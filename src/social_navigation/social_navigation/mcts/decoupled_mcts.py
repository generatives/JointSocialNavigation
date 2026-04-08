from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import Any, Callable, Dict, List, Optional, Tuple
import numpy as np
import itertools


Player = int
# Action is now a continuous 2D vector: (v, omega)
Action = Tuple[float, float]
ValueMap = List[float]


@dataclass(frozen=True, slots=True)
class MCTSConfig:
    num_actors: int
    rng: np.random.Generator 
    max_actions: Tuple[int, ...]  
    
    max_depth: int = 6
    c_puct: float = 1.4
    
    # Progressive Widening parameters
    pw_c: float = 2.0 
    pw_alpha: float = 0.3

    # Fields generated in __post_init__
    child_index_steps: Tuple[int, ...] = field(init=False)
    sampled_actions: Tuple[Tuple[Action, ...], ...] = field(init=False)

    def __post_init__(self) -> None:
        if self.num_actors <= 0:
            raise ValueError("num_actors must be > 0")
        if len(self.max_actions) != self.num_actors:
            raise ValueError("max_actions must have length equal to num_actors")
        if any(action_count <= 0 for action_count in self.max_actions):
            raise ValueError("all entries in max_actions must be > 0")
        
        # Calculate steps for Cartesian indexing
        child_index_steps = []
        radix = 1
        for action_count in self.max_actions:
            child_index_steps.append(radix)
            radix *= action_count
        
        object.__setattr__(self, "child_index_steps", tuple(child_index_steps))
        # Placeholder for discrete action mapping
        object.__setattr__(
            self,
            "sampled_actions",
            tuple(tuple(range(action_count)) for action_count in self.max_actions),
        )

class GameStateProtocol:
    """
    Protocol expected by Continuous Decoupled MCTS:
      
      - sample_actions(rng) -> List[Action]
      - apply_actions(actions) -> GameStateProtocol
      - is_terminal() -> bool
      - terminal_values() -> ValueMap
    """

    def sample_action(self, actor_idx: int, rng: np.random.Generator, existing_actions: Optional[List[Tuple[Action, float]]] = None) -> Tuple[Action, float]:
            raise NotImplementedError

    def apply_actions(self, actions: List[Tuple[Action, float]]) -> "GameStateProtocol":
        raise NotImplementedError

    def is_terminal(self) -> bool:
        raise NotImplementedError

    def terminal_values(self) -> ValueMap:
        raise NotImplementedError

RolloutFn = Callable[[GameStateProtocol], ValueMap]

class _Node:
    __slots__ = (
        "state", "config", "parent", "action_taken_to_reach",
        "children", "action_visits", "action_values", "action_definitions", "visits", "action_pools"
    )

    def __init__(
        self,
        state: GameStateProtocol,
        config: MCTSConfig,
        parent: Optional["_Node"],
        action_taken_to_reach: Optional[List[Action]],
    ) -> None:
        self.state = state
        self.config = config
        self.parent = parent
        self.action_taken_to_reach = action_taken_to_reach
        self.visits = 0
        
        # Parallel lists to track continuous action branches
        self.children: Dict[Tuple[int, ...], '_Node'] = {}
        self.action_visits: List[List[int]] = [[] for _ in range(self.config.num_actors)]
        self.action_values: List[List[float]] = [[] for _ in range(self.config.num_actors)]
        self.action_definitions: List[List[Tuple[Action, float]]] = [[] for _ in range(self.config.num_actors)]

    def get_child(self, actions: Tuple[int, ...]) -> "_Node":
        if actions in self.children:
            return self.children[actions]
        else:
            action_definitions = [self.action_definitions[actor_idx][action_idx] for actor_idx, action_idx in enumerate(actions)]
            new_child_state = self.state.apply_actions(action_definitions)
            new_child_node = _Node(new_child_state, self.config, self, actions)
            self.children[actions] = new_child_node
            return new_child_node

    def get_target_action_counts(self) -> List[int]:
        if self.visits == 0: # Edge case if the node was not expanded yet. PW sucky sucky
            return [1] * self.config.num_actors

        pw_limit = math.ceil(self.config.pw_c * (self.visits ** self.config.pw_alpha))

        target_counts = [
            pw_limit if pw_limit < max_actions else max_actions
            for max_actions
            in self.config.max_actions
        ]
        return target_counts

    def is_fully_expanded_pw(self) -> bool:
        """Determines if we should sample a new action or select an existing one."""
        target_action_counts = self.get_target_action_counts()
        action_counts = [len(actions) for actions in self.action_visits]
        actor_is_fully_expanded = [
            action_count >= target_count
            for action_count, target_count
            in zip(action_counts, target_action_counts)
        ]
        return all(actor_is_fully_expanded)
    
    def print_robot_scores(self):
        actor_idx = 0
        action_visits = self.action_visits[actor_idx]
        action_values = self.action_values[actor_idx]
        num_actions = len(action_visits)
        sqrt_visits = math.sqrt(self.visits + 1)
        
        q_values = [
            action_values[action_idx] / action_visits[action_idx] if action_visits[action_idx] > 0 else 0.0
            for action_idx
            in range(num_actions)
        ]
        min_q = min(q_values)
        max_q = max(q_values)
        scaled_q_values = [(q - min_q) / (max_q - min_q + 1e-8) for q in q_values]
        
        score_sum = sum([score for _, score in self.action_definitions[actor_idx]])
        scores = []
        for action_idx in range(num_actions):
            visits = action_visits[action_idx]
            _, action_score = self.action_definitions[actor_idx][action_idx]
            probability = action_score / score_sum
            q = scaled_q_values[action_idx]
            u = self.config.c_puct * probability * (sqrt_visits / (1 + visits))
            score = q + u
            scores.append((q, u, score))
            
        print(scores)

    def select_or_expand(self) -> "_Node":
        #print(f"Current node {self.state}")
        #print(f"Current fully expanded {self.is_fully_expanded_pw()}")
        selected_actions = []
        target_action_counts = self.get_target_action_counts()
        for actor_idx in range(self.config.num_actors):
            action_visits = self.action_visits[actor_idx]
            action_values = self.action_values[actor_idx]
            num_actions = len(action_visits)
            if num_actions < target_action_counts[actor_idx]:
                existing_actions = self.action_definitions[actor_idx]

                # Pass them to the state
                target_action_counts = self.get_target_action_counts()
                new_action = self.state.sample_action(actor_idx, self.config.rng, existing_actions)

                self.action_visits[actor_idx].append(0)
                self.action_values[actor_idx].append(0.0)
                self.action_definitions[actor_idx].append(new_action)

                selected_actions.append(len(self.action_visits[actor_idx]) - 1)
            else:
                sqrt_visits = math.sqrt(self.visits + 1)
                best_score = -math.inf
                best_idx = 0
                score_sum = sum([score for _, score in self.action_definitions[actor_idx]])

                q_values = [
                    action_values[action_idx] / action_visits[action_idx] if action_visits[action_idx] > 0 else 0.0
                    for action_idx
                    in range(num_actions)
                ]
                min_q = min(q_values)
                max_q = max(q_values)
                scaled_q_values = [(q - min_q) / (max_q - min_q + 1e-8) for q in q_values]
                
                for action_idx in range(num_actions):
                    visits = action_visits[action_idx]
                    _, action_score = self.action_definitions[actor_idx][action_idx]
                    probability = action_score / score_sum
                    q = scaled_q_values[action_idx]
                    u = self.config.c_puct * probability * (sqrt_visits / (1 + visits))
                    score = q + u
                    
                    if score > best_score:
                        best_score = score
                        best_idx = action_idx

                selected_actions.append(best_idx)

        child_node = self.get_child(tuple(selected_actions))

        return child_node

    def backpropagate(self, values: ValueMap) -> None:
        self.visits += 1
        parent = self.parent
        if parent is not None:
            for actor, action in enumerate(self.action_taken_to_reach):
                parent.action_visits[actor][action] += 1
                parent.action_values[actor][action] += values[actor]
            parent.backpropagate(values)


class MCTS:
    __slots__ = ("rollout_fn", "config")

    def __init__(self, config: MCTSConfig, rollout_fn: RolloutFn) -> None:
        self.rollout_fn = rollout_fn
        self.config = config

    def search(
        self, root_state: GameStateProtocol, *, num_simulations: int
    ) -> Tuple[List[Action], GameStateProtocol, List[GameStateProtocol], None]:
        
        if root_state.is_terminal():
            print("Root state is terminal")
            return None, None, None, None

        root = _Node(root_state, self.config, None, None)

        for i in range(num_simulations):
            self._search_iteration(root)
            #if i % 100 == 0:
                #print(f"Iteration: {i}")
                #root.print_robot_scores()
                #print(root.action_visits[0])

        state_trajectory = []
        for child in sorted(root.children.values(), key=lambda n: n.visits, reverse=True)[:5]:
            trajectory = self._get_expected_trajectory(child)
            state_trajectory.extend([state for node in trajectory for state in [node.parent.state, node.state] if node.parent])

        next_node = root
        actions = []
        states = []
        #robot_action_percentages = []
        while next_node.visits > 0 and len(next_node.children) > 0:
            best_actions = self._most_visited_actions(next_node)
            action_definition = [next_node.action_definitions[actor_idx][action_idx][0] for actor_idx, action_idx in enumerate(best_actions)]
            actions.append(action_definition)
            states.append(next_node.state)

            #robot_action_percentages.append(next_node.action_visits[0][best_actions[0]] / next_node.visits)

            next_node = next_node.get_child(tuple(best_actions))

        #print(robot_action_percentages)
        
        return actions, states, state_trajectory, None
        
    def _search_iteration(self, root: _Node):
        node = root
        depth = 0

        #if not root.is_fully_expanded_pw():
        #    print("Expanding Root")

        # Traverse tree
        while node.is_fully_expanded_pw() and not node.state.is_terminal() and depth < self.config.max_depth:
            node = node.select_or_expand()
            depth += 1

        # Evaluate
        if node.state.is_terminal():
            values = node.state.terminal_values()
        else:
            if depth < self.config.max_depth:
                node = node.select_or_expand() # Force an expansion step
            values = self.rollout_fn(node.state)

        # Backpropagate
        node.backpropagate(values)

    def _get_expected_trajectory(self, node: _Node):
        nodes = []
        while not node.state.is_terminal() and node.visits > 0 and len(node.children) > 0:
            best_actions = self._most_visited_actions(node)
            best_child = node.get_child(tuple(best_actions))
            nodes.append(best_child)
            node = best_child
        return nodes

    def _most_visited_actions(self, root: _Node) -> List[Action]:
        actions = []
        for actor in range(self.config.num_actors):
            action_visits = root.action_visits[actor]
            max_visits = max(action_visits)
            action = self.config.rng.choice([action for action, visits in enumerate(action_visits) if visits >= max_visits])
            actions.append(action)

        return actions