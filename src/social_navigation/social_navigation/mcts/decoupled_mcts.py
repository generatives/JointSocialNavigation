from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable, Dict, List, Optional, Tuple
import numpy as np

Player = int
# Action is now a continuous 2D vector: (v, omega)
Action = Tuple[float, float]
ValueMap = List[float]

@dataclass(frozen=True, slots=True)
class MCTSConfig:
    num_actors: int
    rng: np.random.Generator 
    max_depth: int = 6
    c_puct: float = 1.4
    
    # Progressive Widening parameters
    pw_c: float = 2.0 
    pw_alpha: float = 0.6

    def __post_init__(self) -> None:
        if self.num_actors <= 0:
            raise ValueError("num_actors must be > 0")
        if self.max_depth <= 0:
            raise ValueError("max_depth must be > 0")

class GameStateProtocol:
    """
    Protocol expected by Continuous Decoupled MCTS:
      - sample_actions(rng) -> List[Action]
      - apply_actions(actions) -> GameStateProtocol
      - is_terminal() -> bool
      - terminal_values() -> ValueMap
    """
    def sample_actions(self, rng: np.random.Generator, existing_actions: Optional[List[Action]] = None) -> List[Action]:
            raise NotImplementedError

    def apply_actions(self, actions: List[Action]) -> "GameStateProtocol":
        raise NotImplementedError

    def is_terminal(self) -> bool:
        raise NotImplementedError

    def terminal_values(self) -> ValueMap:
        raise NotImplementedError

RolloutFn = Callable[[GameStateProtocol], ValueMap]

class _Node:
    __slots__ = (
        "state", "config", "parent", "action_taken_to_reach",
        "children", "action_visits", "action_values", "visits"
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
        self.children: List['_Node'] = []
        self.action_visits: List[int] = []
        self.action_values: List[List[float]] = [] 

    def is_fully_expanded_pw(self) -> bool:
        """Determines if we should sample a new action or select an existing one."""
        if self.visits == 0: # Edge case if the node was not expanded yet. PW sucky sucky
            return False
        #print(f"Exponential in PW: {self.visits ** self.config.pw_alpha}")
        #print(f"Progressive widening coefficient: {self.config.pw_c}")
        #print(f"Current children: {self.children}")
        max_children = math.ceil(self.config.pw_c * (self.visits ** self.config.pw_alpha))
        #print(f"Max children: {max_children}")
        return len(self.children) >= max_children

    def select_or_expand(self) -> "_Node":
        #print(f"Current node {self.state}")
        #print(f"Current fully expanded {self.is_fully_expanded_pw()}")
        if not self.is_fully_expanded_pw():
            # EXPAND
            existing_actions = [
                child.action_taken_to_reach[0] # get the [float, float] tuple
                for child in self.children 
                if child.action_taken_to_reach is not None
            ]
            
            # Pass existing_actions to the sampler
            new_actions = self.state.sample_actions(self.config.rng, existing_actions)
            
            child_state = self.state.apply_actions(new_actions)
            child_node = _Node(child_state, self.config, self, new_actions)
            
            self.children.append(child_node)
            self.action_visits.append(0)
            self.action_values.append([0.0] * self.config.num_actors)
            return child_node
        else:
            # SELECT
            sqrt_visits = math.sqrt(self.visits + 1)
            best_score = -math.inf
            best_idx = 0
            #print(f"Children {self.children}")
            for i in range(len(self.children)):
                action_visits = self.action_visits[i]
                # We optimize selection based on the Robot's (Actor 0) value
                q = self.action_values[i][0] / action_visits if action_visits > 0 else 0.0
                u = self.config.c_puct * (sqrt_visits / (1 + action_visits))
                score = q + u
                
                if score > best_score:
                    best_score = score
                    best_idx = i
                    
            return self.children[best_idx]

    def backpropagate(self, values: ValueMap) -> None:
        self.visits += 1
        if self.parent is not None:
            idx = self.parent.children.index(self)
            self.parent.action_visits[idx] += 1
            for actor in range(self.config.num_actors):
                self.parent.action_values[idx][actor] += values[actor]
            self.parent.backpropagate(values)


class MCTS:
    __slots__ = ("rollout_fn", "config")

    def __init__(self, config: MCTSConfig, rollout_fn: RolloutFn) -> None:
        self.rollout_fn = rollout_fn
        self.config = config

    def search(
        self, root_state: GameStateProtocol, *, num_simulations: int
    ) -> Tuple[List[Action], GameStateProtocol, List[GameStateProtocol], None]:
        
        if root_state.is_terminal():
            return None, None, None, None

        root = _Node(root_state, self.config, None, None)

        for _ in range(num_simulations):
            self._search_iteration(root)
        
        best_actions = self._most_visited_actions(root)
        #print(f"Best actions at {root_state}, : {best_actions}")
        #print(f"Root of children {root.children}")
        
        best_child_idx = root.children.index(next(c for c in root.children if c.action_taken_to_reach == best_actions))

        child_node = root.children[best_child_idx]
        
        state_trajectory = []
        for child in sorted(root.children, key=lambda n: n.visits, reverse=True)[:5]:
            trajectory = self._get_expected_trajectory(child)
            state_trajectory.extend([state for node in trajectory for state in [node.parent.state, node.state] if node.parent])
        
        return best_actions, child_node.state, state_trajectory, None
        
    def _search_iteration(self, root: _Node):
        node = root
        depth = 0

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
            best_child = next(c for c in node.children if c.action_taken_to_reach == best_actions)
            nodes.append(best_child)
            node = best_child
        return nodes

    def _most_visited_actions(self, root: _Node) -> List[Action]:
        if not root.children:
            return [(0.0, 0.0)] * self.config.num_actors
        best_idx = np.argmax(root.action_visits)
        return root.children[best_idx].action_taken_to_reach