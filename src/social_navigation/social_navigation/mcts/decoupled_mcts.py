from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple


Player = int
Action = Any
ValueMap = List[float]

@dataclass(frozen=True, slots=True)
class MCTSConfig:
    num_actors: int
    max_actions: Tuple[int, ...]
    rng: random.Random
    max_depth: int = 6
    c_puct: float = 1.4
    child_index_steps: Tuple[int, ...] = field(init=False)
    legal_actions: Tuple[Tuple[int, ...], ...] = field(init=False)

    def __post_init__(self) -> None:
        if self.num_actors <= 0:
            raise ValueError("num_actors must be > 0")
        if len(self.max_actions) != self.num_actors:
            raise ValueError("num_actions must have exactly num_actors entries")
        if any(action_count <= 0 for action_count in self.max_actions):
            raise ValueError("all entries in num_actions must be > 0")
        if self.max_depth <= 0:
            raise ValueError("max_depth must be > 0")

        max_actions = tuple(self.max_actions)
        object.__setattr__(self, "max_actions", max_actions)

        child_index_steps = []
        radix = 1
        for action_count in max_actions:
            child_index_steps.append(radix)
            radix *= action_count
        object.__setattr__(
            self,
            "child_index_steps",
            tuple(child_index_steps),
        )
        object.__setattr__(
            self,
            "legal_actions",
            tuple(tuple(range(action_count)) for action_count in max_actions),
        )


class GameStateProtocol:
    """
    Minimal protocol expected by Decoupled MCTS:
      - legal_actions() -> Iterable[Iterable[Action]]
      - apply_action(actions) -> GameStateProtocol
      - is_terminal() -> bool
      - terminal_values() -> ValueMap
    """

    def legal_actions(self) -> Iterable[Iterable[Action]]:  # pragma: no cover - interface
        raise NotImplementedError

    def apply_actions(self, action: Action) -> "GameStateProtocol":  # pragma: no cover - interface
        raise NotImplementedError

    def is_terminal(self) -> bool:  # pragma: no cover - interface
        raise NotImplementedError

    def terminal_values(self) -> ValueMap:  # pragma: no cover - interface
        raise NotImplementedError


HeuristicFn = Callable[[GameStateProtocol, Player, Action], float]
RolloutFn = Callable[[GameStateProtocol], ValueMap]
ActionKeyFn = Callable[[GameStateProtocol, Action], Any]


class _Node:
    __slots__ = (
        "state",
        "config",
        "parent",
        "actions",
        "_children",
        "visits",
        "visits_by_action",
        "value_by_action",
        "_fully_expanded"
    )

    def __init__(
        self,
        state: GameStateProtocol,
        config: MCTSConfig,
        parent: Optional["_Node"],
        actions: Optional[List[int]],
    ) -> None:
        self.state = state
        self.config = config
        self.parent = parent
        self.actions = actions
        # First index is agent, second is action
        self.visits = 0
        self.visits_by_action: List[List[int]] = [[0] * action_count for action_count in config.max_actions]
        self.value_by_action: List[List[float]] = [[0.0] * action_count for action_count in config.max_actions]
        self._children: Dict[int, "_Node"] = {}
        self._fully_expanded = False

    def _get_child_index(self, actions: List[int]):
        index = 0
        for actor_idx, action in enumerate(actions):
            index += self.config.child_index_steps[actor_idx] * action
        return index


    def get_child(self, actions: List[int]):
        index = self._get_child_index(actions)
        child_node = self._children.get(index, None)
        if child_node is None:
            child_state = self.state.apply_actions(actions)
            child_node = _Node(child_state, self.config, self, actions)
            self._children[index] = child_node

        return child_node
    
    def fully_expanded(self) -> bool:
        if not self._fully_expanded:
            self._fully_expanded = all(count > 0 for action_count in self.visits_by_action for count in action_count)
        
        return self._fully_expanded
    

    def select_child(self) -> _Node:
        sqrt_visits = math.sqrt(self.visits + 1)

        selected_actions = [0] * self.config.num_actors
        legal_actions = self.state.legal_actions()

        for actor, actions in enumerate(legal_actions):
            best_score = -math.inf
            best_action = 0
            action_data = []
            heuristic_values = []
            
            for action in actions:
                action_visits = self.visits_by_action[actor][action]
                q = self.value_by_action[actor][action] / action_visits if action_visits > 0 else 0.0
                u = self.config.c_puct * (sqrt_visits / (1 + action_visits))
                
                if actor == 0:
                    h = self.state.heuristic_robot_goal_score_for_action(action)
                else:
                    h = 0.0
                action_data.append((action, q, u, h))
                heuristic_values.append(h)
            
            h_min = min(heuristic_values)
            h_max = max(heuristic_values)

            for action, q, u, h in action_data:
                if h_max > h_min:
                    normalized_score = (h - h_min) / (h_max - h_min)
                else:
                    normalized_score = 1.0

                score = q + u * normalized_score

                if score > best_score:
                    best_score = score
                    best_action = action
            selected_actions[actor] = best_action

        return self.get_child(selected_actions)
    

    def expand(self) -> _Node:
        selected_actions = []
        legal_actions = self.state.legal_actions()
        for actor, actions in enumerate(legal_actions):
            unvisited_actions = [
                action for action in actions
                if self.visits_by_action[actor][action] == 0
            ]
            if len(unvisited_actions) > 0:
                selected_actions.append(self.config.rng.choice(unvisited_actions))
            else:
                selected_actions.append(self.config.rng.choice(range(self.config.max_actions[actor])))

        return self.get_child(selected_actions)
    

    def backpropagate(self, values: ValueMap) -> None:
        self.visits += 1
        parent = self.parent
        if parent is not None:
            for actor, action in enumerate(self.actions):
                parent.visits_by_action[actor][action] += 1
                parent.value_by_action[actor][action] += values[actor]
            parent.backpropagate(values)



class MCTS:
    """
    High-performance MCTS with per-player heuristic priors and pluggable rollouts.

    Rollout function must return a mapping of player -> value.
    Heuristic function returns a non-negative prior for (state, player, action).
    """

    __slots__ = (
        "rollout_fn",
        "heuristic_fn",
        "config"
    )

    def __init__(
        self,
        config: MCTSConfig,
        rollout_fn: RolloutFn,
        heuristic_fn: Optional[HeuristicFn] = None
    ) -> None:
        self.rollout_fn = rollout_fn
        self.heuristic_fn = heuristic_fn
        self.config = config

    def search(
        self,
        root_state: GameStateProtocol,
        *,
        num_simulations: int,
    ) -> Tuple[Action, GameStateProtocol]:
        
        root, stats = self._search_internal(root_state, num_simulations=num_simulations)
        
        best_actions = self._most_visited_actions(root)
        child_node = root.get_child(best_actions)
        trajectory = self._get_expected_trajectory(child_node)
        state_trajectory = [node.state for node in trajectory]
        return best_actions, child_node.state, state_trajectory, stats
        
    def _search_iteration(self, root: _Node, stats: Dict | None):
        node = root
        depth = 0

        fully_expanded = node.fully_expanded()
        is_terminal = node.state.is_terminal()
        
        #stats["check_expanded"] += 1
        #stats["cached_check_expanded"] += 1 if node._fully_expanded else 0
        #stats["check_terminal"] += 1
        #print(f"Selecting node for simulation {i}")
        while fully_expanded and not is_terminal and depth < self.config.max_depth:
            node = node.select_child()
            fully_expanded = node.fully_expanded()
            is_terminal = node.state.is_terminal()
            depth += 1
            #stats["select_child"] += 1
            #stats["check_expanded"] += 1
            #stats["cached_check_expanded"] += 1 if node._fully_expanded else 0
            #stats["check_terminal"] += 1

        if is_terminal:
            #print(f"Reached terminal state for simulation {i}")
            values = node.state.terminal_values()
        else:
            #print(f"Rolling out simulation {i}")
            if not fully_expanded and depth < self.config.max_depth:
                #stats["select_child_expand"] += 1
                node = node.expand()
            #stats["rollout"] += 1
            #stats["rollout_total_depth"] += node.config.max_depth - node.state.depth
            values = self.rollout_fn(node.state)

        #print(f"Backpropagating simulation {i}")
        #stats["backpropogate"] += 1
        #stats["backpropogate_total_depth"] += node.state.depth
        node.backpropagate(values)
        #print(f"Completed simulation {i}")

    def _search_internal(
        self,
        root_state: GameStateProtocol,
        *,
        num_simulations: int,
    ) -> Tuple[_Node, Dict | None]:
        #print("Starting search")
        root = _Node(root_state, self.config, None, None)

        stats = None
        #stats = {
        #    "check_expanded": 0,
        #    "cached_check_expanded": 0,
        #    "check_terminal": 0,
        #    "select_child": 0,
        #    "select_child_expand": 0,
        #    "rollout": 0,
        #    "rollout_total_depth": 0,
        #    "backpropogate": 0,
        #    "backpropogate_total_depth": 0
        #}
        chosen_actions = []
        for i in range(num_simulations):
            self._search_iteration(root, stats)

            if i % 100 == 0:
                best_actions = self._most_visited_actions(root)
                chosen_actions.append(best_actions[0])

        #print(chosen_actions)

        return root, stats
    
    def _get_expected_trajectory(self, node: _Node):
        nodes = [node]
        while not node.state.is_terminal() and node.visits > 0:
            best_actions = self._most_visited_actions(node)
            node = node.get_child(best_actions)
            nodes.append(node)
        return nodes


    def _highest_scoring_actions(self, root: _Node) -> Action:
        actions = []
        for actor in range(self.config.num_actors):
            values = root.value_by_action[actor]
            visits = root.visits_by_action[actor]
            action = max(range(len(values)), key=lambda i: values[i] / visits[i] if visits[i] > 0 else 0)
            actions.append(action)

        return actions


    def _most_visited_actions(self, root: _Node) -> Action:
        actions = []
        for actor in range(self.config.num_actors):
            max_visits = max(root.visits_by_action[actor])
            action = random.choice([action for action, visits in enumerate(root.visits_by_action[actor]) if visits >= max_visits])
            actions.append(action)

        return actions
    
