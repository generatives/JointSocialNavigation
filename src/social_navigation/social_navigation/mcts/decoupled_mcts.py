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
    num_actions: Tuple[int, ...]
    max_depth: int = 6
    child_index_steps: Tuple[int, ...] = field(init=False)
    legal_actions: Tuple[Tuple[int, ...], ...] = field(init=False)

    def __post_init__(self) -> None:
        if self.num_actors <= 0:
            raise ValueError("num_actors must be > 0")
        if len(self.num_actions) != self.num_actors:
            raise ValueError("num_actions must have exactly num_actors entries")
        if any(action_count <= 0 for action_count in self.num_actions):
            raise ValueError("all entries in num_actions must be > 0")
        if self.max_depth <= 0:
            raise ValueError("max_depth must be > 0")

        num_actions = tuple(self.num_actions)
        object.__setattr__(self, "num_actions", num_actions)

        child_index_steps = []
        radix = 1
        for action_count in num_actions:
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
            tuple(tuple(range(action_count)) for action_count in num_actions),
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
        self.visits_by_action: List[List[int]] = [[0] * action_count for action_count in config.num_actions]
        self.value_by_action: List[List[float]] = [[0.0] * action_count for action_count in config.num_actions]
        self._children: Dict[int, "_Node"] = {}
        self.visits = 0
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



class MCTS:
    """
    High-performance MCTS with per-player heuristic priors and pluggable rollouts.

    Rollout function must return a mapping of player -> value.
    Heuristic function returns a non-negative prior for (state, player, action).
    """

    __slots__ = (
        "rollout_fn",
        "heuristic_fn",
        "c_puct",
        "config",
        "rng",
    )

    def __init__(
        self,
        config: MCTSConfig,
        rollout_fn: RolloutFn,
        heuristic_fn: Optional[HeuristicFn] = None,
        *,
        c_puct: float = 1.4,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.rollout_fn = rollout_fn
        self.heuristic_fn = heuristic_fn
        self.c_puct = c_puct
        self.config = config
        self.rng = rng or random.Random()

    def search(
        self,
        root_state: GameStateProtocol,
        *,
        num_simulations: int,
    ) -> Tuple[Action, GameStateProtocol]:
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

        for i in range(num_simulations):
            #print(f"Starting simulation {i}")
            node = root
            depth = 0

            fully_expanded = node.fully_expanded()
            is_terminal = node.state.is_terminal()
            
            #stats["check_expanded"] += 1
            #stats["cached_check_expanded"] += 1 if node._fully_expanded else 0
            #stats["check_terminal"] += 1
            #print(f"Selecting node for simulation {i}")
            while fully_expanded and not is_terminal and depth < self.config.max_depth:
                node = self._select_child(node)
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
                    node = self._select_child_to_expand(node)
                #stats["rollout"] += 1
                #stats["rollout_total_depth"] += node.config.max_depth - node.state.depth
                values = self.rollout_fn(node.state)

            #print(f"Backpropagating simulation {i}")
            #stats["backpropogate"] += 1
            #stats["backpropogate_total_depth"] += node.state.depth
            self._backpropagate(node, values)
            #print(f"Completed simulation {i}")

        best_actions = self._best_actions(root)
        child_node = root.get_child(best_actions)
        return best_actions, child_node.state, stats

    def _best_actions(self, root: _Node) -> Action:
        actions = []
        for actor in range(self.config.num_actors):
            action, visits = max(enumerate(root.visits_by_action[actor]), key=lambda t: t[1])
            actions.append(action)

        return actions

    def _select_child(self, node: _Node) -> _Node:
        sqrt_visits = math.sqrt(node.visits + 1)

        actions = [0] * self.config.num_actors

        for actor in range(self.config.num_actors):
            best_score = -math.inf
            best_action = 0
            for action in range(self.config.num_actions[actor]):
                action_visits = node.visits_by_action[actor][action]
                q = node.value_by_action[actor][action] / action_visits if action_visits > 0 else 0.0
                u = self.c_puct * (sqrt_visits / (1 + action_visits))
                score = q + u
                if score > best_score:
                    best_score = score
                    best_action = action
            actions[actor] = best_action

        return node.get_child(actions)
    
    def _select_child_to_expand(self, node: _Node) -> _Node:
        selected_actions = []
        for actor in range(self.config.num_actors):
            unvisited_actions = [
                action
                for action in range(self.config.num_actions[actor])
                if node.visits_by_action[actor][action] == 0
            ]
            if any(unvisited_actions):
                selected_actions.append(self.rng.choice(unvisited_actions))
            else:
                selected_actions.append(self.rng.choice(range(self.config.num_actions[actor])))

        return node.get_child(selected_actions)

    def _backpropagate(self, node: _Node, values: ValueMap) -> None:
        while node is not None:
            node.visits += 1
            parent = node.parent
            if parent is not None:
                for actor, action in enumerate(node.actions):
                    parent.visits_by_action[actor][action] += 1
                    parent.value_by_action[actor][action] += values[actor]
                    
            node = parent
