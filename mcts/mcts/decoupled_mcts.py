from __future__ import annotations

import math
import random
from typing import Any, Callable, Dict, Hashable, Iterable, List, Optional, Tuple
import itertools


Player = int
Action = Any
ValueMap = List[float]


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
        "parent",
        "actions",
        "_children",
        "visits",
        "num_actors",
        "num_actions"
    )

    def __init__(
        self,
        state: GameStateProtocol,
        parent: Optional["_Node"],
        actions: Optional[List[int]],
        num_actors: int,
        num_actions: int
    ) -> None:
        self.state = state
        self.parent = parent
        self.actions = actions
        self.num_actors = num_actors
        self.num_actions = num_actions
        # First index is agent, second is action
        self.visits_by_action: List[List[int]] = [[0]*num_actions for i in range(num_actors)]
        self.value_by_action: List[List[float]] = [[0]*num_actions for i in range(num_actors)]
        self._children: List["_Node"] = [None] * (num_actions ** num_actors)
        self.visits = 0

    def _get_child_index(self, actions: List[int]):
        index = 0
        for actor_idx in range(self.num_actors, 0, -1):
            step = self.num_actions ** actor_idx
            index += step * actions[actor_idx]
        return index


    def get_child(self, actions: List[int]):
        index = self._get_child_index(actions)
        child_node = self._children[index]
        if child_node is None:
            child_state = self.state.apply_actions(actions)
            child_node = _Node(child_state, self, self.num_actors, self.num_actions)
            self._children[index] = child_node

        return child_node



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
        "max_depth",
        "num_actors",
        "num_actions",
        "rng",
    )

    def __init__(
        self,
        rollout_fn: RolloutFn,
        num_actors: int,
        num_actions: int,
        heuristic_fn: Optional[HeuristicFn] = None,
        *,
        c_puct: float = 1.4,
        max_depth: int = 6,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.rollout_fn = rollout_fn
        self.heuristic_fn = heuristic_fn
        self.c_puct = c_puct
        self.max_depth = max_depth
        self.num_actors = num_actors
        self.num_actions = num_actions
        self.rng = rng or random.Random()

    def search(
        self,
        root_state: GameStateProtocol,
        *,
        num_simulations: int,
    ) -> Tuple[Action, GameStateProtocol]:
        #print("Starting search")
        root = _Node(root_state, None, self.num_actors, self.num_actions)

        for i in range(num_simulations):
            #print(f"Starting simulation {i}")
            node = root
            depth = 0
            #print(f"Selecting node for simulation {i}")
            while node._children and depth < self.max_depth:
                node = self._select_child(node)
                depth += 1

            if node.state.is_terminal():
                #print(f"Reached terminal state for simulation {i}")
                values = node.state.terminal_values()
            else:
                #print(f"Rolling out simulation {i}")
                values = self.rollout_fn(node.state)

            #print(f"Backpropagating simulation {i}")
            self._backpropagate(node, values)
            #print(f"Completed simulation {i}")

        best_actions = self._best_actions(root)
        child_state = root.get_child(best_actions)
        return best_actions, child_state

    def _best_actions(self, root: _Node) -> Action:
        actions = []
        for actor in range(self.num_actors):
            action, visits = max(enumerate(root.visits_by_action[actor]), key=lambda t: t[1])
            actions.append(action)

        return actions

    def _select_child(self, node: _Node) -> _Node:
        player = node.state.current_player()
        sqrt_visits = math.sqrt(node.visits + 1)

        actions = [0] * self.num_actors

        for actor in range(self.num_actors):
            best_score = -math.inf
            best_action = 0
            for action in range(self.num_actions):
                q = node.value_by_action[actor][action] / node.visits
                action_visits = node.visits_by_action[actor][action]
                u = self.c_puct * (sqrt_visits / (1 + action_visits))
                score = q + u
                if score > best_score:
                    best_score = score
                    best_action = action
            actions[actor] = best_action

        return node.get_child(actions)

    def _backpropagate(self, node: _Node, values: ValueMap) -> None:
        while node is not None:
            node.visits += 1
            parent = node.parent
            if parent is not None:
                for actor, action in enumerate(node.actions):
                    parent.visits_by_action[actor][action] += 1
                    parent.value_by_action[actor][action] += values[actor]
                    
            node = parent
