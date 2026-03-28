import random
from typing import Iterable, List

import pytest
import numpy as np

from social_navigation.mcts.decoupled_mcts import MCTS, _Node, Action, GameStateProtocol, MCTSConfig, ValueMap

class NeverendingGameState(GameStateProtocol):

    def __init__(
        self,
        config: MCTSConfig,
        depth: int = 0,
        values: List[int] | None = None,
    ):
        self.config = config
        self.depth = depth
        self.values = values or [0] * self.config.num_actors

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        actors_legal_actions = [list(((action_idx, 1.0) for action_idx in range(num_actions))) for num_actions in self.config.max_actions]
        return actors_legal_actions

    def apply_actions(self, actions: Action) -> GameStateProtocol:
        return NeverendingGameState(
            values=[value + action for value, action in zip(self.values, actions)],
            depth = self.depth + 1,
            config=self.config,
        )

    def is_terminal(self) -> bool:
        return False

    def terminal_values(self) -> ValueMap:
        return [0.0] * self.config.num_actors
    
class AddingGameState(GameStateProtocol):

    def __init__(
        self,
        config: MCTSConfig,
        depth: int = 0,
        values: List[int] | None = None,
    ):
        self.config = config
        self.depth = depth
        self.values = values or [0] * self.config.num_actors

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        actors_legal_actions = [list(((action_idx, 1.0) for action_idx in range(num_actions))) for num_actions in self.config.max_actions]
        return actors_legal_actions

    def apply_actions(self, actions: Action) -> GameStateProtocol:
        return AddingGameState(
            values=[value + action for value, action in zip(self.values, actions)],
            depth = self.depth + 1,
            config=self.config,
        )

    def is_terminal(self) -> bool:
        return all(value == 2 for value in self.values)

    def terminal_values(self) -> ValueMap:
        if self.is_terminal():
            return [1.0] * self.config.num_actors
        else:
            return [0.0] * self.config.num_actors


def simple_rollout_fn(max_depth: int = 4):

    def simple_rollout(state: GameStateProtocol):
        rollout_depth = 0
        while not state.is_terminal() and rollout_depth < max_depth:
            legal_actions = state.legal_actions()
            selected_actions = []
            for actions in legal_actions:
                if len(actions) > 1:
                    action, _ = random.choice(actions)
                else:
                    action, _ = actions[0]

                selected_actions.append(action)

            state = state.apply_actions(selected_actions)
            rollout_depth += 1

        return state.terminal_values()
    
    return simple_rollout

def test_search_depth():
    config = MCTSConfig(
        num_actors=1,
        max_actions=[2],
        rng=random.Random(2),
        max_depth=3,
    )
    root_state = NeverendingGameState(config)
    mcts = MCTS(config, simple_rollout_fn())
    root, stats = mcts._search_internal(root_state, num_simulations=14)

    nodes_found = [[], [], [], []]
    nodes_to_search = [root]

    while any(nodes_to_search):
        node = nodes_to_search.pop()
        nodes_to_search.extend(list(node._children.values()))
        assert node.state.depth in [0, 1, 2, 3]
        nodes_found[node.state.depth].append(node)

    total_visits_per_depth = [sum(node.visits for node in nodes) for nodes in nodes_found]

    assert len(nodes_found[0]) == 1
    assert total_visits_per_depth[0] == 14

    assert len(nodes_found[1]) == 2
    assert total_visits_per_depth[1] == 14

    assert len(nodes_found[2]) == 4
    assert total_visits_per_depth[2] == 12

    assert len(nodes_found[3]) == 8
    assert total_visits_per_depth[3] == 8


def test_expand_terminal():
    config = MCTSConfig(
        num_actors=1,
        max_actions=[2],
        rng=random.Random(2),
        max_depth=3,
    )
    root_state = AddingGameState(config)
    mcts = MCTS(config, simple_rollout_fn())
    root, stats = mcts._search_internal(root_state, num_simulations=15)

    nodes_to_search = [root]

    while any(nodes_to_search):
        node = nodes_to_search.pop()
        children = list(node._children.values())

        if node.state.values[0] == 2:
            assert len(children) == 0

        nodes_to_search.extend(children)
    

def test_find_only_option():
    config = MCTSConfig(
        num_actors=1,
        max_actions=[2],
        rng=random.Random(2),
        max_depth=2,
    )
    root_state = AddingGameState(config)
    mcts = MCTS(config, simple_rollout_fn(0))
    best_actions, best_child_state, state_trajectory, stats = mcts.search(root_state, num_simulations=100)

    assert best_actions[0] == 1
    assert len(state_trajectory) == 2
    assert state_trajectory[1].values[0] == 2