import random
from typing import Iterable, List

import pytest
import numpy as np

from social_navigation.mcts.decoupled_mcts import _Node, Action, GameStateProtocol, MCTSConfig, ValueMap

class SimpleGameState(GameStateProtocol):

    def __init__(
        self,
        config: MCTSConfig,
        values: List[int] | None = None,
    ):
        self.config = config
        self.values = values or [0] * self.config.num_actors

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        actors_legal_actions = [list(range(num_actions)) for num_actions in self.config.max_actions]
        return actors_legal_actions

    def apply_actions(self, actions: Action) -> GameStateProtocol:
        return SimpleGameState(
            values=[value + action for value, action in zip(self.values, actions)],
            config=self.config,
        )

    def is_terminal(self) -> bool:
        return all(value == 2 for value in self.values)

    def terminal_values(self) -> ValueMap:
        if self.is_terminal():
            return [1.0] * self.config.num_actors
        else:
            return [0.0] * self.config.num_actors


def test_gets_child():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(config)
    root = _Node(root_state, config, None, None)

    child_1 = root.get_child([0, 0])
    child_2 = root.get_child([0, 0])

    assert child_1 == child_2

def test_expansion_logic():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(config)
    root = _Node(root_state, config, None, None)

    child = root.get_child([0, 0])
    child.backpropagate([1.0, 1.0])
    assert not root.fully_expanded()

    child = root.get_child([1, 0])
    child.backpropagate([1.0, 1.0])
    assert not root.fully_expanded()

    child = root.get_child([0, 1])
    child.backpropagate([1.0, 1.0])
    assert root.fully_expanded()

def test_backpropogation():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(config)
    root = _Node(root_state, config, None, None)
    child_1 = root.get_child([0, 0])
    child_2 = child_1.get_child([1, 0])
    child_3 = child_2.get_child([0, 1])
    
    child_3.backpropagate([2., 2.])

    assert child_3.visits == 1
    assert child_3.visits_by_action[0][0] == 0
    assert child_3.visits_by_action[0][1] == 0
    assert child_3.visits_by_action[1][0] == 0
    assert child_3.visits_by_action[1][1] == 0
    assert child_3.value_by_action[0][0] == 0
    assert child_3.value_by_action[0][1] == 0
    assert child_3.value_by_action[1][0] == 0
    assert child_3.value_by_action[1][1] == 0

    assert child_2.visits == 1
    assert child_2.visits_by_action[0][0] == 1
    assert child_2.visits_by_action[0][1] == 0
    assert child_2.visits_by_action[1][0] == 0
    assert child_2.visits_by_action[1][1] == 1
    assert child_2.value_by_action[0][0] == 2.0
    assert child_2.value_by_action[0][1] == 0
    assert child_2.value_by_action[1][0] == 0
    assert child_2.value_by_action[1][1] == 2.0

    assert child_1.visits == 1
    assert child_1.visits_by_action[0][0] == 0
    assert child_1.visits_by_action[0][1] == 1
    assert child_1.visits_by_action[1][0] == 1
    assert child_1.visits_by_action[1][1] == 0
    assert child_1.value_by_action[0][0] == 0
    assert child_1.value_by_action[0][1] == 2.0
    assert child_1.value_by_action[1][0] == 2.0
    assert child_1.value_by_action[1][1] == 0

    assert root.visits == 1
    assert root.visits_by_action[0][0] == 1
    assert root.visits_by_action[0][1] == 0
    assert root.visits_by_action[1][0] == 1
    assert root.visits_by_action[1][1] == 0
    assert root.value_by_action[0][0] == 2.0
    assert root.value_by_action[0][1] == 0
    assert root.value_by_action[1][0] == 2.0
    assert root.value_by_action[1][1] == 0

    child_4 = child_1.get_child([0, 0])
    child_4.backpropagate([2.0, 2.0])

    assert child_4.visits == 1
    assert child_4.visits_by_action[0][0] == 0
    assert child_4.visits_by_action[0][1] == 0
    assert child_4.visits_by_action[1][0] == 0
    assert child_4.visits_by_action[1][1] == 0
    assert child_4.value_by_action[0][0] == 0
    assert child_4.value_by_action[0][1] == 0
    assert child_4.value_by_action[1][0] == 0
    assert child_4.value_by_action[1][1] == 0

    assert child_1.visits == 2
    assert child_1.visits_by_action[0][0] == 1
    assert child_1.visits_by_action[0][1] == 1
    assert child_1.visits_by_action[1][0] == 2
    assert child_1.visits_by_action[1][1] == 0
    assert child_1.value_by_action[0][0] == 2.0
    assert child_1.value_by_action[0][1] == 2.0
    assert child_1.value_by_action[1][0] == 4.0
    assert child_1.value_by_action[1][1] == 0

    assert root.visits == 2
    assert root.visits_by_action[0][0] == 2
    assert root.visits_by_action[0][1] == 0
    assert root.visits_by_action[1][0] == 2
    assert root.visits_by_action[1][1] == 0
    assert root.value_by_action[0][0] == 4.0
    assert root.value_by_action[0][1] == 0
    assert root.value_by_action[1][0] == 4.0
    assert root.value_by_action[1][1] == 0

def test_preference_build_up():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(config)
    root = _Node(root_state, config, None, None)
    child_1 = root.get_child([0, 0])
    child_2 = root.get_child([1, 0])
    child_3 = root.get_child([0, 1])
    child_4 = root.get_child([1, 1])

    for i in range(100):
        child = root.select_child()
        if child == child_1:
            child.backpropagate([1., 2.])

        if child == child_2:
            child.backpropagate([2., 2.])

        if child == child_3:
            child.backpropagate([1., 1.])

        if child == child_4:
            child.backpropagate([2., 1.])

    assert root.visits_by_action[0][0] < root.visits_by_action[0][1]
    assert root.visits_by_action[1][1] < root.visits_by_action[1][0]

def test_expansion():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(config)
    root = _Node(root_state, config, None, None)
    assert not root.fully_expanded()

    child = root.expand()
    child.backpropagate([0., 0.])
    assert not root.fully_expanded()

    child = root.expand()
    child.backpropagate([0., 0.])
    assert root.fully_expanded()

def test_expansion_uneven_actions():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[3, 1],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(config)
    root = _Node(root_state, config, None, None)
    assert not root.fully_expanded()

    child = root.expand()
    child.backpropagate([0., 0.])
    assert not root.fully_expanded()

    child = root.expand()
    child.backpropagate([0., 0.])
    assert not root.fully_expanded()

    child = root.expand()
    child.backpropagate([0., 0.])
    assert root.fully_expanded()

def test_expansion_unbiased():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )

    expand_1_first = [0, 0]
    expand_1_second = [0, 0]
    num_simulations = 5000
    for i in range(num_simulations):
        root_state = SimpleGameState(config)
        root = _Node(root_state, config, None, None)

        child_1 = root.expand()
        if child_1.actions[0] == 1:
            expand_1_first[0] += 1
        if child_1.actions[1] == 1:
            expand_1_first[1] += 1
        child_1.backpropagate([0., 0.])

        child_2 = root.expand()
        if child_2.actions[0] == 1:
            expand_1_second[0] += 1
        if child_2.actions[1] == 1:
            expand_1_second[1] += 1
        child_2.backpropagate([0., 0.])

    assert (expand_1_first[0] + expand_1_second[0]) == num_simulations
    assert (expand_1_first[1] + expand_1_second[1]) == num_simulations

    five_percent_error = num_simulations * 0.05
    assert -five_percent_error < expand_1_first[0] - expand_1_second[0] < five_percent_error
    assert -five_percent_error < expand_1_first[1] - expand_1_second[1] < five_percent_error