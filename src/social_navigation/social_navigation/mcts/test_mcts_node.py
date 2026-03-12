import random
from typing import Iterable

import pytest
import numpy as np

from social_navigation.mcts.decoupled_mcts import _Node, Action, GameStateProtocol, MCTSConfig, ValueMap

class SimpleGameState(GameStateProtocol):

    def __init__(
        self,
        positions: np.ndarray,
    ):
        self.positions = positions

        self.possible_actions = np.array(
            [
                +1,
                -1,
            ],
            dtype=np.int32,
        )

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        actors_legal_actions = [list(range(self.possible_actions.shape[0]))] * self.positions.shape[0]

        return actors_legal_actions

    def apply_actions(self, actions: Action) -> GameStateProtocol:
        new_positions = self.positions.copy()
        for actor, action_idx in enumerate(actions):
            new_positions[actor] = new_positions[actor] + self.possible_actions[int(action_idx)]

        return SimpleGameState(
            positions=new_positions,
        )

    def is_terminal(self) -> bool:
        return np.all(self.positions == 2)

    def terminal_values(self) -> ValueMap:
        if self.is_terminal():
            return [1.0] * self.positions.shape[0]
        else:
            return [0.0] * self.positions.shape[0]


def test_gets_child():
    config = MCTSConfig(
        num_actors=2,
        max_actions=[2, 2],
        rng=random.Random(2),
        max_depth=6,
    )
    root_state = SimpleGameState(
        np.array([
            0,
            0
        ],
        dtype=np.int32)
    )
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
    root_state = SimpleGameState(
        np.array([
            0,
            0
        ],
        dtype=np.int32)
    )
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
    root_state = SimpleGameState(
        np.array([
            0,
            0
        ],
        dtype=np.int32)
    )
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