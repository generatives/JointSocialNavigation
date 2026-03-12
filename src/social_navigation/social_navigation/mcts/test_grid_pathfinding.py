import random
from typing import Iterable
import numpy as np

from social_navigation.mcts.decoupled_mcts import (
    Action,
    GameStateProtocol,
    MCTS,
    MCTSConfig,
    ValueMap,
)

class OrderedTwoAgentGameState(GameStateProtocol):

    def __init__(
        self,
        grid_map: np.ndarray,
        positions: np.ndarray,
        goal_position: np.ndarray,
        scheduled_order: tuple[int, ...],
        depth: int = 0,
        arrival_steps: tuple[int | None, ...] | None = None,
    ):
        self.map = grid_map
        self.positions = positions
        self.goal_position = goal_position
        self.scheduled_order = scheduled_order
        self.depth = depth
        if arrival_steps is None:
            self.arrival_steps = tuple(None for _ in range(len(self.positions)))
        else:
            self.arrival_steps = arrival_steps

        self.possible_actions = np.array(
            [
                [0, 0],
                [+1, 0],
                [-1, 0],
                [0, +1],
                [0, -1],
            ],
            dtype=np.int32,
        )

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        actors_legal_actions = []
        for actor in range(len(self.positions)):
            if self.arrival_steps[actor] is not None:
                actors_legal_actions.append([0])
            else:
                legal_actions = []
                for action_idx, delta in enumerate(self.possible_actions):
                    new_position = self.positions[actor] + delta
                    if self._is_free(new_position):
                        legal_actions.append(action_idx)
                actors_legal_actions.append(legal_actions)

        return actors_legal_actions

    def apply_actions(self, actions: Action) -> GameStateProtocol:
        new_positions = self.positions.copy()
        for actor, action_idx in enumerate(actions):
            new_positions[actor] = new_positions[actor] + self.possible_actions[int(action_idx)]

        new_arrival_steps = list(self.arrival_steps)
        next_depth = self.depth + 1
        for actor in range(len(new_positions)):
            if new_arrival_steps[actor] is None and np.array_equal(new_positions[actor], self.goal_position):
                new_arrival_steps[actor] = next_depth

        return OrderedTwoAgentGameState(
            grid_map=self.map,
            positions=new_positions,
            goal_position=self.goal_position,
            scheduled_order=self.scheduled_order,
            depth=next_depth,
            arrival_steps=tuple(new_arrival_steps),
        )

    def is_terminal(self) -> bool:
        everyone_arrived = all(step is not None for step in self.arrival_steps)
        return everyone_arrived

    def terminal_values(self) -> ValueMap:
        scores = [0.0 for _ in range(len(self.positions))]
        if not all(step is not None for step in self.arrival_steps):
            return scores

        observed_order = list(range(len(self.positions)))
        observed_order.sort(key=lambda actor: self.arrival_steps[actor])

        tie_steps = len(set(self.arrival_steps)) != len(self.arrival_steps)
        for scheduled_index, actor in enumerate(self.scheduled_order):
            if tie_steps:
                scores[actor] += 0.0
            elif observed_order[scheduled_index] == actor:
                scores[actor] += 1.0
            else:
                scores[actor] += 0.0

        return scores

    def _is_free(self, position: np.ndarray) -> bool:
        width, height = self.map.shape
        return 0 <= position[0] < width and \
            0 <= position[1] < height and \
            not self.map[position[0], position[1]]


def coordinated_ordered_rollout(state: OrderedTwoAgentGameState):
    rollout_depth = 0
    while not state.is_terminal() and rollout_depth < 8:
        legal_actions = state.legal_actions()
        selected_actions = []
        for actions in legal_actions:
            if len(actions) > 1:
                selected_actions.append(random.choice(actions))
            else:
                selected_actions.append(actions[0])

        state = state.apply_actions(selected_actions)
        rollout_depth += 1

    return state.terminal_values()


def test_straight_path():
    map = np.array([
        [False, False, False, False],
    ])
    goal = np.array([0, 2], dtype=np.int32)

    # Agent 0 is scheduled to arrive first, but starts farther from goal than agent 1.
    start_positions = np.array(
        [
            [0, 0],
            [0, 3],
        ],
        dtype=np.int32,
    )

    mcts = MCTS(
        config=MCTSConfig(num_actors=2, max_actions=(5, 5), max_depth=10, rng=random.Random(7)),
        rollout_fn=coordinated_ordered_rollout,
    )
    state = OrderedTwoAgentGameState(
        grid_map=map,
        positions=start_positions,
        goal_position=goal,
        scheduled_order=(0, 1),
    )
    states = [state]
    actions = []
    for _ in range(6):
        action, state, _, _ = mcts.search(state, num_simulations=500)
        states.append(state)
        actions.append(action)
        if state.is_terminal():
            break

    assert state.is_terminal()
    assert state.arrival_steps[0] is not None and state.arrival_steps[1] is not None
    assert state.arrival_steps[0] < state.arrival_steps[1]
    assert state.terminal_values() == [1.0, 1.0]

def test_curved_path():
    map = np.array([
        [False, False, False],
        [False, True , True ],
        [False, False, False],
        [False, True , True ],
    ])
    goal = np.array([2, 2], dtype=np.int32)

    # Agent 0 is scheduled to arrive first, but starts farther from goal than agent 1.
    start_positions = np.array(
        [
            [0, 2],
            [3, 0],
        ],
        dtype=np.int32,
    )

    mcts = MCTS(
        config=MCTSConfig(num_actors=2, max_actions=(5, 5), max_depth=10, rng=random.Random(7)),
        rollout_fn=coordinated_ordered_rollout,
    )
    state = OrderedTwoAgentGameState(
        grid_map=map,
        positions=start_positions,
        goal_position=goal,
        scheduled_order=(0, 1),
    )
    states = [state]
    actions = []
    for _ in range(8):
        action, state, _, _ = mcts.search(state, num_simulations=25000)
        states.append(state)
        actions.append(action)
        if state.is_terminal():
            break

    assert state.is_terminal()
    assert state.arrival_steps[0] is not None and state.arrival_steps[1] is not None
    assert state.arrival_steps[0] < state.arrival_steps[1]
    assert state.terminal_values() == [1.0, 1.0]
