import random
from dataclasses import dataclass, field
from typing import Iterable, List

from mcts.decoupled_mcts import Action, GameStateProtocol, MCTSConfig, ValueMap
import numpy as np

from simulator.map import ScenarioMap


@dataclass(frozen=True, slots=True)
class MCTSGameStateConfig:
    mcts_config: MCTSConfig
    movement_distance: float
    angle: float
    uncomfortable_distance: float
    map: ScenarioMap
    orientation_changes: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "orientation_changes",
            np.array([-self.angle, 0.0, self.angle], dtype=np.float32),
        )


class MCTSGameState(GameStateProtocol):

    __slots__ = (
        "config",
        "positions",
        "orientations",
        "agent_goal_positions",
        "depth",
        "_value_accumulator",
        "_is_terminal_cache",
        "_terminal_value_cache",
    )

    def __init__(self,
                 positions: np.ndarray,
                 orientations: np.ndarray,
                 agent_goal_positions: np.ndarray,
                 value_accumulator: np.ndarray | None,
                 config: MCTSGameStateConfig,
                 depth: int):
        super().__init__()
        self.config = config

        self.positions = positions
        self.orientations = orientations
        self.agent_goal_positions = agent_goal_positions
        if value_accumulator is None:
            value_accumulator = np.zeros((positions.shape[0],))
        self._value_accumulator = self._accumulate_value(value_accumulator)
        self.depth = depth

        self._is_terminal_cache = None
        self._terminal_value_cache = None

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        return self.config.mcts_config.legal_actions

    def apply_actions(self, actions: List[int]) -> "GameStateProtocol":
        new_orientations = self.orientations + self.config.orientation_changes[actions]

        new_positions = np.empty_like(self.positions)
        new_positions[:, 0] = self.positions[:, 0] + self.config.movement_distance * np.cos(new_orientations)
        new_positions[:, 1] = self.positions[:, 1] + self.config.movement_distance * np.sin(new_orientations)

        return MCTSGameState(
            new_positions,
            new_orientations,
            self.agent_goal_positions,
            self._value_accumulator,
            self.config,
            self.depth + 1
        )

    def is_terminal(self) -> bool:
        if self._is_terminal_cache is None:
            collided_with_wall = any(
                (not self.config.map.position_is_free(self.positions[i, :]))
                for i in range(self.config.mcts_config.num_actors)
            )
            reached_depth = self.depth >= self.config.mcts_config.max_depth
            self._is_terminal_cache = collided_with_wall or reached_depth

        return self._is_terminal_cache
    
    def _uncomfortable_distance(self) -> np.ndarray:
        robot_position = self.positions[0, :]
        other_positions = self.positions[1:, :]
        distances = np.linalg.norm(other_positions - robot_position, axis=1)
        uncomfortable_distances = np.clip(distances, 0, self.config.uncomfortable_distance)
        return np.sum(uncomfortable_distances)
    
    def _accumulate_value(self, value_accumulator) -> np.ndarray:
        value_accumulator = value_accumulator.copy()
        value_accumulator[0] += self._uncomfortable_distance()
        return value_accumulator

    def terminal_values(self) -> ValueMap:
        if self._terminal_value_cache is None:
            negative_distances = -np.linalg.norm(self.agent_goal_positions - self.positions, axis=1)
            self._terminal_value_cache = (negative_distances \
                                            + self._value_accumulator
                                        ).tolist()

        return self._terminal_value_cache
    

def navigation_rollout(state: MCTSGameState):
    while not state.is_terminal():
        legal_actions = state.legal_actions()
        random_actions = [actions[random.randint(0, len(actions) - 1)] for actions in legal_actions]
        state = state.apply_actions(random_actions)
    return state.terminal_values()
