import random
from typing import Iterable, List

from mcts.decoupled_mcts import Action, GameStateProtocol, ValueMap
import numpy as np

from simulator.map import ScenarioMap

class MCTSGameState(GameStateProtocol):

    def __init__(self,
                 positions: np.ndarray,
                 orientations: np.ndarray,
                 agent_goal_positions: np.ndarray,
                 num_actors: int,
                 num_actions: int,
                 movement_distance: float,
                 angle: float,
                 map: ScenarioMap,
                 depth: int):
        super().__init__()
        self.num_actors = num_actors
        self.num_actions = num_actions
        self.movement_distance = movement_distance
        self.angle = angle
        self.map = map

        self.positions = positions
        self.orientations = orientations
        self.agent_goal_positions = agent_goal_positions
        self.depth = depth

        self.is_terminal_cache = None

    def legal_actions(self) -> Iterable[Iterable[Action]]:
        return [list(range(self.num_actions))] * self.num_actors

    def apply_actions(self, actions: List[int]) -> "GameStateProtocol":
        orientation_changes = np.array([-self.angle, 0, self.angle])

        new_orientations = self.orientations + orientation_changes[actions]

        new_positions = np.zeros_like(self.positions)
        new_positions[:, 0] = self.positions[:, 0] + self.movement_distance * np.cos(new_orientations)
        new_positions[:, 1] = self.positions[:, 1] + self.movement_distance * np.sin(new_orientations)

        return MCTSGameState(
            new_positions,
            new_orientations,
            self.num_actors,
            self.num_actions,
            self.movement_distance,
            self.angle,
            self.map,
            self.depth + 1
        )

    def is_terminal(self) -> bool:
        if self.is_terminal_cache is None:
            collided = any((not self.map.position_is_free(self.positions[i, :]) for i in range(self.num_actors)))
            reached_depth = self.depth >= 6
            self.is_terminal_cache = collided or reached_depth

        return self.is_terminal_cache

    def terminal_values(self) -> ValueMap:
        distances = -np.linalg.norm(self.agent_goal_positions - self.positions, axis=1)
        return distances.tolist()
    

def navigation_rollout(state: MCTSGameState):
    while not state.is_terminal():
        legal_actions = state.legal_actions()
        random_actions = [actions[random.randint(0, len(actions) - 1)] for actions in legal_actions]
        state = state.apply_actions(random_actions)
    return state.terminal_values()