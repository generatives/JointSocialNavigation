from .simulation import NavigationSimulation, StepMetrics

__all__ = ["Simulator", "NavigationSimulation", "StepMetrics"]


def __getattr__(name: str):
    if name == "Simulator":
        from .ui import Simulator

        return Simulator
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
