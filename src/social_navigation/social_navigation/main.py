import random

import numpy as np

from social_navigation.simulator.ui import SimulatorUI


def main() -> None:
    random.seed(7)
    np.random.seed(7)
    SimulatorUI().run()


if __name__ == "__main__":
    main()
