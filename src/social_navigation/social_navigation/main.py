import random

import numpy as np

from simulator import Simulator


def main() -> None:
    random.seed(7)
    np.random.seed(7)
    Simulator().run()


if __name__ == "__main__":
    main()
