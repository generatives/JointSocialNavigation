import argparse
import random

import numpy as np

from social_navigation.simulator.ui import SimulatorUI


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--record',
        type=str,
        default=None,
        help='Optional output video path, for example recordings/sim.mp4',
    )
    parser.add_argument(
        '--record-fps',
        type=int,
        default=30,
        help='Frame rate for the recorded video.',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    random.seed(7)
    np.random.seed(7)
    SimulatorUI(recording_path=args.record, recording_fps=args.record_fps).run()


if __name__ == '__main__':
    main()
