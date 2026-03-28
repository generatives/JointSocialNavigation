To run simple simulator and harness outside ROS2
1. cd src/social_navigation
1. create a virtual environment (uv venv --python 3.10)
1. install packages (uv pip install numpy pygame ipykernel imageio imageio-ffmpeg)
1. install own module (uv pip install -e .)
1. activate (source .venv/bin/activate)
1. run (python social_navigation/main.py)

To record a simulation to disk
1. run `python social_navigation/main.py --record recordings/sim.mp4`

To record the harness
1. run `python social_navigation/harness_main.py --record recordings/harness.mp4`
1. the harness records only the first run in the batch and still reports aggregate stats for all runs
