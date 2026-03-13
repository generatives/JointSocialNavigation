To run simple simulator and harness outside ROS2
1. cd src/social_navigation
1. create a virtual environment (uv venv --python 3.10)
1. install packages (uv pip install numpy pygame ipykernel)
1. install own module (uv pip install -e .)
1. activate (source .venv/bin/activate)
1. run (python social_navigation/main.py)