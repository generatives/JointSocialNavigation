import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'social_simulator'

def add_subfolders(folder_name, data_files):
    for filepath in glob(f"{folder_name}/**/*", recursive=True):
        if os.path.isfile(filepath):
            relative_path = os.path.relpath(filepath, folder_name)
            install_dir = os.path.join(
                "share", package_name, folder_name, os.path.dirname(relative_path)
            )
            data_files.append((install_dir, [filepath]))

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "maps"), glob("maps/*")),
    (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    (os.path.join("share", package_name, "launch"), glob("launch/*")),
    (os.path.join("share", package_name, "config"), glob("config/*")),
    (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    (os.path.join("share", package_name, "params"), glob("params/*")),
]

add_subfolders("models", data_files)
add_subfolders("scenarios", data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='cameronsiu02@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'evaluation_runner = social_simulator.evaluation_runner:main'
        ],
    },
)
