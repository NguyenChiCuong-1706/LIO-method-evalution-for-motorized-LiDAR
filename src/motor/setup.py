"""Install MHF9025-V3 joint feedback and experimental oscillation nodes."""

from setuptools import setup


PACKAGE_NAME = "mhf9025_joint_state"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    py_modules=[
        "ros_joint_state_node",
        "ros_oscillating_motor_node",
        "joint_state_publisher",
        "absolute_encoder_diagnostic",
        "waveshare_motor",
        "waveshare_probe",
        "motor_can",
        "speed_analysis",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", ["launch/joint_states.launch.py"]),
        ("share/" + PACKAGE_NAME + "/launch", ["launch/oscillate_45.launch.py"]),
        ("share/" + PACKAGE_NAME + "/launch", ["launch/rotate_360.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="cuong",
    maintainer_email="cuong@example.com",
    description="MHF9025-V3 /joint_states publisher and experimental oscillation node",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "joint_state_node = ros_joint_state_node:main",
            "oscillating_motor_node = ros_oscillating_motor_node:main",
        ],
    },
)
