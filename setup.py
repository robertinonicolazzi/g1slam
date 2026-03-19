from setuptools import setup

setup(
    name="g1_slam",
    version="0.1.0",
    packages=["g1_slam"],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "restamp_cloud  = g1_slam.restamp_cloud:main",
            "restamp_odom   = g1_slam.restamp_odom:main",
            "odom_to_tf     = g1_slam.odom_to_tf:main",
            "pose_publisher = g1_slam.pose_publisher:main",
        ],
    },
)