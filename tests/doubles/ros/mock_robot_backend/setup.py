from setuptools import find_packages, setup

setup(
    name="mock_robot_backend",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mock_robot_backend"]),
        ("share/mock_robot_backend", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    description="mock_robot_backend stage 1-2 implementation",
    entry_points={"console_scripts": ["mock_backend = mock_robot_backend.node:main"]},
)
