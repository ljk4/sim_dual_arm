from setuptools import find_packages, setup

setup(
    name="dual_arm_runtime",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dual_arm_runtime"]),
        ("share/dual_arm_runtime", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    description="dual_arm_runtime stage 1-2 implementation",
    entry_points={
        "console_scripts": [
            "runtime = dual_arm_runtime.node:main",
            "inspect_episode = dual_arm_runtime.audit:main",
        ]
    },
)
