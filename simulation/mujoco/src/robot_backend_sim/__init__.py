"""MuJoCo dual-Panda scene, diagnostics and named SI robot I/O.

The package provides the authored scene/config checks plus the reusable
16-dimensional ``PandaIO`` binding.  It does not implement ROS adapters,
trajectory scheduling, reset/lease protocols or the physics control core.
Importing the execution modules requires MuJoCo; configuration helpers remain
independent of it.
"""

__all__ = ["probe_environment"]
