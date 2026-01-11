"""
dual_arm_controller package

Provides ROS2 nodes and controller interfaces for
dual-arm Cartesian coordination of Fanuc CRX-10iA robots.

This package is intentionally modular:
- Controller logic lives in dedicated modules
- __init__.py exposes the public API only
"""

from .cartesian_controller import DualArmCartesianController
from .synchronizer import DualArmSynchronizer

__all__ = [
    "DualArmCartesianController",
    "DualArmSynchronizer",
]
