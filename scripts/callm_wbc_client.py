#!/usr/bin/env python3
"""ROS2 client for CallmWbcController.

Publishes a WbcData command (flat std_msgs/Float64MultiArray, 20 doubles) and
subscribes to the measured state republished by the controller. The flat layout
matches src/WbcData.h exactly:

    [ 0.. 3) eef_pos         (x, y, z)              world frame
    [ 3.. 7) eef_quat        (w, x, y, z)           world frame (SVA-frame rotation)
    [ 7..13) posture_arm     (6 UR5e joints)        ref_joint_order
    [13..16) posture_base    (base_x, base_y, base_yaw)
    [16..17) gripper_opening (0 = open, 1 = closed) Robotiq convention
    [17..20) velocity_base   (vx, vy, wyaw)         base body frame
    [20..23) task_weights    (w_ee, w_posture_arm, w_base)  <0 = keep controller default

Mirrors explicit_compliance_controller/scripts/explicit_comp_client.py.

"Modes" are just weight presets chosen on this (client) side -- the controller is a
generic weighted-QP executor. See MODE_PRESETS / WbcData.set_mode().
"""

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

WBC_DATA_SIZE = 32

# Per-task gains, order [ee, posture_arm, base, base_posture]. A value < 0 tells the
# controller to keep its current/YAML-default value; damping is a ratio zeta
# (damping = 2*zeta*sqrt(stiffness); zeta = 1 => critically damped).
_KEEP = np.array([-1.0, -1.0, -1.0, -1.0])

# "Modes" are gain presets chosen entirely on this (client) side. Each preset gives
# weights/stiffness/damping (length-4, -1 = keep default).
#   se3     : Cartesian arm (EE dominates, posture regularizes); velocity base
#   direct  : joint-space arm (posture dominates, EE off); velocity base
#   joint   : joint-space arm AND joint-space base (both transform tasks off)
#   compliant: se3 authority but soft EE/base stiffness
MODE_PRESETS = {
    #                  weights                          stiffness                 damping(zeta)
    "se3":     {"weights": np.array([1000., 5., 1000., 1.]), "stiffness": _KEEP.copy(),                    "damping": _KEEP.copy()},
    "direct":  {"weights": np.array([0., 1000., 1000., 1.]), "stiffness": _KEEP.copy(),                    "damping": _KEEP.copy()},
    "joint":   {"weights": np.array([0., 1000., 0., 1000.]), "stiffness": _KEEP.copy(),                    "damping": _KEEP.copy()},
    "compliant": {"weights": np.array([1000., 5., 1000., 1.]), "stiffness": np.array([50., 5., 50., 1.]), "damping": _KEEP.copy()},
    "default": {"weights": _KEEP.copy(), "stiffness": _KEEP.copy(), "damping": _KEEP.copy()},
}


@dataclass
class WbcData:
    eef_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))          # (3,) [x, y, z]
    eef_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # (4,) [w, x, y, z]
    posture_arm: np.ndarray = field(default_factory=lambda: np.zeros(6))      # (6,) UR5e joints
    posture_base: np.ndarray = field(default_factory=lambda: np.zeros(3))     # (3,) base_x, base_y, base_yaw (plumbed only)
    gripper_opening: float = 0.0                                              # 0 = open, 1 = closed
    velocity_base: np.ndarray = field(default_factory=lambda: np.zeros(3))    # (3,) [vx, vy, wyaw] body frame
    # Per-task gains, order [ee, posture_arm, base, base_posture]; <0 = keep controller default.
    task_weights: np.ndarray = field(default_factory=lambda: _KEEP.copy())
    task_stiffness: np.ndarray = field(default_factory=lambda: _KEEP.copy())
    task_damping_ratio: np.ndarray = field(default_factory=lambda: _KEEP.copy())

    def set_mode(self, mode: str) -> "WbcData":
        """Select a gain preset ("mode"). See MODE_PRESETS. Returns self for chaining."""
        if mode not in MODE_PRESETS:
            raise KeyError(f"unknown mode '{mode}'; choose from {sorted(MODE_PRESETS)}")
        preset = MODE_PRESETS[mode]
        self.task_weights = preset["weights"].copy()
        self.task_stiffness = preset["stiffness"].copy()
        self.task_damping_ratio = preset["damping"].copy()
        return self


def _as_numpy(name: str, value, size: int) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.shape != (size,):
        raise ValueError(f"{name} must have {size} elements, got {array.shape}")
    return array


def pack_wbc_data(data: WbcData) -> list:
    packed = np.concatenate(
        [
            _as_numpy("eef_pos", data.eef_pos, 3),
            _as_numpy("eef_quat", data.eef_quat, 4),
            _as_numpy("posture_arm", data.posture_arm, 6),
            _as_numpy("posture_base", data.posture_base, 3),
            np.array([float(data.gripper_opening)], dtype=np.float64),
            _as_numpy("velocity_base", data.velocity_base, 3),
            _as_numpy("task_weights", data.task_weights, 4),
            _as_numpy("task_stiffness", data.task_stiffness, 4),
            _as_numpy("task_damping_ratio", data.task_damping_ratio, 4),
        ]
    )
    return packed.tolist()


def unpack_wbc_data(raw) -> WbcData:
    array = np.asarray(raw, dtype=np.float64).reshape(-1)
    if array.shape != (WBC_DATA_SIZE,):
        raise ValueError(f"WbcData must have {WBC_DATA_SIZE} elements, got {array.shape}")
    return WbcData(
        eef_pos=array[0:3].copy(),
        eef_quat=array[3:7].copy(),
        posture_arm=array[7:13].copy(),
        posture_base=array[13:16].copy(),
        gripper_opening=float(array[16]),
        velocity_base=array[17:20].copy(),
        task_weights=array[20:24].copy(),
        task_stiffness=array[24:28].copy(),
        task_damping_ratio=array[28:32].copy(),
    )


class CallmWbcClient(Node):
    def __init__(
        self,
        node_name: str = "callm_wbc_client",
        command_topic: str = "callm_wbc/command",
        measured_topic: str = "callm_wbc/measured",
    ) -> None:
        super().__init__(node_name)
        self._publisher = self.create_publisher(Float64MultiArray, command_topic, 1)
        self._latest_measured: Optional[WbcData] = None
        self._subscription = self.create_subscription(
            Float64MultiArray, measured_topic, self._measured_callback, 1
        )

    def _measured_callback(self, msg: Float64MultiArray) -> None:
        self._latest_measured = unpack_wbc_data(msg.data)

    def send_command(self, data: WbcData) -> None:
        msg = Float64MultiArray()
        msg.data = pack_wbc_data(data)
        self._publisher.publish(msg)

    def spin_until_measured(
        self, timeout_sec: Optional[float] = None, spin_timeout_sec: float = 0.1
    ) -> WbcData:
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=spin_timeout_sec)
            if self._latest_measured is not None:
                return self._latest_measured
            if timeout_sec is not None and time.monotonic() - start >= timeout_sec:
                raise TimeoutError("Timed out waiting for callm_wbc/measured")
        raise RuntimeError("rclpy shutdown before receiving callm_wbc/measured")

    def spin_measured_loop(
        self, callback: Callable[[WbcData], None], spin_timeout_sec: float = 0.1
    ) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=spin_timeout_sec)
            if self._latest_measured is not None:
                callback(self._latest_measured)

    @property
    def latest_measured(self) -> Optional[WbcData]:
        return self._latest_measured


def create_client(
    node_name: str = "callm_wbc_client",
    command_topic: str = "callm_wbc/command",
    measured_topic: str = "callm_wbc/measured",
) -> CallmWbcClient:
    if not rclpy.ok():
        rclpy.init()
    return CallmWbcClient(node_name=node_name, command_topic=command_topic, measured_topic=measured_topic)


def shutdown_client(client: CallmWbcClient) -> None:
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    client = create_client()
    try:
        # Read the current state first, then nudge the end-effector and drive the base.
        measured = client.spin_until_measured(timeout_sec=2.0)
        print("measured eef_pos:", measured.eef_pos)
        print("measured posture_arm:", measured.posture_arm)

        # SE3 (Cartesian) mode: nudge the hand, drive the base, close the gripper.
        command = WbcData().set_mode("se3")
        command.eef_pos = measured.eef_pos + np.array([0.05, 0.0, 0.05])
        command.eef_quat = measured.eef_quat
        command.posture_arm = measured.posture_arm
        command.velocity_base = np.array([0.1, 0.0, 0.0])  # 0.1 m/s forward (body frame)
        command.gripper_opening = 1.0  # close the gripper
        client.send_command(command)
        print("sent SE3 command; active weights:", client.spin_until_measured(2.0).task_weights)

        # direct (joint-space) mode: arm follows joint targets, EE task off.
        joints = WbcData().set_mode("direct")
        joints.posture_arm = measured.posture_arm + np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        joints.velocity_base = np.array([0.0, 0.0, 0.2])  # spin in place
        client.send_command(joints)
        print("sent direct command.")
    finally:
        shutdown_client(client)
