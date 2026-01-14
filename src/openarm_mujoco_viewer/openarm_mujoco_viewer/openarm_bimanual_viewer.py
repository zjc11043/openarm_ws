import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco
import glfw


class OpenArmBimanualViewer(Node):
    """MuJoCo viewer for OpenArm bimanual scene driven by /joint_states."""

    def __init__(self) -> None:
        super().__init__("openarm_bimanual_viewer")

        # Subscribe to joint_states produced by ros2_control / MoveIt etc.
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        # Path to MuJoCo scene.xml. Adjust here if your workspace path differs.
        default_scene_path = "/home/zcj/openarm_ws/openarm_mujoco/v1/scene.xml"
        self.declare_parameter("scene_path", default_scene_path)
        scene_path = self.get_parameter("scene_path").get_parameter_value().string_value

        if not os.path.isfile(scene_path):
            self.get_logger().error(f"MuJoCo scene file not found: {scene_path}")
            raise FileNotFoundError(scene_path)

        # === 1. Load MuJoCo scene (bimanual + ground) ===
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)

        self.get_logger().info(f"Loaded MuJoCo model from {scene_path}")

        # Build joint name -> qpos index mapping
        self.joint_qpos_index = {}
        for j in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if name is None:
                continue
            qpos_adr = self.model.jnt_qposadr[j]
            self.joint_qpos_index[name] = qpos_adr

        self.get_logger().info("Available MuJoCo joints:")
        for name, idx in self.joint_qpos_index.items():
            self.get_logger().info(f"  {name} -> qpos[{idx}]")

        # === 2. Initialize GLFW + MuJoCo rendering ===
        if not glfw.init():
            self.get_logger().error("Failed to initialize GLFW")
            raise RuntimeError("Failed to initialize GLFW")

        self.window = glfw.create_window(1200, 900, "OpenArm Bimanual (MuJoCo)", None, None)
        if not self.window:
            glfw.terminate()
            self.get_logger().error("Failed to create GLFW window")
            raise RuntimeError("Failed to create GLFW window")

        glfw.make_context_current(self.window)

        # Mouse interactions: scroll = zoom, drag = rotate/pan
        glfw.set_scroll_callback(self.window, self.scroll_callback)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move_callback)
        glfw.set_mouse_button_callback(self.window, self.mouse_button_callback)

        # MuJoCo rendering objects
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.pert = mujoco.MjvPerturb()

        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)

        # Pull camera back a bit so both arms are visible
        self.cam.distance = 2.0

        # Disable shadows to avoid "Shadow framebuffer is not complete" errors
        # on virtual machines or systems with limited OpenGL support.
        # Setting shadowsize=0 prevents MuJoCo from creating the shadow FBO.
        self.model.vis.quality.shadowsize = 0

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)

        # Try to create rendering context. On some systems (e.g. limited GPU/driver)
        # this can fail with "Shadow framebuffer is not complete". In that case,
        # fall back to a headless mode (no MuJoCo rendering window) so that the
        # node does not crash and /joint_states handling still works.
        self.render_enabled = True
        try:
            self.ctx = mujoco.MjrContext(
                self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value
            )
        except mujoco.FatalError as e:
            self.get_logger().error(
                f"Failed to create MuJoCo rendering context: {e}. "
                "Running without MuJoCo rendering (only RViz will show the robot)."
            )
            self.render_enabled = False

        # Latest joint positions from /joint_states
        self.last_positions: dict[str, float] = {}

        # Mouse state for camera control
        self._mouse_left = False
        self._mouse_right = False
        self._mouse_middle = False
        self._last_mouse_x = 0.0
        self._last_mouse_y = 0.0

        # Timer at 100 Hz: sync qpos + step simulation + render
        self.timer = self.create_timer(0.01, self.timer_cb)

    # Mouse wheel zoom
    def scroll_callback(self, window, xoffset, yoffset):
        self.cam.distance *= 1.0 - 0.1 * yoffset

    # Mouse button callback: track which buttons are pressed
    def mouse_button_callback(self, window, button, action, mods):
        self._mouse_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self._mouse_right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        self._mouse_middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        self._last_mouse_x, self._last_mouse_y = glfw.get_cursor_pos(window)

    # Mouse move callback: rotate/pan camera while dragging
    def mouse_move_callback(self, window, xpos, ypos):
        dx = xpos - self._last_mouse_x
        dy = ypos - self._last_mouse_y
        self._last_mouse_x = xpos
        self._last_mouse_y = ypos

        if not (self._mouse_left or self._mouse_right or self._mouse_middle):
            return

        width, height = glfw.get_window_size(window)
        if width <= 0 or height <= 0:
            return

        # Map mouse buttons to MuJoCo camera actions (similar to the
        # official MuJoCo viewer):
        # - Left drag: rotate
        # - Right drag: zoom (already handled by scroll, but keep for parity)
        # - Middle drag: move
        if self._mouse_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif self._mouse_right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H

        mujoco.mjv_moveCamera(
            self.model,
            action,
            dx / width,
            dy / height,
            self.scene,
            self.cam,
        )

    # /joint_states callback: keep only OpenArm arm + gripper joints
    def joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            # 包含左/右臂关节，以及左右夹爪指关节
            if (
                name.startswith("openarm_left_joint")
                or name.startswith("openarm_right_joint")
                or name.startswith("openarm_left_finger_joint")
                or name.startswith("openarm_right_finger_joint")
            ):
                self.last_positions[name] = float(pos)

    # Timer: write joint positions into MuJoCo and render
    def timer_cb(self) -> None:
        if glfw.window_should_close(self.window):
            self.get_logger().info("GLFW window closed, shutting down node.")
            rclpy.shutdown()
            return

        # Write joint states into MuJoCo qpos
        for name, pos in self.last_positions.items():
            idx = self.joint_qpos_index.get(name)
            if idx is not None:
                self.data.qpos[idx] = pos

        # Forward kinematics + small simulation step
        # For visualization we only need forward kinematics; the actual
        # physics simulation already runs inside mujoco_ros2_control.
        mujoco.mj_forward(self.model, self.data)

        # Render (if rendering is available on this system)
        if self.render_enabled:
            width, height = glfw.get_framebuffer_size(self.window)
            viewport = mujoco.MjrRect(0, 0, width, height)

            mujoco.mjv_updateScene(
                self.model,
                self.data,
                self.opt,
                self.pert,
                self.cam,
                mujoco.mjtCatBit.mjCAT_ALL.value,
                self.scene,
            )
            mujoco.mjr_render(viewport, self.scene, self.ctx)

            glfw.swap_buffers(self.window)
            glfw.poll_events()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenArmBimanualViewer()
    try:
        rclpy.spin(node)
    finally:
        glfw.terminate()


if __name__ == "__main__":  # pragma: no cover
    main()
