"""Keyboard control for Roomi robot in Mujoco"""

import time
import mujoco_viewer
import numpy as np
import glfw
import mujoco


class RoomiController:
    def __init__(self, mjcf_path):
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        self.qCmd = np.copy(self.data.qpos)
        self.qdCmd = np.zeros_like(self.data.qvel)
        self.kp = 5

        self.key_map = {
            glfw.KEY_Q: 0,  # Revolute-1
            glfw.KEY_W: 1,  # Revolute-2
            glfw.KEY_E: 2,  # Revolute-3
            glfw.KEY_R: 3,  # Revolute-4
            glfw.KEY_T: 4,  # Revolute-5
            glfw.KEY_Y: 5,  # Revolute-6
        }

        self.step_size = 0.02

    def key_callback(self, window, key, scancode, action, mods):
        if action == glfw.PRESS or action == glfw.REPEAT:
            if key in self.key_map:
                self.qCmd[self.key_map[key]] += self.step_size
            elif key in [glfw.KEY_A, glfw.KEY_S, glfw.KEY_D, glfw.KEY_F, glfw.KEY_G, glfw.KEY_H]:
                rev_key = {
                    glfw.KEY_A: 0,
                    glfw.KEY_S: 1,
                    glfw.KEY_D: 2,
                    glfw.KEY_F: 3,
                    glfw.KEY_G: 4,
                    glfw.KEY_H: 5,
                }
                self.qCmd[rev_key[key]] -= self.step_size

    def run(self):
        glfw.set_key_callback(self.viewer.window, self.key_callback)
        print("Roomi Mujoco Controller started. Use Q/W/E/R/T/Y and A/S/D/F/G/H to control joints.")

        while self.viewer.is_alive:
            mujoco.mj_step1(self.model, self.data)
            self.data.ctrl[:6] = self.kp * (self.qCmd[:6] - self.data.qpos[:6])
            mujoco.mj_step2(self.model, self.data)
            self.viewer.render()

        self.viewer.close()


def main():
    mjcf_path = "assets/roomi.xml"
    controller = RoomiController(mjcf_path)
    controller.run()


if __name__ == "__main__":
    main()
