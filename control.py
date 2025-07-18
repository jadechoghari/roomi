import mujoco
import mujoco_viewer
import glfw
import numpy as np

class RoomiController:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        self.num_actuators = self.model.nu
        self.ctrl = np.zeros(self.num_actuators)

        # print actuator information for debugging
        print(f"Number of actuators: {self.num_actuators}")
        for i in range(self.num_actuators):
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f"Actuator {i}: {actuator_name}")

        # key-to-actuator map including wheels and cart movement
        self.key_map = {
            # Arm joints (0-5)
            glfw.KEY_1: (0, +0.3),  # act_cart_x positive
            glfw.KEY_Q: (0, -0.3),  # act_cart_x negative
            glfw.KEY_2: (1, +0.3),  # act_cart_y positive
            glfw.KEY_W: (1, -0.3),  # act_cart_y negative
            glfw.KEY_3: (2, +0.3),  # act_Revolute_1 positive
            glfw.KEY_E: (2, -0.3),  # act_Revolute_1 negative
            glfw.KEY_4: (3, +0.3),  # act_Revolute_2 positive
            glfw.KEY_R: (3, -0.3),  # act_Revolute_2 negative
            glfw.KEY_5: (4, +0.3),  # act_Revolute_3 positive
            glfw.KEY_T: (4, -0.3),  # act_Revolute_3 negative
            glfw.KEY_6: (5, +0.3),  # act_Revolute_4 positive
            glfw.KEY_Y: (5, -0.3),  # act_Revolute_4 negative
            glfw.KEY_7: (6, +0.3),  # act_Revolute_5 positive
            glfw.KEY_U: (6, -0.3),  # act_Revolute_5 negative
            glfw.KEY_8: (7, +0.3),  # act_Revolute_6 positive
            glfw.KEY_I: (7, -0.3),  # act_Revolute_6 negative
            
            # Wheel controls
            glfw.KEY_UP: (8, +2.0),     # act_Wheel1 forward
            glfw.KEY_DOWN: (8, -2.0),   # act_Wheel1 backward
            glfw.KEY_LEFT: (9, +2.0),   # act_Wheel2 left
            glfw.KEY_RIGHT: (9, -2.0),  # act_Wheel2 right
            
            # wheel controls using WASD
            glfw.KEY_Z: (8, +2.0),   # Both wheels forward
            glfw.KEY_X: (8, -2.0),   # Both wheels backward
            glfw.KEY_A: (9, +1.0),   # Turn left (differential)
            glfw.KEY_D: (9, -1.0),   # Turn right (differential)
        }

        # Differential drive controls (both wheels)
        self.differential_map = {
            glfw.KEY_Z: [(8, +2.0), (9, +2.0)],  # Forward (both wheels)
            glfw.KEY_X: [(8, -2.0), (9, -2.0)],  # Backward (both wheels)
            glfw.KEY_A: [(8, +1.0), (9, -1.0)],  # Turn left
            glfw.KEY_D: [(8, -1.0), (9, +1.0)],  # Turn right
        }

        self.active_keys = set()
        glfw.set_key_callback(self.viewer.window, self.key_callback)

    def key_callback(self, window, key, scancode, action, mods):
        if action == glfw.PRESS:
            self.active_keys.add(key)
        elif action == glfw.RELEASE and key in self.active_keys:
            self.active_keys.remove(key)

    def run(self):
        print("Roomi controller active!")
        print("Controls:")
        print("Cart Movement: 1/Q (X-axis), 2/W (Y-axis)")
        print("Arm Joints: 3/E, 4/R, 5/T, 6/Y, 7/U, 8/I")
        print("Individual Wheels: UP/DOWN (Wheel1), LEFT/RIGHT (Wheel2)")
        print("Differential Drive: Z/X (forward/back), A/D (turn left/right)")
        print("Press ESC to quit.")

        while not glfw.window_should_close(self.viewer.window):
            self.ctrl[:] = 0

            # we handle differential drive first (takes priority)
            differential_active = False
            for key in self.active_keys:
                if key in self.differential_map:
                    differential_active = True
                    for joint_index, velocity in self.differential_map[key]:
                        if joint_index < len(self.ctrl):
                            self.ctrl[joint_index] += velocity

            # and handle individual controls only if no differential drive is active
            if not differential_active:
                for key in self.active_keys:
                    if key in self.key_map:
                        joint_index, velocity = self.key_map[key]
                        if joint_index < len(self.ctrl):
                            self.ctrl[joint_index] = velocity

            # so apply control to simulation
            self.data.ctrl[:] = self.ctrl
            mujoco.mj_step(self.model, self.data)
            
            # and print status but less frequently for better performance)
            if int(self.data.time * 100) % 50 == 0:  # Print every 0.5 seconds
                print(f"Time: {self.data.time:.2f}s, Active keys: {len(self.active_keys)}")
            
            self.viewer.render()

        self.viewer.close()


if __name__ == "__main__":
    model_file = "assets/roomi.xml"
    controller = RoomiController(model_file)
    controller.run()
