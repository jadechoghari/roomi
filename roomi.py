import numpy as np
import sapien
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.utils.common import deepcopy_dict

@register_agent()
class Roomi(BaseAgent):
    uid = "roomi"
    mjcf_path = "assets/roomi.xml"

    # Define a keyframe to avoid spawning inside ground
    keyframes = dict(
        rest=Keyframe(
            pose=sapien.Pose(p=[0, 0, 0.2]),  # Raise base slightly above ground
            qpos=np.zeros(10),  # Adjust length to match your robot's DOFs
        )
    )

    # Joint names from your MJCF
    arm_joint_names = [
        "Revolute-1", "Revolute-2", "Revolute-3",
        "Revolute-4", "Revolute-5", "Revolute-6",
    ]
    cart_joint_names = ["cart_slide_x", "cart_slide_y"]
    wheel_joint_names = ["Revolute-8", "Revolute-9"]

    # Control parameters (you can tune these)
    arm_stiffness = 500
    arm_damping = 50
    arm_force_limit = 100

    wheel_damping = 1
    wheel_force_limit = 30

    cart_stiffness = 1000
    cart_damping = 100
    cart_force_limit = 200

    @property
    def _controller_configs(self):
        # Arm position controller
        arm_ctrl = PDJointPosControllerConfig(
            self.arm_joint_names,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit
        )

        # Cartesian slider controller
        cart_ctrl = PDJointPosControllerConfig(
            self.cart_joint_names,
            stiffness=self.cart_stiffness,
            damping=self.cart_damping,
            force_limit=self.cart_force_limit
        )

        # Wheels velocity controller
        wheel_ctrl = PDJointVelControllerConfig(
            self.wheel_joint_names,
            damping=self.wheel_damping,
            force_limit=self.wheel_force_limit
        )

        return deepcopy_dict(dict(
            pd_joint_pos=dict(
                arm=arm_ctrl,
                cart=cart_ctrl,
                wheels=wheel_ctrl,
                balance_passive_force=False  # good for mobile robots
            ),
        ))
