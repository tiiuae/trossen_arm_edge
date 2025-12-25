# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Purpose:
    Control two follower arms using Cartesian action vectors.
    Each action vector has 7 dimensions: [x, y, z, roll, pitch, yaw, gripper]

Hardware setup:
    1. WXAI V0 arm with follower end effector at IP 192.168.1.2 (Arm 1)
    2. WXAI V0 arm with follower end effector at IP 192.168.1.3 (Arm 2)

The script:
    1. Initializes and configures both follower arms
    2. Records sleep positions
    3. Executes movements based on action vectors
    4. Returns to sleep position on completion or Ctrl+C
"""

import signal
import sys
import time

import numpy as np

import trossen_arm


class DualFollowerController:
    """Controller for two follower arms with graceful shutdown."""

    def __init__(self, arm1_ip='192.168.1.2', arm2_ip='192.168.1.3'):
        """
        Initialize the dual follower controller.

        Args:
            arm1_ip: IP address of first follower arm
            arm2_ip: IP address of second follower arm
        """
        self.arm1_ip = arm1_ip
        self.arm2_ip = arm2_ip
        self.arm1_driver = None
        self.arm2_driver = None
        self.arm1_sleep_positions = None
        self.arm2_sleep_positions = None
        self.shutdown_requested = False

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C signal."""
        print("\n[INFO] Ctrl+C detected. Returning to sleep positions...")
        self.shutdown_requested = True

    def initialize(self):
        """Initialize and configure both arm drivers."""
        print("[INFO] Initializing drivers...")
        self.arm1_driver = trossen_arm.TrossenArmDriver()
        self.arm2_driver = trossen_arm.TrossenArmDriver()

        print("[INFO] Configuring drivers...")
        self.arm1_driver.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            self.arm1_ip,
            False
        )
        self.arm2_driver.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            self.arm2_ip,
            False
        )

        print("[INFO] Setting position mode...")
        self.arm1_driver.set_arm_modes(trossen_arm.Mode.position)
        self.arm2_driver.set_arm_modes(trossen_arm.Mode.position)

        # Record sleep positions
        self.arm1_sleep_positions = np.array(self.arm1_driver.get_arm_positions())
        self.arm2_sleep_positions = np.array(self.arm2_driver.get_arm_positions())

        print("[INFO] Initialization complete.")
        print(f"[INFO] Arm 1 sleep position: {self.arm1_sleep_positions}")
        print(f"[INFO] Arm 2 sleep position: {self.arm2_sleep_positions}")

    def execute_action(self, arm1_action, arm2_action, duration=2.0):
        """
        Execute a Cartesian action on both arms.

        Args:
            arm1_action: Action vector for arm 1 [x, y, z, roll, pitch, yaw, gripper]
            arm2_action: Action vector for arm 2 [x, y, z, roll, pitch, yaw, gripper]
            duration: Time to execute the movement (seconds)

        Returns:
            bool: True if successful, False if shutdown requested
        """
        if self.shutdown_requested:
            return False

        arm1_action = np.array(arm1_action)
        arm2_action = np.array(arm2_action)

        if len(arm1_action) != 7 or len(arm2_action) != 7:
            raise ValueError("Action vectors must have 7 dimensions: [x, y, z, roll, pitch, yaw, gripper]")

        # Extract Cartesian positions (x, y, z, roll, pitch, yaw)
        arm1_cartesian = arm1_action[:6]
        arm2_cartesian = arm2_action[:6]

        # Extract gripper commands
        arm1_gripper = arm1_action[6]
        arm2_gripper = arm2_action[6]

        print(f"[INFO] Executing action:")
        print(f"  Arm 1: position={arm1_cartesian}, gripper={arm1_gripper}")
        print(f"  Arm 2: position={arm2_cartesian}, gripper={arm2_gripper}")

        # Set Cartesian positions for both arms
        self.arm1_driver.set_cartesian_positions(
            arm1_cartesian,
            trossen_arm.InterpolationSpace.cartesian
        )
        self.arm2_driver.set_cartesian_positions(
            arm2_cartesian,
            trossen_arm.InterpolationSpace.cartesian
        )

        # Wait for movement to complete or check for shutdown
        elapsed = 0.0
        while elapsed < duration:
            if self.shutdown_requested:
                return False
            time.sleep(0.1)
            elapsed += 0.1

        # Control grippers
        self._set_gripper(self.arm1_driver, arm1_gripper)
        self._set_gripper(self.arm2_driver, arm2_gripper)

        return True

    def _set_gripper(self, driver, gripper_value):
        """
        Set gripper state based on value.

        Args:
            driver: Arm driver instance
            gripper_value: Gripper command (positive=open, negative=close, 0=no change)
        """
        if gripper_value > 0:
            # Open gripper
            driver.set_gripper_mode(trossen_arm.Mode.external_effort)
            driver.set_gripper_external_effort(20.0, 2.0, True)
        elif gripper_value < 0:
            # Close gripper
            driver.set_gripper_mode(trossen_arm.Mode.external_effort)
            driver.set_gripper_external_effort(-20.0, 2.0, True)
        # If gripper_value == 0, do nothing

    def return_to_sleep(self):
        """Return both arms to their sleep positions."""
        if self.arm1_driver is None or self.arm2_driver is None:
            return

        print("[INFO] Returning to sleep positions...")

        try:
            # Set position mode
            self.arm1_driver.set_arm_modes(trossen_arm.Mode.position)
            self.arm2_driver.set_arm_modes(trossen_arm.Mode.position)

            # Move to sleep positions
            self.arm1_driver.set_arm_positions(
                self.arm1_sleep_positions,
                3.0,
                True
            )
            self.arm2_driver.set_arm_positions(
                self.arm2_sleep_positions,
                3.0,
                True
            )

            print("[INFO] Returned to sleep positions.")
        except Exception as e:
            print(f"[ERROR] Failed to return to sleep: {e}")

    def cleanup(self):
        """Cleanup resources and return to sleep."""
        self.return_to_sleep()
        print("[INFO] Cleanup complete.")


def main():
    """Main execution function with example usage."""
    # Initialize controller
    controller = DualFollowerController(
        arm1_ip='192.168.1.2',
        arm2_ip='192.168.1.3'
    )

    try:
        # Initialize arms
        controller.initialize()

        # Example action vectors: [x, y, z, roll, pitch, yaw, gripper]
        # Note: Adjust these values based on your specific arm configuration
        # and workspace limits

        # Action 1: Move both arms forward and up, open grippers
        action1_arm1 = [0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]  # Open gripper (1.0)
        action1_arm2 = [0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]

        print("\n[INFO] Executing Action 1...")
        if not controller.execute_action(action1_arm1, action1_arm2, duration=3.0):
            return

        time.sleep(1.0)

        # Action 2: Move arms to different positions, close grippers
        action2_arm1 = [0.35, 0.1, 0.15, 0.0, 0.0, 0.5, -1.0]  # Close gripper (-1.0)
        action2_arm2 = [0.35, -0.1, 0.15, 0.0, 0.0, -0.5, -1.0]

        print("\n[INFO] Executing Action 2...")
        if not controller.execute_action(action2_arm1, action2_arm2, duration=3.0):
            return

        time.sleep(1.0)

        # Add more actions as needed...

        print("\n[INFO] All actions completed successfully.")

    except Exception as e:
        print(f"[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Always cleanup and return to sleep
        controller.cleanup()


if __name__ == '__main__':
    main()
