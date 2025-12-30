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

# Purpose:
# This script demonstrates how to move both follower robots to home position and then to sleep position.

# Hardware setup:
# 1. Two WXAI V0 arms with follower end effectors
#    - Arm 1: IP 192.168.1.4
#    - Arm 2: IP 192.168.1.5

# The script does the following:
# 1. Initializes both drivers
# 2. Configures both drivers
# 3. Moves both robots to home position
# 4. Moves both robots to sleep position
# 5. The drivers automatically set the mode to idle at the destructor

import numpy as np

import trossen_arm

if __name__=='__main__':
    # Define IP addresses for both follower arms
    arm1_ip = "192.168.1.4"
    arm2_ip = "192.168.1.5"

    print("Initializing the drivers...")
    driver1 = trossen_arm.TrossenArmDriver()
    driver2 = trossen_arm.TrossenArmDriver()

    print("Configuring the drivers...")
    driver1.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        arm1_ip,
        False
    )
    driver2.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        arm2_ip,
        False
    )

    # Define home and sleep positions
    home_positions = np.array([0.0, np.pi/2, np.pi/2, 0.0, 0.0, 0.0, 0.0])
    sleep_positions = np.zeros(driver1.get_num_joints())

    print("Moving both arms to home position...")
    driver1.set_all_modes(trossen_arm.Mode.position)
    driver2.set_all_modes(trossen_arm.Mode.position)

    driver1.set_all_positions(
        home_positions,
        2.0,  # Duration in seconds
        True  # Wait for completion
    )
    driver2.set_all_positions(
        home_positions,
        2.0,  # Duration in seconds
        True  # Wait for completion
    )
    print("Both arms reached home position.")

    print("Moving both arms to sleep position...")
    driver1.set_all_modes(trossen_arm.Mode.position)
    driver2.set_all_modes(trossen_arm.Mode.position)

    driver1.set_all_positions(
        sleep_positions,
        2.0,  # Duration in seconds
        True  # Wait for completion
    )
    driver2.set_all_positions(
        sleep_positions,
        2.0,  # Duration in seconds
        True  # Wait for completion
    )
    print("Both arms reached sleep position.")

    print("Done!")
