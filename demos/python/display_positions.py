#!/usr/bin/env python3
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
This script continuously displays both robot arms' joint positions using rich formatting.

Hardware setup:
1. Two WXAI V0 arms with follower end effectors
   - Left Arm: IP 192.168.1.4
   - Right Arm: IP 192.168.1.5

The script does the following:
1. Initializes both drivers
2. Configures both drivers
3. Continuously reads and displays all joint positions for both arms
4. Exits on Ctrl+C
"""

import time
from datetime import datetime

import numpy as np
from rich.console import Console
from rich.table import Table
from rich.live import Live

import trossen_arm


def create_positions_table(positions, velocities=None, timestamp=None, title="🤖 Robot Arm Joint Positions"):
    """Create a rich table displaying joint positions."""
    table = Table(title=title, show_header=True, header_style="bold magenta")

    table.add_column("Joint", style="cyan", width=15, justify="center")
    table.add_column("Position (rad/m)", style="green", width=20, justify="right")

    if velocities is not None:
        table.add_column("Velocity (rad/s or m/s)", style="yellow", width=25, justify="right")

    # Joint names
    joint_names = [
        "waist",
        "shoulder",
        "elbow",
        "forearm_roll",
        "wrist_angle",
        "wrist_rotate",
        "gripper",
    ]

    # Display each joint
    for i, pos in enumerate(positions):
        joint_name = joint_names[i] if i < len(joint_names) else f"Joint {i}"

        if velocities is not None:
            table.add_row(
                joint_name,
                f"{pos:.6f}",
                f"{velocities[i]:.6f}"
            )
        else:
            table.add_row(
                joint_name,
                f"{pos:.6f}"
            )

    # Add timestamp if provided
    if timestamp:
        table.caption = f"Last updated: {timestamp}"

    return table


def main():
    console = Console()

    try:
        # Initialize and configure drivers for both arms
        console.print("[bold yellow]Initializing the drivers...[/bold yellow]")
        driver_left = trossen_arm.TrossenArmDriver()
        driver_right = trossen_arm.TrossenArmDriver()

        console.print("[bold yellow]Configuring the left arm (192.168.1.4)...[/bold yellow]")
        driver_left.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            '192.168.1.4',
            False
        )

        console.print("[bold yellow]Configuring the right arm (192.168.1.5)...[/bold yellow]")
        driver_right.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            '192.168.1.5',
            False
        )

        num_joints_left = driver_left.get_num_joints()
        num_joints_right = driver_right.get_num_joints()
        console.print(f"[bold green]✓ Connected to both arms! Joints - Left: {num_joints_left}, Right: {num_joints_right}[/bold green]\n")

        # Enable gravity compensation - allows manual movement with torque released
        console.print("[bold yellow]Enabling gravity compensation (releasing torque) for both arms...[/bold yellow]")
        driver_left.set_all_modes(trossen_arm.Mode.external_effort)
        driver_left.set_all_external_efforts(
            [0] * num_joints_left,
            0.0,
            False
        )
        driver_right.set_all_modes(trossen_arm.Mode.external_effort)
        driver_right.set_all_external_efforts(
            [0] * num_joints_right,
            0.0,
            False
        )
        console.print("[bold green]✓ Gravity compensation enabled - you can now move both arms by hand![/bold green]\n")

        # Display continuous updates
        console.print("[bold cyan]Starting continuous position display for both arms...[/bold cyan]")
        console.print("[dim]Press Ctrl+C to exit[/dim]\n")

        with Live(console=console, refresh_per_second=10) as live:
            while True:
                # Get current positions for both arms
                positions_left = np.array(driver_left.get_all_positions())
                velocities_left = np.array(driver_left.get_all_velocities())

                positions_right = np.array(driver_right.get_all_positions())
                velocities_right = np.array(driver_right.get_all_velocities())

                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                # Create tables for both arms
                table_left = create_positions_table(
                    positions_left,
                    velocities_left,
                    timestamp,
                    title="🤖 Left Arm (192.168.1.4)"
                )
                table_right = create_positions_table(
                    positions_right,
                    velocities_right,
                    timestamp,
                    title="🤖 Right Arm (192.168.1.5)"
                )

                # Create a layout to display both tables side by side
                from rich.columns import Columns
                layout = Columns([table_left, table_right])
                live.update(layout)

                # Small delay to reduce CPU usage
                time.sleep(0.1)

    except KeyboardInterrupt:
        console.print("\n[bold yellow]⚠ Interrupted by user[/bold yellow]")
    except Exception as e:
        console.print(f"\n[bold red]✗ Error: {e}[/bold red]")
        raise
    finally:
        console.print("[bold green]✓ Shutdown complete[/bold green]")


if __name__ == '__main__':
    main()
