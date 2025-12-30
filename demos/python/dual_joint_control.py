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
    Control two arms using joint position action vector.
    Action vector has 14 dimensions representing joint positions for both arms:
    [right_waist, right_shoulder, right_elbow, right_forearm_roll, right_wrist_angle,
    right_wrist_rotate, right_gripper,
    left_waist, left_shoulder, left_elbow, left_forearm_roll, left_wrist_angle,
    left_wrist_rotate, left_gripper]

Hardware setup:
    1. WXAI V0 arm at IP 192.168.1.5 (Left Arm)
    2. WXAI V0 arm at IP 192.168.1.4 (Right Arm)

The script:
    1. Initializes and configures both arms
    2. Records sleep positions
    3. Executes movements based on joint position vector
    4. Returns to sleep position on completion or Ctrl+C
"""

import argparse
import json
import logging
import signal
import time
from pathlib import Path

import numpy as np

from rich.console import Console
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn

import trossen_arm

console = Console()

# Joint names for reference
JOINT_NAMES = [
    "right_waist",
    "right_shoulder",
    "right_elbow",
    "right_forearm_roll",
    "right_wrist_angle",
    "right_wrist_rotate",
    "right_gripper",
    "left_waist",
    "left_shoulder",
    "left_elbow",
    "left_forearm_roll",
    "left_wrist_angle",
    "left_wrist_rotate",
    "left_gripper"
]


# Configure logging with Rich handler
def setup_logging(verbosity: str) -> logging.Logger:
    """
    Setup logging with Rich handler.

    Args:
        verbosity: 'error', 'warning', 'info', or 'debug'
    """
    level_map = {
        'error': logging.ERROR,
        'warning': logging.WARNING,
        'info': logging.INFO,
        'debug': logging.DEBUG
    }
    level = level_map.get(verbosity.lower(), logging.INFO)

    logging.basicConfig(
        level=level,
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(console=console, rich_tracebacks=True, show_path=False)]
    )

    logger = logging.getLogger("dual_joint_control")
    logger.setLevel(level)
    return logger


# Global logger (will be initialized in main)
logger = None


class DualJointController:
    """Controller for two arms using joint position control with graceful shutdown."""

    def __init__(self, arm_left_ip='192.168.1.5', arm_right_ip='192.168.1.4'):
        """
        Initialize the dual joint controller.

        Args:
            arm_left_ip: IP address of left arm
            arm_right_ip: IP address of right arm
        """
        self.arm_left_ip = arm_left_ip
        self.arm_right_ip = arm_right_ip
        self.arm_left_driver = None
        self.arm_right_driver = None
        self.arm_left_sleep_positions = None
        self.arm_right_sleep_positions = None
        self.shutdown_requested = False

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C signal."""
        console.print("\n[bold yellow]⚠ Ctrl+C detected. Returning to sleep positions...[/bold yellow]")
        logger.warning("Shutdown signal received")
        self.shutdown_requested = True

    def initialize(self):
        """Initialize and configure both arm drivers."""
        console.print(Panel.fit("🤖 [bold cyan]Initializing Dual Joint Controller[/bold cyan]"))
        logger.info("Starting initialization process")

        # Initialize drivers with progress bar
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            console=console,
        ) as progress:
            task1 = progress.add_task("[cyan]Initializing drivers...", total=None)
            logger.debug("Creating TrossenArmDriver instances")
            self.arm_left_driver = trossen_arm.TrossenArmDriver()
            self.arm_right_driver = trossen_arm.TrossenArmDriver()
            progress.update(task1, completed=True)

            task2 = progress.add_task("[cyan]Configuring drivers...", total=None)
            logger.debug(f"Configuring Left Arm at {self.arm_left_ip}")
            self.arm_left_driver.configure(
                trossen_arm.Model.wxai_v0,
                trossen_arm.StandardEndEffector.wxai_v0_follower,
                self.arm_left_ip,
                False
            )
            logger.debug(f"Configuring Right Arm at {self.arm_right_ip}")
            self.arm_right_driver.configure(
                trossen_arm.Model.wxai_v0,
                trossen_arm.StandardEndEffector.wxai_v0_follower,
                self.arm_right_ip,
                False
            )
            progress.update(task2, completed=True)

            task3 = progress.add_task("[cyan]Setting position mode...", total=None)
            logger.debug("Setting arms to position mode")
            self.arm_left_driver.set_arm_modes(trossen_arm.Mode.position)
            self.arm_right_driver.set_arm_modes(trossen_arm.Mode.position)
            progress.update(task3, completed=True)

        # Record sleep positions
        logger.debug("Recording sleep positions")
        self.arm_left_sleep_positions = np.array(self.arm_left_driver.get_arm_positions())
        self.arm_right_sleep_positions = np.array(self.arm_right_driver.get_arm_positions())

        logger.info(f"Left Arm sleep position: {self.arm_left_sleep_positions}")
        logger.info(f"Right Arm sleep position: {self.arm_right_sleep_positions}")

        # Create sleep positions table
        table = Table(title="💤 Sleep Positions", show_header=True, header_style="bold magenta")
        table.add_column("Arm", style="cyan", width=20)
        table.add_column("Joint Positions", style="green", width=80)

        table.add_row(
            f"Left Arm ({self.arm_left_ip})",
            f"[{', '.join([f'{x:.3f}' for x in self.arm_left_sleep_positions])}]"
        )
        table.add_row(
            f"Right Arm ({self.arm_right_ip})",
            f"[{', '.join([f'{x:.3f}' for x in self.arm_right_sleep_positions])}]"
        )

        console.print(table)
        console.print("[bold green]✓ Initialization complete[/bold green]\n")
        logger.info("Initialization complete")

    def execute_joint_action(self, action, duration=2.0):
        """
        Execute a joint position action on both arms.

        Args:
            action: Action vector for both arms [14 elements]
                    [right_waist, right_shoulder, right_elbow, right_forearm_roll,
                     right_wrist_angle, right_wrist_rotate, right_gripper,
                     left_waist, left_shoulder, left_elbow, left_forearm_roll,
                     left_wrist_angle, left_wrist_rotate, left_gripper]
            duration: Time to execute the movement (seconds)

        Returns:
            bool: True if successful, False if shutdown requested
        """
        if self.shutdown_requested:
            logger.debug("Shutdown requested, skipping action")
            return False

        action = np.array(action)

        if len(action) != 14:
            logger.error("Invalid action vector dimensions")
            raise ValueError("Action vector must have 14 dimensions: [right arm (7), left arm (7)]")

        # Split action into right and left arm actions
        # First 7 values are for right arm, last 7 for left arm
        arm_right_joints = action[:6]  # First 6 joints (excluding gripper)
        arm_right_gripper = action[6]
        arm_left_joints = action[7:13]  # Next 6 joints (excluding gripper)
        arm_left_gripper = action[13]

        logger.info(f"Executing joint action - Right Arm joints: {arm_right_joints}, gripper: {arm_right_gripper}")
        logger.info(f"Executing joint action - Left Arm joints: {arm_left_joints}, gripper: {arm_left_gripper}")

        # Create action execution table
        table = Table(title="🎯 Executing Joint Action", show_header=True, header_style="bold yellow")
        table.add_column("Joint", style="cyan", width=20)
        table.add_column("Right Arm", style="green", justify="right")
        table.add_column("Left Arm", style="blue", justify="right")

        # Display joint values
        for i in range(6):
            right_name = JOINT_NAMES[i]
            # left_name = JOINT_NAMES[i + 7]
            table.add_row(
                right_name.replace("right_", "").replace("left_", "").title().replace("_", " "),
                f"{arm_right_joints[i]:7.3f}",
                f"{arm_left_joints[i]:7.3f}"
            )

        # Display gripper values
        gripper_icons = {1.0: "🟢 Open", -1.0: "🔴 Close", 0.0: "⚪ Hold"}
        table.add_row(
            "Gripper",
            gripper_icons.get(arm_right_gripper, f"{arm_right_gripper:.3f}"),
            gripper_icons.get(arm_left_gripper, f"{arm_left_gripper:.3f}")
        )

        console.print(table)

        # Set joint positions for both arms in parallel
        logger.debug("Setting joint positions for both arms in parallel")

        self.arm_left_driver.set_arm_positions(arm_left_joints, duration, blocking=False)
        self.arm_right_driver.set_arm_positions(arm_right_joints, duration, blocking=False)

        # Wait for movement to complete with progress bar
        logger.debug(f"Waiting for movement completion ({duration}s)")
        with Progress(console=console) as progress:
            task = progress.add_task("[cyan]Moving arms...", total=duration * 10)
            elapsed = 0.0
            while elapsed < duration:
                if self.shutdown_requested:
                    logger.warning("Shutdown requested during movement")
                    return False
                time.sleep(0.1)
                elapsed += 0.1
                progress.update(task, advance=1)

        # Control grippers in parallel
        logger.debug(f"Controlling grippers in parallel - Right Arm: {arm_right_gripper}, Left Arm: {arm_left_gripper}")

        self._set_gripper(self.arm_left_driver, arm_left_gripper)
        self._set_gripper(self.arm_right_driver, arm_right_gripper)

        console.print("[bold green]✓ Action completed[/bold green]\n")
        logger.info("Action execution completed successfully")
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
            logger.debug("Opening gripper")
            driver.set_gripper_mode(trossen_arm.Mode.external_effort)
            driver.set_gripper_external_effort(20.0, 2.0, blocking=False)
        elif gripper_value < 0:
            # Close gripper
            logger.debug("Closing gripper")
            driver.set_gripper_mode(trossen_arm.Mode.external_effort)
            driver.set_gripper_external_effort(-20.0, 2.0, blocking=False)
        else:
            logger.debug("Gripper state unchanged")

    def return_to_sleep(self):
        """Return both arms to their sleep positions."""
        if self.arm_left_driver is None or self.arm_right_driver is None:
            logger.warning("Drivers not initialized, skipping return to sleep")
            return

        console.print(Panel("💤 [bold cyan]Returning to Sleep Positions[/bold cyan]"))
        logger.info("Returning arms to sleep positions")

        try:
            # Set position mode
            logger.debug("Setting position mode for sleep return")
            self.arm_left_driver.set_arm_modes(trossen_arm.Mode.position)
            self.arm_right_driver.set_arm_modes(trossen_arm.Mode.position)

            # Move to sleep positions in parallel
            logger.debug(f"Moving Left Arm to sleep position: {self.arm_left_sleep_positions}")
            logger.debug(f"Moving Right Arm to sleep position: {self.arm_right_sleep_positions}")

            self.arm_left_driver.set_arm_positions(self.arm_left_sleep_positions, 3.0, blocking=True)
            self.arm_right_driver.set_arm_positions(self.arm_right_sleep_positions, 3.0, blocking=True)

            console.print("[bold green]✓ Returned to sleep positions[/bold green]")
            logger.info("Successfully returned to sleep positions")
        except Exception as e:
            console.print(f"[bold red]✗ Failed to return to sleep: {e}[/bold red]")
            logger.error(f"Failed to return to sleep: {e}", exc_info=True)

    def cleanup(self):
        """Cleanup resources and return to sleep."""
        logger.info("Starting cleanup")
        self.return_to_sleep()
        console.print("[bold green]✓ Cleanup complete[/bold green]")
        logger.info("Cleanup complete")


def main():
    """Main execution function with example usage."""
    global logger

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Dual Arm Joint Control Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Verbosity levels:
  error   - Only critical errors
  warning - Errors and warnings
  info    - General information (default)
  debug   - Detailed debugging information

Joint Names (14 values):
  Right Arm: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper
  Left Arm:  waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper

Examples:
  python dual_joint_control.py -v error       # Minimal output
  python dual_joint_control.py -v debug       # Maximum verbosity
  python dual_joint_control.py --arm-left 192.168.1.5 --arm-right 192.168.1.4
        """
    )
    parser.add_argument(
        '-v', '--verbosity',
        type=str,
        choices=['error', 'warning', 'info', 'debug'],
        default='info',
        help='Set logging verbosity (default: info)'
    )
    parser.add_argument(
        '--arm-left',
        type=str,
        default='192.168.1.5',
        help='IP address of left arm (default: 192.168.1.5)'
    )
    parser.add_argument(
        '--arm-right',
        type=str,
        default='192.168.1.4',
        help='IP address of right arm (default: 192.168.1.4)'
    )
    parser.add_argument(
        '--actions-file',
        type=str,
        default=None,
        help='Path to JSON file containing actions to execute'
    )
    parser.add_argument(
        '--action-duration',
        type=float,
        default=1.0,
        help='Duration for each action in seconds (default: 1.0)'
    )
    parser.add_argument(
        '--action-delay',
        type=float,
        default=0.0,
        help='Delay between actions in seconds (default: 0.0)'
    )

    args = parser.parse_args()

    # Setup logging
    logger = setup_logging(args.verbosity)
    logger.info(f"Logging verbosity set to {args.verbosity.upper()}")

    console.print(Panel.fit(
        "[bold cyan]Dual Arm Joint Control Demo[/bold cyan]\n"
        "Press [bold red]Ctrl+C[/bold red] at any time to safely return to sleep positions",
        border_style="blue"
    ))

    # Initialize controller
    logger.info(f"Initializing controller with Left Arm: {args.arm_left}, Right Arm: {args.arm_right}")
    controller = DualJointController(
        arm_left_ip=args.arm_left,
        arm_right_ip=args.arm_right
    )

    try:
        # Initialize arms
        controller.initialize()

        # Load actions from file or use example actions
        if args.actions_file:
            # Load actions from JSON file
            actions_path = Path(args.actions_file)
            if not actions_path.exists():
                console.print(f"[bold red]✗ Actions file not found: {args.actions_file}[/bold red]")
                logger.error(f"Actions file not found: {args.actions_file}")
                return

            logger.info(f"Loading actions from {args.actions_file}")
            with open(actions_path, 'r') as f:
                actions_data = json.load(f)

            # Extract actions from JSON structure
            # Expected format: {"shape": [n, 1, 14], "dtype": "...", "actions": [[[14 values]], [[14 values]], ...]}
            if "actions" not in actions_data:
                console.print("[bold red]✗ Invalid JSON format: 'actions' key not found[/bold red]")
                logger.error("Invalid JSON format: 'actions' key not found")
                return

            actions_list = actions_data["actions"]
            num_actions = len(actions_list)

            console.print(Panel.fit(
                f"📋 [bold cyan]Loaded {num_actions} actions from file[/bold cyan]\n"
                f"Shape: {actions_data.get('shape', 'unknown')}\n"
                f"Duration per action: {args.action_duration}s\n"
                f"Delay between actions: {args.action_delay}s",
                border_style="cyan"
            ))
            logger.info(f"Loaded {num_actions} actions from file")

            # Execute all actions from file
            for i, action_data in enumerate(actions_list, 1):
                if controller.shutdown_requested:
                    logger.warning(f"Shutdown requested at action {i}/{num_actions}")
                    break

                # Handle nested array structure: action_data is [[14 values]]
                if isinstance(action_data[0], list):
                    action = action_data[0]
                else:
                    action = action_data

                console.print(Panel(
                    f"📍 [bold yellow]Action {i}/{num_actions}[/bold yellow]"
                ))
                logger.info(f"Starting Action {i}/{num_actions}")

                if not controller.execute_joint_action(action, duration=args.action_duration):
                    logger.warning(f"Action {i} interrupted")
                    break

                if args.action_delay > 0 and i < num_actions:
                    logger.debug(f"Waiting {args.action_delay}s before next action")
                    time.sleep(args.action_delay)

            console.print(Panel.fit(
                f"🎉 [bold green]Completed {min(i, num_actions)}/{num_actions} Actions![/bold green]",
                border_style="green"
            ))
            logger.info(f"Completed {min(i, num_actions)}/{num_actions} actions")

        else:
            # Run example actions if no file provided
            console.print(Panel.fit(
                "[bold yellow]No actions file provided. Running example actions.[/bold yellow]\n"
                "Use --actions-file to load actions from JSON.",
                border_style="yellow"
            ))

            # Example action vector: 14 elements representing joint positions
            # [right_waist, right_shoulder, right_elbow, right_forearm_roll,
            #  right_wrist_angle, right_wrist_rotate, right_gripper,
            #  left_waist, left_shoulder, left_elbow, left_forearm_roll,
            #  left_wrist_angle, left_wrist_rotate, left_gripper]

            # Action 1: Move to specific joint positions
            action = [
                0.0,    # right_waist
                -0.5,   # right_shoulder
                1.0,    # right_elbow
                0.0,    # right_forearm_roll
                0.5,    # right_wrist_angle
                0.0,    # right_wrist_rotate
                1.0,    # right_gripper (open)
                0.0,    # left_waist
                -0.5,   # left_shoulder
                1.0,    # left_elbow
                0.0,    # left_forearm_roll
                0.5,    # left_wrist_angle
                0.0,    # left_wrist_rotate
                1.0     # left_gripper (open)
            ]

            console.print(Panel("📍 [bold yellow]Action 1: Move to Position & Open Grippers[/bold yellow]"))
            logger.info("Starting Action 1")
            if not controller.execute_joint_action(action, duration=2.0):
                logger.warning("Action 1 interrupted")
                return
            time.sleep(1.0)

            # Action 2: Move to different positions and close grippers
            action = [
                0.2,    # right_waist
                -0.3,   # right_shoulder
                0.8,    # right_elbow
                0.1,    # right_forearm_roll
                0.3,    # right_wrist_angle
                0.0,    # right_wrist_rotate
                -1.0,   # right_gripper (close)
                -0.2,   # left_waist
                -0.3,   # left_shoulder
                0.8,    # left_elbow
                -0.1,   # left_forearm_roll
                0.3,    # left_wrist_angle
                0.0,    # left_wrist_rotate
                -1.0    # left_gripper (close)
            ]

            console.print(Panel("📍 [bold yellow]Action 2: Move to New Position & Close Grippers[/bold yellow]"))
            logger.info("Starting Action 2")
            if not controller.execute_joint_action(action, duration=2.0):
                logger.warning("Action 2 interrupted")
                return
            time.sleep(1.0)

            console.print(Panel.fit(
                "🎉 [bold green]All Example Actions Completed Successfully![/bold green]",
                border_style="green"
            ))
            logger.info("All example actions completed successfully")

    except Exception as e:
        console.print(Panel(
            f"[bold red]✗ An error occurred:[/bold red]\n{e}",
            border_style="red"
        ))
        logger.error(f"An error occurred: {e}", exc_info=True)
        import traceback
        console.print(traceback.format_exc())

    finally:
        # Always cleanup and return to sleep
        controller.cleanup()


if __name__ == '__main__':
    main()
