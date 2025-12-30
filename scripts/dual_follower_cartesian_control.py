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
    Control two follower arms using Cartesian action vector.
    Action vector has 14 dimensions: [x, y, z, roll, pitch, yaw, gripper, x, y, z, roll, pitch, yaw, gripper]

Hardware setup:
    1. WXAI V0 arm with follower end effector at IP 192.168.1.5 (Left Arm)
    2. WXAI V0 arm with follower end effector at IP 192.168.1.4 (Right Arm)

The script:
    1. Initializes and configures both follower arms
    2. Records sleep positions
    3. Executes movements based on action vector
    4. Returns to sleep position on completion or Ctrl+C
"""

import argparse
import logging
import signal
import time

import numpy as np

from rich.console import Console
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn

import trossen_arm

console = Console()


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

    logger = logging.getLogger("dual_follower")
    logger.setLevel(level)
    return logger


# Global logger (will be initialized in main)
logger = None


class DualFollowerController:
    """Controller for two follower arms with graceful shutdown."""

    def __init__(self, arm_left_ip='192.168.1.2', arm_right_ip='192.168.1.3'):
        """
        Initialize the dual follower controller.

        Args:
            arm_left_ip: IP address of left follower arm
            arm_right_ip: IP address of right follower arm
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
        console.print(Panel.fit("🤖 [bold cyan]Initializing Dual Follower Controller[/bold cyan]"))
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
        table.add_column("Arm", style="cyan", width=40)
        table.add_column("Position Vector", style="green", width=80)

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

    def execute_action(self, action, duration=2.0):
        """
        Execute a Cartesian action on both arms.

        Args:
            action: Action vector for both arms [14 elements]
                    [left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper,
                     right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper]
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
            raise ValueError("Action vector must have 14 dimensions: [left arm (7), right arm (7)]")

        # Split action into left and right arm actions
        arm_left_action = action[:7]
        arm_right_action = action[7:]

        # Extract Cartesian deltas (x, y, z, roll, pitch, yaw)
        arm_left_delta = arm_left_action[:6]
        arm_right_delta = arm_right_action[:6]

        # Get current Cartesian positions
        logger.debug("Getting current Cartesian positions")
        arm_left_current = np.array(self.arm_left_driver.get_cartesian_positions())
        arm_right_current = np.array(self.arm_right_driver.get_cartesian_positions())

        # Compute new target positions as current + delta
        arm_left_cartesian = arm_left_current + arm_left_delta
        arm_right_cartesian = arm_right_current + arm_right_delta

        # Extract gripper commands
        arm_left_gripper = arm_left_action[6]
        arm_right_gripper = arm_right_action[6]

        logger.info(f"Executing action - Left Arm target: {arm_left_cartesian}, gripper: {arm_left_gripper}")
        logger.info(f"Executing action - Right Arm target: {arm_right_cartesian}, gripper: {arm_right_gripper}")
        logger.debug(f"Left Arm delta: {arm_left_delta}")
        logger.debug(f"Right Arm delta: {arm_right_delta}")

        # Create action execution table
        table = Table(title="🎯 Executing Action", show_header=True, header_style="bold yellow")
        table.add_column("Arm", style="cyan", width=10)
        table.add_column("Current", style="dim")
        table.add_column("Delta", style="blue")
        table.add_column("Target", style="green")
        table.add_column("Gripper", style="magenta", justify="center")

        def format_vec(vec):
            return f"[{', '.join([f'{x:6.3f}' for x in vec])}]"

        gripper_icons = {1.0: "🟢 Open", -1.0: "🔴 Close", 0.0: "⚪ Hold"}

        table.add_row(
            "Left Arm",
            format_vec(arm_left_current),
            format_vec(arm_left_delta),
            format_vec(arm_left_cartesian),
            gripper_icons.get(arm_left_gripper, f"{arm_left_gripper:.1f}")
        )
        table.add_row(
            "Right Arm",
            format_vec(arm_right_current),
            format_vec(arm_right_delta),
            format_vec(arm_right_cartesian),
            gripper_icons.get(arm_right_gripper, f"{arm_right_gripper:.1f}")
        )

        console.print(table)

        # Set Cartesian positions for both arms in parallel
        logger.debug("Setting Cartesian positions for both arms in parallel")

        self.arm_left_driver.set_cartesian_positions(
            arm_left_delta,
            trossen_arm.InterpolationSpace.cartesian,
            blocking=False
        )
        self.arm_right_driver.set_cartesian_positions(
            arm_right_delta,
            trossen_arm.InterpolationSpace.cartesian,
            blocking=False
        )

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
        logger.debug(f"Controlling grippers in parallel - Left Arm: {arm_left_gripper}, Right Arm: {arm_right_gripper}")

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

            # Move to sleep positions
            logger.debug(f"Moving Left Arm to sleep position: {self.arm_left_sleep_positions}")
            self.arm_left_driver.set_arm_positions(
                self.arm_left_sleep_positions,
                3.0,
                True
            )
            logger.debug(f"Moving Right Arm to sleep position: {self.arm_right_sleep_positions}")
            self.arm_right_driver.set_arm_positions(
                self.arm_right_sleep_positions,
                3.0,
                True
            )

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
        description="Dual Follower Cartesian Control Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Verbosity levels:
  error   - Only critical errors
  warning - Errors and warnings
  info    - General information (default)
  debug   - Detailed debugging information

Examples:
  python dual_follower_cartesian_control.py -v error       # Minimal output
  python dual_follower_cartesian_control.py -v debug       # Maximum verbosity
  python dual_follower_cartesian_control.py --arm-left 192.168.1.4 --arm-right 192.168.1.5
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
        default='192.168.1.4',
        help='IP address of left follower arm (default: 192.168.1.4)'
    )
    parser.add_argument(
        '--arm-right',
        type=str,
        default='192.168.1.5',
        help='IP address of right follower arm (default: 192.168.1.5)'
    )

    args = parser.parse_args()

    # Setup logging
    logger = setup_logging(args.verbosity)
    logger.info(f"Logging verbosity set to {args.verbosity.upper()}")

    console.print(Panel.fit(
        "[bold cyan]Dual Follower Cartesian Control Demo[/bold cyan]\n"
        "Press [bold red]Ctrl+C[/bold red] at any time to safely return to sleep positions",
        border_style="blue"
    ))

    # Initialize controller
    logger.info(f"Initializing controller with Left Arm: {args.arm_left}, Right Arm: {args.arm_right}")
    controller = DualFollowerController(
        arm_left_ip=args.arm_left,
        arm_right_ip=args.arm_right
    )

    try:
        # Initialize arms
        controller.initialize()

        # Example action vector: 14 elements
        # [left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper,
        #  right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper]
        # Note: Adjust these values based on your specific arm configuration
        # and workspace limits

        # Action 1: from generated by VLA
        action = [
            0.5929722785949707,   # left_x
            -0.8700209259986877,  # left_y
            0.9497268199920654,   # left_z
            0.1804877519607544,   # left_roll
            -0.5096222162246704,  # left_pitch
            -0.4125874638557434,  # left_yaw
            -0.9965096116065979,  # left_gripper
            -0.6737160086631775,  # right_x
            -1.0,                 # right_y
            0.7450045347213745,   # right_z
            -0.6993166208267212,  # right_roll
            -0.6443812847137451,  # right_pitch
            0.9514955282211304,   # right_yaw
            -0.9932773113250732   # right_gripper
        ]

        console.print(Panel("📍 [bold yellow]Action 1: Move Forward & Open Grippers[/bold yellow]"))
        logger.info("Starting Action 1")
        if not controller.execute_action(action, duration=1.0):
            logger.warning("Action 1 interrupted")
            return
        time.sleep(1.0)

        # # Action 2: Move arms to different positions, close grippers
        # action = [
        #     0.0, 0.0, 0.3, 0.0, 0.0, 0.0, -1.0,  # Left arm
        #     0.0, 0.0, 0.3, 0.0, 0.0, 0.0, -1.0   # Right arm
        # ]

        # console.print(Panel("📍 [bold yellow]Action 2: Move to Position & Close Grippers[/bold yellow]"))
        # if not controller.execute_action(action, duration=3.0):
        #     return

        # Add more actions as needed...

        console.print(Panel.fit(
            "🎉 [bold green]All Actions Completed Successfully![/bold green]",
            border_style="green"
        ))
        logger.info("All actions completed successfully")

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
