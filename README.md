# Trossen Arm

## Installation

This package requires installation of dependencies. Install using pip:

```bash
pip install trossen-arm
```

## Python Demo Codes

The package includes various Python demo scripts located in the `demos/python/` directory. These demos demonstrate different functionalities of the Trossen Arm, including:

- Cartesian position and velocity control
- Joint control
- Gravity compensation
- Teleoperation
- Gripper control
- And more...

To use the demos, navigate to the `demos/python/` directory and run any of the example scripts.

## Quick Access Setup for Home-to-Sleep Script

For easy access to the `home_to_sleep.py` script from anywhere in the terminal, a wrapper script and alias have been set up.

### Setup Steps

1. **Wrapper Script Created**: A shell script `home-to-sleep.sh` has been created in the project root that:
   - Automatically activates the required conda environment (`trossen-arm`)
   - Runs the Python script with the correct environment
   - Accepts any command-line arguments

2. **Alias Configuration**: An alias has been added to `~/.bashrc`:
   ```bash
   alias home-to-sleep='/home/edgeai/trossen_arm/home-to-sleep.sh'
   ```

### Usage

Simply run from any directory:
```bash
home-to-sleep
```

The script will automatically:
- Activate the `trossen-arm` conda environment
- Execute the home-to-sleep sequence for both robotic arms
- Handle all environment dependencies

**Note**: The alias is permanent and will be available in all new terminal sessions.
