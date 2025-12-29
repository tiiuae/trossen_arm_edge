#!/bin/bash
# Wrapper script to run home_to_sleep.py with the correct conda environment

# Activate the conda environment
source /home/edgeai/miniconda3/etc/profile.d/conda.sh
conda activate /home/edgeai/miniconda3/envs/trossen-arm

# Run the Python script
python /home/edgeai/trossen_arm/demos/python/home_to_sleep.py "$@"
