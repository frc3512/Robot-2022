#!/usr/bin/env python3
"""Retrieves all CSV log files from target."""

import os
import subprocess

import frcutils

if not os.path.exists("csvs"):
    os.makedirs("csvs")

ip = frcutils.get_roborio_ip()

subprocess.run(["scp", f"lvuser@{ip}:*.csv", "csvs/"])
