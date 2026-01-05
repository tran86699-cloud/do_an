# utils/helpers.py
"""Helper functions"""

import time

def wait(seconds):
    """Wait for seconds"""
    time.sleep(seconds)

def get_timestamp():
    """Get current timestamp"""
    return time.time()
