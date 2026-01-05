# utils/logger.py
"""Logging utility"""

import logging
import sys

# Create logger
logger = logging.getLogger("raspbot")
logger.setLevel(logging.DEBUG)

# Console handler
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)

# Formatter
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
handler.setFormatter(formatter)

# Add handler
logger.addHandler(handler)
