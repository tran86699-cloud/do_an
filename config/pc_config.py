#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC CONFIGURATION
Store PC IP address and settings

File location: /home/heiina/raspbot_intelligent/config/pc_config.py
"""

# ============================================================================
# PC SERVER CONFIGURATION
# ============================================================================

# PC IP Address
# IMPORTANT: Change this to your PC's actual IP address!
# Find PC IP:
#   - Windows: ipconfig
#   - Mac/Linux: ifconfig
#   - Look for address in same subnet as Pi (192.168.X.X)

PC_IP = "192.168.100.11"  # ← CHANGE THIS!

# PC Server Port
PC_PORT = 8000

# Connection Settings
PC_TIMEOUT = 2  # seconds
PC_RETRY_INTERVAL = 5  # seconds
PC_UPDATE_INTERVAL = 0.5  # seconds (send data every 0.5s)
PC_POLL_INTERVAL = 1.0  # seconds (poll commands every 1s)

# ============================================================================
# VALIDATION
# ============================================================================

def validate_config():
    """Validate PC configuration"""
    
    # Check IP format
    parts = PC_IP.split('.')
    if len(parts) != 4:
        raise ValueError(f"Invalid PC_IP format: {PC_IP}")
    
    for part in parts:
        if not part.isdigit() or not (0 <= int(part) <= 255):
            raise ValueError(f"Invalid PC_IP format: {PC_IP}")
    
    # Check port
    if not (1 <= PC_PORT <= 65535):
        raise ValueError(f"Invalid PC_PORT: {PC_PORT}")
    
    print(f"[CONFIG] PC Server: http://{PC_IP}:{PC_PORT}")
    print(f"[CONFIG] Update interval: {PC_UPDATE_INTERVAL}s")
    print(f"[CONFIG] Poll interval: {PC_POLL_INTERVAL}s")
    
    return True


if __name__ == '__main__':
    # Test config
    print("="*70)
    print("PC CONFIGURATION TEST")
    print("="*70)
    
    try:
        validate_config()
        print("\n✅ Configuration valid!")
    except Exception as e:
        print(f"\n❌ Configuration error: {e}")
        print("\nPlease edit pc_config.py and set:")
        print(f"  PC_IP = \"YOUR_PC_IP_HERE\"")
    
    print("="*70)


