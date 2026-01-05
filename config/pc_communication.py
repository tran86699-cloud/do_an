#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC Communication Module - WORKING VERSION
Compatible with fixed robot_api.py
"""

import requests
import json
import time

class PCCommunication:
    def __init__(self, pc_ip, pc_port):
        self.pc_ip = pc_ip
        self.pc_port = pc_port
        self.base_url = f"http://{pc_ip}:{pc_port}"
        self.timeout = 5
        print(f"[PC-COMM] Initialized for {self.base_url}")
    
    def test_connection(self):
        try:
            response = requests.get(f"{self.base_url}/api/pc/status", timeout=3)
            if response.status_code == 200:
                print(f"[PC-COMM] ✅ Connected")
                return True
            return False
        except Exception as e:
            print(f"[PC-COMM] ⚠️ Failed: {e}")
            return False
    
    def send_sensor_data(self, data):
        try:
            response = requests.post(f"{self.base_url}/api/robot/update", json=data, timeout=self.timeout)
            return response.json() if response.status_code == 200 else None
        except:
            return None
    
    def get_commands(self):
        try:
            response = requests.get(f"{self.base_url}/api/robot/commands", timeout=self.timeout)
            return response.json() if response.status_code == 200 else None
        except:
            return None
    
    def send_chat(self, message, image=None):
        """Send chat to PC AI with image support"""
        try:
            # ? ALWAYS include image in payload
            payload = {
                'message': message,
                'image': image  # ? CRITICAL! Must forward image!
            }
            
            # Log for debugging
            if image:
                print(f"[PC-COMM] Sending message WITH image ({len(image)} chars)")
            else:
                print(f"[PC-COMM] Sending message (text only)")
            
            # Increase timeout for vision queries
            timeout = 360 if image else 30
            
            response = requests.post(
                f"{self.base_url}/api/ai/chat",
                json=payload,
                timeout=timeout
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"[PC-COMM] Error: {response.status_code}")
                return {
                    'status': 'error',
                    'ai_response': f'PC server error: {response.status_code}',
                    'command': {'action': 'none'}
                }
                
        except requests.Timeout:
            print("[PC-COMM] Timeout!")
            return {
                'status': 'error',
                'ai_response': 'Timeout - vision takes 20-30 seconds',
                'command': {'action': 'none'}
            }
        except Exception as e:
            print(f"[PC-COMM] Error: {e}")
            return {
                'status': 'error',
                'ai_response': f'Error: {str(e)}',
                'command': {'action': 'none'}
            }

