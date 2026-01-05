#!/usr/bin/env python3
"""
PC SERVER WITH VISION - LLaVA-Llama3
Windows compatible version
Supports both text chat and vision analysis for robot control
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
import time
import json
import requests
import re
import base64
from datetime import datetime
import socket

app = Flask(__name__)
CORS(app)

# ============================================================================
# CONFIGURATION
# ============================================================================

OLLAMA_URL = "http://localhost:11434"
OLLAMA_MODEL = "qwen2.5vl:7b"  # Vision + Chat model (Qwen2.5-VL 7B)

print(f"[CONFIG] Using model: {OLLAMA_MODEL}")

# ============================================================================
# GLOBAL STATE
# ============================================================================

robot_data = {
    'last_update': None,
    'distance': 0,
    'sensors': {'pan': 90, 'tilt': 45},
    'mode': 'manual',
    'pc_control_enabled': False,
    'status': 'disconnected',
    'last_vision_analysis': None,
    'last_camera_frame': None
}

ai_commands = []  # Command queue

# ============================================================================
# COMMAND PARSER
# ============================================================================

class CommandParser:
    """Parse natural language to robot commands"""
    
    COMMANDS = {
        # Movement
        'go forward': 'move_forward',
        'move forward': 'move_forward',
        'forward': 'move_forward',
        'go back': 'move_backward',
        'backward': 'move_backward',
        'turn left': 'rotate_left',
        'rotate left': 'rotate_left',
        'turn right': 'rotate_right',
        'rotate right': 'rotate_right',
        'stop': 'stop',
        
        # Autonomous
        'find face': 'start_face_tracking',
        'track face': 'start_face_tracking',
        'find person': 'start_face_tracking',
        'follow': 'start_face_tracking',
        'patrol': 'start_patrol',
        'avoid': 'start_obstacle_avoidance',
        
        # Camera
        'look left': 'pan_left',
        'look right': 'pan_right',
        'look up': 'tilt_up',
        'look down': 'tilt_down',
        'center camera': 'camera_center',
    }
    
    @classmethod
    def parse(cls, text):
        """Parse text to command"""
        text_lower = text.lower().strip()
        
        for keyword, action in cls.COMMANDS.items():
            if keyword in text_lower:
                # Extract speed
                speed_match = re.search(r'speed\s*(\d+)|(\d+)\s*(?:km|%)', text_lower)
                if speed_match:
                    speed = int([g for g in speed_match.groups() if g][0])
                else:
                    speed = 150
                
                return {
                    'action': action,
                    'speed': min(255, max(50, speed)),
                    'confidence': 0.9,
                    'raw_text': text
                }
        
        return {
            'action': 'unknown',
            'speed': 0,
            'confidence': 0.0,
            'raw_text': text
        }

# ============================================================================
# VISION-LANGUAGE MODEL
# ============================================================================

class VisionLanguageModel:
    """LLaVA-Llama3 vision + chat model"""
    
    def __init__(self, model=OLLAMA_MODEL):
        self.model = model
        self.conversation_history = []
        
        self.system_prompt = """You are an AI controlling a robot called Raspbot V2.

Your capabilities:
1. VISION: Analyze camera images to understand surroundings
2. CONTROL: Send movement commands to robot
3. NAVIGATION: Plan paths based on visual information

Available commands:
- move_forward, move_backward, rotate_left, rotate_right, stop
- start_face_tracking (find and follow person)
- start_patrol (explore area)
- start_obstacle_avoidance (auto navigate)
- pan_left, pan_right, tilt_up, tilt_down, camera_center

When analyzing images:
- Identify objects with positions (left/center/right)
- Estimate distances if possible (near/far, or meters)
- Detect people and their gestures
- Identify obstacles and clear paths
- Suggest navigation actions

IMPORTANT:
- Keep responses concise (1-3 sentences)
- For commands, confirm action: "Moving forward now"
- For vision, describe briefly then suggest action
- If unclear, ask for clarification
"""
    
    def chat(self, user_message, image_data=None):
        """Chat with or without image"""
        try:
            # Check Ollama
            try:
                requests.get(f"{OLLAMA_URL}/api/tags", timeout=2)
            except:
                return {
                    'response': 'Error: Ollama not running. Start with: ollama serve',
                    'command': {'action': 'error'},
                    'success': False
                }
            
            # Add to history
            self.conversation_history.append({
                'role': 'user',
                'content': user_message
            })
            
            # Keep last 10 messages
            if len(self.conversation_history) > 10:
                self.conversation_history = self.conversation_history[-10:]
            
            # Prepare request
            messages = [
                {'role': 'system', 'content': self.system_prompt}
            ] + self.conversation_history
            
            # Call Ollama
            payload = {
                'model': self.model,
                'messages': messages,
                'stream': False
            }
            
            # Add images if provided
            if image_data:
                payload['images'] = [image_data]
            
            response = requests.post(
                f"{OLLAMA_URL}/api/chat",
                json=payload,
                timeout=120
            )
            
            if response.status_code == 200:
                data = response.json()
                ai_response = data['message']['content']
                
                # Add to history
                self.conversation_history.append({
                    'role': 'assistant',
                    'content': ai_response
                })
                
                # Parse command
                parsed_command = CommandParser.parse(user_message)
                
                return {
                    'response': ai_response,
                    'command': parsed_command,
                    'success': True,
                    'has_vision': image_data is not None
                }
            else:
                return {
                    'response': 'AI did not respond',
                    'command': {'action': 'unknown'},
                    'success': False
                }
                
        except Exception as e:
            print(f"[VLM] Error: {e}")
            return {
                'response': f'Error: {str(e)}',
                'command': {'action': 'error'},
                'success': False
            }
    
    def reset(self):
        """Clear conversation history"""
        self.conversation_history = []

# Init VLM
vlm = VisionLanguageModel()

# ============================================================================
# API ENDPOINTS - Robot Data
# ============================================================================

@app.route('/api/robot/update', methods=['POST'])
def receive_robot_update():
    """Receive sensor data from Pi"""
    global robot_data
    
    try:
        data = request.json
        
        if not data:
            return jsonify({'status': 'error', 'message': 'No data'}), 400
        
        robot_data.update({
            'last_update': datetime.now().isoformat(),
            'distance': data.get('distance', 0),
            'sensors': {
                'pan': data.get('pan', 90),
                'tilt': data.get('tilt', 45)
            },
            'mode': data.get('mode', 'manual'),
            'pc_control_enabled': data.get('pc_control_enabled', False),
            'status': 'connected'
        })
        
        print(f"[UPDATE] Mode: {robot_data['mode']}, Dist: {robot_data['distance']:.1f}cm")
        
        return jsonify({
            'status': 'ok',
            'timestamp': robot_data['last_update']
        }), 200
        
    except Exception as e:
        print(f"[ERROR] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/robot/camera', methods=['POST'])
def receive_camera_frame():
    """Receive camera frame from Pi for vision analysis"""
    global robot_data
    
    try:
        # Get image data (base64 or binary)
        if request.is_json:
            data = request.json
            frame_data = data.get('frame')
        else:
            frame_data = base64.b64encode(request.data).decode('utf-8')
        
        if frame_data:
            # Store latest frame
            robot_data['last_camera_frame'] = frame_data
            robot_data['last_update'] = datetime.now().isoformat()
            
            print(f"[CAMERA] Frame received")
            
            return jsonify({'status': 'ok'}), 200
        else:
            return jsonify({'status': 'error', 'message': 'No frame data'}), 400
            
    except Exception as e:
        print(f"[ERROR] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/robot/commands', methods=['GET'])
def get_robot_commands():
    """Pi polls for commands"""
    global ai_commands
    
    try:
        if len(ai_commands) == 0:
            return jsonify({
                'status': 'ok',
                'commands': [],
                'count': 0
            }), 200
        
        commands = ai_commands.copy()
        ai_commands.clear()
        
        print(f"[POLL] Sending {len(commands)} commands to Pi")
        
        return jsonify({
            'status': 'ok',
            'commands': commands,
            'count': len(commands)
        }), 200
        
    except Exception as e:
        print(f"[ERROR] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

# ============================================================================
# API ENDPOINTS - AI Chat & Vision
# ============================================================================

@app.route('/api/ai/chat', methods=['POST'])
def ai_chat():
    """Chat with robot AI (text or text+image)"""
    global ai_commands
    
    try:
        data = request.json
        user_message = data.get('message', '')
        image_data = data.get('image', None)
        
        if not user_message:
            return jsonify({
                'status': 'error',
                'message': 'No message'
            }), 400
        
        print(f"[AI-CHAT] User: {user_message}")
        if image_data:
            print(f"[AI-CHAT] With image ({len(image_data)} chars)")
        
        # Chat with VLM
        result = vlm.chat(user_message, image_data)
        
        print(f"[AI-CHAT] Bot: {result['response']}")
        
        # Queue command if valid
        command_queued = False
        if result['command']['action'] not in ['unknown', 'error']:
            command = {
                'action': result['command']['action'],
                'speed': result['command'].get('speed', 150),
                'timestamp': time.time(),
                'source': 'ai_chat'
            }
            
            ai_commands.append(command)
            command_queued = True
            print(f"[AI-CHAT] Queued: {command['action']}")
        
        return jsonify({
            'status': 'ok',
            'ai_response': result['response'],
            'command': result['command'],
            'command_queued': command_queued
        }), 200
        
    except Exception as e:
        print(f"[AI-CHAT] Error: {e}")
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500


@app.route('/api/ai/test', methods=['GET'])
def test_ollama():
    """Test Ollama connection"""
    try:
        response = requests.get(f"{OLLAMA_URL}/api/tags", timeout=5)
        
        if response.status_code == 200:
            models = response.json().get('models', [])
            model_names = [m['name'] for m in models]
            
            # Check if vision model is available
            has_vision = any(OLLAMA_MODEL in name for name in model_names)
            
            return jsonify({
                'status': 'ok',
                'ollama_running': True,
                'models': model_names,
                'current_model': OLLAMA_MODEL,
                'vision_available': has_vision
            }), 200
        else:
            return jsonify({
                'status': 'error',
                'ollama_running': False
            }), 500
            
    except Exception as e:
        return jsonify({
            'status': 'error',
            'ollama_running': False,
            'message': str(e)
        }), 500

# ============================================================================
# STATUS & INFO
# ============================================================================

@app.route('/api/pc/status', methods=['GET'])
def pc_status():
    """Get PC server status"""
    return jsonify({
        'status': 'online',
        'robot_connected': robot_data['status'] == 'connected',
        'last_update': robot_data['last_update'],
        'robot_mode': robot_data['mode'],
        'pc_control_enabled': robot_data['pc_control_enabled'],
        'command_queue_size': len(ai_commands),
        'ollama_model': OLLAMA_MODEL,
        'vision_enabled': True
    }), 200


@app.route('/', methods=['GET'])
def index():
    """Status page"""
    
    # Test Ollama
    ollama_status = "🔴 Offline"
    vision_status = "❌ No"
    try:
        r = requests.get(f"{OLLAMA_URL}/api/tags", timeout=2)
        if r.status_code == 200:
            ollama_status = "🟢 Online"
            models = r.json().get('models', [])
            if any(OLLAMA_MODEL in m['name'] for m in models):
                vision_status = "✅ Yes"
    except:
        pass
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>PC Server - Vision AI</title>
        <meta http-equiv="refresh" content="3">
        <style>
            body {{ font-family: 'Consolas', monospace; background: #1a1a1a; color: #00ff00; padding: 20px; }}
            .status {{ background: #2a2a2a; padding: 15px; border-radius: 5px; margin: 10px 0; }}
            .connected {{ color: #00ff00; }}
            .disconnected {{ color: #ff0000; }}
            h1 {{ color: #0ea5e9; }}
            code {{ background: #0f172a; padding: 2px 8px; border-radius: 3px; }}
        </style>
    </head>
    <body>
        <h1>🧠👁️ PC SERVER - VISION AI (Windows)</h1>
        
        <div class="status">
            <h2>Server Status</h2>
            <p>Status: <span class="connected">ONLINE</span></p>
            <p>Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        </div>
        
        <div class="status">
            <h2>Ollama AI</h2>
            <p>Status: {ollama_status}</p>
            <p>Model: <code>{OLLAMA_MODEL}</code></p>
            <p>Vision Support: {vision_status}</p>
        </div>
        
        <div class="status">
            <h2>Robot Connection</h2>
            <p>Status: <span class="{'connected' if robot_data['status'] == 'connected' else 'disconnected'}">{robot_data['status'].upper()}</span></p>
            <p>Mode: <code>{robot_data['mode'].upper()}</code></p>
        </div>
        
        <div class="status">
            <h2>Command Queue</h2>
            <p>Pending: {len(ai_commands)} commands</p>
        </div>
        
        <hr>
        <h3>API Endpoints</h3>
        <ul>
            <li>POST /api/robot/update - Receive sensor data</li>
            <li>POST /api/robot/camera - Receive camera frames</li>
            <li>GET  /api/robot/commands - Poll commands</li>
            <li>POST /api/ai/chat - AI chat (+ vision)</li>
            <li>GET  /api/ai/test - Test Ollama</li>
        </ul>
    </body>
    </html>
    """
    return html

# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    hostname = socket.gethostname()
    try:
        pc_ip = socket.gethostbyname(hostname)
    except:
        pc_ip = '127.0.0.1'
    
    print("="*70)
    print("PC SERVER - VISION AI CONTROL (Windows)")
    print("="*70)
    print(f"PC Hostname: {hostname}")
    print(f"PC IP: {pc_ip}")
    print(f"Ollama Model: {OLLAMA_MODEL}")
    print(f"Vision: ENABLED")
    print("\nStarting server on http://0.0.0.0:8000")
    print("\nAPI Endpoints:")
    print("  POST /api/robot/update   - Receive sensor data")
    print("  POST /api/robot/camera   - Receive camera frames")
    print("  GET  /api/robot/commands - Poll commands")
    print("  POST /api/ai/chat        - AI chat (+ vision)")
    print("  GET  /api/ai/test        - Test Ollama")
    print("\nWeb Interface:")
    print(f"  http://{pc_ip}:8000")
    print(f"  http://localhost:8000")
    print("="*70 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")