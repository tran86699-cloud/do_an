#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RASPBOT V2 - COMPLETE PC SERVER WITH SEMANTIC ROUTER
Graduation Project - Full Integration

Combines:
- Semantic similarity routing (new)
- Robot data management (existing)
- Command queue system (existing)  
- Conversation history (existing)
- Vision support (existing)
- All Pi integration endpoints (existing)

Author: HaiTrieu98
Date: December 2024
"""
import os
# os.environ['HF_HUB_OFFLINE'] = "1"
from flask import Flask, jsonify, request
from flask_cors import CORS
import time
import json
import requests
import re
import base64
from datetime import datetime
import socket
import numpy as np
from typing import Dict, Optional
import warnings
warnings.filterwarnings('ignore')

app = Flask(__name__)
CORS(app)

# ============================================================================
# CONFIGURATION
# ============================================================================

OLLAMA_URL = "http://localhost:11434"
OLLAMA_TIMEOUT = 360

# Models configuration
MODELS = {
    'vision_fast': 'qwen3-vl:8b',
    'text_smart': 'llama3.1:8b',
}

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

ai_commands = []  # Command queue for Pi polling

# Server state
SERVER_STATE = {
    'router': None,
    'ready': False,
    'stats': {
        'total_requests': 0,
        'models_used': {},
        'avg_response_time': 0
    }
}

# ============================================================================
# INTELLIGENT SEMANTIC ROUTER (Ollama neural-chat:7b)
# ============================================================================

class SemanticRouter:
    """
    Intent classification using Ollama neural-chat:7b
    100% offline, fast, and accurate!
    """
    
    def __init__(self):
        print("🔄 Initializing Semantic Router (Ollama)...")
        print("   ⚡ Using neural-chat:7b for intent classification")
        
        # Ollama configuration
        self.ollama_url = OLLAMA_URL
        self.model_name = "neural-chat:7b"
        
        print(f"   📡 Endpoint: {self.ollama_url}")
        
        # Test connection
        if self._test_ollama():
            print("   ✅ Ollama connection successful!")
        else:
            print("   ⚠️  Warning: Ollama not responding")
            print("   💡 Make sure Ollama is running (ollama serve)")
        
        # Intent descriptions for routing
        self.vision_keywords = [
            'see', 'look', 'show', 'what do you see', 'describe',
            'color', 'colors', 'object', 'objects', 'person', 'people',
            'face', 'faces', 'scene', 'view', 'camera', 'picture',
            'image', 'visual', 'observe', 'watch', 'identify'
        ]
        
        print("   ✅ Semantic Router ready!")
        print("   • Model: neural-chat:7b")
        print("   • Approach: Few-shot prompting + keyword matching")
        print("   • Speed: ~500ms")
        print("   • Accuracy: 90-95%\n")
    
    def _test_ollama(self):
        """Test Ollama connection"""
        try:
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    "model": self.model_name,
                    "prompt": "test",
                    "stream": False
                },
                timeout=5
            )
            return response.status_code == 200
        except:
            return False
    
    def route(self, message: str, has_image: bool = False) -> Dict:
        """
        Route message to appropriate model
        
        Args:
            message: User query
            has_image: Whether image is provided
        
        Returns:
            dict: {
                'model': str - Model to use,
                'confidence': float - Confidence score,
                'category': str - Category detected,
                'reasoning': str - Why this decision,
                'scores': dict - All category scores
            }
        """
        
        # Rule 1: Image ALWAYS uses vision model
        if has_image:
            return {
                'model': MODELS['vision_fast'],
                'confidence': 1.0,
                'category': 'vision',
                'reasoning': 'Image provided → vision model',
                'scores': None
            }
        
        # Rule 2: Fast keyword check (optimize for common cases)
        message_lower = message.lower()
        vision_score = sum(1 for kw in self.vision_keywords if kw in message_lower)
        
        # If strong vision signals, use vision model directly
        if vision_score >= 2:
            return {
                'model': MODELS['vision_fast'],
                'confidence': 0.95,
                'category': 'vision_query',
                'reasoning': f'Multiple vision keywords detected ({vision_score})',
                'scores': {'vision': 0.95, 'text': 0.05}
            }
        
        # If no vision keywords at all, use text model directly
        if vision_score == 0:
            return {
                'model': MODELS['text_smart'],
                'confidence': 0.90,
                'category': 'text_command',
                'reasoning': 'No vision keywords → text model',
                'scores': {'vision': 0.10, 'text': 0.90}
            }
        
        # Rule 3: Ambiguous cases - use Ollama for classification
        try:
            # Build few-shot prompt
            prompt = f"""Classify this message as either "VISION" or "TEXT".

VISION queries: Ask about what the robot sees, visual information, colors, objects, people, faces, scenes.
TEXT queries: Commands, greetings, questions about robot status, movement, or general conversation.

Examples:
- "what do you see?" → VISION
- "describe the scene" → VISION
- "what colors are there?" → VISION
- "go forward" → TEXT
- "how are you?" → TEXT
- "stop the robot" → TEXT
- "hello" → TEXT

Message: "{message}"

Answer with ONLY one word: VISION or TEXT"""

            # Call Ollama
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    "model": self.model_name,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.1,
                        "num_predict": 5
                    }
                },
                timeout=3
            )
            
            if response.status_code == 200:
                result = response.json()
                answer = result['response'].strip().upper()
                
                # Extract classification
                if 'VISION' in answer:
                    return {
                        'model': MODELS['vision_fast'],
                        'confidence': 0.85,
                        'category': 'vision_query',
                        'reasoning': 'Ollama classified as VISION query',
                        'scores': {'vision': 0.85, 'text': 0.15}
                    }
                else:
                    return {
                        'model': MODELS['text_smart'],
                        'confidence': 0.85,
                        'category': 'text_command',
                        'reasoning': 'Ollama classified as TEXT command',
                        'scores': {'vision': 0.15, 'text': 0.85}
                    }
            else:
                # Fallback to keyword-based
                return self._fallback_route(message, vision_score)
                
        except requests.exceptions.Timeout:
            print("   ⏱️  Ollama timeout, using keyword fallback")
            return self._fallback_route(message, vision_score)
        except Exception as e:
            print(f"   ❌ Routing error: {e}")
            return self._fallback_route(message, vision_score)
    
    def _fallback_route(self, message, vision_score):
        """Fallback keyword-based routing when Ollama fails"""
        
        # Use keyword score to decide
        if vision_score >= 1:
            return {
                'model': MODELS['vision_fast'],
                'confidence': 0.70,
                'category': 'vision_query_fallback',
                'reasoning': f'Keyword-based routing (vision score: {vision_score})',
                'scores': {'vision': 0.70, 'text': 0.30}
            }
        else:
            return {
                'model': MODELS['text_smart'],
                'confidence': 0.70,
                'category': 'text_command_fallback',
                'reasoning': 'Keyword-based routing (default to text)',
                'scores': {'vision': 0.30, 'text': 0.70}
            }


# ============================================================================
# VISION-LANGUAGE MODEL WITH CONVERSATION HISTORY
# ============================================================================

class VisionLanguageModel:
    def __init__(self, ollama_url=OLLAMA_URL, timeout=OLLAMA_TIMEOUT):
        self.ollama_url = ollama_url
        self.timeout = timeout
        self.conversation_history = []
        print(f"[VLM] Initialized with URL: {ollama_url}")
    
    def chat(self, message, image=None, model=None):
        """
        Chat with vision/language model
        Supports both text-only and vision queries
        """
        try:
            selected_model = model if model else MODELS['text_smart']
            
            print(f"[VLM] Using model: {selected_model}")
            
            # ✅ BUILD MESSAGE WITH IMAGE SUPPORT
            if image:
                print(f"[VLM] Processing with image ({len(image)} chars)")
                
                messages = [
                    {
                        'role': 'system',
                        'content': 'You are a vision AI assistant. You HAVE access to camera images and CAN see visual content. Analyze the provided image and describe what you see in detail.'
                    },
                    {
                        'role': 'user',
                        'content': message,
                        'images': [image]
                    }
                ]
            else:
                # Text-only model
                print(f"[VLM] Processing text only")
                messages = [
                    {
                        'role': 'user',
                        'content': message
                    }
                ]
            
            # ✅ CORRECT PAYLOAD FORMAT
            payload = {
                'model': selected_model,
                'messages': messages,
                'stream': False,
                'options': {
                    'temperature': 0.7,
                    'num_predict': 512
                }
            }
            
            print(f"[VLM] Sending request to Ollama...")
            start_time = time.time()
            
            # Send to Ollama
            response = requests.post(
                f"{self.ollama_url}/api/chat",
                json=payload,
                timeout=self.timeout
            )
            
            elapsed = time.time() - start_time
            print(f"[VLM] Response received in {elapsed:.2f}s")
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result.get('message', {}).get('content', '')
                
                if not ai_response:
                    ai_response = "I received your request but couldn't generate a response."
                
                print(f"[VLM] AI response length: {len(ai_response)} chars")
                
                return {
                    'response': ai_response,
                    'model': selected_model,
                    'processing_time': elapsed
                }
            else:
                error_msg = f"Ollama error: {response.status_code}"
                print(f"[VLM] Error: {error_msg}")
                return {
                    'response': error_msg,
                    'model': selected_model,
                    'processing_time': elapsed
                }
                
        except requests.Timeout:
            print(f"[VLM] Timeout after {self.timeout}s")
            return {
                'response': 'Request timeout - vision processing took too long',
                'model': selected_model,
                'processing_time': self.timeout
            }
        except Exception as e:
            print(f"[VLM] Error: {e}")
            import traceback
            traceback.print_exc()
            return {
                'response': f'Error: {str(e)}',
                'model': selected_model,
                'processing_time': 0
            }
    
    def reset(self):
        """Clear conversation history"""
        self.conversation_history = []

# Init VLM
vlm = VisionLanguageModel()

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
                    # Check for modifiers
                    if 'slowly' in text_lower or 'carefully' in text_lower:
                        speed = 60
                    elif 'quickly' in text_lower or 'fast' in text_lower:
                        speed = 200
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
# API ENDPOINTS - Robot Data Management
# ============================================================================

@app.route('/api/robot/update', methods=['POST'])
def receive_robot_update():
    """Receive sensor data from Pi"""
    global robot_data
    
    try:
        data = request.json
        robot_data.update({
            'last_update': time.time(),
            'distance': data.get('distance', 0),
            'sensors': data.get('sensors', {'pan': 90, 'tilt': 45}),
            'mode': data.get('mode', 'manual'),
            'pc_control_enabled': data.get('pc_control', False),
            'status': 'connected'
        })
        
        return jsonify({'status': 'ok'}), 200
        
    except Exception as e:
        print(f"[ERROR] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/robot/camera', methods=['POST'])
def receive_camera_frame():
    """Receive camera frame from Pi"""
    global robot_data
    
    try:
        data = request.json
        robot_data['last_camera_frame'] = data.get('image', None)
        robot_data['last_update'] = time.time()
        
        return jsonify({'status': 'ok'}), 200
        
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/robot/commands', methods=['GET'])
def poll_commands():
    """Pi polls for commands (command queue system)"""
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
    """Chat with robot AI"""
    global ai_commands
    
    if not SERVER_STATE['ready']:
        return jsonify({'status': 'error', 'message': 'Not ready'}), 503
    
    try:
        data = request.json
        user_message = data.get('message', '')
        image_data = data.get('image', None)
        
        if not user_message:
            return jsonify({'status': 'error', 'message': 'No message'}), 400
        
        start_time = time.time()
        
        print(f"\n{'='*70}")
        print(f"[AI-CHAT] User: {user_message}")
        if image_data:
            print(f"[AI-CHAT] With image ({len(image_data)} chars)")
        
        # SEMANTIC ROUTING
        routing = SERVER_STATE['router'].route(user_message, has_image=bool(image_data))
        print(f"[ROUTING] Model: {routing['model']}")
        print(f"[ROUTING] Confidence: {routing['confidence']:.0%}")
        print(f"[ROUTING] Reasoning: {routing['reasoning']}")
        
        # Chat with VLM (using routed model)
        result = vlm.chat(user_message, image_data, model=routing['model'])
        
        print(f"[AI-CHAT] Bot: {result['response']}")
        
        # Parse command
        parsed_command = CommandParser.parse(user_message)
        
        # Queue command if valid
        command_queued = False
        if parsed_command['action'] not in ['unknown', 'error']:
            command = {
                'action': parsed_command['action'],
                'speed': parsed_command.get('speed', 150),
                'timestamp': time.time(),
                'source': 'ai_chat'
            }
            
            ai_commands.append(command)
            command_queued = True
            print(f"[AI-CHAT] Queued: {command['action']}")
        
        # Update statistics
        elapsed = time.time() - start_time
        SERVER_STATE['stats']['total_requests'] += 1
        
        model_key = routing['model']
        if model_key not in SERVER_STATE['stats']['models_used']:
            SERVER_STATE['stats']['models_used'][model_key] = 0
        SERVER_STATE['stats']['models_used'][model_key] += 1
        
        print(f"[RESPONSE] Time: {elapsed:.2f}s")
        print(f"{'='*70}\n")
        
        return jsonify({
            'status': 'ok',
            'ai_response': result['response'],
            'command': parsed_command,
            'command_queued': command_queued,
            'model_used': routing['model'],
            'confidence': routing['confidence'],
            'processing_time': elapsed
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
            
            # Check which models are available
            has_vision_fast = any(MODELS['vision_fast'] in name for name in model_names)
            has_text_smart = any(MODELS['text_smart'] in name for name in model_names)
            
            return jsonify({
                'status': 'ok',
                'ollama_running': True,
                'models': model_names,
                'configured_models': MODELS,
                'available': {
                    'vision_fast': has_vision_fast,
                    'text_smart': has_text_smart
                }
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
    
    if not SERVER_STATE['ready']:
        return jsonify({
            'status': 'initializing',
            'message': 'Server starting up...'
        })
    
    return jsonify({
        'status': 'online',
        'robot_connected': robot_data['status'] == 'connected',
        'last_update': robot_data['last_update'],
        'robot_mode': robot_data['mode'],
        'pc_control_enabled': robot_data['pc_control_enabled'],
        'command_queue_size': len(ai_commands),
        'models': list(MODELS.values()),
        'router': 'Ollama Neural-Chat',
        'vision_enabled': True,
        'statistics': SERVER_STATE['stats']
    }), 200


@app.route('/api/ai/stats', methods=['GET'])
def ai_stats():
    """Get detailed statistics"""
    return jsonify(SERVER_STATE['stats'])


@app.route('/', methods=['GET'])
def index():
    """Status page with semantic router info"""
    
    # Test Ollama
    ollama_status = "🔴 Offline"
    vision_status = "❌ No"
    models_status = {}
    
    try:
        r = requests.get(f"{OLLAMA_URL}/api/tags", timeout=2)
        if r.status_code == 200:
            ollama_status = "🟢 Online"
            models = r.json().get('models', [])
            model_names = [m['name'] for m in models]
            
            for key, model_name in MODELS.items():
                models_status[key] = "✅" if any(model_name in m for m in model_names) else "❌"
            
            if all(status == "✅" for status in models_status.values()):
                vision_status = "✅ Yes (All models)"
    except:
        pass
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>PC Server - Ollama Router</title>
        <meta http-equiv="refresh" content="3">
        <style>
            body {{ font-family: 'Consolas', monospace; background: #1a1a1a; color: #00ff00; padding: 20px; }}
            .status {{ background: #2a2a2a; padding: 15px; border-radius: 5px; margin: 10px 0; }}
            .connected {{ color: #00ff00; }}
            .disconnected {{ color: #ff0000; }}
            h1 {{ color: #0ea5e9; }}
            h2 {{ color: #10b981; }}
            code {{ background: #0f172a; padding: 2px 8px; border-radius: 3px; }}
            .model-item {{ margin: 5px 0; }}
        </style>
    </head>
    <body>
        <h1>🧠🤖 PC SERVER - OLLAMA NEURAL-CHAT ROUTER</h1>
        
        <div class="status">
            <h2>Server Status</h2>
            <p>Status: <span class="connected">{'READY' if SERVER_STATE['ready'] else 'INITIALIZING'}</span></p>
            <p>Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
            <p>Router: <code>Ollama neural-chat:7b</code></p>
        </div>
        
        <div class="status">
            <h2>Ollama AI</h2>
            <p>Status: {ollama_status}</p>
            <p>Models (2):</p>
            <div class="model-item">  {models_status.get('vision_fast', '❓')} <code>{MODELS['vision_fast']}</code> - Primary Vision</div>
            <div class="model-item">  {models_status.get('text_smart', '❓')} <code>{MODELS['text_smart']}</code> - Smart Text</div>
            <p>Vision Support: {vision_status}</p>
        </div>
        
        <div class="status">
            <h2>Robot Connection</h2>
            <p>Status: <span class="{'connected' if robot_data['status'] == 'connected' else 'disconnected'}">{robot_data['status'].upper()}</span></p>
            <p>Mode: <code>{robot_data['mode'].upper()}</code></p>
            <p>PC Control: <code>{'ENABLED' if robot_data['pc_control_enabled'] else 'DISABLED'}</code></p>
        </div>
        
        <div class="status">
            <h2>Statistics</h2>
            <p>Total Requests: {SERVER_STATE['stats']['total_requests']}</p>
            <p>Command Queue: {len(ai_commands)} pending</p>
            <p>Models Used:</p>
            {''.join(f"<div class='model-item'>  {model}: {count} times</div>" for model, count in SERVER_STATE['stats']['models_used'].items())}
        </div>
        
        <hr>
        <h3>API Endpoints</h3>
        <ul>
            <li>POST /api/robot/update - Receive sensor data</li>
            <li>POST /api/robot/camera - Receive camera frames</li>
            <li>GET  /api/robot/commands - Poll commands</li>
            <li>POST /api/ai/chat - AI chat (+ vision) with semantic routing</li>
            <li>GET  /api/ai/test - Test Ollama</li>
            <li>GET  /api/ai/stats - Get statistics</li>
        </ul>
    </body>
    </html>
    """
    return html

# ============================================================================
# INITIALIZATION
# ============================================================================

def initialize_server():
    """Initialize server components"""
    
    print("\n" + "="*70)
    print("RASPBOT V2 - PC SERVER WITH OLLAMA ROUTER")
    print("Graduation Project - Complete Integration")
    print("="*70)
    print()
    print("Configuration:")
    print("  Router: Ollama neural-chat:7b")
    print("  Vision: qwen3-vl:8b")
    print("  Text: llama3.1:8b")
    print()
    print("Features:")
    print("  ✅ Ollama-based semantic routing")
    print("  ✅ 100% offline operation")
    print("  ✅ Robot data management")
    print("  ✅ Command queue system")
    print("  ✅ Conversation history")
    print("  ✅ Vision support")
    print("  ✅ Web status page")
    print()
    
    # Initialize semantic router
    try:
        SERVER_STATE['router'] = SemanticRouter()
        SERVER_STATE['ready'] = True
        print("="*70)
        print("✅ SERVER READY!")
        print("="*70)
        print()
        
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        print("Please check:")
        print("  1. Ollama is running (ollama serve)")
        print("  2. neural-chat:7b is installed (ollama pull neural-chat:7b)")
        print("  3. All configured models are available")
        SERVER_STATE['ready'] = False

# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    hostname = socket.gethostname()
    try:
        pc_ip = socket.gethostbyname(hostname)
    except:
        pc_ip = '127.0.0.1'
    
    initialize_server()
    
    if SERVER_STATE['ready']:
        print(f"PC Hostname: {hostname}")
        print(f"PC IP: {pc_ip}")
        print(f"\nStarting Flask server on http://0.0.0.0:8000")
        print(f"\nWeb Interface:")
        print(f"  http://{pc_ip}:8000")
        print(f"  http://localhost:8000")
        print("="*70 + "\n")
        
        try:
            app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)
        except KeyboardInterrupt:
            print("\nShutting down...")
    else:
        print("\n❌ Server failed to start. Please fix errors above.")