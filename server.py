#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RASPBOT V2 - HYBRID ARCHITECTURE
Pi: Real-time execution (face tracking, servo, motors)
PC: AI & Strategy (chat, complex vision, monitoring)
"""

import os
import sys
import json
import requests
from datetime import datetime
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

# Import AI modules
from ai_engine import AIEngine
from command_parser import CommandParser

# Import Vision Detector
try:
    from vision_detector import get_vision_detector
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    print("⚠️ Vision detector not available")

app = Flask(__name__, static_folder='templates', static_url_path='')
CORS(app)

# ============================================================================
# GLOBAL STATE
# ============================================================================

class ServerState:
    """Server state management"""
    
    def __init__(self):
        self.chat_history = []
        self.max_history = 100
        self.start_time = datetime.now()
        
        # Initialize AI Engine
        self.ai_engine = AIEngine(
            robot_api_url="http://192.168.0.102:5000",
            ollama_url="http://localhost:11434"
        )
        
        # Initialize Vision Detector (PC-based complex vision)
        self.vision = None
        if VISION_AVAILABLE:
            try:
                self.vision = get_vision_detector()
                print("✅ Vision detector initialized (PC)")
            except Exception as e:
                print(f"⚠️ Vision detector init failed: {e}")
        
        # Pi robot state (monitored from PC)
        self.pi_state = {
            'is_running': False,
            'autonomous_mode': 'none',  # 'face_tracking', 'patrol', etc
            'last_update': datetime.now().isoformat()
        }

state = ServerState()

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def add_chat(role, message):
    """Add message to chat history"""
    entry = {
        'timestamp': datetime.now().isoformat(),
        'role': role,
        'message': message
    }
    state.chat_history.append(entry)
    if len(state.chat_history) > state.max_history:
        state.chat_history.pop(0)
    return entry

# ============================================================================
# STATIC FILES (WEB UI)
# ============================================================================

@app.route('/')
def index():
    """Serve Web UI"""
    return send_from_directory('templates', 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    """Serve CSS, JS, images"""
    return send_from_directory('templates', path)

# ============================================================================
# HEALTH & STATUS
# ============================================================================

@app.route('/api/health', methods=['GET'])
def health():
    """Health check"""
    return jsonify({
        'status': 'ok',
        'server': 'PC',
        'timestamp': datetime.now().isoformat(),
        'uptime': str(datetime.now() - state.start_time),
        'components': {
            'ai_engine': 'active',
            'vision_detector': 'active' if state.vision else 'disabled',
            'robot_api': 'http://192.168.100.13:5000',
            'ollama': 'http://localhost:11434'
        }
    }), 200

@app.route('/api/status', methods=['GET'])
def server_status():
    """Server status"""
    return jsonify({
        'server': 'online',
        'role': 'PC (Brain)',
        'components': {
            'ai_engine': 'active',
            'vision': 'active' if state.vision else 'disabled'
        },
        'chat_messages': len(state.chat_history),
        'uptime': str(datetime.now() - state.start_time),
        'timestamp': datetime.now().isoformat()
    }), 200

@app.route('/api/info', methods=['GET'])
def info():
    """System information"""
    return jsonify({
        'name': 'RASPBOT V2 - HYBRID ARCHITECTURE',
        'architecture': {
            'brain': 'PC (AI, Strategy, Vision)',
            'body': 'Pi (Real-time, Motors, Servos, Sensors)'
        },
        'features': [
            'Natural language understanding (Neural-Chat:7b)',
            'Real-time face tracking (Pi-based)',
            'AI vision analysis (PC-based)',
            'Autonomous patrol & search (Pi-based)',
            'Obstacle avoidance (Pi-based)',
            'Chat history management',
            'Live status monitoring'
        ],
        'endpoints': {
            'chat': '/api/chat',
            'pi_autonomous': {
                'start': '/api/pi/autonomous/start',
                'stop': '/api/pi/autonomous/stop',
                'status': '/api/pi/autonomous/status'
            },
            'pc_vision': {
                'face': '/api/detect/face',
                'obstacle': '/api/detect/obstacle',
                'pose': '/api/detect/pose',
                'hands': '/api/detect/hands'
            },
            'chat_history': '/api/chat/history'
        }
    }), 200

# ============================================================================
# CHAT & AI (PC-based)
# ============================================================================

@app.route('/api/chat', methods=['POST'])
def chat():
    """Main chat endpoint - PC AI processing"""
    try:
        data = request.json or {}
        user_input = data.get('message', '').strip()
        
        if not user_input:
            return jsonify({'error': 'message required'}), 400
        
        print(f"\n{'='*60}")
        print(f"[USER] {user_input}")
        
        add_chat('user', user_input)
        
        # Process through AI Engine (PC)
        result = state.ai_engine.process(user_input)
        input_type = result.get('input_type', 'UNKNOWN')
        response_obj = result.get('response', {})
        message = response_obj.get('message', 'No response')
        
        add_chat('assistant', message)
        
        print(f"[TYPE] {input_type}")
        print(f"[RESPONSE] {message}")
        print(f"{'='*60}\n")
        
        return jsonify({
            'input_type': input_type,
            'message': message,
            'type': response_obj.get('type', ''),
            'status': response_obj.get('status', 'unknown'),
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        print(f"[ERROR] Chat error: {e}")
        add_chat('system', f'Error: {str(e)}')
        return jsonify({
            'error': str(e),
            'message': '❌ Error processing request'
        }), 500

@app.route('/api/chat/history', methods=['GET'])
def get_chat_history():
    """Get chat history"""
    limit = request.args.get('limit', 50, type=int)
    return jsonify({
        'history': state.chat_history[-limit:],
        'count': len(state.chat_history)
    }), 200

@app.route('/api/chat/clear', methods=['POST'])
def clear_chat_history():
    """Clear chat history"""
    state.chat_history = []
    state.ai_engine.clear_history()
    return jsonify({'status': 'cleared'}), 200

# ============================================================================
# PI AUTONOMOUS CONTROL (Real-time on Pi)
# ============================================================================

@app.route('/api/pi/autonomous/start', methods=['POST'])
def start_pi_autonomous():
    """Start autonomous face tracking on Pi (real-time)"""
    try:
        data = request.json or {}
        mode = data.get('mode', 'face_tracking')
        
        # Send to Pi to start autonomous mode
        response = requests.post(
            "http://192.168.100.13:5000/api/autonomous/start",
            json={'mode': mode, 'duration': 300},
            timeout=5
        )
        
        if response.status_code == 200:
            state.pi_state['is_running'] = True
            state.pi_state['autonomous_mode'] = mode
            state.pi_state['last_update'] = datetime.now().isoformat()
            
            add_chat('bot', f'🤖 Pi autonomous mode started: {mode}')
            print(f"✅ Pi autonomous STARTED: {mode}")
            
            return jsonify({
                'status': 'started',
                'mode': mode,
                'message': f'Real-time face tracking on Pi started',
                'note': 'Pi controls robots in real-time, PC monitors'
            }), 200
        else:
            return jsonify({
                'error': 'Pi not responding',
                'details': response.text
            }), 503
    
    except Exception as e:
        print(f"[ERROR] Start Pi autonomous: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/pi/autonomous/stop', methods=['POST'])
def stop_pi_autonomous():
    """Stop autonomous mode on Pi"""
    try:
        # Send to Pi
        response = requests.post(
            "http://192.168.100.13:5000/api/autonomous/stop",
            timeout=5
        )
        
        if response.status_code == 200:
            state.pi_state['is_running'] = False
            state.pi_state['autonomous_mode'] = 'none'
            state.pi_state['last_update'] = datetime.now().isoformat()
            
            add_chat('bot', '⏹️ Pi autonomous mode stopped')
            print(f"⏹️ Pi autonomous STOPPED")
            
            return jsonify({
                'status': 'stopped',
                'message': 'Autonomous mode on Pi deactivated'
            }), 200
        else:
            return jsonify({'error': 'Pi not responding'}), 503
    
    except Exception as e:
        print(f"[ERROR] Stop Pi autonomous: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/pi/autonomous/status', methods=['GET'])
def get_pi_autonomous_status():
    """Get Pi autonomous status"""
    try:
        # Get from Pi
        response = requests.get(
            "http://192.168.100.13:5000/api/autonomous/status",
            timeout=3
        )
        
        if response.status_code == 200:
            pi_status = response.json()
            
            return jsonify({
                'pi_running': state.pi_state['is_running'],
                'pi_mode': state.pi_state['autonomous_mode'],
                'pi_details': pi_status,
                'last_update': state.pi_state['last_update']
            }), 200
        else:
            return jsonify({
                'pi_running': False,
                'error': 'Pi not responding'
            }), 503
    
    except Exception as e:
        return jsonify({
            'pi_running': False,
            'error': str(e)
        }), 500

# ============================================================================
# PC VISION DETECTION (Complex analysis)
# ============================================================================

@app.route('/api/detect/face', methods=['GET'])
def detect_face():
    """Face detection - PC analyzes camera"""
    try:
        if not state.vision:
            return jsonify({
                'error': 'Vision detector not available',
                'detected': False,
                'count': 0
            }), 503
        
        result = state.vision.get_face_detection()
        
        if result['detected']:
            add_chat('bot', f"👤 {result['count']} face(s) detected")
        
        return jsonify({
            'detected': result['detected'],
            'count': result['count'],
            'faces': result.get('boxes', []),
            'processor': 'PC',
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        print(f"[ERROR] Face detection: {e}")
        return jsonify({'error': str(e), 'detected': False}), 500

@app.route('/api/detect/obstacle', methods=['GET'])
def detect_obstacle():
    """Obstacle detection - PC analyzes camera"""
    try:
        if not state.vision:
            return jsonify({
                'error': 'Vision detector not available',
                'detected': False
            }), 503
        
        result = state.vision.get_obstacle_detection()
        
        return jsonify({
            'detected': result['detected'],
            'position': result.get('position', 'unknown'),
            'area': result.get('area', 0),
            'processor': 'PC',
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        print(f"[ERROR] Obstacle detection: {e}")
        return jsonify({'error': str(e), 'detected': False}), 500

@app.route('/api/detect/pose', methods=['GET'])
def detect_pose():
    """Pose detection - PC analyzes camera"""
    try:
        if not state.vision:
            return jsonify({'error': 'Vision detector not available'}), 503
        
        result = state.vision.get_pose_detection()
        return jsonify({
            **result,
            'processor': 'PC',
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        print(f"[ERROR] Pose detection: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/detect/hands', methods=['GET'])
def detect_hands():
    """Hand detection - PC analyzes camera"""
    try:
        if not state.vision:
            return jsonify({'error': 'Vision detector not available'}), 503
        
        result = state.vision.get_hand_detection()
        return jsonify({
            **result,
            'processor': 'PC',
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        print(f"[ERROR] Hand detection: {e}")
        return jsonify({'error': str(e)}), 500

# ============================================================================
# ERROR HANDLERS
# ============================================================================

@app.errorhandler(404)
def not_found(e):
    """404 handler"""
    return jsonify({'error': 'Endpoint not found'}), 404

@app.errorhandler(500)
def server_error(e):
    """500 handler"""
    return jsonify({'error': 'Server error'}), 500

# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    print("\n" + "="*70)
    print("🤖 RASPBOT V2 - HYBRID ARCHITECTURE")
    print("="*70)
    
    print("\n🧠 PC (Brain):")
    print("  • AI chat (Neural-Chat:7b)")
    print("  • Complex vision analysis")
    print("  • Strategy & learning")
    print("  • User interface")
    
    print("\n⚡ Pi (Body):")
    print("  • Real-time face tracking")
    print("  • Servo control (pan/tilt)")
    print("  • Motor control")
    print("  • Obstacle avoidance")
    print("  • Sensor reading")
    print("  • Camera streaming")
    
    print("\n📡 Communication:")
    print("  • PC → Pi: High-level commands (HTTP)")
    print("  • Pi → PC: Status updates (HTTP)")
    print("  • Camera: MJPEG streaming")
    
    print("\n✅ Features:")
    if VISION_AVAILABLE:
        print("  • Vision detection (face, obstacle, pose, hands)")
    print("  • Chat history management")
    print("  • Live status monitoring")
    print("  • Real-time autonomous control")
    
    print("\n🌐 Web Interface: http://localhost:3000")
    print("📚 API Docs: http://localhost:8000/api/info")
    
    print("\n" + "="*70)
    print("Starting server on http://0.0.0.0:8000...\n")
    
    try:
        app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n⏹️  Shutdown")
    except Exception as e:
        print(f"\n❌ Error: {e}")