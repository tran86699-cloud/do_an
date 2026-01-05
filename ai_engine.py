# ~/raspbot_server/ai_engine.py

import requests
import json
from command_parser import CommandParser

class AIEngine:
    def __init__(self, robot_api_url="http://192.168.100.13:5000", 
                 ollama_url="http://localhost:11434"):
        self.parser = CommandParser(ollama_url)
        self.robot_api = robot_api_url
        self.ollama_url = ollama_url
        self.chat_history = []
    
    def process(self, user_input):
        """Xử lý input - Smart AI"""
        print(f"\n{'='*60}")
        print(f"[INPUT] {user_input}")
        print(f"{'='*60}")
        
        # Check: Robot command hay question?
        is_command, confidence = self.parser.is_robot_command(user_input)
        
        print(f"[CHECK] Is robot command? {is_command} ({confidence:.0%})")
        
        if is_command and confidence > 0.5:
            return self.execute_robot_command(user_input)
        else:
            return self.answer_question(user_input)
    
    def execute_robot_command(self, user_input):
        """Execute lệnh robot"""
        print(f"[COMMAND] Executing robot command...")
        
        parsed = self.parser.parse(user_input)
        
        if not parsed['success']:
            response = {
                'type': 'command',
                'message': '❌ Could not understand the command',
                'status': 'error'
            }
        else:
            robot_cmd = self.parser.build_robot_command(parsed)
            response = self.execute_robot_action(robot_cmd)
        
        self.chat_history.append({
            'user': user_input,
            'type': 'ROBOT_COMMAND',
            'response': response.get('message', '')
        })
        
        return {
            'input_type': 'ROBOT_COMMAND',
            'response': response
        }
    
    def execute_robot_action(self, robot_cmd):
        """Execute robot action"""
        action = robot_cmd.get('robot_action', 'unknown')
        params = robot_cmd.get('parameters', {})
        
        print(f"[EXECUTE] {action} {params}")
        
        try:
            if action in ['forward', 'backward', 'left', 'right', 'stop']:
                speed = params.get('speed', 150)
                duration = params.get('duration', 2)
                return self.execute_move(action, speed, duration)
            
            elif action in ['rotate_left', 'rotate_right', 'rotate']:
                count = params.get('count', 1)
                speed = params.get('speed', 120)
                return self.execute_rotate(action, count, speed)
            
            elif action in ['detect_face', 'detect_obstacle']:
                return self.detect(action)
            
            elif action in ['read_distance', 'read_line']:
                return self.read_sensor(action)
            
            elif action.startswith('autonomous_'):
                return self.start_autonomous(action)
            
            else:
                return {
                    'type': 'command',
                    'message': f'❌ Unknown action: {action}',
                    'status': 'error'
                }
        
        except Exception as e:
            print(f"[ERROR] {e}")
            return {
                'type': 'command',
                'message': f'❌ Error: {str(e)}',
                'status': 'error'
            }
    
    def execute_move(self, action, speed=150, duration=2):
        """Move robot"""
        try:
            print(f"[MOVE] {action} (speed: {speed}, duration: {duration}s)")
            response = requests.post(
                f"{self.robot_api}/api/command",
                json={'action': action, 'speed': speed, 'duration': duration},
                timeout=5
            )
            
            if response.status_code == 200:
                return {
                    'type': 'command',
                    'message': f"✅ Robot is {action}",
                    'status': 'success'
                }
            else:
                return {'type': 'command', 'message': '❌ Failed', 'status': 'error'}
        except Exception as e:
            return {'type': 'command', 'message': f'❌ Error: {str(e)}', 'status': 'error'}
    
    def execute_rotate(self, action, count=1, speed=120):
        """Rotate robot"""
        try:
            print(f"[ROTATE] {action} x{count}")
            for i in range(count):
                response = requests.post(
                    f"{self.robot_api}/api/command",
                    json={'action': action, 'speed': speed, 'duration': 3},
                    timeout=5
                )
                if response.status_code != 200:
                    return {'type': 'command', 'message': '❌ Failed', 'status': 'error'}
            
            return {
                'type': 'command',
                'message': f"✅ Robot rotated {count} times",
                'status': 'success'
            }
        except Exception as e:
            return {'type': 'command', 'message': f'❌ Error: {str(e)}', 'status': 'error'}
    
    def detect(self, detection_type):
        """Detect objects"""
        try:
            print(f"[DETECT] {detection_type}")
            endpoint = f'/api/detect/{detection_type.split("_")[1]}'
            response = requests.get(f"{self.robot_api}{endpoint}", timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if detection_type == 'detect_face':
                    count = data.get('count', 0)
                    msg = f"👤 Found {count} face(s)" if count > 0 else "👤 No faces"
                elif detection_type == 'detect_obstacle':
                    detected = data.get('detected', False)
                    msg = "🚫 Obstacle detected" if detected else "✅ No obstacles"
                else:
                    msg = str(data)
                
                return {'type': 'detection', 'message': msg, 'status': 'success'}
            else:
                return {'type': 'detection', 'message': '❌ Detection error', 'status': 'error'}
        except Exception as e:
            return {'type': 'detection', 'message': f'❌ Error: {str(e)}', 'status': 'error'}
    
    def read_sensor(self, sensor_type):
        """Read sensors"""
        try:
            print(f"[SENSOR] {sensor_type}")
            endpoint = f'/api/sensor/{sensor_type.split("_")[1]}'
            response = requests.get(f"{self.robot_api}{endpoint}", timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if sensor_type == 'read_distance':
                    distance = data.get('distance', 0)
                    msg = f"📍 Distance: {distance:.1f}cm"
                elif sensor_type == 'read_line':
                    direction = data.get('direction', 'UNKNOWN')
                    msg = f"📍 Line: {direction}"
                else:
                    msg = str(data)
                
                return {'type': 'sensor', 'message': msg, 'status': 'success'}
            else:
                return {'type': 'sensor', 'message': '❌ Sensor error', 'status': 'error'}
        except Exception as e:
            return {'type': 'sensor', 'message': f'❌ Error: {str(e)}', 'status': 'error'}
    
    def start_autonomous(self, mode):
        """Start autonomous mode"""
        try:
            print(f"[AUTONOMOUS] {mode}")
            mode_name = mode.split('_')[1]
            response = requests.post(
                f"{self.robot_api}/api/autonomous",
                json={'mode': mode_name, 'duration': 30},
                timeout=5
            )
            
            if response.status_code == 200:
                return {
                    'type': 'autonomous',
                    'message': f"🤖 {mode_name.upper()} started",
                    'status': 'success'
                }
            else:
                return {'message': '❌ Failed to start mode'}
        except Exception as e:
            return {'type': 'autonomous', 'message': f'❌ Error: {str(e)}', 'status': 'error'}
    
    def answer_question(self, question):
        """Answer question with neural-chat:7b"""
        try:
            print(f"[QUESTION] {question}")
            
            prompt = f"""You are Raspbot, an intelligent robot.
Answer naturally and briefly in the same language as the question.
Keep it 1-3 sentences.

Question: {question}
Answer:"""
            
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    'model': 'neural-chat:7b',
                    'prompt': prompt,
                    'stream': False,
                    'temperature': 0.7
                },
                timeout=30
            )
            
            if response.status_code == 200:
                answer = response.json().get('response', '').strip()
                self.chat_history.append({
                    'user': question,
                    'type': 'QUESTION',
                    'response': answer
                })
                
                return {
                    'input_type': 'QUESTION',
                    'response': {
                        'type': 'question',
                        'message': answer,
                        'status': 'success'
                    }
                }
            else:
                return self.fallback_answer(question)
        except Exception as e:
            print(f"[ERROR] {e}")
            return self.fallback_answer(question)
    
    def fallback_answer(self, question):
        """Fallback answer"""
        msg = "I'm not sure. Try asking me to move or check sensors! 🤔"
        self.chat_history.append({
            'user': question,
            'type': 'QUESTION',
            'response': msg
        })
        
        return {
            'input_type': 'QUESTION',
            'response': {
                'type': 'question',
                'message': msg,
                'status': 'fallback'
            }
        }
    
    def get_history(self, limit=50):
        return self.chat_history[-limit:]
    
    def clear_history(self):
        self.chat_history = []