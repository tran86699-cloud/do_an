# ~/raspbot_server/command_parser.py

import requests
import json
import re

class CommandParser:
    """
    Parse bất kỳ lệnh robot nào (đã biết hoặc mới)
    Không cần classifier - hoạt động độc lập
    """
    
    def __init__(self, ollama_url="http://localhost:11434"):
        self.ollama_url = ollama_url
        
        # Known command patterns (cache để nhanh)
        self.known_patterns = {
            'MOVE_FORWARD': ['forward', 'ahead', 'go', 'tiến', 'đi tới', 'advance'],
            'MOVE_BACKWARD': ['back', 'backward', 'lùi', 'reverse', 'go back'],
            'TURN_LEFT': ['left', 'trái', 'turn left', 'quay trái'],
            'TURN_RIGHT': ['right', 'phải', 'turn right', 'quay phải'],
            'ROTATE': ['rotate', 'spin', 'xoay', 'quay vòng'],
            'STOP': ['stop', 'dừng', 'halt', 'pause'],
            'DETECT_FACE': ['face', 'khuôn mặt', 'người', 'person', 'detect face'],
            'DETECT_OBSTACLE': ['obstacle', 'chướng ngại', 'detect obstacle'],
            'GET_DISTANCE': ['distance', 'far', 'khoảng cách', 'bao xa', 'ultrasonic'],
            'GET_LINE': ['line', 'line tracking', 'line status'],
            'PATROL': ['patrol', 'tuần tra'],
            'AVOID': ['avoid', 'obstacle avoidance', 'tránh'],
            'FOLLOW_LINE': ['follow line', 'line follow', 'theo dõi đường'],
        }
    
    def is_robot_command(self, user_input):
        """
        Check xem có phải lệnh robot không
        Returns: True/False, confidence
        """
        # Pattern matching
        result = self.parse_pattern(user_input)
        if result and result['confidence'] > 0.6:
            return True, result['confidence']
        
        # Keyword check
        robot_keywords = [
            'move', 'forward', 'backward', 'turn', 'rotate', 'stop',
            'detect', 'distance', 'line', 'patrol', 'avoid',
            'go', 'tiến', 'lùi', 'dừng', 'tránh', 'đi'
        ]
        
        user_lower = user_input.lower()
        found = sum(1 for keyword in robot_keywords if keyword in user_lower)
        
        if found > 0:
            confidence = min(found / len(robot_keywords), 0.8)
            return True, confidence
        
        return False, 0.2
    
    def parse(self, user_input):
        """
        Parse lệnh - 2 cách:
        1. Pattern matching (nhanh)
        2. LLM parsing (smart, cho lệnh mới)
        """
        print(f"\n[PARSE] Input: {user_input}")
        
        # Cách 1: Pattern matching (nhanh)
        print(f"[PARSE] Trying pattern matching...")
        result = self.parse_pattern(user_input)
        
        if result and result['confidence'] > 0.7:
            print(f"[PARSE] Pattern matched: {result['action']}")
            return result
        
        # Cách 2: LLM parsing (smart)
        print(f"[PARSE] Pattern not confident, trying LLM...")
        result = self.parse_with_llm(user_input)
        
        return result
    
    def parse_pattern(self, user_input):
        """
        Pattern matching - nhanh
        """
        user_lower = user_input.lower()
        best_action = None
        best_score = 0
        
        for action, keywords in self.known_patterns.items():
            for keyword in keywords:
                if keyword in user_lower:
                    score = len(keyword) / len(user_lower)
                    if score > best_score:
                        best_score = score
                        best_action = action
        
        if best_action and best_score > 0.3:
            # Extract parameters
            params = self.extract_parameters(user_input)
            
            return {
                'success': True,
                'action': best_action,
                'parameters': params,
                'confidence': min(best_score, 0.9),
                'method': 'pattern'
            }
        
        return None
    
    def parse_with_llm(self, user_input):
        """
        LLM parsing - smart, cho lệnh mới
        """
        try:
            prompt = f"""Analyze this robot command and extract the action and parameters.

Possible actions:
- move_forward / move_backward (move robot)
- turn_left / turn_right / rotate (turn robot)
- stop (stop moving)
- detect_face / detect_obstacle (detect objects)
- read_distance / read_line (read sensors)
- patrol / avoid / follow_line (autonomous modes)

Return ONLY a JSON object with:
- action: (the action from list above)
- speed: (0-255, optional, default 150)
- duration: (seconds, optional, default 2)
- distance: (in cm, optional)
- count: (repeat count, optional)
- any_other_param: value

Example 1: "spin around 5 times fast"
{{"action": "rotate", "count": 5, "speed": 200}}

Example 2: "move forward slowly for 10 seconds"
{{"action": "move_forward", "speed": 80, "duration": 10}}

Command: "{user_input}"

Return ONLY valid JSON:"""
            
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    'model': 'neural-chat:7b',
                    'prompt': prompt,
                    'stream': False,
                    'temperature': 0.1
                },
                timeout=15
            )
            
            if response.status_code == 200:
                answer = response.json().get('response', '').strip()
                print(f"[PARSE] LLM response: {answer}")
                
                try:
                    parsed = json.loads(answer)
                    
                    action = parsed.get('action', 'unknown')
                    parameters = {k: v for k, v in parsed.items() 
                                if k != 'action'}
                    
                    print(f"[PARSE] Parsed: {action} {parameters}")
                    
                    return {
                        'success': True,
                        'action': action,
                        'parameters': parameters,
                        'confidence': 0.85,
                        'method': 'llm'
                    }
                
                except json.JSONDecodeError:
                    print(f"[PARSE] JSON parse failed")
                    return self.parse_fallback(user_input)
            
            return self.parse_fallback(user_input)
        
        except Exception as e:
            print(f"[ERROR] {e}")
            return self.parse_fallback(user_input)
    
    def extract_parameters(self, user_input):
        """
        Extract parameters từ text
        speed, duration, distance, count, etc.
        """
        params = {}
        user_lower = user_input.lower()
        
        # Speed (0-255)
        speed_match = re.search(r'speed\s+(\d+)', user_lower)
        if speed_match:
            params['speed'] = int(speed_match.group(1))
        
        if 'slow' in user_lower or 'từ từ' in user_lower:
            params['speed'] = 80
        elif 'fast' in user_lower or 'nhanh' in user_lower:
            params['speed'] = 200
        
        # Duration/Time
        time_match = re.search(r'(\d+)\s*(?:second|sec|s|giây)', user_lower)
        if time_match:
            params['duration'] = int(time_match.group(1))
        
        # Distance
        dist_match = re.search(r'(\d+)\s*(?:cm|m|meter|centimet)', user_lower)
        if dist_match:
            params['distance'] = int(dist_match.group(1))
        
        # Count/Repeat
        count_match = re.search(r'(\d+)\s*times?', user_lower)
        if count_match:
            params['count'] = int(count_match.group(1))
        
        return params
    
    def parse_fallback(self, user_input):
        """
        Fallback nếu tất cả fail
        """
        print(f"[PARSE] Using fallback")
        
        return {
            'success': False,
            'action': 'unknown',
            'parameters': {},
            'confidence': 0.3,
            'method': 'fallback'
        }
    
    def build_robot_command(self, parsed):
        """
        Convert parsed thành robot API command
        """
        print(f"[BUILD] Building robot command: {parsed['action']}")
        
        action = parsed['action']
        params = parsed['parameters']
        
        speed = params.get('speed', 150)
        duration = params.get('duration', 2)
        count = params.get('count', 1)
        
        # Map to robot actions
        action_map = {
            'move_forward': {'robot_action': 'forward', 'speed': speed, 'duration': duration},
            'move_backward': {'robot_action': 'backward', 'speed': speed, 'duration': duration},
            'turn_left': {'robot_action': 'left', 'speed': speed, 'duration': duration},
            'turn_right': {'robot_action': 'right', 'speed': speed, 'duration': duration},
            'rotate': {'robot_action': 'rotate_right', 'speed': speed, 'count': count},
            'stop': {'robot_action': 'stop'},
            'detect_face': {'robot_action': 'detect_face'},
            'detect_obstacle': {'robot_action': 'detect_obstacle'},
            'read_distance': {'robot_action': 'read_distance'},
            'read_line': {'robot_action': 'read_line'},
            'patrol': {'robot_action': 'autonomous_patrol'},
            'avoid': {'robot_action': 'autonomous_avoid'},
            'follow_line': {'robot_action': 'autonomous_line'},
        }
        
        if action in action_map:
            cmd = action_map[action].copy()
            cmd['parameters'] = params
            cmd['confidence'] = parsed['confidence']
            return cmd
        
        return {
            'robot_action': 'unknown',
            'parameters': params,
            'confidence': parsed['confidence']
        }