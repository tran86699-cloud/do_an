# ~/raspbot_server/smart_classifier.py

import requests

class SmartClassifier:
    """
    Phân loại loại input trước
    Giúp robot hiểu: Đây là lệnh? Câu hỏi? Chat thường?
    """
    
    ROBOT_COMMANDS = [
        'MOVE_FORWARD', 'MOVE_BACKWARD', 'TURN_LEFT', 'TURN_RIGHT',
        'ROTATE_LEFT', 'ROTATE_RIGHT', 'STOP',
        'GET_DISTANCE', 'GET_LINE', 'DETECT_FACE', 'DETECT_OBSTACLE',
        'AUTONOMOUS_PATROL', 'AUTONOMOUS_AVOID', 'AUTONOMOUS_LINE'
    ]
    
    INPUT_TYPES = {
        'ROBOT_COMMAND': 'Đây là lệnh điều khiển robot',
        'QUESTION': 'Đây là câu hỏi',
        'GREETING': 'Đây là lời chào',
        'CONVERSATION': 'Đây là cuộc trò chuyện thường',
        'HELP': 'Người yêu cầu trợ giúp'
    }
    
    def __init__(self, ollama_url="http://localhost:11434"):
        self.ollama_url = ollama_url
    
    def classify(self, user_input):
        """
        Phân loại input bằng LLM
        Returns: {input_type, confidence, description}
        """
        print(f"\n[CLASSIFY] Input: {user_input}")
        
        # Gửi tới Ollama để phân loại
        prompt = f"""Classify this user input into ONE of these categories:
1. ROBOT_COMMAND - A command to control the robot (move, turn, detect, etc.)
2. QUESTION - A question asking for information
3. GREETING - A greeting or salutation
4. HELP - A request for help or usage information
5. CONVERSATION - Regular conversation or chat

Input: "{user_input}"

Return ONLY the category name and a confidence score 0-100.
Example: ROBOT_COMMAND 95
Example: QUESTION 80
Example: CONVERSATION 75"""
        
        try:
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    'model': 'orca-mini:3b',
                    'prompt': prompt,
                    'stream': False,
                    'temperature': 0.1
                },
                timeout=15
            )
            
            if response.status_code == 200:
                answer = response.json().get('response', '').strip()
                print(f"[CLASSIFY] LLM response: {answer}")
                
                # Parse response
                parts = answer.split()
                if len(parts) >= 2:
                    input_type = parts[0].upper()
                    try:
                        confidence = int(parts[1]) / 100.0
                    except:
                        confidence = 0.7
                    
                    if input_type in self.INPUT_TYPES:
                        print(f"[CLASSIFY] Type: {input_type} ({confidence:.0%})")
                        return {
                            'input_type': input_type,
                            'confidence': round(confidence, 2),
                            'description': self.INPUT_TYPES[input_type]
                        }
            
            # Fallback
            return self.fallback_classify(user_input)
        
        except Exception as e:
            print(f"[ERROR] Classification error: {e}")
            return self.fallback_classify(user_input)
    
    def fallback_classify(self, user_input):
        """
        Fallback classification nếu Ollama error
        """
        print(f"[CLASSIFY] Using fallback...")
        
        user_lower = user_input.lower()
        
        # Quick keyword check
        if any(word in user_lower for word in ['move', 'go', 'turn', 'forward', 'backward', 'distance', 'detect', 'face', 'obstacle', 'patrol']):
            return {
                'input_type': 'ROBOT_COMMAND',
                'confidence': 0.8,
                'description': self.INPUT_TYPES['ROBOT_COMMAND']
            }
        
        if any(word in user_lower for word in ['what', 'how', 'why', 'when', 'where', 'which', '?', 'gì', 'như thế nào']):
            return {
                'input_type': 'QUESTION',
                'confidence': 0.75,
                'description': self.INPUT_TYPES['QUESTION']
            }
        
        if any(word in user_lower for word in ['hello', 'hi', 'hey', 'xin chào', 'chào', 'mình là']):
            return {
                'input_type': 'GREETING',
                'confidence': 0.7,
                'description': self.INPUT_TYPES['GREETING']
            }
        
        if any(word in user_lower for word in ['help', 'support', 'guide', 'hướng dẫn', 'giúp']):
            return {
                'input_type': 'HELP',
                'confidence': 0.7,
                'description': self.INPUT_TYPES['HELP']
            }
        
        # Default
        return {
            'input_type': 'CONVERSATION',
            'confidence': 0.5,
            'description': self.INPUT_TYPES['CONVERSATION']
        }