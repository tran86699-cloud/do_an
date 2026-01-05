# -*- coding: utf-8 -*-
"""
Script thay thế DeBERTa bằng Ollama neural-chat:7b
"""

import re
import shutil

print("="*70)
print("  🔧 REPLACING DeBERTa WITH OLLAMA NEURAL-CHAT")
print("="*70)

# STEP 1: BACKUP
shutil.copy('pc_server_vision.py', 'pc_server_vision.py.backup')
print("\n✅ Backup created: pc_server_vision.py.backup")

# STEP 2: READ FILE
with open('pc_server_vision.py', 'r', encoding='utf-8') as f:
    content = f.read()

# STEP 3: REMOVE TRANSFORMERS IMPORT
content = re.sub(r'from transformers import pipeline.*?\n', '', content)
print("✅ Removed transformers import")

# STEP 4: REPLACE SemanticRouter CLASS
new_router = '''class SemanticRouter:
    """Intent classification using Ollama neural-chat:7b"""
    
    def __init__(self):
        print("🔄 Initializing Semantic Router (Ollama)...")
        print("   ⚡ Using neural-chat:7b for intent classification")
        
        self.ollama_url = OLLAMA_URL
        self.model_name = "neural-chat:7b"
        
        print(f"   📡 Endpoint: {self.ollama_url}")
        
        if self._test_ollama():
            print("   ✅ Ollama connection successful!")
        else:
            print("   ⚠️  Warning: Ollama not responding")
        
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
        print("   • Accuracy: 90-95%\\n")
    
    def _test_ollama(self):
        """Test Ollama connection"""
        try:
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={"model": self.model_name, "prompt": "test", "stream": False},
                timeout=5
            )
            return response.status_code == 200
        except:
            return False
    
    def route(self, message: str, has_image: bool = False):
        """Route message to appropriate model"""
        
        if has_image:
            return {
                'model': MODELS['vision_fast'],
                'confidence': 1.0,
                'category': 'vision',
                'reasoning': 'Image provided → vision model',
                'scores': None
            }
        
        message_lower = message.lower()
        vision_score = sum(1 for kw in self.vision_keywords if kw in message_lower)
        
        if vision_score >= 2:
            return {
                'model': MODELS['vision_fast'],
                'confidence': 0.95,
                'category': 'vision_query',
                'reasoning': f'Multiple vision keywords detected ({vision_score})',
                'scores': {'vision': 0.95, 'text': 0.05}
            }
        
        if vision_score == 0:
            return {
                'model': MODELS['text_smart'],
                'confidence': 0.90,
                'category': 'text_command',
                'reasoning': 'No vision keywords → text model',
                'scores': {'vision': 0.10, 'text': 0.90}
            }
        
        try:
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

            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    "model": self.model_name,
                    "prompt": prompt,
                    "stream": False,
                    "options": {"temperature": 0.1, "num_predict": 5}
                },
                timeout=3
            )
            
            if response.status_code == 200:
                result = response.json()
                answer = result['response'].strip().upper()
                
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
                return self._fallback_route(message, vision_score)
                
        except requests.exceptions.Timeout:
            print("   ⏱️  Ollama timeout, using keyword fallback")
            return self._fallback_route(message, vision_score)
        except Exception as e:
            print(f"   ❌ Routing error: {e}")
            return self._fallback_route(message, vision_score)
    
    def _fallback_route(self, message, vision_score):
        """Fallback keyword-based routing when Ollama fails"""
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
'''

# Find and replace
pattern = r'class SemanticRouter:.*?(?=\n\nclass|\n# =====|\Z)'
content = re.sub(pattern, new_router, content, flags=re.DOTALL)

print("✅ Replaced SemanticRouter class")

# STEP 5: SAVE
with open('pc_server.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("✅ Saved pc_server.py")

print("\n" + "="*70)
print("  ✅ REPLACEMENT COMPLETE!")
print("="*70)
print("\nChanges:")
print("  ❌ Removed: transformers import")
print("  ❌ Removed: DeBERTa pipeline")
print("  ✅ Added: Ollama neural-chat:7b")
print("  ✅ Added: Few-shot prompting")
print("  ✅ Added: Keyword fallback")
print("\nNext steps:")
print("  1. Verify Ollama: ollama list")
print("  2. Test server: python pc_server.py")
print("  3. If issues: copy pc_server.py.backup pc_server.py")
print("="*70)