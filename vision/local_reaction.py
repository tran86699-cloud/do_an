#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TensorFlow Lite Detector
Real-time object detection on Raspbot Pi 5 (< 100ms latency)
Supports: Person detection, Face detection, Gesture recognition
"""

import numpy as np
import cv2
import time
from typing import Dict, List, Optional, Tuple
from pathlib import Path
import sys

# Try to import TFLite (optional - will use mock if not available)
try:
    import tensorflow as tf
    TFLITE_AVAILABLE = True
except ImportError:
    TFLITE_AVAILABLE = False
    print("âš ï¸  TensorFlow not installed. Using mock mode.")

from utils import logger


class TFLiteModel:
    """Single TFLite model wrapper"""
    
    def __init__(self, model_path: str, input_size: Tuple[int, int] = (320, 320)):
        """
        Load and initialize TFLite model
        
        Args:
            model_path: Path to .tflite model file
            input_size: Model input size (width, height)
        """
        self.model_path = model_path
        self.input_size = input_size
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.loaded = False
        
        if TFLITE_AVAILABLE and Path(model_path).exists():
            self._load_model()
        else:
            logger.warning(f"âŒ Model not found: {model_path}")
    
    def _load_model(self):
        """Load TFLite model"""
        try:
            self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
            self.interpreter.allocate_tensors()
            
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            self.loaded = True
            logger.info(f"âœ… Loaded TFLite model: {Path(self.model_path).name}")
        except Exception as e:
            logger.error(f"Failed to load TFLite model: {e}")
            self.loaded = False
    
    def infer(self, image: np.ndarray) -> Dict:
        """
        Run inference on image
        
        Args:
            image: Input image (BGR, any size)
            
        Returns:
            dict: Detection results
        """
        if not self.loaded:
            return {"success": False, "detections": []}
        
        try:
            # Preprocess
            input_tensor = self._preprocess(image)
            
            # Inference
            self.interpreter.set_tensor(self.input_details[0]['index'], input_tensor)
            self.interpreter.invoke()
            
            # Postprocess
            results = self._postprocess(image)
            return {"success": True, "detections": results}
        
        except Exception as e:
            logger.error(f"Inference error: {e}")
            return {"success": False, "detections": []}
    
    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for model"""
        img = cv2.resize(image, self.input_size)
        img = img.astype(np.float32) / 255.0
        return np.expand_dims(img, axis=0)
    
    def _postprocess(self, image: np.ndarray) -> List[Dict]:
        """Postprocess model output"""
        results = []
        
        try:
            # Get output tensors
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
            
            # Parse detections (format depends on model)
            # Typical: [batch, detections, classes] with confidence threshold
            detections = output_data[0]
            
            h, w = image.shape[:2]
            
            for det in detections:
                if len(det) < 5:
                    continue
                    
                confidence = det[4]
                if confidence < 0.5:  # Confidence threshold
                    continue
                
                # Bounding box
                y1, x1, y2, x2 = det[:4]
                x1, y1 = int(x1 * w), int(y1 * h)
                x2, y2 = int(x2 * w), int(y2 * h)
                
                results.append({
                    'bbox': (x1, y1, x2, y2),
                    'confidence': float(confidence),
                    'class': int(det[5]) if len(det) > 5 else 0
                })
        
        except Exception as e:
            logger.debug(f"Postprocess error: {e}")
        
        return results


class TFLiteDetector:
    """
    Main TFLite Detector
    Manages multiple models and runs inference
    """
    
    def __init__(self, models_dir: str = "vision/models"):
        """
        Initialize detector
        
        Args:
            models_dir: Directory containing .tflite files
        """
        self.models_dir = Path(models_dir)
        self.models = {}
        self.latencies = {}
        
        self._load_models()
    
    def _load_models(self):
        """Load all available models"""
        if not self.models_dir.exists():
            logger.warning(f"âŒ Models directory not found: {self.models_dir}")
            logger.info("â„¹ï¸  Creating mock detector (no real detection)")
            return
        
        # Load person detector
        person_model = self.models_dir / "person_detection.tflite"
        if person_model.exists():
            self.models['person'] = TFLiteModel(str(person_model))
            logger.info("âœ… Loaded person detector")
        else:
            logger.warning("âš ï¸  Person detector not found")
        
        # Load face detector
        face_model = self.models_dir / "face_detection.tflite"
        if face_model.exists():
            self.models['face'] = TFLiteModel(str(face_model))
            logger.info("âœ… Loaded face detector")
        else:
            logger.warning("âš ï¸  Face detector not found")
        
        # Load gesture detector (optional)
        gesture_model = self.models_dir / "gesture.tflite"
        if gesture_model.exists():
            self.models['gesture'] = TFLiteModel(str(gesture_model))
            logger.info("âœ… Loaded gesture detector")
    
    def detect_person(self, image: np.ndarray) -> Dict:
        """
        Detect person in image
        
        Args:
            image: Input image (BGR)
            
        Returns:
            dict: {detected: bool, bbox: tuple, confidence: float, latency: float}
        """
        if 'person' not in self.models or not self.models['person'].loaded:
            return self._mock_detect_person(image)
        
        start = time.time()
        result = self.models['person'].infer(image)
        latency = time.time() - start
        
        self.latencies['person'] = latency
        
        if result['detections']:
            det = result['detections'][0]  # Highest confidence
            return {
                'detected': True,
                'bbox': det['bbox'],
                'confidence': det['confidence'],
                'latency': latency
            }
        else:
            return {
                'detected': False,
                'bbox': None,
                'confidence': 0.0,
                'latency': latency
            }
    
    def detect_face(self, image: np.ndarray) -> Dict:
        """
        Detect face in image
        
        Args:
            image: Input image (BGR)
            
        Returns:
            dict: {detected: bool, bbox: tuple, confidence: float, latency: float}
        """
        if 'face' not in self.models or not self.models['face'].loaded:
            return self._mock_detect_face(image)
        
        start = time.time()
        result = self.models['face'].infer(image)
        latency = time.time() - start
        
        self.latencies['face'] = latency
        
        if result['detections']:
            det = result['detections'][0]
            return {
                'detected': True,
                'bbox': det['bbox'],
                'confidence': det['confidence'],
                'latency': latency
            }
        else:
            return {
                'detected': False,
                'bbox': None,
                'confidence': 0.0,
                'latency': latency
            }
    
    def detect_gesture(self, image: np.ndarray) -> Dict:
        """
        Detect gesture in image
        
        Args:
            image: Input image (BGR)
            
        Returns:
            dict: {detected: bool, gesture: str, confidence: float, latency: float}
        """
        if 'gesture' not in self.models or not self.models['gesture'].loaded:
            return {
                'detected': False,
                'gesture': None,
                'confidence': 0.0,
                'latency': 0.0
            }
        
        start = time.time()
        result = self.models['gesture'].infer(image)
        latency = time.time() - start
        
        self.latencies['gesture'] = latency
        
        if result['detections']:
            det = result['detections'][0]
            gestures = ['wave', 'thumbsup', 'peace']
            gesture_name = gestures[det.get('class', 0)] if det.get('class', 0) < len(gestures) else 'unknown'
            
            return {
                'detected': True,
                'gesture': gesture_name,
                'confidence': det['confidence'],
                'latency': latency
            }
        else:
            return {
                'detected': False,
                'gesture': None,
                'confidence': 0.0,
                'latency': latency
            }
    
    def _mock_detect_person(self, image: np.ndarray) -> Dict:
        """Mock person detection for testing"""
        h, w = image.shape[:2]
        return {
            'detected': np.random.random() > 0.5,
            'bbox': (int(w*0.2), int(h*0.2), int(w*0.8), int(h*0.8)) if np.random.random() > 0.5 else None,
            'confidence': np.random.random() * 0.5 + 0.5,
            'latency': np.random.random() * 0.05  # 0-50ms
        }
    
    def _mock_detect_face(self, image: np.ndarray) -> Dict:
        """Mock face detection for testing"""
        h, w = image.shape[:2]
        return {
            'detected': np.random.random() > 0.6,
            'bbox': (int(w*0.3), int(h*0.2), int(w*0.7), int(h*0.6)) if np.random.random() > 0.6 else None,
            'confidence': np.random.random() * 0.4 + 0.6,
            'latency': np.random.random() * 0.03  # 0-30ms
        }
    
    def get_latency(self, model_name: str) -> float:
        """Get last inference latency for model"""
        return self.latencies.get(model_name, 0.0)
    
    def get_average_latency(self) -> float:
        """Get average latency across all detections"""
        if not self.latencies:
            return 0.0
        return sum(self.latencies.values()) / len(self.latencies)
    
    def is_loaded(self) -> bool:
        """Check if any models are loaded"""
        return len(self.models) > 0 and any(m.loaded for m in self.models.values())


# Global detector instance
_detector = None


def get_detector() -> TFLiteDetector:
    """Get or create detector instance"""
    global _detector
    if _detector is None:
        _detector = TFLiteDetector()
    return _detector


# Test
if __name__ == '__main__':
    print("\n" + "="*70)
    print("TFLite Detector Test")
    print("="*70 + "\n")
    
    detector = get_detector()
    print(f"Detector loaded: {detector.is_loaded()}\n")
    
    # Create dummy image
    dummy_img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
    
    print("Testing person detection...")
    person = detector.detect_person(dummy_img)
    print(f"  Result: {person}\n")
    
    print("Testing face detection...")
    face = detector.detect_face(dummy_img)
    print(f"  Result: {face}\n")
    
    print("Testing gesture detection...")
    gesture = detector.detect_gesture(dummy_img)
    print(f"  Result: {gesture}\n")
    
    print(f"Average latency: {detector.get_average_latency()*1000:.1f}ms")
    print("âœ… Test complete!")




