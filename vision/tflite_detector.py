#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TFLite Detector - Real Models"""

import os
import time
import numpy as np
import cv2
from utils import logger

class TFLiteDetector:
    """Real TensorFlow Lite Detector"""
    
    def __init__(self, models_dir="/home/heiina/raspbot_intelligent/vision/models"):
        self.models_dir = models_dir
        self.interpreter_person = None
        self.interpreter_face = None
        self.latency_history = []
        
        self._load_models()
    
    def _load_models(self):
        """Load real TFLite models"""
        try:
            import tensorflow as tf
            
            # Load person detection
            person_path = os.path.join(self.models_dir, "person_detection.tflite")
            if os.path.exists(person_path):
                self.interpreter_person = tf.lite.Interpreter(model_path=person_path)
                self.interpreter_person.allocate_tensors()
                size = os.path.getsize(person_path) / (1024*1024)
                logger.info(f"✅ Person model loaded ({size:.1f} MB)")
            else:
                logger.error(f"❌ Person model not found: {person_path}")
            
            # Load face detection
            face_path = os.path.join(self.models_dir, "face_detection.tflite")
            if os.path.exists(face_path):
                self.interpreter_face = tf.lite.Interpreter(model_path=face_path)
                self.interpreter_face.allocate_tensors()
                size = os.path.getsize(face_path) / (1024*1024)
                logger.info(f"✅ Face model loaded ({size:.1f} MB)")
            else:
                logger.error(f"❌ Face model not found: {face_path}")
        
        except ImportError:
            logger.error("❌ TensorFlow not installed!")
            logger.info("Install: pip install tensorflow")
    
    def detect_person(self, frame):
        """Detect person in frame"""
        if self.interpreter_person is None or frame is None:
            return {'detected': False, 'bbox': None, 'confidence': 0.0}
        
        start = time.time()
        
        try:
            # Preprocess
            h, w = frame.shape[:2]
            input_size = 256  # MoveNet uses 256x256
            
            # Resize frame
            resized = cv2.resize(frame, (input_size, input_size))
            resized = resized.astype(np.float32) / 255.0
            
            # Set input
            input_details = self.interpreter_person.get_input_details()
            self.interpreter_person.set_tensor(
                input_details[0]['index'],
                np.expand_dims(resized, axis=0)
            )
            
            # Run inference
            self.interpreter_person.invoke()
            
            # Get output
            output_details = self.interpreter_person.get_output_details()
            keypoints = self.interpreter_person.get_tensor(output_details[0]['index'])
            
            # Parse keypoints (nose, eyes, shoulders, etc)
            detected = False
            confidence = 0.0
            bbox = None
            
            if keypoints is not None and len(keypoints) > 0:
                # Get bounding box from keypoints
                y_coords = keypoints[0, :, 0]
                x_coords = keypoints[0, :, 1]
                scores = keypoints[0, :, 2]
                
                # Filter by confidence
                valid_points = scores > 0.2
                if np.sum(valid_points) > 5:
                    detected = True
                    confidence = float(np.mean(scores[valid_points]))
                    
                    # Calculate bbox
                    y_min, y_max = np.min(y_coords[valid_points]), np.max(y_coords[valid_points])
                    x_min, x_max = np.min(x_coords[valid_points]), np.max(x_coords[valid_points])
                    
                    # Scale back to original image
                    bbox = (
                        int(x_min * w / input_size),
                        int(y_min * h / input_size),
                        int((x_max - x_min) * w / input_size),
                        int((y_max - y_min) * h / input_size)
                    )
            
            latency = (time.time() - start) * 1000
            self.latency_history.append(latency)
            
            return {
                'detected': detected,
                'bbox': bbox,
                'confidence': confidence,
                'latency': latency
            }
        
        except Exception as e:
            logger.error(f"Person detection error: {e}")
            return {'detected': False, 'bbox': None, 'confidence': 0.0}
    
    def detect_face(self, frame):
        """Detect face in frame"""
        if self.interpreter_face is None or frame is None:
            return {'detected': False, 'bbox': None, 'confidence': 0.0}
        
        start = time.time()
        
        try:
            h, w = frame.shape[:2]
            input_size = 128  # BlazeFace uses 128x128
            
            # Preprocess
            resized = cv2.resize(frame, (input_size, input_size))
            resized = resized.astype(np.float32) / 127.5 - 1.0
            
            # Set input
            input_details = self.interpreter_face.get_input_details()
            self.interpreter_face.set_tensor(
                input_details[0]['index'],
                np.expand_dims(resized, axis=0)
            )
            
            # Run inference
            self.interpreter_face.invoke()
            
            # Get output
            output_details = self.interpreter_face.get_output_details()
            detections = self.interpreter_face.get_tensor(output_details[0]['index'])
            
            detected = False
            confidence = 0.0
            bbox = None
            
            if detections is not None and len(detections) > 0:
                # Get top detection
                det = detections[0, 0]
                confidence = float(det[0])
                
                if confidence > 0.5:
                    detected = True
                    
                    # Parse bbox
                    ymin = int(det[1] * h)
                    xmin = int(det[2] * w)
                    ymax = int(det[3] * h)
                    xmax = int(det[4] * w)
                    
                    bbox = (xmin, ymin, xmax - xmin, ymax - ymin)
            
            latency = (time.time() - start) * 1000
            self.latency_history.append(latency)
            
            return {
                'detected': detected,
                'bbox': bbox,
                'confidence': confidence,
                'latency': latency
            }
        
        except Exception as e:
            logger.error(f"Face detection error: {e}")
            return {'detected': False, 'bbox': None, 'confidence': 0.0}
    
    def get_avg_latency(self):
        """Average latency"""
        if not self.latency_history:
            return 0.0
        return np.mean(self.latency_history[-100:])  # Last 100

_detector = None

def get_detector():
    global _detector
    if _detector is None:
        _detector = TFLiteDetector()
    return _detector


