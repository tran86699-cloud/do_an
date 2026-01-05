#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vision Interface - Abstract layer for vision integration"""

from abc import ABC, abstractmethod
from typing import Dict, Any

class VisionInterface(ABC):
    """Abstract vision interface - any vision module must implement this"""
    
    @abstractmethod
    def get_obstacle_status(self) -> Dict[str, Any]:
        """Get obstacle detection status
        
        Returns:
            {
                'detected': bool,
                'distance': float (cm),
                'confidence': float (0-1),
                'position': str (center/left/right)
            }
        """
        pass
    
    @abstractmethod
    def get_face_status(self) -> Dict[str, Any]:
        """Get face detection status
        
        Returns:
            {
                'detected': bool,
                'count': int,
                'position': str (left/center/right),
                'distance': str (close/medium/far)
            }
        """
        pass
    
    @abstractmethod
    def get_path_status(self) -> str:
        """Get path status
        
        Returns:
            'clear' / 'blocked' / 'uncertain'
        """
        pass
    
    @abstractmethod
    def get_line_status(self) -> Dict[str, Any]:
        """Get line detection status
        
        Returns:
            {
                'detected': bool,
                'position': str (left/center/right),
                'confidence': float
            }
        """
        pass
    
    @abstractmethod
    def capture(self):
        """Capture frame from camera"""
        pass
    
    @abstractmethod
    def release(self):
        """Release camera resources"""
        pass


class MockVisionModule(VisionInterface):
    """Mock implementation - for testing without actual camera"""
    
    def get_obstacle_status(self):
        return {
            'detected': False,
            'distance': 100.0,
            'confidence': 0.8,
            'position': 'center'
        }
    
    def get_face_status(self):
        return {
            'detected': False,
            'count': 0,
            'position': 'none',
            'distance': 'far'
        }
    
    def get_path_status(self):
        return 'clear'
    
    def get_line_status(self):
        return {
            'detected': False,
            'position': 'center',
            'confidence': 0.0
        }
    
    def capture(self):
        return None
    
    def release(self):
        pass


# Global vision instance
_vision_module: VisionInterface = None

def set_vision_module(vision: VisionInterface):
    """Set global vision module"""
    global _vision_module
    _vision_module = vision

def get_vision_module() -> VisionInterface:
    """Get global vision module (defaults to Mock)"""
    global _vision_module
    if _vision_module is None:
        _vision_module = MockVisionModule()
    return _vision_module
