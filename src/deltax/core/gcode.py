from typing import List, Dict, Optional
import re

class GCodeParser:
    """Parser for G-code commands"""
    
    # G-code command patterns
    MOVE_PATTERN = r"G[0-1]\s+X?([-\d.]+)?\s*Y?([-\d.]+)?\s*Z?([-\d.]+)?\s*F?(\d+)?"
    HOME_PATTERN = r"G28(\s+[XYZ])?"
    
    def __init__(self):
        self.position = [0, 0, 0]  # Current position
        self.speed = 1000  # Default speed
        
    def parse_command(self, line: str) -> Dict:
        """Parse single G-code command
        
        Args:
            line: G-code command string
            
        Returns:
            Dict containing command type and parameters
        """
        line = line.strip().upper()
        
        # Skip comments and empty lines
        if not line or line.startswith(';'):
            return None
            
        # Parse move command
        if line.startswith('G0') or line.startswith('G1'):
            return self._parse_move(line)
            
        # Parse home command
        elif line.startswith('G28'):
            return self._parse_home(line)
            
        return None
        
    def _parse_move(self, line: str) -> Dict:
        """Parse G0/G1 move command"""
        match = re.match(self.MOVE_PATTERN, line)
        if not match:
            return None
            
        x, y, z, f = match.groups()
        
        # Update position
        if x is not None:
            self.position[0] = float(x)
        if y is not None:
            self.position[1] = float(y)
        if z is not None:
            self.position[2] = float(z)
            
        # Update speed
        if f is not None:
            self.speed = float(f)
            
        return {
            'type': 'move',
            'x': float(x) if x else None,
            'y': float(y) if y else None,
            'z': float(z) if z else None,
            'speed': float(f) if f else self.speed
        }
        
    def _parse_home(self, line: str) -> Dict:
        """Parse G28 home command"""
        match = re.match(self.HOME_PATTERN, line)
        if not match:
            return None
            
        axes = match.group(1)
        if axes:
            axes = [axis for axis in 'XYZ' if axis in axes]
        else:
            axes = ['X','Y','Z']  # Home all axes
            
        return {
            'type': 'home',
            'axes': axes
        } 