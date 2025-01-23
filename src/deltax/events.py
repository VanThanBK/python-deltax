from typing import Callable, Dict, List
from dataclasses import dataclass

@dataclass
class Event:
    """Event data container"""
    type: str
    data: any = None

class EventManager:
    """Event management system"""
    
    def __init__(self):
        self._handlers: Dict[str, List[Callable]] = {}
        
    def register_handler(self, event_type: str, handler: Callable):
        """Register event handler"""
        if event_type not in self._handlers:
            self._handlers[event_type] = []
        self._handlers[event_type].append(handler)
        
    def emit_event(self, event: Event):
        """Emit event to registered handlers"""
        if event.type in self._handlers:
            for handler in self._handlers[event.type]:
                handler(event) 