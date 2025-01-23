from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from .logger import Logger
from .errors import PluginError

class Plugin(ABC):
    """Base class for DeltaX plugins"""
    
    def __init__(self):
        self.name = self.__class__.__name__
        self.logger = Logger(self.name)
        self.config: Dict[str, Any] = {}
        
    @abstractmethod
    def initialize(self) -> None:
        """Initialize the plugin"""
        pass
        
    @abstractmethod
    def cleanup(self) -> None:
        """Cleanup plugin resources"""
        pass
        
    def configure(self, config: Dict[str, Any]) -> None:
        """Configure the plugin"""
        self.config = config

class PluginManager:
    """Manage DeltaX plugins"""
    
    def __init__(self):
        self.plugins: Dict[str, Plugin] = {}
        self.logger = Logger(__name__)
        
    def register_plugin(self, plugin: Plugin) -> None:
        """Register a new plugin"""
        if plugin.name in self.plugins:
            raise PluginError(f"Plugin {plugin.name} already registered")
            
        try:
            plugin.initialize()
            self.plugins[plugin.name] = plugin
            self.logger.info(f"Plugin {plugin.name} registered successfully")
        except Exception as e:
            raise PluginError(f"Failed to initialize plugin {plugin.name}: {e}")
            
    def unregister_plugin(self, name: str) -> None:
        """Unregister a plugin"""
        if name in self.plugins:
            try:
                self.plugins[name].cleanup()
                del self.plugins[name]
                self.logger.info(f"Plugin {name} unregistered successfully")
            except Exception as e:
                raise PluginError(f"Failed to cleanup plugin {name}: {e}")
                
    def get_plugin(self, name: str) -> Optional[Plugin]:
        """Get a registered plugin by name"""
        return self.plugins.get(name)
        
    def cleanup(self) -> None:
        """Cleanup all plugins"""
        for name in list(self.plugins.keys()):
            self.unregister_plugin(name) 