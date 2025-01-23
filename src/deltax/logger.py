import logging
from pathlib import Path
from typing import Optional

class Logger:
    """Centralized logging system for DeltaX"""
    
    def __init__(self, name: str, log_file: Optional[str] = None):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        
        # Console handler
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        console.setFormatter(formatter)
        self.logger.addHandler(console)
        
        # File handler if specified
        if log_file:
            log_path = Path(log_file)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(logging.DEBUG)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
            
    def debug(self, msg: str, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)
        
    def info(self, msg: str, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)
        
    def warning(self, msg: str, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)
        
    def error(self, msg: str, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)
        
    def critical(self, msg: str, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs) 