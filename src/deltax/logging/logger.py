import logging
import logging.handlers
import os
from pathlib import Path
from typing import Optional
from datetime import datetime

class LogFormatter(logging.Formatter):
    """Custom log formatter with colors"""
    
    COLORS = {
        'DEBUG': '\033[94m',     # Blue
        'INFO': '\033[92m',      # Green
        'WARNING': '\033[93m',   # Yellow
        'ERROR': '\033[91m',     # Red
        'CRITICAL': '\033[95m',  # Purple
        'RESET': '\033[0m'       # Reset
    }
    
    def format(self, record):
        # Add color to level name
        if record.levelname in self.COLORS:
            record.levelname = (f"{self.COLORS[record.levelname]}"
                              f"{record.levelname}"
                              f"{self.COLORS['RESET']}")
        return super().format(record)

class DeltaXLogger:
    """Enhanced logging system for DeltaX"""
    
    def __init__(self, name: str, log_dir: Optional[str] = None):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        
        # Setup log directory
        if log_dir is None:
            log_dir = os.path.expanduser("~/.deltax/logs")
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Setup handlers
        self._setup_console_handler()
        self._setup_file_handlers()
        
    def _setup_console_handler(self):
        """Setup colored console output"""
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        
        formatter = LogFormatter(
            '%(levelname)s - %(name)s - %(message)s'
        )
        console.setFormatter(formatter)
        self.logger.addHandler(console)
        
    def _setup_file_handlers(self):
        """Setup file handlers for different log levels"""
        # Daily rotating file handler
        daily_file = self.log_dir / "deltax.log"
        daily_handler = logging.handlers.TimedRotatingFileHandler(
            daily_file,
            when='midnight',
            interval=1,
            backupCount=30
        )
        daily_handler.setLevel(logging.DEBUG)
        daily_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        self.logger.addHandler(daily_handler)
        
        # Error log file
        error_file = self.log_dir / "errors.log"
        error_handler = logging.FileHandler(error_file)
        error_handler.setLevel(logging.ERROR)
        error_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s\n'
            'Exception:\n%(exc_info)s\n'
        ))
        self.logger.addHandler(error_handler)
        
    def start_session(self):
        """Start new logging session"""
        session_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_file = self.log_dir / f"session_{session_time}.log"
        
        handler = logging.FileHandler(session_file)
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        self.logger.addHandler(handler)
        
        self.logger.info(f"Started new session: {session_time}")
        return handler
        
    def end_session(self, handler):
        """End logging session"""
        self.logger.info("Ending session")
        self.logger.removeHandler(handler)
        handler.close()
        
    def debug(self, msg: str, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)
        
    def info(self, msg: str, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)
        
    def warning(self, msg: str, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)
        
    def error(self, msg: str, *args, **kwargs):
        self.logger.error(msg, exc_info=True, *args, **kwargs)
        
    def critical(self, msg: str, *args, **kwargs):
        self.logger.critical(msg, exc_info=True, *args, **kwargs) 