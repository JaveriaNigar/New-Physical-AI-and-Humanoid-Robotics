"""
Comprehensive logging system for the RAG Chatbot backend.
Provides structured logging for debugging, monitoring, and analytics.
"""
import logging
import logging.handlers
import os
from datetime import datetime
from typing import Optional
from src.config.settings import settings


class LoggingManager:
    """Manages logging configuration and provides logging utilities."""

    def __init__(self):
        """Initialize the logging manager with configuration from settings."""
        self.logger = logging.getLogger("rag_chatbot")
        self.logger.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))
        
        # Prevent adding multiple handlers if already configured
        if not self.logger.handlers:
            self._setup_handlers()
        
    def _setup_handlers(self):
        """Set up logging handlers based on configuration."""
        # Create formatter
        formatter = logging.Formatter(
            settings.LOG_FORMAT,
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)
        
        # File handler with rotation
        if settings.LOG_FILE:
            file_handler = logging.handlers.RotatingFileHandler(
                settings.LOG_FILE,
                maxBytes=10*1024*1024,  # 10MB
                backupCount=5
            )
            file_handler.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
    
    def get_logger(self) -> logging.Logger:
        """Get the configured logger instance."""
        return self.logger


# Initialize the logging manager
logging_manager = LoggingManager()
logger = logging_manager.get_logger()


def log_api_request(
    endpoint: str,
    method: str,
    user_id: Optional[str],
    session_id: str,
    query: str,
    response_status: int,
    response_time: float
):
    """Log API requests with relevant metadata."""
    logger.info(
        f"API Request: {method} {endpoint} | "
        f"User: {user_id or 'unknown'} | "
        f"Session: {session_id} | "
        f"Status: {response_status} | "
        f"Response Time: {response_time:.3f}s | "
        f"Query: {query[:100]}..."
    )


def log_retrieval_event(
    query: str,
    session_id: str,
    num_chunks_found: int,
    retrieval_time: float,
    similarity_threshold: float
):
    """Log retrieval events with performance metrics."""
    logger.info(
        f"Retrieval Event | "
        f"Session: {session_id} | "
        f"Chunks Found: {num_chunks_found} | "
        f"Retrieval Time: {retrieval_time:.3f}s | "
        f"Threshold: {similarity_threshold} | "
        f"Query: {query[:100]}..."
    )


def log_generation_event(
    query: str,
    session_id: str,
    response_length: int,
    generation_time: float,
    confidence_score: float
):
    """Log generation events with quality metrics."""
    logger.info(
        f"Generation Event | "
        f"Session: {session_id} | "
        f"Response Length: {response_length} chars | "
        f"Generation Time: {generation_time:.3f}s | "
        f"Confidence: {confidence_score:.2f} | "
        f"Query: {query[:100]}..."
    )


def log_error(
    error_type: str,
    error_message: str,
    session_id: Optional[str] = None,
    query: Optional[str] = None
):
    """Log errors with context for debugging."""
    context = []
    if session_id:
        context.append(f"Session: {session_id}")
    if query:
        context.append(f"Query: {query[:100]}...")
    
    context_str = " | ".join(context)
    if context_str:
        context_str = f" | {context_str}"
    
    logger.error(
        f"Error: {error_type} | "
        f"Message: {error_message}"
        f"{context_str}"
    )


def log_security_event(event_type: str, details: str, user_id: Optional[str] = None):
    """Log security-related events."""
    user_info = f" | User: {user_id}" if user_id else ""
    logger.warning(f"Security Event: {event_type} | Details: {details}{user_info}")


def log_validation_event(
    validation_type: str,
    is_valid: bool,
    details: str,
    session_id: Optional[str] = None
):
    """Log validation events."""
    status = "PASS" if is_valid else "FAIL"
    session_info = f" | Session: {session_id}" if session_id else ""
    logger.info(f"Validation: {validation_type} | Status: {status} | Details: {details}{session_info}")