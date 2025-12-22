"""
Configuration management for the RAG Chatbot backend.
Loads settings from environment variables with appropriate defaults.
"""
import os
from typing import Optional
import logging

# Load environment variables from .env file if it exists
from dotenv import load_dotenv
load_dotenv()


class Settings:
    """Application settings loaded from environment variables."""

    # Google AI and Gemini settings
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_MODEL_NAME: str = os.getenv("GEMINI_MODEL_NAME", "models/gemini-2.5-flash")

    # Qdrant vector database settings
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook-content")

    # Neon Postgres settings
    NEON_DATABASE_URL: str = os.getenv("NEON_DATABASE_URL", "")

    # Application settings
    ENVIRONMENT: str = os.getenv("ENVIRONMENT", "development")
    HOST: str = os.getenv("HOST", "0.0.0.0")
    PORT: int = int(os.getenv("PORT", 8000))
    RELOAD: bool = os.getenv("RELOAD", "false").lower() == "true"
    ALLOWED_ORIGINS: list = os.getenv("ALLOWED_ORIGINS", "").split(",")

    # Performance and operational settings
    RETRIEVAL_TIMEOUT: float = float(os.getenv("RETRIEVAL_TIMEOUT", "5.0"))
    MAX_CONTEXT_LENGTH: int = int(os.getenv("MAX_CONTEXT_LENGTH", "3000"))
    MIN_SIMILARITY_THRESHOLD: float = float(os.getenv("MIN_SIMILARITY_THRESHOLD", "0.35"))  # Lowered to improve recall
    MAX_QUESTION_LENGTH: int = int(os.getenv("MAX_QUESTION_LENGTH", "1000"))
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "60"))  # per minute per IP

    # Embedding settings
    EMBEDDING_MODEL_NAME: str = os.getenv("EMBEDDING_MODEL_NAME", "models/text-embedding-004")

    # Security settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "")
    ALGORITHM: str = os.getenv("ALGORITHM", "HS256")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # Logging settings
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")
    LOG_FILE: str = os.getenv("LOG_FILE", "rag_chatbot.log")
    LOG_FORMAT: str = os.getenv("LOG_FORMAT", "%(asctime)s - %(name)s - %(levelname)s - %(message)s")


# Singleton instance of settings
settings = Settings()


def validate_settings():
    """Validate that critical settings are properly configured."""
    errors = []

    if not settings.GEMINI_API_KEY or "your_actual_" in settings.GEMINI_API_KEY:
        if settings.ENVIRONMENT == "development":
            logging.warning("GEMINI_API_KEY is using placeholder value in development mode")
        else:
            errors.append("GEMINI_API_KEY environment variable is required")

    if not settings.QDRANT_API_KEY or "your_actual_" in settings.QDRANT_API_KEY:
        if settings.ENVIRONMENT == "development":
            logging.warning("QDRANT_API_KEY is using placeholder value in development mode")
        else:
            errors.append("QDRANT_API_KEY environment variable is required")

    if not settings.QDRANT_URL or "your_actual_" in settings.QDRANT_URL:
        if settings.ENVIRONMENT == "development":
            logging.warning("QDRANT_URL is using placeholder value in development mode")
        else:
            errors.append("QDRANT_URL environment variable is required")

    if not settings.NEON_DATABASE_URL or "your_actual_" in settings.NEON_DATABASE_URL:
        if settings.ENVIRONMENT == "development":
            logging.warning("NEON_DATABASE_URL is using placeholder value in development mode")
        else:
            errors.append("NEON_DATABASE_URL environment variable is required")

    # For production environments, SECRET_KEY should be set
    if settings.ENVIRONMENT != "development" and not settings.SECRET_KEY:
        errors.append("SECRET_KEY environment variable is required for non-development environments")

    if errors:
        raise ValueError(f"Configuration errors found: {'; '.join(errors)}")


# Validate settings on import
validate_settings()