"""
Constants for the RAG Chatbot backend.
Contains fixed values that shouldn't change during runtime.
"""
from enum import Enum


# API Response constants
class ResponseStatus(Enum):
    SUCCESS = "success"
    ERROR = "error"
    REFUSED = "refused"


# Content chunking constants
CHUNK_SIZE_TOKENS = 512
CHUNK_OVERLAP_TOKENS = 50
MAX_CONTENT_CHUNKS_PER_RETRIEVAL = 10
DEFAULT_EMBEDDING_DIMENSION = 768


# Database constants
POSTGRES_POOL_SIZE = 20
POSTGRES_POOL_OVERFLOW = 10


# Vector database constants
QDRANT_TIMEOUT_SECONDS = 30
QDRANT_GRPC_ENABLED = True


# Generation service constants
MAX_GENERATION_TOKENS = 1024
TEMPERATURE = 0.1  # Low temperature for factual responses
TOP_P = 0.9
TOP_K = 40


# Validation constants
MIN_QUESTION_CHARACTERS = 5
MAX_QUESTION_CHARACTERS = 1000
MIN_ANSWER_CHARACTERS = 10
MAX_ANSWER_CHARACTERS = 5000


# Logging constants
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
LOG_DEFAULT_LEVEL = "INFO"


# Service health check constants
HEALTH_CHECK_TIMEOUT = 5.0