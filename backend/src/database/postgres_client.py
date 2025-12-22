"""
Neon Postgres database client implementation for the RAG Chatbot backend.
Handles all interactions with the Neon Postgres database for metadata storage.
"""
import logging
import os
from typing import List, Optional, Dict, Any
from sqlalchemy import create_engine, text, Column, String, Integer, DateTime, Boolean, Text, ARRAY
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID
from src.config.settings import settings


# Create base class for declarative models
Base = declarative_base()


class ChunkReference(Base):
    """Metadata for content chunks stored in the relational database for traceability."""

    __tablename__ = 'chunk_references'

    id = Column(UUID(as_uuid=True), primary_key=True)
    chunk_id = Column(String, nullable=False, unique=True)  # Reference to the content chunk in vector store
    chapter = Column(String, nullable=False)  # Chapter name or identifier
    section = Column(String, nullable=False)  # Section name or identifier
    page_number = Column(Integer)  # Page number in the textbook
    source_file = Column(String, nullable=False)  # Original source file name
    text_preview = Column(Text)  # Short preview of the text content (first 200 chars)
    vector_id = Column(String, nullable=False)  # ID in the vector database (Qdrant)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class ChatSession(Base):
    """Represents a user session for tracking conversations with the RAG chatbot."""

    __tablename__ = 'chat_sessions'

    session_id = Column(UUID(as_uuid=True), primary_key=True)
    user_id = Column(String)  # Identifies the user (for analytics)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    active = Column(Boolean, default=True)  # Whether the session is currently active


class ChatMessage(Base):
    """Represents a message within a chat session, including the retrieved context and response."""

    __tablename__ = 'chat_messages'

    message_id = Column(UUID(as_uuid=True), primary_key=True)
    session_id = Column(UUID(as_uuid=True), nullable=False)  # Reference to the parent session
    role = Column(String, nullable=False)  # "user" or "assistant"
    content = Column(Text, nullable=False)  # The actual message content
    retrieved_chunks = Column(ARRAY(String))  # IDs of chunks used in response
    selected_text_used = Column(Boolean, default=False)  # Whether user-selected text was used as context
    confidence_score = Column(Integer)  # Confidence level of the response (0-100)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class QueryResponseLog(Base):
    """Logs for tracking queries and responses for analysis and debugging."""

    __tablename__ = 'query_response_logs'

    log_id = Column(UUID(as_uuid=True), primary_key=True)
    query = Column(Text, nullable=False)  # The original query from the user
    response = Column(Text)  # The system's response
    retrieved_chunks = Column(ARRAY(String))  # IDs of chunks used in response
    query_type = Column(String, default='general')  # 'general' or 'selected_text'
    response_time = Column(Integer)  # Time in milliseconds to generate response
    confidence_level = Column(Integer)  # Confidence level of the response (0-100)
    refused_answer = Column(Boolean, default=False)  # Whether the system refused to answer
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class PostgresManager:
    """Manages all interactions with the Neon Postgres database."""

    def __init__(self):
        """Initialize the Postgres client with configuration from settings."""
        # Get database URL from settings
        DATABASE_URL = settings.NEON_DATABASE_URL

        # For development, if the Neon URL is not accessible, create an in-memory SQLite database
        # This is only for development/testing purposes
        import os
        if os.getenv("ENVIRONMENT") == "development" and "neon.tech" in DATABASE_URL:
            # Check if we can connect to the Neon database
            import socket
            from urllib.parse import urlparse
            parsed_url = urlparse(DATABASE_URL)
            hostname = parsed_url.hostname

            # Test if we can resolve the hostname
            try:
                socket.gethostbyname(hostname)
            except socket.gaierror:
                # If we can't resolve the hostname, use a local SQLite database for development
                DATABASE_URL = "sqlite:///./test.db"
                print(f"Using local SQLite database for development: {DATABASE_URL}")

        # Create engine
        self.engine = create_engine(
            DATABASE_URL,
            pool_size=20,
            max_overflow=10
        )

        # Create session factory
        self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
        self.logger = logging.getLogger(__name__)

        # Test connection only if in production or if specifically enabled
        import os
        if os.getenv("TESTING", "false").lower() != "true":
            try:
                with self.engine.connect() as conn:
                    conn.execute(text("SELECT 1"))  # Use text() in SQLAlchemy 2.x
                    self.logger.info("Postgres connected successfully")
            except Exception as e:
                self.logger.warning(f"Failed to connect to Postgres (this is expected during testing): {str(e)}")
        else:
            self.logger.info("Skipping Postgres connection during testing")


    def init_db(self):
        """Initialize the database tables."""
        try:
            # Create all tables in the database
            Base.metadata.create_all(bind=self.engine)
            self.logger.info("Database tables created successfully")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize database: {str(e)}")
            return False

    def get_db_session(self):
        """Get a database session."""
        db = self.SessionLocal()
        try:
            yield db
        finally:
            db.close()

    def health_check(self) -> bool:
        """Perform a health check on the Postgres connection."""
        try:
            # Try to connect and execute a simple query
            with self.SessionLocal() as session:
                session.execute(text("SELECT 1"))  # Use text() in SQLAlchemy 2.x
            return True
        except Exception as e:
            self.logger.error(f"Postgres health check failed: {str(e)}")
            return False


# Singleton instance
postgres_client = PostgresManager()