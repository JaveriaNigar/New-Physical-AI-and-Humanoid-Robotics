"""
Logging functionality for the RAG Chatbot backend.
Provides logging for retrieved chunk IDs for traceability and debugging.
"""
import logging
from typing import List, Dict, Any
from datetime import datetime

from src.database.postgres_client import postgres_client
from src.config.settings import settings


class LoggingService:
    """Service for logging retrieved chunk IDs for traceability and debugging."""
    
    def __init__(self):
        """Initialize the logging service."""
        self.logger = logging.getLogger(__name__)
        self.postgres_client = postgres_client
    
    def log_retrieved_chunks(
        self,
        query_id: str,
        query_text: str,
        retrieved_chunk_ids: List[str],
        session_id: str = None,
        user_id: str = None
    ) -> bool:
        """
        Log the retrieved chunk IDs for traceability.
        
        Args:
            query_id: Unique identifier for the query
            query_text: The original query text
            retrieved_chunk_ids: List of chunk IDs that were retrieved
            session_id: Session identifier
            user_id: User identifier (optional)
            
        Returns:
            True if logging was successful, False otherwise
        """
        try:
            timestamp = datetime.utcnow()
            
            self.logger.info(
                f"Query ID: {query_id}, Session: {session_id}, "
                f"User: {user_id}, Retrieved chunks: {len(retrieved_chunk_ids)}, "
                f"Query: '{query_text[:100]}...'"
            )
            
            # In a complete implementation, we would store this data in the database
            # For now, we'll just log to the application logs
            for i, chunk_id in enumerate(retrieved_chunk_ids):
                self.logger.debug(f"  Chunk {i+1}: {chunk_id}")
            
            # In a real implementation, we would store in the QueryResponseLog table
            # with self.postgres_client.get_db_session() as db:
            #     log_entry = QueryResponseLog(
            #         log_id=uuid4(),
            #         query=query_text,
            #         retrieved_chunks=retrieved_chunk_ids,
            #         query_type="general",
            #         created_at=timestamp,
            #         # ... other fields
            #     )
            #     db.add(log_entry)
            #     db.commit()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error logging retrieved chunks: {str(e)}")
            return False
    
    def log_query_response(
        self,
        query: str,
        response: str,
        retrieved_chunk_ids: List[str],
        session_id: str,
        confidence_score: float = None,
        response_time_ms: int = None
    ) -> bool:
        """
        Log a complete query-response interaction for monitoring and analysis.
        
        Args:
            query: The original query
            response: The generated response
            retrieved_chunk_ids: List of chunk IDs used to generate the response
            session_id: Session identifier
            confidence_score: Confidence score of the response
            response_time_ms: Time taken to generate the response in milliseconds
            
        Returns:
            True if logging was successful, False otherwise
        """
        try:
            timestamp = datetime.utcnow()
            
            log_msg = (
                f"Session: {session_id}, "
                f"Query: '{query[:100]}...', "
                f"Response: '{response[:100]}...', "
                f"Chunks used: {len(retrieved_chunk_ids)}, "
                f"Confidence: {confidence_score}, "
                f"Response time: {response_time_ms}ms"
            )
            
            self.logger.info(log_msg)
            
            # Log individual chunk IDs for traceability
            if retrieved_chunk_ids:
                self.logger.debug(f"Chunks used for session {session_id}: {retrieved_chunk_ids}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error logging query-response: {str(e)}")
            return False
    
    def log_refusal_event(
        self,
        query: str,
        session_id: str,
        reason: str,
        retrieved_chunk_ids: List[str] = None
    ) -> bool:
        """
        Log when the system refuses to answer a query.
        
        Args:
            query: The original query that was refused
            session_id: Session identifier
            reason: Reason for refusal
            retrieved_chunk_ids: List of chunk IDs that were retrieved (if any)
            
        Returns:
            True if logging was successful, False otherwise
        """
        try:
            timestamp = datetime.utcnow()
            
            log_msg = (
                f"REFUSAL - Session: {session_id}, "
                f"Query: '{query[:100]}...', "
                f"Reason: {reason}, "
                f"Chunks retrieved: {len(retrieved_chunk_ids) if retrieved_chunk_ids else 0}"
            )
            
            self.logger.warning(log_msg)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error logging refusal event: {str(e)}")
            return False


# Singleton instance
logging_service = LoggingService()