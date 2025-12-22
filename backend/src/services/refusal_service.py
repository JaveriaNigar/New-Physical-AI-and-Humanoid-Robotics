"""
Refusal response implementation for the RAG Chatbot backend.
Handles generating appropriate refusal responses when no relevant context is found.
"""
import logging
from typing import Union

from src.models.chat import ChatQueryRequest, SelectedTextQueryRequest, RefusalResponse, ChatQueryResponse
from src.services.validation_service import validation_service


class RefusalService:
    """Service for handling refusal responses when no relevant context is found."""
    
    def __init__(self):
        """Initialize the refusal service."""
        self.logger = logging.getLogger(__name__)
    
    def create_refusal_response(
        self,
        request: Union[ChatQueryRequest, SelectedTextQueryRequest],
        reason: str
    ) -> RefusalResponse:
        """
        Create an appropriate refusal response for the given request and reason.
        
        Args:
            request: The original request that led to a refusal
            reason: The specific reason for refusal
            
        Returns:
            RefusalResponse with appropriate message and metadata
        """
        # Set appropriate refusal message based on reason
        if reason == "no_context_found":
            message = "I cannot answer this question as it's not covered in the provided textbook content."
        elif reason == "insufficient_context":
            message = "I don't have sufficient context from the textbook to answer your question."
        elif reason == "context_not_applicable":
            message = "The provided context does not contain information relevant to your question."
        elif reason == "selected_text_insufficient":
            message = "The selected text does not contain sufficient information to answer your question."
        else:
            message = "I'm unable to answer your question due to insufficient relevant context."
        
        self.logger.info(f"Creating refusal response for session {request.session_id} with reason: {reason}")
        
        return RefusalResponse(
            message=message,
            reason=reason,
            session_id=request.session_id
        )
    
    def should_refuse_response(
        self,
        query: str,
        retrieved_chunks: list
    ) -> tuple[bool, str]:
        """
        Determine if a response should be refused based on query and retrieved chunks.
        
        Args:
            query: The original query
            retrieved_chunks: List of chunks retrieved from vector store
            
        Returns:
            Tuple of (should_refuse, reason)
        """
        # Check if no chunks were retrieved
        if not retrieved_chunks:
            return True, "no_context_found"
        
        # Check if retrieved chunks have sufficient content
        for chunk in retrieved_chunks:
            text_content = chunk.get('text_content', '')
            if validation_service.validate_response(text_content):
                # Found at least one valid chunk with content
                return False, ""
        
        # If we get here, chunks were retrieved but they don't have sufficient content
        return True, "insufficient_context"


# Singleton instance
refusal_service = RefusalService()