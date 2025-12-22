"""
RAG (Retrieval Augmented Generation) Handler for the Qwen Agent.
Handles the retrieval and generation logic for textbook-based Q&A.
"""
from typing import Optional
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.models.chat import ChatQueryResponse


class RAGHandler:
    """Class to handle RAG logic for textbook-based question answering."""

    def __init__(self):
        """Initialize the RAG handler with required services."""
        self.retrieval_service = retrieval_service
        self.generation_service = generation_service

    def get_answer(self, question: str) -> str:
        """
        Retrieve context and generate an answer for the given question.

        Args:
            question: The question to answer

        Returns:
            Generated answer as a string, or empty string if no context found
        """
        try:
            # Retrieve context from Qdrant
            context = self.retrieval_service.retrieve_relevant_chunks(
                query=question,
                limit=10,
                min_similarity_score=0.5
            )

            if not context:
                return ""  # Will trigger fallback in API endpoint

            # Generate answer from context using Gemini API
            response = self.generation_service.generate_response_from_context(
                question=question,
                context_chunks=context,
                session_id="rag-handler-session"
            )

            return response.response

        except Exception as e:
            # Log the error (in a real implementation, you'd use proper logging)
            print(f"Error in RAGHandler.get_answer: {str(e)}")
            return ""  # Return empty string to trigger fallback in API