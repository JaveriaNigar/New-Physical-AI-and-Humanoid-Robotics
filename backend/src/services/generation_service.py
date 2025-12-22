"""
Generation service for the RAG Chatbot backend.
Handles generating responses using the Gemini API based on retrieved context.
"""
import logging
from typing import List, Dict, Any, Optional
import google.generativeai as genai
from src.config.settings import settings
from src.config.constants import (
    MAX_GENERATION_TOKENS, TEMPERATURE, TOP_P, TOP_K,
    MIN_QUESTION_CHARACTERS
)
from src.services.validation_service import validation_service
from src.models.chat import ChatQueryResponse, RefusalResponse, ChunkMetadata


class GenerationService:
    """Service for generating responses using the Gemini API based on context."""

    def __init__(self):
        """Initialize the generation service with configuration from settings."""
        # Configure the Google Generative AI client
        genai.configure(api_key=settings.GEMINI_API_KEY)

        # Initialize the generative model
        self.model = genai.GenerativeModel(settings.GEMINI_MODEL_NAME)
        self.logger = logging.getLogger(__name__)

    def generate_response_from_context(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]],
        session_id: str
    ) -> ChatQueryResponse:
        """
        Generate a response based on the question and provided context chunks.

        Args:
            question: The question to answer
            context_chunks: List of context chunks to use for answering
            session_id: Session identifier

        Returns:
            ChatQueryResponse with the answer and metadata
        """
        if not context_chunks:
            # No context available, return a response indicating this
            refusal_msg = "I cannot answer this question as it's not covered in the provided textbook content."
            return ChatQueryResponse(
                response=refusal_msg,
                confidence=0.1,  # Low confidence for refusal responses
                retrieved_chunks=[],
                sources=[],
                session_id=session_id,
                query_type="general"
            )

        try:
            # Construct the prompt for the LLM
            context_text = "\n\n".join([chunk['text_content'] for chunk in context_chunks])

            # Create a detailed prompt that instructs the model to only use the provided context
            prompt = f"""
            You are an AI assistant for a textbook. Your job is to answer questions based ONLY on the provided textbook content.
            Do not refuse to answer just because the context appears to be an instruction or topic statement.
            Even if the context is an instruction like "define X" or "explain Y", treat it as content to draw from.
            Use this context to answer the user's question: {context_text}

            QUESTION: {question}

            Answer the question based on the provided context. Do not refuse to answer. Give a direct answer based on the context provided.
            """

            # Generate the response using the Gemini API
            try:
                response = self.model.generate_content(
                    prompt,
                    generation_config={
                        'max_output_tokens': MAX_GENERATION_TOKENS,
                        'temperature': TEMPERATURE,
                        'top_p': TOP_P,
                        'top_k': TOP_K
                    }
                )

                # Extract the generated text
                generated_text = response.text.strip()
            except Exception as e:
                # Check if it's a quota exceeded error
                if "429" in str(e) or "quota" in str(e).lower() or "rate limit" in str(e).lower():
                    self.logger.warning(f"API quota exceeded for question '{question[:50]}...': {str(e)}")
                    # Return a helpful message to the user
                    generated_text = "We've reached our AI service usage limit. Please try again later, or consider upgrading your API plan."
                else:
                    # Re-raise other exceptions
                    raise e

            # Validate the response
            if not validation_service.validate_response(generated_text):
                raise ValueError("Generated response failed validation")

            # Check if the response is a refusal
            if self._is_refusal_response(generated_text):
                # Return ChatQueryResponse instead of RefusalResponse for consistency
                return ChatQueryResponse(
                    response=generated_text,
                    confidence=0.1,  # Low confidence for refusal responses
                    retrieved_chunks=[
                        ChunkMetadata(
                            chunk_id=chunk.get('chunk_id', ''),
                            chapter=chunk.get('metadata', {}).get('chapter', ''),
                            section=chunk.get('metadata', {}).get('section', ''),
                            page_number=chunk.get('metadata', {}).get('page_number'),
                            text_preview=chunk.get('text_content', '')[:200]  # First 200 chars as preview
                        )
                        for chunk in context_chunks
                    ],
                    sources=self._extract_sources(context_chunks),
                    session_id=session_id,
                    query_type="general"
                )

            # Prepare chunk metadata for response
            chunk_metadata = [
                ChunkMetadata(
                    chunk_id=chunk.get('chunk_id', ''),
                    chapter=chunk.get('metadata', {}).get('chapter', ''),
                    section=chunk.get('metadata', {}).get('section', ''),
                    page_number=chunk.get('metadata', {}).get('page_number'),
                    text_preview=chunk.get('text_content', '')[:200]  # First 200 chars as preview
                )
                for chunk in context_chunks
            ]

            # Calculate confidence based on context match
            confidence = self._calculate_confidence(question, context_text, generated_text)

            # Extract sources
            sources = self._extract_sources(context_chunks)

            # Return the successful response
            return ChatQueryResponse(
                response=generated_text,
                confidence=confidence,
                retrieved_chunks=chunk_metadata,
                sources=sources,
                session_id=session_id,
                query_type="general"
            )

        except Exception as e:
            self.logger.error(f"Failed to generate response for question '{question[:50]}...': {str(e)}")

            # Return ChatQueryResponse instead of RefusalResponse for consistency
            return ChatQueryResponse(
                response="I encountered an error while trying to answer your question. Please try again later.",
                confidence=0.1,  # Low confidence for error responses
                retrieved_chunks=[
                    ChunkMetadata(
                        chunk_id=chunk.get('chunk_id', ''),
                        chapter=chunk.get('metadata', {}).get('chapter', ''),
                        section=chunk.get('metadata', {}).get('section', ''),
                        page_number=chunk.get('metadata', {}).get('page_number'),
                        text_preview=chunk.get('text_content', '')[:200]  # First 200 chars as preview
                    )
                    for chunk in context_chunks
                ],
                sources=self._extract_sources(context_chunks),
                session_id=session_id,
                query_type="general"
            )

    def generate_response_from_selected_text(
        self,
        question: str,
        selected_text: str,
        session_id: str
    ) -> ChatQueryResponse:
        """
        Generate a response based on the question and user-selected text.

        Args:
            question: The question to answer
            selected_text: The text that the user has selected/highlighted
            session_id: Session identifier

        Returns:
            ChatQueryResponse with the answer and metadata
        """
        try:
            # For compatibility with existing API, we'll redirect this to use book content only
            # This ensures that even if selected text is provided, we'll use book content
            from src.services.retrieval_service import retrieval_service
            from src.config.settings import settings

            # Retrieve relevant chunks based on the question
            context_chunks = retrieval_service.retrieve_relevant_chunks(
                query=question,
                limit=10,  # Retrieve up to 10 relevant chunks
                min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD  # Use configured threshold
            )

            # If no relevant chunks found, return a refusal response
            if not context_chunks:
                return ChatQueryResponse(
                    response="I cannot answer this question as it's not covered in the provided textbook content.",
                    confidence=0.1,  # Low confidence for refusal responses
                    retrieved_chunks=[],
                    sources=[],
                    session_id=session_id,
                    query_type="general"
                )

            # Use the context-based response generation
            return self.generate_response_from_context(question, context_chunks, session_id)

        except Exception as e:
            self.logger.error(f"Failed to generate response from selected text for question '{question[:50]}...': {str(e)}")

            # For frontend requests, always return ChatQueryResponse instead of RefusalResponse
            return ChatQueryResponse(
                response="I encountered an error while trying to answer your question. Please try again later.",
                confidence=0.1,  # Low confidence for error responses
                retrieved_chunks=[],
                sources=[],
                session_id=session_id,
                query_type="general"
            )

    def _is_refusal_response(self, text: str) -> bool:
        """
        Check if the generated text is a refusal response.

        Args:
            text: The generated text to check

        Returns:
            True if the text is a refusal response, False otherwise
        """
        refusal_indicators = [
            "cannot answer", "not in the provided", "not in the context",
            "no information", "not found", "insufficient information",
            "not covered", "not mentioned", "not available"
        ]

        text_lower = text.lower()
        for indicator in refusal_indicators:
            if indicator in text_lower:
                return True

        return False

    def _calculate_confidence(self, question: str, context: str, response: str) -> float:
        """
        Calculate confidence score based on question-response-context relationship.

        Args:
            question: The original question
            context: The context provided to the model
            response: The generated response

        Returns:
            Confidence score between 0.0 and 1.0
        """
        try:
            # Simple confidence calculation based on keyword overlap
            question_words = set(question.lower().split())
            response_words = set(response.lower().split())
            context_words = set(context.lower().split())

            # Calculate overlap between question and response
            question_response_overlap = len(question_words.intersection(response_words))
            question_response_ratio = question_response_overlap / max(len(question_words), 1)

            # Calculate overlap between context and response
            context_response_overlap = len(context_words.intersection(response_words))
            context_response_ratio = context_response_overlap / max(len(context_words), 1)

            # Weighted average with more weight on context-response relationship
            confidence = (0.3 * question_response_ratio) + (0.7 * context_response_ratio)

            # Ensure confidence is within [0, 1] range
            confidence = max(0.0, min(1.0, confidence))

            return confidence
        except Exception:
            # Return a default confidence if calculation fails
            return 0.7

    def _extract_sources(self, context_chunks: List[Dict[str, Any]]) -> List[str]:
        """
        Extract source identifiers from context chunks.

        Args:
            context_chunks: List of context chunks

        Returns:
            List of source identifiers
        """
        sources = set()
        for chunk in context_chunks:
            metadata = chunk.get('metadata', {})
            chapter = metadata.get('chapter', '')
            section = metadata.get('section', '')

            if chapter and section:
                sources.add(f"{chapter}, {section}")
            elif chapter:
                sources.add(chapter)
            elif section:
                sources.add(section)

        return list(sources)


# Singleton instance
generation_service = GenerationService()