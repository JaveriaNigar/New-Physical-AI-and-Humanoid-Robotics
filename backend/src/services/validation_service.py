"""
Validation service for the RAG Chatbot backend.
Handles input validation, content validation, and response validation.
"""
import logging
import re
from typing import Optional, Dict, Any
from src.config.constants import (
    MIN_QUESTION_CHARACTERS, MAX_QUESTION_CHARACTERS,
    MIN_ANSWER_CHARACTERS, MAX_ANSWER_CHARACTERS
)


class ValidationService:
    """Service for validating inputs, content, and responses."""

    def __init__(self):
        """Initialize the validation service."""
        self.logger = logging.getLogger(__name__)

    def validate_question(self, question: str) -> bool:
        """
        Validate a question string.

        Args:
            question: The question to validate

        Returns:
            True if valid, False otherwise
        """
        if not question:
            self.logger.warning("Question is empty")
            return False

        if len(question) < MIN_QUESTION_CHARACTERS:
            self.logger.warning(f"Question too short: {len(question)} characters, minimum {MIN_QUESTION_CHARACTERS}")
            return False

        if len(question) > MAX_QUESTION_CHARACTERS:
            self.logger.warning(f"Question too long: {len(question)} characters, maximum {MAX_QUESTION_CHARACTERS}")
            return False

        # Sanitize the question by removing potential harmful content
        sanitized_question = self.sanitize_input(question)

        # Check if the question contains actual content (not just punctuation)
        if not re.search(r'[a-zA-Z0-9]', sanitized_question):
            self.logger.warning("Question doesn't contain meaningful content")
            return False

        # Additional security checks
        if self._contains_malicious_content(sanitized_question):
            self.logger.warning("Question contains potentially malicious content")
            return False

        return True

    def validate_selected_text(self, selected_text: str) -> bool:
        """
        Validate selected text for the selected-text-only mode.

        Args:
            selected_text: The selected text to validate

        Returns:
            True if valid, False otherwise
        """
        if not selected_text:
            self.logger.warning("Selected text is empty")
            return False

        if len(selected_text) < MIN_QUESTION_CHARACTERS:  # Using same min as question
            self.logger.warning(f"Selected text too short: {len(selected_text)} characters")
            return False

        if len(selected_text) > MAX_ANSWER_CHARACTERS:  # Using max answer length as an upper bound
            self.logger.warning(f"Selected text too long: {len(selected_text)} characters")
            return False

        # Sanitize the selected text
        sanitized_text = self.sanitize_input(selected_text)

        # Additional validation for selected text
        # It should contain meaningful content (not just whitespaces or punctuation)
        if not re.search(r'[a-zA-Z0-9]', sanitized_text):
            self.logger.warning("Selected text doesn't contain meaningful content")
            return False

        # Check for potential security issues
        if self._contains_malicious_content(sanitized_text):
            self.logger.warning("Selected text contains potentially malicious content")
            return False

        return True

    def validate_sufficient_selected_text(self, selected_text: str, question: str) -> bool:
        """
        Validate that the selected text is sufficient to answer the question.

        Args:
            selected_text: The selected text
            question: The question to be answered

        Returns:
            True if selected text is sufficient to answer the question, False otherwise
        """
        if not selected_text or not question:
            return False

        # Check if the selected text contains enough information to address the question
        # This is a basic implementation - in reality, you would use more sophisticated NLP techniques
        import re
        from collections import Counter

        # Get key terms from the question
        question_words = set(re.findall(r'\b\w+\b', question.lower()))

        # Get words from selected text
        text_words = set(re.findall(r'\b\w+\b', selected_text.lower()))

        # Calculate overlap between question terms and text terms
        overlap = len(question_words.intersection(text_words))
        overlap_percent = overlap / max(len(question_words), 1)

        # If less than 30% of question terms appear in the selected text, it may not be sufficient
        if overlap_percent < 0.3:
            self.logger.warning(
                f"Selected text may be insufficient for question. "
                f"Term overlap: {overlap_percent:.2%} ({overlap}/{len(question_words)} terms)"
            )
            return False

        # Additional check: verify that the selected text contains substantial content
        if len(selected_text.strip()) < 50:  # At least 50 characters of non-whitespace
            self.logger.warning(f"Selected text too short: {len(selected_text.strip())} characters")
            return False

        return True

    def validate_response(self, response: str) -> bool:
        """
        Validate a response string.

        Args:
            response: The response to validate

        Returns:
            True if valid, False otherwise
        """
        if not response:
            self.logger.warning("Response is empty")
            return False

        if len(response) < MIN_ANSWER_CHARACTERS:
            self.logger.warning(f"Response too short: {len(response)} characters, minimum {MIN_ANSWER_CHARACTERS}")
            return False

        if len(response) > MAX_ANSWER_CHARACTERS:
            self.logger.warning(f"Response too long: {len(response)} characters, maximum {MAX_ANSWER_CHARACTERS}")
            return False

        # Check if the response contains actual content (not just punctuation)
        if not re.search(r'[a-zA-Z0-9]', response):
            self.logger.warning("Response doesn't contain meaningful content")
            return False

        # Check for potential security issues in the response
        if self._contains_malicious_content(response):
            self.logger.warning("Response contains potentially malicious content")
            return False

        return True

    def validate_chunk_content(self, text_content: str, metadata: Dict[str, Any]) -> bool:
        """
        Validate a content chunk.

        Args:
            text_content: The text content of the chunk
            metadata: Metadata associated with the chunk

        Returns:
            True if valid, False otherwise
        """
        if not text_content or len(text_content.strip()) == 0:
            self.logger.warning("Chunk content is empty")
            return False

        # Sanitize chunk content
        sanitized_content = self.sanitize_input(text_content)

        if not metadata:
            self.logger.warning("Chunk metadata is empty")
            return False

        # Check if required metadata fields exist
        required_fields = ['source_file']
        for field in required_fields:
            if field not in metadata:
                self.logger.warning(f"Required metadata field '{field}' missing")
                return False

        # Check for chapter and section in metadata
        if 'chapter' not in metadata:
            self.logger.info(f"Optional metadata field 'chapter' missing for chunk")

        if 'section' not in metadata:
            self.logger.info(f"Optional metadata field 'section' missing for chunk")

        return True

    def validate_context_fidelity(self, response: str, context: str) -> bool:
        """
        Validate that the response is consistent with the provided context.
        This is a simplified version - in a real implementation you might use more sophisticated techniques.

        Args:
            response: The generated response
            context: The context provided to the LLM

        Returns:
            True if response appears to be consistent with context, False otherwise
        """
        if not response or not context:
            return False

        # Simple heuristic: check if some key terms from context appear in response
        context_words = set(context.lower().split()[:50])  # Use first 50 words as representative
        response_words = set(response.lower().split())

        # If less than 10% of context words appear in response, it might not be contextually consistent
        common_words = context_words.intersection(response_words)
        if len(common_words) < max(1, len(context_words) * 0.1):
            self.logger.warning("Response may not be sufficiently connected to the provided context")
            return False

        return True

    def is_external_knowledge_injected(self, response: str, context: str) -> bool:
        """
        Check if the response contains information not present in the context.
        This is a simplified version - real implementation would require more sophisticated NLP.

        Args:
            response: The generated response
            context: The context provided to the LLM

        Returns:
            True if external knowledge appears to be injected, False otherwise
        """
        # In a real implementation, you would use more sophisticated techniques
        # like semantic similarity, entity extraction, or other NLP methods
        # For now, we'll return False to indicate no external knowledge detected
        return False

    def sanitize_input(self, text: str) -> str:
        """
        Sanitize input text by removing potentially harmful content.

        Args:
            text: The input text to sanitize

        Returns:
            Sanitized version of the input text
        """
        if not text:
            return text

        # Remove potential script tags
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.IGNORECASE | re.DOTALL)

        # Remove potential iframe tags
        sanitized = re.sub(r'<iframe[^>]*>.*?</iframe>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove potential object/embed tags
        sanitized = re.sub(r'<(object|embed)[^>]*>.*?</\1>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove potential javascript: urls
        sanitized = re.sub(r'javascript:\s*\w+', '', sanitized, flags=re.IGNORECASE)

        # Remove potential data: urls that could contain scripts
        sanitized = re.sub(r'data:\s*[^,]*base64', '', sanitized, flags=re.IGNORECASE)

        # Remove potential SQL injection patterns
        sql_patterns = [
            r'\b(union|select|insert|update|delete|drop|create|alter|exec|execute)\b',
            r'(\b(or|and)\s+1\s*=\s*1\b)',
            r'(\b(or|and)\s+\'1\'\s*=\s*\'1\'\b)'
        ]

        for pattern in sql_patterns:
            sanitized = re.sub(pattern, '', sanitized, flags=re.IGNORECASE)

        return sanitized.strip()

    def _contains_malicious_content(self, text: str) -> bool:
        """
        Check if the text contains potentially malicious content.

        Args:
            text: The text to check

        Returns:
            True if malicious content is detected, False otherwise
        """
        if not text:
            return False

        # Check for potential XSS patterns
        xss_patterns = [
            r'<script',  # Script tags
            r'javascript:',  # JavaScript URLs
            r'on\w+\s*=',  # Event handlers (onclick, onload, etc.)
            r'<iframe',  # Iframe tags
            r'<object',  # Object tags
            r'<embed',  # Embed tags
            r'<link',  # Link tags
            r'<meta',  # Meta tags
        ]

        for pattern in xss_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return True

        # Check for potential SQL injection patterns
        sql_patterns = [
            r'\b(union|select|insert|update|delete|drop|create|alter|exec|execute)\b\s+\w',
            r'(\b(or|and)\s+1\s*=\s*1\b)',
            r'(\b(or|and)\s+\'1\'\s*=\s*\'1\'\b)',
            r'(\b(or|and)\s+\"1\"\s*=\s*\"1\"\b)',
        ]

        for pattern in sql_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return True

        # Check for potential command injection patterns
        cmd_patterns = [
            r'[;&|][\s]*(rm|mv|cp|ls|cat|echo|wget|curl|nc|ncat|telnet|ssh|ftp)',
            r'[\s>][\s]*[<>/]*[a-zA-Z0-9_]*\s*[|;]',
        ]

        for pattern in cmd_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return True

        return False


# Singleton instance
validation_service = ValidationService()