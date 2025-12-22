"""
Unit tests for the generation service in the RAG Chatbot backend.
Tests Gemini API integration and response generation functionality.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.generation_service import GenerationService
from src.models.chat import ChatQueryResponse, RefusalResponse, ChunkMetadata
from src.config.constants import MAX_GENERATION_TOKENS, TEMPERATURE, TOP_P, TOP_K


class TestGenerationService:
    """Test class for the generation service."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        with patch('src.config.settings.settings') as mock_settings:
            mock_settings.GEMINI_API_KEY = "test_api_key"
            mock_settings.GEMINI_MODEL_NAME = "models/gemini-pro"
            self.generation_service = GenerationService()
            # Mock the Gemini model
            self.generation_service.model = Mock()

    def test_generate_response_from_context_success(self):
        """Test successful response generation from context."""
        # Arrange
        question = "What are neural networks?"
        context_chunks = [
            {
                'chunk_id': 'chunk-1',
                'text_content': 'Neural networks are computing systems inspired by the human brain...',
                'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 127}
            }
        ]
        session_id = "test-session-123"
        
        mock_response = Mock()
        mock_response.text = "Neural networks are computing systems inspired by the human brain..."
        self.generation_service.model.generate_content.return_value = mock_response

        # Act
        result = self.generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, ChatQueryResponse)
        assert result.response == "Neural networks are computing systems inspired by the human brain..."
        assert result.session_id == session_id
        assert result.query_type == "general"
        assert len(result.retrieved_chunks) == 1
        assert result.retrieved_chunks[0].chunk_id == 'chunk-1'
        assert result.retrieved_chunks[0].chapter == 'Chapter 5'
        assert result.retrieved_chunks[0].section == '5.1'
        assert result.retrieved_chunks[0].page_number == 127

        # Verify the model was called with the right parameters
        self.generation_service.model.generate_content.assert_called_once()
        call_args = self.generation_service.model.generate_content.call_args
        assert question in call_args[0][0]  # Check that question is in the prompt
        assert context_chunks[0]['text_content'] in call_args[0][0]  # Check that context is in the prompt

    def test_generate_response_from_context_no_context(self):
        """Test response when no context is provided."""
        # Arrange
        question = "What are neural networks?"
        context_chunks = []
        session_id = "test-session-123"

        # Act
        result = self.generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, RefusalResponse)
        assert "not covered in the provided textbook content" in result.message
        assert result.reason == "no_context_found"
        assert result.session_id == session_id

    def test_generate_response_from_context_refusal_response(self):
        """Test that refusal responses from Gemini are properly handled."""
        # Arrange
        question = "What is the meaning of life?"
        context_chunks = [
            {
                'chunk_id': 'chunk-1',
                'text_content': 'This textbook covers artificial intelligence topics...',
                'metadata': {'chapter': 'Chapter 1', 'section': '1.1'}
            }
        ]
        session_id = "test-session-123"
        
        mock_response = Mock()
        mock_response.text = "I cannot answer this question as it's not covered in the provided textbook content."
        self.generation_service.model.generate_content.return_value = mock_response

        # Act
        result = self.generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, RefusalResponse)
        assert "not covered in the provided textbook content" in result.message
        assert result.reason == "insufficient_context"
        assert result.session_id == session_id

    def test_generate_response_from_context_generation_failure(self):
        """Test handling of generation failures."""
        # Arrange
        question = "What are neural networks?"
        context_chunks = [
            {
                'chunk_id': 'chunk-1',
                'text_content': 'Neural networks are computing systems inspired by the human brain...',
                'metadata': {'chapter': 'Chapter 5', 'section': '5.1'}
            }
        ]
        session_id = "test-session-123"
        
        self.generation_service.model.generate_content.side_effect = Exception("API Error")

        # Act
        result = self.generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, RefusalResponse)
        assert "encountered an error" in result.message
        assert result.reason == "insufficient_context"
        assert result.session_id == session_id

    def test_generate_response_from_selected_text_success(self):
        """Test successful response generation from selected text."""
        # Arrange
        question = "Explain this concept?"
        selected_text = "Backpropagation is a method used in artificial neural networks to calculate the gradient..."
        session_id = "test-session-123"
        
        mock_response = Mock()
        mock_response.text = "Backpropagation is a fundamental algorithm for training neural networks..."
        self.generation_service.model.generate_content.return_value = mock_response

        # Act
        result = self.generation_service.generate_response_from_selected_text(
            question=question,
            selected_text=selected_text,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, ChatQueryResponse)
        assert "Backpropagation is a fundamental algorithm" in result.response
        assert result.session_id == session_id
        assert result.query_type == "selected_text"
        assert len(result.retrieved_chunks) == 1
        assert result.retrieved_chunks[0].chunk_id == 'selected_text_chunk'
        assert result.retrieved_chunks[0].chapter == 'User Selected Text'
        assert result.retrieved_chunks[0].section == 'Selected Text'

        # Verify the model was called with the right parameters
        self.generation_service.model.generate_content.assert_called_once()
        call_args = self.generation_service.model.generate_content.call_args
        assert question in call_args[0][0]  # Check that question is in the prompt
        assert selected_text in call_args[0][0]  # Check that selected text is in the prompt

    def test_generate_response_from_selected_text_refusal_response(self):
        """Test that refusal responses from Gemini for selected text are properly handled."""
        # Arrange
        question = "How does this relate to quantum computing?"
        selected_text = "Backpropagation is a method used in artificial neural networks..."
        session_id = "test-session-123"
        
        mock_response = Mock()
        mock_response.text = "I cannot answer this question based on the selected text."
        self.generation_service.model.generate_content.return_value = mock_response

        # Act
        result = self.generation_service.generate_response_from_selected_text(
            question=question,
            selected_text=selected_text,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, RefusalResponse)
        assert "cannot answer" in result.message.lower()
        assert result.reason == "selected_text_insufficient"
        assert result.session_id == session_id

    def test_generate_response_from_selected_text_generation_failure(self):
        """Test handling of generation failures for selected text."""
        # Arrange
        question = "Explain this concept?"
        selected_text = "Backpropagation is a method used in artificial neural networks..."
        session_id = "test-session-123"
        
        self.generation_service.model.generate_content.side_effect = Exception("API Error")

        # Act
        result = self.generation_service.generate_response_from_selected_text(
            question=question,
            selected_text=selected_text,
            session_id=session_id
        )

        # Assert
        assert isinstance(result, RefusalResponse)
        assert "encountered an error" in result.message
        assert result.reason == "selected_text_insufficient"
        assert result.session_id == session_id

    def test_is_refusal_response_detection(self):
        """Test the refusal response detection logic."""
        # Test cases that should be detected as refusal responses
        refusal_texts = [
            "I cannot answer this question",
            "This is not in the provided context",
            "No information available",
            "Not covered in the textbook",
            "I don't have enough information",
            "The information is not mentioned",
            "Not available in the provided content"
        ]

        for text in refusal_texts:
            assert self.generation_service._is_refusal_response(text) is True, f"Failed for: {text}"

        # Test cases that should NOT be detected as refusal responses
        non_refusal_texts = [
            "Neural networks are computing systems",
            "Backpropagation is an algorithm",
            "The answer is 42",
            "Artificial intelligence is a field of study"
        ]

        for text in non_refusal_texts:
            assert self.generation_service._is_refusal_response(text) is False, f"Failed for: {text}"

    def test_calculate_confidence(self):
        """Test the confidence calculation logic."""
        # Arrange
        question = "What is backpropagation?"
        context = "Backpropagation is an algorithm used in artificial neural networks."
        response = "Backpropagation is an algorithm used in neural networks for training."

        # Act
        confidence = self.generation_service._calculate_confidence(question, context, response)

        # Assert
        assert 0.0 <= confidence <= 1.0
        # The confidence should be relatively high since there's good overlap
        assert confidence > 0.5

    def test_calculate_confidence_edge_cases(self):
        """Test confidence calculation with edge cases."""
        # Test with empty inputs
        confidence = self.generation_service._calculate_confidence("", "", "")
        assert 0.0 <= confidence <= 1.0

        # Test with completely different content
        confidence = self.generation_service._calculate_confidence(
            "What is quantum computing?",
            "Neural networks are computing systems inspired by the brain.",
            "Chemistry is the study of matter and its properties."
        )
        assert 0.0 <= confidence <= 1.0

    def test_extract_sources(self):
        """Test the source extraction logic."""
        # Arrange
        context_chunks = [
            {
                'metadata': {'chapter': 'Chapter 5', 'section': '5.1'}
            },
            {
                'metadata': {'chapter': 'Chapter 5', 'section': '5.2'}
            },
            {
                'metadata': {'chapter': 'Chapter 6', 'section': '6.1'}
            }
        ]

        # Act
        sources = self.generation_service._extract_sources(context_chunks)

        # Assert
        assert len(sources) == 3
        assert "Chapter 5, 5.1" in sources
        assert "Chapter 5, 5.2" in sources
        assert "Chapter 6, 6.1" in sources

    def test_extract_sources_with_missing_metadata(self):
        """Test source extraction with missing metadata."""
        # Arrange
        context_chunks = [
            {
                'metadata': {'chapter': 'Chapter 5'}  # Missing section
            },
            {
                'metadata': {'section': '5.2'}  # Missing chapter
            },
            {
                'metadata': {}  # Missing both
            }
        ]

        # Act
        sources = self.generation_service._extract_sources(context_chunks)

        # Assert
        # Should contain 'Chapter 5' and '5.2' but not empty strings
        assert 'Chapter 5' in sources
        assert '5.2' in sources
        # Filter out any empty strings
        assert '' not in sources