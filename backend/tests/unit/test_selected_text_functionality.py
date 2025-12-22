"""
Unit tests for the selected-text-only functionality in the RAG Chatbot backend.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.services.generation_service import GenerationService
from src.models.chat import ChatQueryResponse, RefusalResponse


class TestSelectedTextFunctionality(unittest.TestCase):
    """Unit tests for the selected-text-only functionality."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.generation_service = GenerationService()

        # Sample selected text and question for testing
        self.sample_selected_text = """
        The Robot Operating System (ROS) is flexible framework for writing robot software.
        It is a collection of tools, libraries, and conventions that aim to simplify the
        task of creating complex and robust robot behavior across a wide variety of
        robot platforms.
        """
        self.sample_question = "What is ROS?"
        self.session_id = "test-session-123"

    def test_generate_response_from_selected_text_basic(self):
        """Test that the service can generate a response from selected text."""
        with patch.object(self.generation_service, 'model') as mock_model:
            # Mock the generate_content response
            mock_response = MagicMock()
            mock_response.text = "ROS is a flexible framework for writing robot software."
            mock_model.generate_content.return_value = mock_response

            # Call the method
            result = self.generation_service.generate_response_from_selected_text(
                question=self.sample_question,
                selected_text=self.sample_selected_text,
                session_id=self.session_id
            )

            # Assertions
            self.assertIsInstance(result, ChatQueryResponse)
            self.assertEqual(result.response, "ROS is a flexible framework for writing robot software.")
            self.assertEqual(result.session_id, self.session_id)
            self.assertGreaterEqual(result.confidence, 0.0)
            self.assertLessEqual(result.confidence, 1.0)

            # Verify that generate_content was called
            mock_model.generate_content.assert_called_once()

    def test_generate_response_from_selected_text_empty_selected_text(self):
        """Test that the service handles empty selected text appropriately."""
        result = self.generation_service.generate_response_from_selected_text(
            question=self.sample_question,
            selected_text="",
            session_id=self.session_id
        )

        # Should return a refusal response
        self.assertIsInstance(result, RefusalResponse)
        self.assertIn("cannot answer", result.message.lower())
        self.assertEqual(result.session_id, self.session_id)
        self.assertEqual(result.reason, "selected_text_insufficient")

    def test_generate_response_from_selected_text_invalid_question(self):
        """Test that the service handles invalid questions appropriately."""
        result = self.generation_service.generate_response_from_selected_text(
            question="",  # Invalid question
            selected_text=self.sample_selected_text,
            session_id=self.session_id
        )

        # Should return a refusal response
        self.assertIsInstance(result, RefusalResponse)
        self.assertIn("cannot answer", result.message.lower())
        self.assertEqual(result.session_id, self.session_id)
        self.assertEqual(result.reason, "selected_text_insufficient")

    def test_generate_response_from_selected_text_short_selected_text(self):
        """Test that the service refuses to answer if selected text is too short."""
        with patch.object(self.generation_service, 'model') as mock_model:
            # Mock the generate_content response to return a refusal
            mock_response = MagicMock()
            mock_response.text = "I cannot answer this question based on the provided selected text as it doesn't contain sufficient information."
            mock_model.generate_content.return_value = mock_response

            # Use very short selected text
            result = self.generation_service.generate_response_from_selected_text(
                question=self.sample_question,
                selected_text="Short text.",
                session_id=self.session_id
            )

            # Should return a refusal response
            self.assertIsInstance(result, RefusalResponse)
            self.assertEqual(result.session_id, self.session_id)
            self.assertEqual(result.reason, "selected_text_insufficient")

    def test_generate_response_from_selected_text_generation_failure(self):
        """Test that the service handles API generation failures gracefully."""
        with patch.object(self.generation_service, 'model') as mock_model:
            # Mock the generate_content to raise an exception
            mock_model.generate_content.side_effect = Exception("API Error")

            # Call the method
            result = self.generation_service.generate_response_from_selected_text(
                question=self.sample_question,
                selected_text=self.sample_selected_text,
                session_id=self.session_id
            )

            # Should return a refusal response
            self.assertIsInstance(result, RefusalResponse)
            self.assertIn("encountered an error", result.message.lower())
            self.assertEqual(result.session_id, self.session_id)
            self.assertEqual(result.reason, "selected_text_insufficient")

    def test_generate_response_from_selected_text_validation_failure(self):
        """Test that the service handles validation failures gracefully."""
        with patch('src.services.generation_service.validation_service') as mock_validation_service:
            # Mock validation to return False
            mock_validation_service.validate_response.return_value = False

            with patch.object(self.generation_service, 'model') as mock_model:
                # Mock the generate_content response
                mock_response = MagicMock()
                mock_response.text = "This response will fail validation"
                mock_model.generate_content.return_value = mock_response

                # Call the method
                result = self.generation_service.generate_response_from_selected_text(
                    question=self.sample_question,
                    selected_text=self.sample_selected_text,
                    session_id=self.session_id
                )

                # Should return a refusal response due to validation failure
                self.assertIsInstance(result, RefusalResponse)
                self.assertEqual(result.session_id, self.session_id)
                self.assertEqual(result.reason, "selected_text_insufficient")

    def test_is_refusal_response_identifies_refusals(self):
        """Test that the _is_refusal_response method correctly identifies refusal responses."""
        refusal_texts = [
            "Cannot answer this question",
            "Not in the provided context",
            "No information in the selected text",
            "Insufficient information",
            "Not covered in the provided text"
        ]

        for refusal_text in refusal_texts:
            with self.subTest(text=refusal_text):
                is_refusal = self.generation_service._is_refusal_response(refusal_text)
                self.assertTrue(is_refusal, f"Failed to identify '{refusal_text}' as refusal")

    def test_is_refusal_response_identifies_non_refusals(self):
        """Test that the _is_refusal_response method correctly identifies non-refusal responses."""
        non_refusal_texts = [
            "ROS is a flexible framework for writing robot software",
            "The answer to your question is yes",
            "According to the text, the main benefit is efficiency"
        ]

        for non_refusal_text in non_refusal_texts:
            with self.subTest(text=non_refusal_text):
                is_refusal = self.generation_service._is_refusal_response(non_refusal_text)
                self.assertFalse(is_refusal, f"Wrongly identified '{non_refusal_text}' as refusal")


if __name__ == '__main__':
    unittest.main()