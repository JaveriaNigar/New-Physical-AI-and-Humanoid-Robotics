"""
Embedding service for the RAG Chatbot backend.
Handles generating embeddings for text content and managing the embedding process.
"""
import logging
from typing import List, Dict, Any
import google.generativeai as genai
from src.config.settings import settings
from src.config.constants import DEFAULT_EMBEDDING_DIMENSION
from src.utils.content_preprocessor import prepare_content_for_embedding


class EmbeddingService:
    """Service for generating and managing text embeddings."""

    def __init__(self):
        """Initialize the embedding service with configuration from settings."""
        # Configure the Google Generative AI client
        genai.configure(api_key=settings.GEMINI_API_KEY)

        # Store the model name for later use in embeddings (EmbeddingModel is deprecated)
        self.model_name = settings.EMBEDDING_MODEL_NAME
        self.logger = logging.getLogger(__name__)

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embeddings, each as a list of floats
        """
        embeddings = []

        for text in texts:
            try:
                # Generate embedding for the text using the new API
                result = genai.embed_content(
                    model=self.model_name,
                    content=text,
                    task_type="RETRIEVAL_DOCUMENT"  # Using appropriate task type
                )

                # Extract the embedding vector
                embedding = result['embedding']

                # Validate the embedding dimensions
                if len(embedding) != DEFAULT_EMBEDDING_DIMENSION:
                    self.logger.warning(
                        f"Embedding dimension mismatch: expected {DEFAULT_EMBEDDING_DIMENSION}, "
                        f"got {len(embedding)}"
                    )

                embeddings.append(embedding)

            except Exception as e:
                self.logger.error(f"Failed to generate embedding for text: {str(e)}")
                # Add a zero vector as a placeholder, though this is not ideal
                embeddings.append([0.0] * DEFAULT_EMBEDDING_DIMENSION)

        return embeddings

    def embed_textbook_content(self, content: str, source_file: str) -> List[Dict[str, Any]]:
        """
        Process textbook content, chunk it, and generate embeddings.

        Args:
            content: Raw textbook content
            source_file: Name of the source file

        Returns:
            List of dictionaries with text content, embeddings, and metadata
        """
        try:
            # Prepare content for embedding (clean and chunk)
            content_chunks = prepare_content_for_embedding(
                text=content,
                source_file=source_file,
                chunk_strategy="semantic"  # Use semantic chunking for textbooks
            )

            # Extract the text content from each chunk
            texts_to_embed = [chunk['text_content'] for chunk in content_chunks]

            # Generate embeddings for all text chunks
            embeddings = self.generate_embeddings(texts_to_embed)

            # Combine chunks with their embeddings
            result = []
            for i, chunk in enumerate(content_chunks):
                chunk_with_embedding = {
                    'text_content': chunk['text_content'],
                    'embedding': embeddings[i],
                    'metadata': chunk['metadata']
                }
                result.append(chunk_with_embedding)

            self.logger.info(f"Generated embeddings for {len(result)} content chunks from {source_file}")
            return result

        except Exception as e:
            self.logger.error(f"Failed to embed textbook content from {source_file}: {str(e)}")
            return []

    def embed_single_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            Embedding as a list of floats
        """
        try:
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="RETRIEVAL_DOCUMENT"  # Using appropriate task type
            )

            embedding = result['embedding']

            # Validate the embedding dimensions
            if len(embedding) != DEFAULT_EMBEDDING_DIMENSION:
                self.logger.warning(
                    f"Embedding dimension mismatch: expected {DEFAULT_EMBEDDING_DIMENSION}, "
                    f"got {len(embedding)}"
                )

            return embedding
        except Exception as e:
            self.logger.error(f"Failed to generate embedding for single text: {str(e)}")
            return [0.0] * DEFAULT_EMBEDDING_DIMENSION


# Singleton instance
embedding_service = EmbeddingService()