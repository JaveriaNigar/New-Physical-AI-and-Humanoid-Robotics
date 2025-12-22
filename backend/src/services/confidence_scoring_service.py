"""
Confidence scoring implementation for the RAG Chatbot backend.
Calculates and provides confidence scores for responses based on context relevance.
"""
import logging
from typing import List, Dict, Any

from src.config.constants import MAX_GENERATION_TOKENS


class ConfidenceScoringService:
    """Service for calculating confidence scores for responses based on context relevance."""
    
    def __init__(self):
        """Initialize the confidence scoring service."""
        self.logger = logging.getLogger(__name__)
    
    def calculate_confidence(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        response: str,
        similarity_scores: List[float] = None
    ) -> float:
        """
        Calculate confidence score for a response based on query, context and response quality.
        
        Args:
            query: The original query
            context_chunks: List of context chunks used to generate the response
            response: The generated response
            similarity_scores: Optional list of similarity scores for each chunk
            
        Returns:
            Confidence score between 0.0 and 1.0
        """
        try:
            # Initialize score components
            relevance_score = 0.0
            context_quality_score = 0.0
            response_quality_score = 0.0
            
            # Calculate relevance score based on similarity scores if provided
            if similarity_scores and len(similarity_scores) > 0:
                avg_similarity = sum(similarity_scores) / len(similarity_scores)
                # Normalize similarity score to 0-1 range
                relevance_score = min(1.0, avg_similarity)
            else:
                # If no similarity scores provided, calculate based on semantic overlap
                relevance_score = self._calculate_semantic_overlap_score(query, context_chunks)
            
            # Calculate context quality score based on number and quality of chunks
            context_quality_score = self._calculate_context_quality_score(context_chunks)
            
            # Calculate response quality score based on response characteristics
            response_quality_score = self._calculate_response_quality_score(query, response, context_chunks)
            
            # Weighted average of the scores
            # Relevance is most important (40%), context quality (30%), response quality (30%)
            confidence = (
                0.4 * relevance_score +
                0.3 * context_quality_score +
                0.3 * response_quality_score
            )
            
            # Ensure the confidence is within [0, 1] range
            confidence = max(0.0, min(1.0, confidence))
            
            self.logger.info(f"Calculated confidence score: {confidence:.3f}")
            return confidence
            
        except Exception as e:
            self.logger.error(f"Error calculating confidence score: {str(e)}")
            # Return a conservative confidence score in case of error
            return 0.5
    
    def _calculate_semantic_overlap_score(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]]
    ) -> float:
        """Calculate a score based on semantic overlap between query and context."""
        if not context_chunks:
            return 0.0
        
        # Simple heuristic: calculate overlap of words between query and context
        query_words = set(query.lower().split())
        
        total_overlap = 0
        total_context_words = 0
        
        for chunk in context_chunks:
            context_text = chunk.get('text_content', '')
            context_words = set(context_text.lower().split())
            
            overlap = len(query_words.intersection(context_words))
            total_overlap += overlap
            total_context_words += len(context_words)
        
        if total_context_words == 0:
            return 0.0
        
        # Normalize overlap to 0-1 range
        overlap_ratio = total_overlap / total_context_words
        return min(1.0, overlap_ratio * 2)  # Boost the ratio to better reflect relevance
    
    def _calculate_context_quality_score(self, context_chunks: List[Dict[str, Any]]) -> float:
        """Calculate a score based on the quality and quantity of context chunks."""
        if not context_chunks:
            return 0.0
        
        # More chunks generally mean better context (up to a point)
        chunk_count_score = min(1.0, len(context_chunks) / 10.0)  # Up to 10 chunks = full score
        
        # Calculate average chunk quality based on length and content
        total_length = 0
        for chunk in context_chunks:
            text_content = chunk.get('text_content', '')
            total_length += len(text_content)
        
        avg_chunk_length = total_length / len(context_chunks) if context_chunks else 0
        # Prefer chunks that are not too short (meaningful content) but not excessively long
        length_quality = max(0.0, min(1.0, avg_chunk_length / 500))  # Optimal around 500 chars
        
        # Weighted combination
        quality_score = 0.7 * chunk_count_score + 0.3 * length_quality
        return quality_score
    
    def _calculate_response_quality_score(
        self,
        query: str,
        response: str,
        context_chunks: List[Dict[str, Any]]
    ) -> float:
        """Calculate a score based on the quality of the response."""
        if not response or not query:
            return 0.0
        
        # Check if response is not too short (indicating low quality)
        response_length_score = max(0.0, min(1.0, len(response) / 50))  # At least 50 chars for full score
        
        # Check if response contains information clearly related to the query
        query_words = set(query.lower().split())
        response_words = set(response.lower().split())
        response_to_query_relevance = len(query_words.intersection(response_words)) / max(1, len(query_words))
        
        # Combine metrics
        response_quality = (
            0.5 * response_length_score +
            0.5 * response_to_query_relevance
        )
        
        return response_quality


# Singleton instance
confidence_scoring_service = ConfidenceScoringService()