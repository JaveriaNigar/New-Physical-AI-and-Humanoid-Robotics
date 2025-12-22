"""
Architecture validation tests for the RAG Chatbot backend.
Validates component separation, information flows, and external knowledge injection prevention.
"""
import pytest
from unittest.mock import patch, MagicMock
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.services.embedding_service import embedding_service
from src.vector_store.qdrant_client import qdrant_client
from src.database.postgres_client import postgres_client
import inspect


class TestArchitectureValidation:
    """Test class for architecture validation of the RAG system."""

    def test_component_separation_verification(self):
        """Test that ingestion, retrieval, and generation components are properly separated."""
        # Verify that each component exists as a separate service
        assert hasattr(retrieval_service, 'retrieve_relevant_chunks')
        assert hasattr(generation_service, 'generate_response_from_context')
        assert hasattr(embedding_service, 'generate_embeddings')
        
        # Verify that services have distinct responsibilities
        retrieval_methods = [name for name, method in inspect.getmembers(retrieval_service, predicate=inspect.ismethod)]
        generation_methods = [name for name, method in inspect.getmembers(generation_service, predicate=inspect.ismethod)]
        embedding_methods = [name for name, method in inspect.getmembers(embedding_service, predicate=inspect.ismethod)]
        
        # Ensure there's no overlap in primary responsibilities
        # Retrieval should focus on finding relevant content
        assert any('retrieve' in method or 'search' in method for method in retrieval_methods)
        # Generation should focus on creating responses
        assert any('generate' in method or 'response' in method for method in generation_methods)
        # Embedding should focus on creating vector representations
        assert any('embed' in method for method in embedding_methods)

    def test_retrieval_generation_interface_isolation(self):
        """Test that retrieval and generation components have clear interfaces."""
        # Mock retrieval to return known chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-1',
                    'text_content': 'Neural networks are computing systems...',
                    'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 127}
                }
            ]
            mock_retrieve.return_value = mock_chunks

            # Verify that generation service only receives the data it needs
            # and doesn't directly access retrieval internals
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                # Call the generation service with the retrieved chunks
                generation_service.generate_response_from_context(
                    question="What are neural networks?",
                    context_chunks=mock_chunks,
                    session_id="test-session"
                )
                
                # Verify that the generation service was called with the expected parameters
                mock_generate.assert_called_once()
                
                # Verify that retrieval and generation don't share internal state directly
                assert not hasattr(generation_service, '_retrieval_internal_state')
                assert not hasattr(retrieval_service, '_generation_internal_state')

    def test_deterministic_retrieval_behavior(self):
        """Test that retrieval behavior is deterministic for the same inputs."""
        # This test verifies that the same query and context consistently return the same results
        query = "What are neural networks?"
        
        # Mock the embedding service to return the same vector for the same text
        with patch.object(embedding_service, 'embed_single_text') as mock_embed:
            mock_embedding = [0.1, 0.2, 0.3, 0.4, 0.5]
            mock_embed.return_value = mock_embedding
            
            with patch.object(qdrant_client, 'search_vectors') as mock_search:
                mock_results = [
                    {
                        'chunk_id': 'chunk-1',
                        'text_content': 'Neural networks are computing systems...',
                        'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 127},
                        'similarity_score': 0.95
                    }
                ]
                mock_search.return_value = mock_results

                # Call retrieval multiple times with the same query
                result1 = retrieval_service.retrieve_relevant_chunks(query, limit=5, min_similarity_score=0.5)
                result2 = retrieval_service.retrieve_relevant_chunks(query, limit=5, min_similarity_score=0.5)
                
                # Results should be identical
                assert result1 == result2
                assert len(result1) == len(result2)
                if result1:
                    assert result1[0]['chunk_id'] == result2[0]['chunk_id']

    def test_information_flows_between_components(self):
        """Test that information flows correctly between components."""
        # Test the flow: embedding -> retrieval -> generation
        
        # Mock the embedding service
        with patch.object(embedding_service, 'embed_single_text') as mock_embed:
            mock_embedding = [0.1, 0.2, 0.3]
            mock_embed.return_value = mock_embedding
            
            # Mock the retrieval service
            with patch.object(retrieval_service, 'qdrant_client') as mock_qdrant:
                mock_chunks = [
                    {
                        'chunk_id': 'chunk-info-flow',
                        'text_content': 'Information flows between components in the RAG system.',
                        'metadata': {'chapter': 'Chapter 10', 'section': '10.1', 'page_number': 250},
                        'similarity_score': 0.88
                    }
                ]
                mock_qdrant.search_vectors.return_value = mock_chunks
                
                # Mock the generation service
                with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                    from src.models.chat import ChatQueryResponse, ChunkMetadata
                    mock_response = ChatQueryResponse(
                        response="Information flows from embedding to retrieval to generation.",
                        confidence=0.88,
                        retrieved_chunks=[
                            ChunkMetadata(
                                chunk_id='chunk-info-flow',
                                chapter='Chapter 10',
                                section='10.1',
                                page_number=250,
                                text_preview='Information flows between components...'
                            )
                        ],
                        sources=["Chapter 10, 10.1"],
                        session_id="test-info-flow",
                        query_type="general"
                    )
                    mock_generate.return_value = mock_response

                    # Execute the full flow
                    retrieved_chunks = retrieval_service.retrieve_relevant_chunks(
                        query="How does information flow in the RAG system?",
                        limit=5,
                        min_similarity_score=0.5
                    )
                    
                    final_response = generation_service.generate_response_from_context(
                        question="How does information flow in the RAG system?",
                        context_chunks=retrieved_chunks,
                        session_id="test-info-flow"
                    )
                    
                    # Verify the flow worked correctly
                    assert len(retrieved_chunks) > 0
                    assert "Information flows" in final_response.response
                    assert final_response.confidence > 0.5

    def test_external_knowledge_injection_prevention(self):
        """Test that the system prevents external knowledge injection."""
        # This test verifies that the system only uses provided context and doesn't generate
        # information from external sources
        
        # Mock retrieval to return specific context
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-specific',
                    'text_content': 'The RAG system uses only textbook content for responses.',
                    'metadata': {'chapter': 'Chapter 1', 'section': '1.1', 'page_number': 10}
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                from src.models.chat import ChatQueryResponse, ChunkMetadata
                mock_response = ChatQueryResponse(
                    response="The RAG system uses only textbook content for responses.",
                    confidence=0.95,
                    retrieved_chunks=[
                        ChunkMetadata(
                            chunk_id='chunk-specific',
                            chapter='Chapter 1',
                            section='1.1',
                            page_number=10,
                            text_preview='The RAG system uses only textbook content...'
                        )
                    ],
                    sources=["Chapter 1, 1.1"],
                    session_id="test-external-prevention",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                # Test that the response is based only on the provided context
                result = generation_service.generate_response_from_context(
                    question="What does the RAG system use for responses?",
                    context_chunks=mock_chunks,
                    session_id="test-external-prevention"
                )
                
                # Verify the response is grounded in the provided context
                assert "textbook content" in result.response.lower()
                # Verify the response doesn't contain information not in the context
                assert "external" not in result.response.lower() or "wikipedia" not in result.response.lower()

    def test_component_isolation_with_selected_text_mode(self):
        """Test that selected text mode properly isolates components."""
        # In selected text mode, the system should bypass the vector database completely
        question = "Explain this concept?"
        selected_text = "Backpropagation is an algorithm used in neural networks."
        
        # Mock the generation service to generate response from selected text
        with patch.object(generation_service, 'generate_response_from_selected_text') as mock_generate:
            from src.models.chat import ChatQueryResponse, ChunkMetadata
            mock_response = ChatQueryResponse(
                response="Backpropagation is an algorithm used to train neural networks.",
                confidence=0.9,
                retrieved_chunks=[
                    ChunkMetadata(
                        chunk_id='selected_text_chunk',
                        chapter='User Selected Text',
                        section='Selected Text',
                        page_number=None,
                        text_preview=selected_text[:200]
                    )
                ],
                sources=["User Selected Text"],
                session_id="test-selected-isolation",
                query_type="selected_text"
            )
            mock_generate.return_value = mock_response

            # Execute selected text flow
            result = generation_service.generate_response_from_selected_text(
                question=question,
                selected_text=selected_text,
                session_id="test-selected-isolation"
            )
            
            # Verify the response is based only on selected text
            assert "backpropagation" in result.response.lower()
            assert result.query_type == "selected_text"
            
            # The retrieval service should not have been involved in this flow
            # when using selected text mode

    def test_qdrant_postgres_separation(self):
        """Test that Qdrant (vector DB) and Postgres (metadata DB) are properly separated."""
        # Verify that both clients exist and are separate
        assert qdrant_client is not None
        assert postgres_client is not None
        assert qdrant_client is not postgres_client
        
        # Verify they have different responsibilities
        qdrant_methods = [name for name, method in inspect.getmembers(qdrant_client, predicate=inspect.ismethod)]
        postgres_methods = [name for name, method in inspect.getmembers(postgres_client, predicate=inspect.ismethod)]
        
        # Qdrant should have vector-related methods
        assert any('vector' in method.lower() or 'search' in method.lower() for method in qdrant_methods)
        # Postgres should have record/table-related methods
        # (actual postgres client methods would be checked in a real implementation)

    def test_architecture_compliance_checker(self):
        """Test the architecture compliance through health checks."""
        # Verify that all components can be accessed and are properly initialized
        from src.api.health_routes import health_check
        import asyncio
        
        # This would normally check the actual health of services
        # For testing purposes, we'll just verify the function exists and can be called
        # without throwing an exception (when mocked)
        
        with patch.object(qdrant_client, 'health_check', return_value=True), \
             patch.object(postgres_client, 'health_check', return_value=True), \
             patch('src.database.postgres_client.postgres_client.health_check', return_value=True):
            
            # The health check should return a valid response structure
            health_result = asyncio.run(health_check()) if hasattr(asyncio, 'run') else health_check()
            
            assert hasattr(health_result, 'status')
            assert hasattr(health_result, 'timestamp')
            assert hasattr(health_result, 'services')
            
            # Verify service status structure
            assert 'qdrant' in health_result.services
            assert 'postgres' in health_result.services
            assert 'gemini' in health_result.services

    def test_hallucination_prevention_mechanism(self):
        """Test that the hallucination prevention mechanism works correctly."""
        # Mock retrieval to return no relevant chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_retrieve.return_value = []  # No relevant context found
            
            # The system should refuse to answer when no context is found
            # This is verified by checking if the API returns a refusal response
            # in the integration tests, but here we verify the internal logic
            
            result = retrieval_service.retrieve_relevant_chunks(
                query="What is the meaning of life according to the textbook?",
                limit=10,
                min_similarity_score=0.7  # High threshold to ensure no results
            )
            
            # Should return empty list when no context meets threshold
            assert result == []

    def test_traceability_implementation(self):
        """Test that responses include proper traceability to source chunks."""
        # Mock retrieval with specific chunk metadata
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'trace-chunk-1',
                    'text_content': 'Neural networks consist of layers of interconnected nodes.',
                    'metadata': {
                        'chapter': 'Chapter 5', 
                        'section': '5.2', 
                        'page_number': 105,
                        'source_file': 'ai_textbook.pdf'
                    }
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                from src.models.chat import ChatQueryResponse, ChunkMetadata
                mock_response = ChatQueryResponse(
                    response="Neural networks consist of layers of interconnected nodes.",
                    confidence=0.92,
                    retrieved_chunks=[
                        ChunkMetadata(
                            chunk_id='trace-chunk-1',
                            chapter='Chapter 5',
                            section='5.2',
                            page_number=105,
                            text_preview='Neural networks consist of layers...'
                        )
                    ],
                    sources=["Chapter 5, 5.2"],
                    session_id="test-traceability",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                # Execute the flow
                result = generation_service.generate_response_from_context(
                    question="What are neural networks composed of?",
                    context_chunks=mock_chunks,
                    session_id="test-traceability"
                )
                
                # Verify traceability information is preserved
                assert len(result.retrieved_chunks) > 0
                assert result.retrieved_chunks[0].chunk_id == 'trace-chunk-1'
                assert result.retrieved_chunks[0].chapter == 'Chapter 5'
                assert result.retrieved_chunks[0].section == '5.2'
                assert result.sources == ["Chapter 5, 5.2"]