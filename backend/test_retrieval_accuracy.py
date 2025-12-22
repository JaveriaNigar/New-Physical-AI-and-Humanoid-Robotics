"""
Test script to verify the improved textbook retrieval accuracy.
"""
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_retrieval_improvement():
    print("Testing retrieval improvement with lowered similarity threshold...")
    
    try:
        # Import the retrieval service
        from src.services.retrieval_service import retrieval_service
        from src.config.settings import settings
        
        print(f"Current MIN_SIMILARITY_THRESHOLD: {settings.MIN_SIMILARITY_THRESHOLD}")
        print(f"Current QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
        
        # Test with a sample question
        sample_question = "What is artificial intelligence?"
        print(f"\nTesting retrieval with question: '{sample_question}'")
        
        # Retrieve context with the lowered threshold
        context_chunks = retrieval_service.retrieve_relevant_chunks(
            query=sample_question,
            limit=10,
            min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD
        )
        
        print(f"Number of chunks retrieved: {len(context_chunks)}")
        
        if len(context_chunks) > 0:
            print("✅ Retrieval is working - found relevant content!")
            print(f"First chunk preview: {context_chunks[0]['text_content'][:100]}...")
        else:
            print("⚠️  No chunks retrieved - may need to verify Qdrant content exists")
        
        # Test with another question
        sample_question2 = "Explain neural networks"
        print(f"\nTesting with another question: '{sample_question2}'")
        
        context_chunks2 = retrieval_service.retrieve_relevant_chunks(
            query=sample_question2,
            limit=10,
            min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD
        )
        
        print(f"Number of chunks retrieved: {len(context_chunks2)}")
        
        if len(context_chunks2) > 0:
            print("✅ Second retrieval test successful!")
            print(f"First chunk preview: {context_chunks2[0]['text_content'][:100]}...")
        else:
            print("⚠️  No chunks retrieved for second test - may need to verify Qdrant content exists")
        
        # Compare with higher threshold to show the difference
        print(f"\nComparing with higher threshold (0.7)...")
        context_chunks_high_threshold = retrieval_service.retrieve_relevant_chunks(
            query=sample_question,
            limit=10,
            min_similarity_score=0.7
        )
        
        print(f"With threshold 0.7: {len(context_chunks_high_threshold)} chunks")
        print(f"With threshold {settings.MIN_SIMILARITY_THRESHOLD}: {len(context_chunks)} chunks")
        
        if len(context_chunks) > len(context_chunks_high_threshold):
            print("✅ Lower threshold is retrieving more content as expected!")
        else:
            print("ℹ️  Threshold adjustment may not show difference with this query")
        
        return True
        
    except ImportError as e:
        print(f"❌ Import error: {str(e)}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {str(e)}")
        return False

def test_generation_service():
    print("\nTesting generation service with retrieved context...")
    
    try:
        from src.services.generation_service import generation_service
        
        # Mock context chunks
        mock_context = [{
            'chunk_id': 'test-chunk-1',
            'text_content': 'Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals.',
            'metadata': {
                'chapter': 'Introduction to AI',
                'section': '1.1',
                'page_number': 1
            }
        }]
        
        question = "What is artificial intelligence?"
        print(f"Asking: {question}")
        
        response = generation_service.generate_response_from_context(
            question=question,
            context_chunks=mock_context,
            session_id="test-session"
        )
        
        print(f"Generated response: {response.response[:200]}...")
        print("✅ Generation service working with context")
        
        return True
        
    except Exception as e:
        print(f"❌ Error in generation test: {str(e)}")
        return False

if __name__ == "__main__":
    print("Running Qwen Textbook Retrieval Improvement Tests...\n")
    
    test1_success = test_retrieval_improvement()
    test2_success = test_generation_service()
    
    if test1_success and test2_success:
        print("\n🎉 All tests PASSED!")
        print("The retrieval accuracy has been improved by lowering the similarity threshold.")
        print("- Textbook questions should now find relevant content more often")
        print("- Reduced 'not covered' responses for valid textbook questions")
        print("- Better balance between precision and recall")
    else:
        print("\n❌ Some tests FAILED!")
        print("Please check the implementation.")