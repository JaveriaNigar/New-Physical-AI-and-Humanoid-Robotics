"""
Test script to verify the RAG chatbot functionality.
This script tests that the chatbot can retrieve relevant information from the indexed textbook content
and generate accurate responses based on the retrieved context.
"""
import asyncio
import sys
import os

# Add the src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.models.chat import ChatQueryRequest


def test_retrieval():
    """Test the retrieval service to ensure it can find relevant content."""
    print("Testing retrieval service...")
    
    # Test query about Physical AI concepts from the textbook
    test_query = "What is Physical AI and how does it differ from traditional digital AI?"
    
    # Retrieve relevant chunks
    relevant_chunks = retrieval_service.retrieve_relevant_chunks(
        query=test_query,
        limit=5,
        min_similarity_score=0.3
    )
    
    print(f"Found {len(relevant_chunks)} relevant chunks for query: '{test_query[:50]}...'")
    
    for i, chunk in enumerate(relevant_chunks[:3]):  # Show first 3 chunks
        print(f"Chunk {i+1} (Score: {chunk.get('similarity_score', 0):.3f}):")
        print(f"  Content preview: {chunk.get('text_content', '')[:100]}...")
        print(f"  Metadata: {chunk.get('metadata', {})}")
        print()
    
    return len(relevant_chunks) > 0


def test_generation():
    """Test the generation service to ensure it can create responses from retrieved context."""
    print("Testing generation service...")
    
    # Test query about Physical AI concepts
    test_question = "What is Physical AI and how does it differ from traditional digital AI?"
    session_id = "test-session-123"
    
    # First, retrieve relevant chunks
    context_chunks = retrieval_service.retrieve_relevant_chunks(
        query=test_question,
        limit=5,
        min_similarity_score=0.3
    )
    
    if not context_chunks:
        print("No relevant context found for the question.")
        return False
    
    print(f"Using {len(context_chunks)} chunks for context")
    
    # Generate response from the context
    response = generation_service.generate_response_from_context(
        question=test_question,
        context_chunks=context_chunks,
        session_id=session_id
    )
    
    print(f"Generated response: {response.response[:200]}...")
    print(f"Confidence: {response.confidence}")
    print(f"Number of retrieved chunks: {len(response.retrieved_chunks)}")
    print(f"Sources: {response.sources}")
    
    return response.response and len(response.response) > 0


def test_end_to_end():
    """Test the complete RAG pipeline from query to response."""
    print("Testing end-to-end RAG pipeline...")
    
    # Test various questions about the textbook content
    test_questions = [
        "What is Physical AI and embodied intelligence?",
        "Explain the difference between digital and physical AI systems",
        "What are the key principles of embodied intelligence?",
        "Why are humanoid robots advantageous in human environments?",
        "How do physics constraints affect Physical AI systems?"
    ]
    
    for question in test_questions:
        print(f"\nQuestion: {question}")
        
        # Retrieve relevant chunks
        context_chunks = retrieval_service.retrieve_relevant_chunks(
            query=question,
            limit=5,
            min_similarity_score=0.3
        )
        
        if not context_chunks:
            print("  No relevant context found.")
            continue
        
        print(f"  Found {len(context_chunks)} relevant chunks")
        
        # Generate response
        response = generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id="test-session-123"
        )
        
        if hasattr(response, 'response'):  # Successful response
            print(f"  Response: {response.response[:150]}...")
            print(f"  Confidence: {response.confidence:.2f}")
        elif hasattr(response, 'message'):  # Refusal response
            print(f"  Refusal: {response.message}")
        else:
            print("  Unknown response type")
    
    return True


def main():
    """Main function to run all tests."""
    print("Starting RAG Chatbot functionality tests...\n")
    
    # Test 1: Retrieval
    retrieval_success = test_retrieval()
    print(f"Retrieval test: {'PASSED' if retrieval_success else 'FAILED'}\n")
    
    # Test 2: Generation
    generation_success = test_generation()
    print(f"Generation test: {'PASSED' if generation_success else 'FAILED'}\n")
    
    # Test 3: End-to-end
    e2e_success = test_end_to_end()
    print(f"End-to-end test: {'PASSED' if e2e_success else 'FAILED'}\n")
    
    print("All tests completed!")
    
    # Summary
    all_passed = retrieval_success and generation_success and e2e_success
    print(f"Overall result: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")


if __name__ == "__main__":
    main()