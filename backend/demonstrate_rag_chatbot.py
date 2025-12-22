"""
Demonstration script for the RAG Chatbot.
Shows how the system can answer questions based on the indexed textbook content.
"""
import sys
import os

# Add the src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service


def demonstrate_rag_chatbot():
    """Demonstrate the RAG chatbot functionality with sample questions."""
    print("RAG Chatbot Demonstration")
    print("=" * 30)

    # Sample questions about the textbook content
    sample_questions = [
        "What is Physical AI and how does it differ from traditional digital AI?",
        "Explain the concept of embodied intelligence and its importance in robotics",
        "What are the key principles of embodied intelligence?",
        "Why are humanoid robots specifically advantageous in human environments?",
        "How do physics constraints affect Physical AI systems?",
        "What are the main differences between digital and physical AI systems?"
    ]

    for i, question in enumerate(sample_questions, 1):
        print(f"\nQuestion {i}: {question}")
        print("-" * (len(question) + 12))

        # Step 1: Retrieve relevant chunks
        context_chunks = retrieval_service.retrieve_relevant_chunks(
            query=question,
            limit=3,  # Get top 3 most relevant chunks
            min_similarity_score=0.3
        )

        print(f"Retrieved {len(context_chunks)} relevant chunks from the textbook")

        if context_chunks:
            # Show the most relevant chunk
            top_chunk = context_chunks[0]
            print(f"   Most relevant content: '{top_chunk['text_content'][:100]}...'")
            print(f"   From: {top_chunk['metadata'].get('source_file', 'Unknown')}")
            print(f"   Similarity: {top_chunk['similarity_score']:.3f}")

        # Step 2: Generate response based on context
        response = generation_service.generate_response_from_context(
            question=question,
            context_chunks=context_chunks,
            session_id="demo-session"
        )

        # Step 3: Display the response
        if hasattr(response, 'response'):  # Successful response
            print(f"Answer: {response.response}")
            print(f"Confidence: {response.confidence:.2f}")
            print(f"Sources: {', '.join(response.sources[:3]) if response.sources else 'None'}")  # Show first 3 sources
        else:  # Refusal response
            print(f"The system refused to answer: {response.message}")

        print("\n" + "="*50)


def main():
    """Main function to run the demonstration."""
    print("Initializing RAG Chatbot demonstration...\n")

    # Verify that the system is working
    try:
        # Test a simple retrieval to make sure everything is connected
        test_chunks = retrieval_service.retrieve_relevant_chunks(
            query="Physical AI concepts",
            limit=1
        )

        if test_chunks:
            print(f"System is ready! Found {len(test_chunks)} test chunks.")
            print(f"   Sample content: {test_chunks[0]['text_content'][:50]}...\n")
        else:
            print("Warning: No test chunks found. System may not be properly initialized.")
            return
    except Exception as e:
        print(f"Error connecting to the system: {str(e)}")
        return

    # Run the demonstration
    demonstrate_rag_chatbot()

    print("\nDemonstration complete!")
    print("\nThe RAG Chatbot is fully functional and can answer questions based on the")
    print("Physical AI textbook content using Retrieval-Augmented Generation.")


if __name__ == "__main__":
    main()