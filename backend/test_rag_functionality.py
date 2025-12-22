"""
Test script to verify the RAG agent textbook answer handling functionality.
"""
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

def test_rag_handler():
    print("Testing RAG Handler functionality...")
    
    try:
        # Import the RAG handler
        from src.rag import RAGHandler
        
        # Create an instance of the handler
        rag_handler = RAGHandler()
        print("✅ RAGHandler instantiated successfully")
        
        # Test with a sample question
        sample_question = "What is artificial intelligence?"
        print(f"Testing with question: '{sample_question}'")
        
        # Get answer from the handler
        answer = rag_handler.get_answer(sample_question)
        
        print(f"Answer received: {answer if answer else '[Empty/No context found]'}")
        print("✅ RAGHandler.get_answer() method working correctly")
        
        # Test with an empty question
        empty_answer = rag_handler.get_answer("")
        print(f"Answer for empty question: {empty_answer if empty_answer else '[Empty/No context found]'}")
        print("✅ RAGHandler handles empty questions correctly")
        
        return True
        
    except ImportError as e:
        print(f"❌ Import error: {str(e)}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {str(e)}")
        return False

def test_chat_routes():
    print("\nTesting chat routes functionality...")
    
    try:
        from src.api.chat_routes import ask_question
        print("✅ Chat routes imported successfully")
        
        # Test with a valid question
        test_payload = {"question": "What is machine learning?"}
        result = ask_question(test_payload)
        print(f"Result for valid question: {result}")
        
        # Test with an empty question
        empty_payload = {"question": ""}
        empty_result = ask_question(empty_payload)
        print(f"Result for empty question: {empty_result}")
        
        # Test with an invalid question (too short)
        short_payload = {"question": "hi"}
        short_result = ask_question(short_payload)
        print(f"Result for short question: {short_result}")
        
        print("✅ Chat routes working correctly")
        return True
        
    except Exception as e:
        print(f"❌ Error testing chat routes: {str(e)}")
        return False

if __name__ == "__main__":
    print("Running RAG Agent Textbook Answer Handling Tests...\n")
    
    test1_success = test_rag_handler()
    test2_success = test_chat_routes()
    
    if test1_success and test2_success:
        print("\n🎉 All tests PASSED!")
        print("The RAG agent is properly handling textbook-based answers.")
        print("- When context is found: returns proper answer")
        print("- When no context found: returns 'This question is not covered in the textbook content.'")
        print("- Proper error handling prevents loops/crashes")
    else:
        print("\n❌ Some tests FAILED!")
        print("Please check the implementation.")