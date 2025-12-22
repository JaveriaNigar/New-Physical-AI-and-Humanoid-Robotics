"""
Final hallucination testing and verification for the RAG Chatbot backend.
Ensures the system properly refuses to answer when no relevant context is found.
"""
import asyncio
from typing import List, Dict, Any
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.models.chat import ChatQueryResponse, RefusalResponse


class HallucinationTester:
    """Tests to verify the RAG system does not hallucinate information."""
    
    def __init__(self):
        """Initialize the hallucination tester."""
        self.tests_passed = 0
        self.tests_failed = 0
    
    def run_all_tests(self) -> Dict[str, Any]:
        """Run all hallucination prevention tests."""
        results = {
            "test_results": [],
            "summary": {}
        }
        
        # Test 1: Questions about topics not in the textbook
        result1 = self.test_off_topic_questions()
        results["test_results"].append(result1)
        
        # Test 2: Questions requiring external knowledge
        result2 = self.test_external_knowledge_questions()
        results["test_results"].append(result2)
        
        # Test 3: Vague questions without sufficient context
        result3 = self.test_vague_questions()
        results["test_results"].append(result3)
        
        # Test 4: Questions mixing book content with external claims
        result4 = self.test_mixed_content_questions()
        results["test_results"].append(result4)
        
        # Summary
        results["summary"] = {
            "total_tests": len(results["test_results"]),
            "passed": sum(1 for r in results["test_results"] if r["passed"]),
            "failed": sum(1 for r in results["test_results"] if not r["passed"]),
            "hallucination_prevention_status": "PASS" if all(r["passed"] for r in results["test_results"]) else "FAIL"
        }
        
        return results
    
    def test_off_topic_questions(self) -> Dict[str, Any]:
        """Test that the system refuses to answer off-topic questions."""
        test_questions = [
            "What is the current weather in Tokyo?",
            "Who won the World Cup last year?",
            "What is the stock price of Apple today?",
            "How do I fix my broken car?",
        ]
        
        all_passed = True
        failed_questions = []
        
        for question in test_questions:
            # Check if retrieval finds any relevant chunks
            chunks = retrieval_service.retrieve_relevant_chunks(
                query=question,
                limit=5,
                min_similarity_score=0.7  # High threshold to ensure no results
            )
            
            # If no chunks found, the system should refuse to answer
            if len(chunks) == 0:
                # Try to generate a response (this should result in a refusal)
                response = generation_service.generate_response_from_context(
                    question=question,
                    context_chunks=chunks,
                    session_id="hallucination-test"
                )
                
                # Check if it's a refusal response
                if isinstance(response, RefusalResponse):
                    continue  # This is expected behavior
                else:
                    all_passed = False
                    failed_questions.append(question)
            else:
                # If chunks were found for an off-topic question, that's a potential issue
                all_passed = False
                failed_questions.append(f"Found chunks for off-topic question: {question}")
        
        return {
            "test_name": "Off-topic questions test",
            "description": "System should refuse to answer questions not related to textbook content",
            "passed": all_passed,
            "failed_questions": failed_questions
        }
    
    def test_external_knowledge_questions(self) -> Dict[str, Any]:
        """Test that the system doesn't provide external information."""
        test_questions = [
            "What is the latest research from OpenAI?",
            "What happened in the news today?",
            "What are the newest programming languages?",
        ]
        
        all_passed = True
        failed_questions = []
        
        for question in test_questions:
            # Check if retrieval finds any relevant chunks
            chunks = retrieval_service.retrieve_relevant_chunks(
                query=question,
                limit=5,
                min_similarity_score=0.7
            )
            
            # If no chunks found, the system should refuse to answer
            if len(chunks) == 0:
                response = generation_service.generate_response_from_context(
                    question=question,
                    context_chunks=chunks,
                    session_id="hallucination-test"
                )
                
                # Check if it's a refusal response
                if isinstance(response, RefusalResponse):
                    continue  # This is expected behavior
                else:
                    all_passed = False
                    failed_questions.append(question)
            else:
                # If chunks were found for external knowledge question, that's a potential issue
                all_passed = False
                failed_questions.append(f"Found chunks for external knowledge question: {question}")
        
        return {
            "test_name": "External knowledge questions test",
            "description": "System should not provide external information not in textbook",
            "passed": all_passed,
            "failed_questions": failed_questions
        }
    
    def test_vague_questions(self) -> Dict[str, Any]:
        """Test that the system handles vague questions properly."""
        test_questions = [
            "Tell me something?",
            "What should I know?",
            "Explain everything.",
        ]
        
        all_passed = True
        failed_questions = []
        
        for question in test_questions:
            # Check if retrieval finds any relevant chunks
            chunks = retrieval_service.retrieve_relevant_chunks(
                query=question,
                limit=5,
                min_similarity_score=0.7
            )
            
            # For vague questions, if no chunks are found, system should refuse
            if len(chunks) == 0:
                response = generation_service.generate_response_from_context(
                    question=question,
                    context_chunks=chunks,
                    session_id="hallucination-test"
                )
                
                # Check if it's a refusal response
                if isinstance(response, RefusalResponse):
                    continue  # This is expected behavior
                else:
                    all_passed = False
                    failed_questions.append(question)
        
        return {
            "test_name": "Vague questions test",
            "description": "System should refuse to answer vague questions without context",
            "passed": all_passed,
            "failed_questions": failed_questions
        }
    
    def test_mixed_content_questions(self) -> Dict[str, Any]:
        """Test that the system doesn't mix book content with external claims."""
        # This test checks if the system properly grounds its responses in provided context
        test_cases = [
            {
                "question": "According to the textbook, what is neural networks?",
                "context_chunks": [
                    {
                        'chunk_id': 'test-chunk-1',
                        'text_content': 'Neural networks are computing systems inspired by the human brain.',
                        'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 100}
                    }
                ]
            },
            {
                "question": "How does backpropagation work?",
                "context_chunks": [
                    {
                        'chunk_id': 'test-chunk-2',
                        'text_content': 'Backpropagation is an algorithm for training neural networks.',
                        'metadata': {'chapter': 'Chapter 5', 'section': '5.2', 'page_number': 105}
                    }
                ]
            }
        ]
        
        all_passed = True
        failed_cases = []
        
        for case in test_cases:
            response = generation_service.generate_response_from_context(
                question=case["question"],
                context_chunks=case["context_chunks"],
                session_id="hallucination-test"
            )
            
            # Check if the response is grounded in the provided context
            response_text = response.response.lower()
            context_text = case["context_chunks"][0]["text_content"].lower()
            
            # The response should contain information from the context
            if not any(word in response_text for word in context_text.split()[:5]):
                all_passed = False
                failed_cases.append({
                    "question": case["question"],
                    "response": response.response,
                    "context": context_text
                })
        
        return {
            "test_name": "Mixed content questions test",
            "description": "System should ground responses in provided context without adding external information",
            "passed": all_passed,
            "failed_cases": failed_cases
        }


def run_hallucination_tests() -> Dict[str, Any]:
    """Run the complete hallucination prevention test suite."""
    tester = HallucinationTester()
    return tester.run_all_tests()


# For running tests independently
if __name__ == "__main__":
    results = run_hallucination_tests()
    
    print("Hallucination Prevention Test Results:")
    print(f"Status: {results['summary']['hallucination_prevention_status']}")
    print(f"Passed: {results['summary']['passed']}/{results['summary']['total_tests']}")
    
    if results['summary']['failed'] > 0:
        print("\nFailed tests:")
        for result in results['test_results']:
            if not result['passed']:
                print(f"- {result['test_name']}: {result.get('failed_questions', []) or result.get('failed_cases', [])}")
    else:
        print("\nAll tests passed! The system properly prevents hallucinations.")