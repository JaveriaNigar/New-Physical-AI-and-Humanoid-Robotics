---
id: 003
title: qwen-rag-agent-textbook-fix-complete
stage: green
date: 2025-12-20
surface: agent
model: gpt-4o
feature: 003-rag-chatbot-integration
branch: 003-rag-chatbot-integration
user: hackathon-participant
command: /sp.implement
labels:
  - backend
  - rag
  - textbook
  - qwen
  - fix
  - api
  - generation
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\src\api\chat_routes.py
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\src\rag.py
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\test_rag_functionality.py
tests:
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\test_rag_functionality.py
prompt_text: |
  /sp.implement /sp.implement
  title: Qwen RAG Agent – Textbook Answer Handling Fix
  desc: Fix Qwen agent responses so it strictly uses textbook content but gives clear, structured answers instead of fallback messages.

  steps:
    1. Verify RAG setup
       - Ensure the Qdrant collection is correctly set in `.env`:
         ```
         QDRANT_COLLECTION_NAME=textbook-content
         QDRANT_URL=your-qdrant-cluster-url
         QDRANT_API_KEY=your-qdrant-api-key
         ```
       - Confirm the Neon Postgres metadata connection is working (NEON_DATABASE_URL).

    2. Update `/chat/ask` endpoint
       - In `src/api/chat_routes.py` (or wherever `/chat/ask` is implemented):
         ```python
         from sqlalchemy import text
         from src.database.postgres_client import postgres_client
         from src.rag import RAGHandler  # Your RAG logic

         rag_handler = RAGHandler()

         @app.post("/chat/ask")
         async def ask_question(payload: dict):
             question = payload.get("question", "").strip()
             if not question:
                 return {"answer": "❌ Please provide a valid question."}

             try:
                 answer = rag_handler.get_answer(question)
                 if not answer:
                     return {"answer": "⚠️ This question is not covered in the textbook content."}
                 return {"answer": answer}
             except Exception as e:
                 return {"answer": f"❌ Error processing the question: {str(e)}"}
         ```

    3. Update RAG logic to fallback gracefully
       - In `src/rag.py` or wherever the retrieval is implemented:
         ```python
         class RAGHandler:
             def __init__(self):
                 # Initialize vector DB, embeddings, etc.
                 pass

             def get_answer(self, question: str) -> str:
                 # Retrieve context from Qdrant
                 context = self.retrieve_context(question)
                 if not context:
                     return ""  # Will trigger fallback in API endpoint

                 # Generate answer from context using Gemini API
                 answer = self.generate_from_context(question, context)
                 return answer
         ```

    4. Test
       - Run backend:
         ```bash
         uvicorn src.main:app --reload --port 8001
         ```
       - Send a sample question from frontend:
         - If covered in textbook → returns proper answer.
         - If not covered → returns “⚠️ This question is not covered in the textbook content.”

  notes:
    - Keeps strict textbook-based answering but avoids generic fallback messages.
    - Make sure your Qdrant collection `textbook-content` has all the textbook pages.
    - Proper error handling ensures the agent doesn't loop or crash on invalid input.
response_text: |
  Successfully implemented the Qwen RAG Agent fix for textbook answer handling:

  1. Updated the `/chat/ask` endpoint in `src/api/chat_routes.py`:
     - Improved error messaging with clear indicators (❌, ⚠️)
     - Enhanced validation for empty and invalid questions
     - Better handling of cases where no context is found in the textbook
     - More informative error responses when exceptions occur

  2. Created a dedicated RAG handler module (`src/rag.py`):
     - Implements the RAGHandler class with clean separation of concerns
     - Handles retrieval and generation logic in a centralized way
     - Returns empty string when no context is found, triggering proper fallback responses

  3. Created comprehensive test script (`test_rag_functionality.py`):
     - Tests the RAG handler functionality
     - Verifies proper response handling for different scenarios
     - Confirms error handling works correctly

  The implementation ensures:
  - Strict textbook-based answering (no hallucinations)
  - Clear, structured responses instead of generic fallback messages
  - Proper handling when questions are not covered in the textbook content
  - Robust error handling to prevent loops or crashes
  - Consistent response format for frontend integration

  When questions are covered in the textbook, the system returns proper answers.
  When questions are not covered, it returns "⚠️ This question is not covered in the textbook content."
  When invalid input is provided, it returns appropriate error messages.
outcome: Successfully fixed the Qwen RAG agent's textbook answer handling to provide clear, structured responses while maintaining strict textbook-based answering.
evaluation: The implementation successfully addresses the requirements by providing clear, structured responses while maintaining strict textbook-based answering. The error handling is robust and prevents the agent from looping or crashing. All tests pass and the functionality works as expected.
---