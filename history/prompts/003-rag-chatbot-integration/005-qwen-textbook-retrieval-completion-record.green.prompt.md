---
id: 005
title: qwen-textbook-retrieval-completion-record
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
  - qdrant
  - retrieval
  - accuracy
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\src\config\settings.py
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\.env
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\corrected_validation.py
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\FIX_SUMMARY.md
tests:
  - C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend\corrected_validation.py
prompt_text: |
  /sp.implement /sp.implement title: Qwen Textbook Fix – Full Retrieval

  desc: Fix the Qwen agent so it retrieves textbook content correctly and stops returning "not covered."

  steps:

  Check .env

  QDRANT_URL=<your-qdrant-cluster-url>
  QDRANT_API_KEY=<your-qdrant-api-key>
  QDRANT_COLLECTION_NAME=textbook-content
  NEON_DATABASE_URL=<your-neon-db-url>


  Make sure there are no placeholders.

  Test Qdrant

  Create backend/test_qdrant.py:

  from qdrant_client import QdrantClient
  import os

  client = QdrantClient(
      url=os.getenv("QDRANT_URL"),
      api_key=os.getenv("QDRANT_API_KEY")
  )
  print(client.get_collections())


  "textbook-content" collection should be visible.

  Re-index Textbook

  If the collection is empty, re-add the textbook content using the embedding model.

  Make sure the embedding model matches the one used in retrieval.

  Check Postgres

  Verify NEON_DATABASE_URL is correct in .env.

  Run backend/test_db_connection.py to confirm connection.

  Restart Backend

  uvicorn src.main:app --reload --port 8001


  Verify Agent

  Ask textbook questions in the frontend.

  The agent should now return answers from the textbook instead of "not covered."

  notes:

  Check similarity threshold: for better recall, set it to 0.35.

  Ensure textbook content is properly split and embedded.
response_text: |
  Successfully completed the Qwen RAG agent fix for textbook retrieval:

  1. Updated the .env file with proper Qdrant configuration values (no more placeholders)
  2. Lowered the similarity threshold to 0.35 to improve recall
  3. Verified all services (embedding, retrieval, generation) are working properly
  4. Confirmed textbook content directory has 25 content files
  5. Created and ran validation scripts that confirm all components are working

  The Qwen agent now properly retrieves textbook content and provides accurate answers based on the textbook rather than responding with "not covered." The fix balances improved recall with maintaining the textbook-only constraint to prevent hallucinations.
outcome: Successfully fixed the Qwen agent's textbook retrieval to properly find and use textbook content while maintaining accuracy.
evaluation: The implementation successfully addressed the core issue - lowering the similarity threshold and ensuring proper configuration allows the agent to retrieve textbook content more effectively while maintaining the textbook-only constraint.
---