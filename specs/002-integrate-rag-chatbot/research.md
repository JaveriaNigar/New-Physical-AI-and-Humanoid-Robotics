# Research Summary: Integrated RAG Chatbot for AI-Powered Textbook

## Decision: Content Preparation Strategy
**Rationale:** Optimal text chunking is critical for RAG performance. After researching various chunking strategies, semantic chunking proves most effective for textbook content as it preserves context and maintains conceptual coherence.

**Alternatives considered:**
- Fixed-length chunking: Simpler to implement but might break up related concepts
- Recursive splitting: More complex to tune, but preserves some context
- Semantic chunking: Best for maintaining meaning in educational content

## Decision: Embedding Model Selection
**Rationale:** Using Google's embedding models makes sense given the existing use of Gemini LLM. Consistency between embedding and generation models from the same provider ensures compatibility and potentially better performance.

**Alternatives considered:**
- OpenAI's text-embedding-ada-002: High quality but violates "No OpenAI APIs" constraint
- Hugging Face open-source models (e.g., all-MiniLM-L6-v2): Free but may not integrate as well with Gemini
- Google's embedding models: Compatible with Gemini, good performance, meets constraints

## Decision: Textbook Content Extraction
**Rationale:** Need to determine the best approach for extracting content from the existing textbook frontend. Since the frontend is already deployed, content likely exists in structured form.

**Alternatives considered:**
- Direct parsing of frontend files: May include UI elements that need filtering
- Accessing content from source files: Cleanest approach if available
- Web scraping the deployed site: Less reliable, may capture UI elements

## Decision: Qdrant Vector Database Configuration
**Rationale:** Qdrant is well-suited for semantic search in RAG applications. For the RAG use case, we need to configure it optimally for textbook content retrieval with appropriate similarity thresholds.

**Alternatives considered:**
- Pinecone: Commercial option but has more features
- Weaviate: Open-source alternative with good documentation
- Qdrant: Free tier available, good for this project, efficient for semantic search

## Decision: Retrieval Strategy
**Rationale:** Implementing a threshold-based retrieval system ensures quality while preventing hallucinations. Using cosine similarity with a minimum threshold (0.7) balances retrieval precision with coverage.

**Alternatives considered:**
- Top-k retrieval without threshold: Risk of retrieving poor matches
- Variable threshold based on query: More complex, harder to tune
- Fixed threshold (0.7): Provides good balance between precision and recall

## Decision: Selected-Text-Only Mode Implementation
**Rationale:** This mode requires bypassing the entire vector database system and using only the user-selected text. This ensures compliance with the constitutional principle of Context Fidelity.

**Alternatives considered:**
- Augmenting selected text with similar chunks: Violates the constitutional requirement
- Using selected text as the sole context: Complies with requirements, simpler implementation
- Hybrid approach: More complex, violates constitutional requirements

## Decision: Gemini Safety Settings
**Rationale:** Need to configure Gemini's safety settings to prevent hallucinations while allowing educational responses. The system prompt should reinforce the requirement to only use provided context.

**Alternatives considered:**
- Default safety settings: May not enforce content-only responses strongly enough
- Maximum safety: Might block valid responses
- Custom safety + system prompt: Best approach for enforcing requirements