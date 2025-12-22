"""
Mock backend server for the RAG Chatbot that simulates API responses without requiring actual API keys.
This is for demonstration purposes when actual backend dependencies aren't available.
"""
import os
import json
import time
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any, Union
import random
from datetime import datetime

# Define the same models as the real backend
class ChunkMetadata(BaseModel):
    chunk_id: str
    chapter: str
    section: str
    page_number: Optional[int] = None
    text_preview: Optional[str] = None

class ChatQueryRequest(BaseModel):
    question: str
    session_id: str
    context: Optional[Dict[str, Any]] = None

class SelectedTextQueryRequest(BaseModel):
    question: str
    selected_text: str
    session_id: str

class ChatQueryResponse(BaseModel):
    response: str
    confidence: float
    retrieved_chunks: List[ChunkMetadata]
    sources: List[str]
    session_id: str
    query_type: str = "general"

class RefusalResponse(BaseModel):
    message: str
    reason: str
    session_id: str

# Create FastAPI application instance
app = FastAPI(
    title="Mock RAG Chatbot API",
    description="Mock API for the Retrieval-Augmented Generation chatbot integrated with AI-powered textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for demo purposes
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mock data for responses
MOCK_CHUNKS = [
    ChunkMetadata(
        chunk_id="chunk_001",
        chapter="Introduction to Physical AI",
        section="1.1",
        page_number=15,
        text_preview="Physical AI combines principles of physics and artificial intelligence..."
    ),
    ChunkMetadata(
        chunk_id="chunk_002",
        chapter="Humanoid Robotics",
        section="3.2",
        page_number=89,
        text_preview="Humanoid robots are designed to mimic human appearance and behavior..."
    ),
    ChunkMetadata(
        chunk_id="chunk_003",
        chapter="Embodied Intelligence",
        section="5.4",
        page_number=156,
        text_preview="Embodied intelligence refers to the concept that intelligence emerges from the interaction..."
    )
]

def generate_mock_response(question: str) -> str:
    """Generate a mock response based on the question"""
    question_lower = question.lower()
    
    if "physical ai" in question_lower:
        return "Physical AI combines principles of physics and artificial intelligence to create systems that interact with the physical world. This involves understanding dynamics, control systems, and how agents can learn from physical interactions. According to the textbook, Physical AI is essential for creating robots that can operate effectively in real-world environments."
    elif "humanoid" in question_lower or "robot" in question_lower:
        return "Humanoid robots are designed to mimic human appearance and behavior. They require sophisticated control systems to achieve human-like movement and interaction. The textbook explains that humanoid robotics involves biomechanics, motor control, and cognitive systems to replicate human capabilities."
    elif "embodied" in question_lower or "intelligence" in question_lower:
        return "Embodied intelligence refers to the concept that intelligence emerges from the interaction between an agent and its environment. Rather than processing information in isolation, embodied agents learn and adapt through physical interaction with the world. This is fundamental to creating adaptive robotic systems."
    elif "control" in question_lower:
        return "Control systems in robotics involve algorithms that determine how a robot should move and act. These systems must account for dynamics, sensor feedback, and environmental interactions. The textbook covers various control approaches including PID controllers, model predictive control, and learning-based control methods."
    else:
        return f"Based on the textbook content, {question} relates to advanced concepts in Physical AI and Humanoid Robotics. The system retrieves relevant information from the textbook to provide accurate, grounded responses. For more specific details, please refer to the relevant chapters in the textbook."

@app.get("/health/status")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now(),
        "services": {
            "qdrant": "mocked",
            "postgres": "mocked", 
            "gemini": "mocked"
        }
    }

@app.post("/chat/query", response_model=Union[ChatQueryResponse, RefusalResponse])
async def chat_query(request: ChatQueryRequest):
    """Handle a chat query about textbook content."""
    try:
        # Simulate processing delay
        time.sleep(0.5)
        
        # Determine if we can answer the question
        question_lower = request.question.lower()
        can_answer = any(word in question_lower for word in ["physical ai", "humanoid", "robot", "control", "embodied", "intelligence"])
        
        if not can_answer:
            # Return a refusal response
            return RefusalResponse(
                message="I cannot answer this question as it's not covered in the provided textbook content.",
                reason="no_context_found",
                session_id=request.session_id
            )
        
        # Generate a mock response
        response_text = generate_mock_response(request.question)
        
        # Randomly select some chunks as "retrieved" (for demonstration)
        selected_chunks = random.sample(MOCK_CHUNKS, min(2, len(MOCK_CHUNKS)))
        
        return ChatQueryResponse(
            response=response_text,
            confidence=random.uniform(0.7, 0.95),
            retrieved_chunks=selected_chunks,
            sources=[f"Chapter {chunk.chapter}, Section {chunk.section}" for chunk in selected_chunks],
            session_id=request.session_id,
            query_type="general"
        )
    except Exception as e:
        print(f"Error processing chat query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="An internal error occurred while processing your request"
        )

@app.post("/chat/selected-text", response_model=Union[ChatQueryResponse, RefusalResponse])
async def chat_selected_text(request: SelectedTextQueryRequest):
    """Handle a chat query with user-selected text context."""
    try:
        # Simulate processing delay
        time.sleep(0.5)
        
        # Generate a response based on the selected text and question
        response_text = f"Based on the selected text: '{request.selected_text[:100]}...', the answer to your question '{request.question}' is that this content relates to concepts in Physical AI and Humanoid Robotics. The textbook explains that when specific text is selected, the AI focuses exclusively on that content to provide contextually relevant answers."
        
        return ChatQueryResponse(
            response=response_text,
            confidence=0.9,
            retrieved_chunks=[MOCK_CHUNKS[0]],  # Use the first chunk as a reference
            sources=["Selected text context"],
            session_id=request.session_id,
            query_type="selected_text"
        )
    except Exception as e:
        print(f"Error processing selected-text query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="An internal error occurred while processing your request"
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "mock_backend:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )