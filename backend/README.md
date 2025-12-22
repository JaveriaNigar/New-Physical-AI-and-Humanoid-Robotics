# RAG Chatbot Backend

This is the backend service for the RAG (Retrieval-Augmented Generation) Chatbot that integrates with an AI-powered textbook. The backend handles chat queries, retrieves relevant textbook content, and generates responses using Google's Gemini AI.

## Features

- **CORS enabled** for frontend communication (allows connections from localhost:3000)
- **RAG functionality** - retrieves textbook content and generates answers
- **Security measures** - input validation, rate limiting, security headers
- **Monitoring** - performance metrics and logging
- **Error handling** - comprehensive error management

## Endpoints

- `POST /chat/ask` - Simple chat endpoint for frontend (returns JSON: `{"answer": "response"}`)
- `POST /chat/query` - Full chat endpoint with session management
- `POST /chat/selected-text` - Endpoint for queries with user-selected text context
- `GET /health` - Health check endpoint
- `GET /metrics` - Performance metrics endpoint

## Setup Instructions

### Prerequisites

- Python 3.11
- pip
- Virtual environment (recommended)

### Installation

1. **Clone the repository** (if not already done)

2. **Navigate to the backend directory**:
   ```bash
   cd C:\Users\M.R Computers\OneDrive\Desktop\hackathon2\backend
   ```

3. **Create and activate virtual environment** (if not already done):
   ```bash
   python -m venv venv
   venv\Scripts\activate  # On Windows
   # source venv/bin/activate  # On Linux/Mac
   ```

4. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

5. **Configure environment variables**:
   - Copy the `.env` template and update with your actual API keys:
   ```
   GEMINI_API_KEY=your-gemini-api-key-here
   QDRANT_API_KEY=your-qdrant-api-key-here
   QDRANT_URL=your-qdrant-cluster-url-here
   NEON_DATABASE_URL=your-neon-database-url-here
   ```

6. **Run the backend server**:
   ```bash
   uvicorn src.main:app --reload --port 8001
   ```

   Or use the startup script:
   ```bash
   start_backend.bat
   ```

## Frontend Integration

The backend is configured to accept requests from `http://localhost:3000` (typical React development server port). 

### Example frontend request:
```javascript
fetch('http://127.0.0.1:8001/chat/ask', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    question: 'What is artificial intelligence?'
  })
})
.then(response => response.json())
.then(data => console.log(data.answer));
```

## Available Scripts

- `start_backend.bat` - Start the backend server with proper configuration
- `test_backend.py` - Test script to verify the backend is responding correctly

## Troubleshooting

1. **CORS Issues**: Make sure your frontend is running on an allowed origin (localhost:3000 by default)
2. **Connection Refused**: Ensure the backend server is running on port 8001
3. **API Keys**: Verify all required API keys are set in the environment variables
4. **Dependencies**: Make sure all required packages are installed in your virtual environment

## Architecture

The backend implements a hallucination-free architecture that strictly answers from textbook content or user-selected text, using:
- FastAPI for the web framework
- Qdrant Cloud for vector storage
- Neon Serverless Postgres for metadata
- Google's Gemini API for response generation