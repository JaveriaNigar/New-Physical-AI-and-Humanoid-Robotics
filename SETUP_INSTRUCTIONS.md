# Qwen Agent Setup Instructions

Follow these steps to set up the Qwen agent with the Qdrant cluster and textbook content:

## Prerequisites

1. **Qdrant Account**: Create an account at [Qdrant Cloud](https://cloud.qdrant.io/)
2. **Google AI API Key**: Get an API key from [Google AI Studio](https://aistudio.google.com/)

## Step 1: Configure Environment Variables

Update the `.env` file in the `backend` directory with your actual credentials:

```bash
# QDRANT CONFIGURATION - REQUIRED FOR AGENT SETUP
# 1. Go to https://cloud.qdrant.io/ to create your account and cluster
# 2. Get your cluster URL and API key from the dashboard
# 3. Replace the placeholder values below with your actual credentials
QDRANT_URL=your-actual-qdrant-cluster-url
QDRANT_API_KEY=your-actual-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook-content

# GOOGLE AI CONFIGURATION - REQUIRED FOR AGENT SETUP
# 1. Go to https://aistudio.google.com/ to get your API key
# 2. Replace the placeholder value below with your actual API key
GEMINI_API_KEY=your-gemini-api-key-here
GEMINI_MODEL_NAME=models/gemini-2.5-flash
```

## Step 2: Populate Qdrant with Textbook Content

Run the populate script to generate embeddings for all book content:

```bash
cd backend
python populate_textbook_qdrant.py
```

This will:
- Read all markdown files from the `physical-ai-book/docs` directory
- Generate embeddings for each content chunk
- Insert the embeddings into your Qdrant cluster

## Step 3: Start the Backend Server

Once the population is complete, start the backend server:

```bash
cd backend
python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Step 4: Test the Agent

You can test the agent using the test script:

```bash
python test_agent.py
```

Or make direct requests to the `/chat/ask` endpoint:

```bash
curl -X POST http://127.0.0.1:8000/chat/ask \
  -H "Content-Type": "application/json" \
  -d '{"question": "What is Physical AI?"}'
```

## Agent Behavior

- The agent will only retrieve content from the textbook
- If a question is not covered in the textbook, it will respond with: "⚠️ This question is not covered in the textbook content."
- The agent handles errors gracefully and avoids crashes

## Troubleshooting

1. **Connection Issues**: Verify your Qdrant URL and API key are correct
2. **Empty Responses**: Make sure the populate script completed successfully
3. **API Limits**: If you get quota exceeded errors, check your Google AI API usage limits