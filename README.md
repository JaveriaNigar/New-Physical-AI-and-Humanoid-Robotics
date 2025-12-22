# Physical AI Book Chatbot

This project integrates a Docusaurus-based frontend with a FastAPI backend to provide an AI assistant for the Physical AI & Humanoid Robotics textbook.

## Project Structure

- `physical-ai-book/` - Docusaurus frontend
- `backend/` - FastAPI backend

## Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   # On Windows:
   venv\Scripts\activate
   # On macOS/Linux:
   source venv/bin/activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Run the backend server:
   ```bash
   uvicorn src.main:app --reload
   ```

The backend will be available at `http://127.0.0.1:8000` with API documentation at `http://127.0.0.1:8000/docs`.

## Frontend Setup

1. Navigate to the physical-ai-book directory:
   ```bash
   cd physical-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run the development server:
   ```bash
   npm start
   ```

The frontend will be available at `http://localhost:3000`.

## Functionality

The AI chatbot provides two modes of interaction:

1. **General Questions**: Ask questions about the textbook content, which will be answered based on the indexed content.

2. **Selected Text Questions**: Select text on the page and ask questions specifically about that text, which will be answered using only the selected content.

The floating chatbot button in the bottom-right corner opens the chat interface where users can interact with the AI assistant.

## API Endpoints Used

- `/chat/query` - For general questions about textbook content
- `/chat/selected-text` - For questions about user-selected text

## Environment Variables

The backend uses the following environment variables (also defined in the `.env` file):

- `ALLOWED_ORIGINS` - Comma-separated list of allowed origins for CORS
- `HOST` - Host for the backend server
- `PORT` - Port for the backend server
- `RELOAD` - Whether to enable auto-reload during development

These are already configured in the `.env` file for development.