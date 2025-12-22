@echo off
REM Startup script for RAG Chatbot backend

echo Starting RAG Chatbot backend...

REM Activate virtual environment
call venv\Scripts\activate.bat

REM Install any missing dependencies
pip install -r requirements.txt

REM Run the backend server
uvicorn src.main:app --reload --port 8001

echo Backend server started on port 8001
pause