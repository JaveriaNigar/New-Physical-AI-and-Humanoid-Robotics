---
id: 1
title: Fix Qwen Selected Text Responses
stage: tasks
date: "2025-12-22"
model: "Qwen"
feature: "003-rag-chatbot-integration"
branch: "main"
user: "M.R Computers"
command: "/sp.implement"
labels: ["frontend", "backend", "qwen", "selected-text"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "physical-ai-book/src/components/AskAIButton.js"
  - "physical-ai-book/src/components/AIChatbot/ChatModal.tsx"
  - "backend/src/api/chat_routes.py"
  - "backend/src/services/generation_service.py"
tests:
  - "Test that selected text is properly captured and stored in localStorage"
  - "Test that ChatModal reads selected text on open"
  - "Test that selected text is sent to backend in request body"
  - "Test that backend handles empty or instruction-like selected text"
  - "Test that responses are always returned with proper JSON format"
---

# Fix Qwen Selected Text Responses

## Prompt Text

```
$ARGUMENTS
```

## Response Text

This implementation addressed the issue where Qwen was not generating answers when selected text was empty or looked like an instruction. The changes made include:

1. **Frontend - AskAIButton.js**:
   - Updated to properly capture and store selected text (including empty strings) in localStorage
   - Ensured old selections are cleared to avoid partial words
   - The selected text is now always stored regardless of content

2. **Frontend - ChatModal.tsx**:
   - Modified to read selected text on modal open
   - Updated to send selectedText in the request body to the backend
   - The selected text is now sent regardless of content

3. **Backend - chat_routes.py**:
   - Enhanced to safely handle selectedText parameter
   - Added logic to prepend fallback prompt when selectedText is empty or looks like an instruction
   - Ensured the backend always returns a valid JSON response to prevent crashes

4. **Backend - generation_service.py**:
   - Updated to properly process selected text even when it's empty or appears to be an instruction
   - Improved error handling to ensure responses are always returned

## Outcome

The implementation now ensures that:
- Full selected text is always stored and sent to the backend
- Old selections are cleared to avoid mixing content
- The backend never crashes on empty or instruction-like selected text
- Responses are always returned in the proper JSON format
- Qwen generates answers even when selected text is empty or looks like an instruction