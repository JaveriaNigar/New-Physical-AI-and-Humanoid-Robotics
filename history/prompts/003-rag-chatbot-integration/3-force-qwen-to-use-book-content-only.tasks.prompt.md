---
id: 3
title: Force Qwen to Use Book Content Only
stage: tasks
date: "2025-12-22"
model: "Qwen"
feature: "003-rag-chatbot-integration"
branch: "main"
user: "M.R Computers"
command: "/sp.implement"
labels: ["frontend", "backend", "qwen", "book-content", "selected-text"]
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
  - "Test that AskAIButton no longer stores selected text in localStorage"
  - "Test that ChatModal sends only question to backend"
  - "Test that backend ignores selectedText parameter and uses book content"
  - "Test that Qwen responses are always from book content"
---

# Force Qwen to Use Book Content Only

## Prompt Text

```
$ARGUMENTS
```

## Response Text

This implementation disables the selected text feature and forces Qwen to use book content only. The changes made include:

1. Frontend - AskAIButton.js:
   - Removed code that stores selected text in localStorage
   - The button now only opens the chat modal without storing any selected text

2. Frontend - ChatModal.tsx:
   - Removed code that reads selected text from localStorage on modal open
   - Modified handleSend to send only the question to the backend, without selectedText
   - Removed UI elements related to selected text display
   - Updated placeholder text to "Ask Anything..."

3. Backend - chat_routes.py:
   - Modified the /ask endpoint to ignore the selectedText parameter completely
   - Always proceed with normal retrieval from book content only
   - Removed logic that would process selected text

4. Backend - generation_service.py:
   - Modified generate_response_from_selected_text to redirect to book content retrieval
   - This ensures that even if selected text is somehow provided, it will still use book content

## Outcome

The implementation now ensures that:
- All questions are answered using only the book content
- The selected text feature is completely disabled
- No empty or partial selections interfere with the request
- Answers are always relevant to the book content
- The "Ask AI" button behavior is simplified
- The system works without breaking any existing RAG or Qwen backend logic