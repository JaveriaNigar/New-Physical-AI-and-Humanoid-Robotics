/**
 * Frontend-Backend Integration for RAG Chatbot
 * Connects the frontend chatbot interface to the backend RAG system
 */

class RAGChatbotIntegration {
    constructor(backendUrl = 'http://localhost:8001') {
        this.backendUrl = backendUrl;
        this.chatContainer = null;
        this.inputField = null;
        this.sendButton = null;
        this.initializeElements();
        this.attachEventListeners();
    }

    initializeElements() {
        // Common selectors for chatbot elements - adjust these based on your actual HTML structure
        this.chatContainer = document.querySelector('.chat-messages') || 
                            document.querySelector('#chat-messages') ||
                            document.querySelector('.messages-container') ||
                            document.querySelector('.chat-container');
                            
        this.inputField = document.querySelector('.chat-input') || 
                         document.querySelector('#chat-input') ||
                         document.querySelector('.message-input') ||
                         document.querySelector('input[type="text"]');
                         
        this.sendButton = document.querySelector('.send-button') || 
                         document.querySelector('#send-button') ||
                         document.querySelector('.send-btn') ||
                         document.querySelector('button[type="submit"]');
    }

    attachEventListeners() {
        // Handle send button click
        if (this.sendButton) {
            this.sendButton.addEventListener('click', (e) => {
                e.preventDefault();
                this.handleSendMessage();
            });
        }

        // Handle Enter key in input field
        if (this.inputField) {
            this.inputField.addEventListener('keypress', (e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    this.handleSendMessage();
                }
            });
        }
    }

    async handleSendMessage() {
        const question = this.inputField?.value?.trim();
        
        if (!question) {
            console.warn('No question entered');
            return;
        }

        // Add user message to chat
        this.addMessageToChat(question, 'user');
        
        // Clear input field
        this.inputField.value = '';
        
        // Show typing indicator
        const typingIndicator = this.addTypingIndicator();
        
        try {
            // Send question to backend
            const answer = await this.sendQuestionToBackend(question);
            
            // Remove typing indicator
            this.removeTypingIndicator(typingIndicator);
            
            // Add answer to chat
            this.addMessageToChat(answer, 'bot');
            
        } catch (error) {
            // Remove typing indicator
            this.removeTypingIndicator(typingIndicator);
            
            // Show error message
            this.addMessageToChat('Sorry, I encountered an error processing your request. Please try again.', 'bot');
            console.error('Error sending question to backend:', error);
        }
    }

    async sendQuestionToBackend(question) {
        try {
            const response = await fetch(`${this.backendUrl}/ask`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ question })
            });

            if (!response.ok) {
                if (response.status === 404) {
                    throw new Error('No relevant answer found in the textbook content.');
                } else {
                    throw new Error(`Backend returned error: ${response.status} ${response.statusText}`);
                }
            }

            const data = await response.json();
            
            if (data && typeof data.answer === 'string') {
                return data.answer;
            } else {
                throw new Error('Invalid response format from backend');
            }
        } catch (error) {
            console.error('Error communicating with backend:', error);
            throw error;
        }
    }

    addMessageToChat(text, sender) {
        if (!this.chatContainer) {
            console.error('Chat container not found');
            return;
        }

        const messageElement = document.createElement('div');
        messageElement.classList.add('message', `${sender}-message`);
        
        // Add styling if not already present
        messageElement.style.padding = '10px 15px';
        messageElement.style.margin = '5px 0';
        messageElement.style.borderRadius = '8px';
        messageElement.style.maxWidth = '80%';
        messageElement.style.wordWrap = 'break-word';
        
        if (sender === 'user') {
            messageElement.style.backgroundColor = '#007bff';
            messageElement.style.color = 'white';
            messageElement.style.alignSelf = 'flex-end';
            messageElement.style.marginLeft = 'auto';
        } else {
            messageElement.style.backgroundColor = '#f8f9fa';
            messageElement.style.color = 'black';
            messageElement.style.alignSelf = 'flex-start';
            messageElement.style.marginRight = 'auto';
        }

        messageElement.textContent = text;
        this.chatContainer.appendChild(messageElement);
        
        // Scroll to bottom of chat
        this.chatContainer.scrollTop = this.chatContainer.scrollHeight;
    }

    addTypingIndicator() {
        if (!this.chatContainer) return null;

        const typingElement = document.createElement('div');
        typingElement.classList.add('typing-indicator');
        typingElement.style.padding = '10px 15px';
        typingElement.style.margin = '5px 0';
        typingElement.style.borderRadius = '8px';
        typingElement.style.backgroundColor = '#f8f9fa';
        typingElement.style.color = 'black';
        typingElement.style.alignSelf = 'flex-start';
        typingElement.style.marginRight = 'auto';
        typingElement.style.maxWidth = '80%';
        typingElement.style.wordWrap = 'break-word';
        typingElement.innerHTML = '<em>Thinking...</em>';
        
        this.chatContainer.appendChild(typingElement);
        this.chatContainer.scrollTop = this.chatContainer.scrollHeight;
        
        return typingElement;
    }

    removeTypingIndicator(typingElement) {
        if (typingElement && typingElement.parentNode) {
            typingElement.parentNode.removeChild(typingElement);
        }
    }
}

// Initialize the integration when the page loads
document.addEventListener('DOMContentLoaded', function() {
    // Initialize with your backend URL
    const chatbotIntegration = new RAGChatbotIntegration('http://localhost:8001');
    
    // Make it globally accessible if needed for debugging
    window.ragChatbot = chatbotIntegration;
});

// Export for use in other modules if needed
if (typeof module !== 'undefined' && module.exports) {
    module.exports = RAGChatbotIntegration;
}