// Define the structure for a chat message
export interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
}

// Define the state for the chat context
export interface ChatState {
  messages: ChatMessage[];
  isChatOpen: boolean;
  isLoading: boolean;
}

// Define the methods for interacting with the chat
export interface ChatMethods {
  addMessage: (message: Omit<ChatMessage, 'id' | 'timestamp'>) => void;
  openChat: () => void;
  closeChat: () => void;
  clearMessages: () => void;
  setIsLoading: (loading: boolean) => void;
}