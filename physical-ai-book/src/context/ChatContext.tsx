import React, { createContext, useContext, useReducer, ReactNode } from 'react';
import { ChatState, ChatMethods, ChatMessage } from '../types/chat';

// Define the context type
interface ChatContextType extends ChatState, ChatMethods {}

// Create the context
const ChatContext = createContext<ChatContextType | undefined>(undefined);

// Action types for the reducer
type ChatAction =
  | { type: 'ADD_MESSAGE'; payload: Omit<ChatMessage, 'id' | 'timestamp'> }
  | { type: 'OPEN_CHAT' }
  | { type: 'CLOSE_CHAT' }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'SET_LOADING'; payload: boolean };

// Initial state
const initialState: ChatState = {
  messages: [],
  isChatOpen: false,
  isLoading: false,
};

// Reducer function
const chatReducer = (state: ChatState, action: ChatAction): ChatState => {
  switch (action.type) {
    case 'ADD_MESSAGE':
      const newMessage: ChatMessage = {
        ...action.payload,
        id: Date.now().toString(),
        timestamp: new Date(),
      };
      return {
        ...state,
        messages: [...state.messages, newMessage],
      };
    case 'OPEN_CHAT':
      return {
        ...state,
        isChatOpen: true,
      };
    case 'CLOSE_CHAT':
      return {
        ...state,
        isChatOpen: false,
      };
    case 'CLEAR_MESSAGES':
      return {
        ...state,
        messages: [],
      };
    case 'SET_LOADING':
      return {
        ...state,
        isLoading: action.payload,
      };
    default:
      return state;
  }
};

// Provider component
interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  const addMessage = (message: Omit<ChatMessage, 'id' | 'timestamp'>) => {
    dispatch({ type: 'ADD_MESSAGE', payload: message });
  };

  const openChat = () => {
    dispatch({ type: 'OPEN_CHAT' });
  };

  const closeChat = () => {
    dispatch({ type: 'CLOSE_CHAT' });
  };

  const clearMessages = () => {
    dispatch({ type: 'CLEAR_MESSAGES' });
  };

  const setIsLoading = (loading: boolean) => {
    dispatch({ type: 'SET_LOADING', payload: loading });
  };

  const value = {
    ...state,
    addMessage,
    openChat,
    closeChat,
    clearMessages,
    setIsLoading,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
};

// Custom hook to use the chat context
export const useChat = (): ChatContextType => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
};