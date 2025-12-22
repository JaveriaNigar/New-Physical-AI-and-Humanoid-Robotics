import React, { useState, useEffect, useRef } from 'react';
import { FaRobot } from 'react-icons/fa';
import { useChat } from '../context/ChatContext';

const AskAIButton = () => {
  const { openChat } = useChat();
  const [position, setPosition] = useState({ top: 0, left: 0 });
  const [showButton, setShowButton] = useState(false);
  const buttonRef = useRef(null);

  useEffect(() => {
    // Remove text selection functionality
    // This button will now just open the chat modal without storing selected text
    const handleSelection = () => {
      // Clear any previous selected text
      localStorage.removeItem('selectedText');

      const selection = window.getSelection();
      const text = selection.toString().trim();

      // Get the position of the selection
      if (selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setPosition({
          top: rect.top + window.pageYOffset - 40, // Position slightly above the selection
          left: rect.left + window.pageXOffset + (rect.width / 2) // Center horizontally
        });
      }

      if (text) {
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    // Listen for mouseup event to detect text selection
    window.addEventListener('mouseup', handleSelection);

    return () => {
      window.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleClick = () => {
    // Clear any selected text from localStorage before opening the chat
    localStorage.removeItem('selectedText');
    openChat(); // Open the chat modal
    setShowButton(false); // Hide the button after clicking
  };

  if (!showButton) {
    return null;
  }

  return (
    <button
      ref={buttonRef}
      onClick={handleClick}
      aria-label="Ask AI about selected text"
      title="Ask AI about selected text"
      style={{
        position: 'absolute',
        top: `${position.top}px`,
        left: `${position.left}px`,
        zIndex: 9999, // Required z-index
        transform: 'translateX(-50%)', // Center the button on the x-axis
        backgroundColor: '#4F46E5', // Primary color matching theme
        border: 'none',
        borderRadius: '50%',
        width: '40px',
        height: '40px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        cursor: 'pointer',
        boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
        transition: 'all 0.2s ease',
        outline: 'none'
      }}
      onMouseEnter={(e) => {
        e.target.style.backgroundColor = '#4338CA';
        e.target.style.transform = 'translateX(-50%) scale(1.1)';
      }}
      onMouseLeave={(e) => {
        e.target.style.backgroundColor = '#4F46E5';
        e.target.style.transform = 'translateX(-50%) scale(1)';
      }}
    >
      <FaRobot
        style={{
          color: 'white',
          fontSize: '18px'
        }}
      />
    </button>
  );
};

export default AskAIButton;