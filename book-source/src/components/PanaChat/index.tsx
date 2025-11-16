import React, { useState, useRef, useEffect } from 'react';
import './panaChat.css';

interface AssistantProps {
  isOpen: boolean;
  onClose: () => void;
}

interface Message {
  id: string;
  text: string;
  isBot: boolean;
  timestamp: Date;
}

const Assistant: React.FC<AssistantProps> = ({ isOpen, onClose }) => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI Assistant. How can I help you today?',
      isBot: true,
      timestamp: new Date(),
    },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isMinimized, setIsMinimized] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && !isMinimized) {
      inputRef.current?.focus();
    }
  }, [isOpen, isMinimized]);

  const handleSendMessage = () => {
    if (!inputValue.trim()) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      isBot: false,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');

    // Simulate bot response
    setTimeout(() => {
      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Thank you for your message! I\'m here to help you with any questions about AI Native Software Development.',
        isBot: true,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, botMessage]);
    }, 1000);
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleSendMessage();
    }
  };

  const toggleMinimize = () => {
    setIsMinimized(!isMinimized);
  };

  if (!isOpen) return null;

  return (
    <div className={`pana-chat ${isMinimized ? 'pana-chat--minimized' : ''}`}>
      {/* Chat Header */}
      <div className="pana-chat__header">
        <div className="pana-chat__header-left">
          <div className="pana-chat__logo">
            <svg viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path
                d="M3 3L8 3L8 21L3 21L3 3Z"
                fill="white"
              />
              <path
                d="M11 3L16 3L16 21L11 21L11 3Z"
                fill="white"
                fillOpacity="0.7"
              />
              <path
                d="M19 3L21 3L21 21L19 21L19 3Z"
                fill="white"
                fillOpacity="0.5"
              />
            </svg>
          </div>
          <h3 className="pana-chat__title">Assistant</h3>
        </div>
        <div className="pana-chat__header-actions">
          <button
            className="pana-chat__header-btn"
            onClick={toggleMinimize}
            aria-label={isMinimized ? 'Maximize' : 'Minimize'}
            title={isMinimized ? 'Maximize' : 'Minimize'}
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path
                d="M19 13H5"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </button>
          <button
            className="pana-chat__header-btn"
            onClick={toggleMinimize}
            aria-label="Expand"
            title="Expand"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path
                d="M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </button>
          <button
            className="pana-chat__header-btn"
            onClick={onClose}
            aria-label="Close"
            title="Close"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
              <path
                d="M18 6L6 18M6 6l12 12"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </button>
        </div>
      </div>

      {/* Chat Body - Only visible when not minimized */}
      {!isMinimized && (
        <>
          <div className="pana-chat__body">
            <div className="pana-chat__messages">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`pana-chat__message ${
                    message.isBot ? 'pana-chat__message--bot' : 'pana-chat__message--user'
                  }`}
                >
                  <div className="pana-chat__message-text">{message.text}</div>
                </div>
              ))}
              <div ref={messagesEndRef} />
            </div>
          </div>

          {/* Chat Footer */}
          <div className="pana-chat__footer">
            <div className="pana-chat__input-wrapper">
              <input
                ref={inputRef}
                type="text"
                className="pana-chat__input"
                placeholder="Type your message..."
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
              />
              <button
                className="pana-chat__send-btn"
                onClick={handleSendMessage}
                disabled={!inputValue.trim()}
                aria-label="Send message"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
                  <path
                    d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              </button>
            </div>
          </div>
        </>
      )}
    </div>
  );
};

export default Assistant;
