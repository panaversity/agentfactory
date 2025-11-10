import React, { useState, useEffect, useRef } from 'react';
import { Rnd } from 'react-rnd';
import { motion, AnimatePresence } from 'framer-motion';
import ReactMarkdown from 'react-markdown';
import lessonController from '../../utils/LessonController';

/**
 * Tutor Chat Window - Main teaching interface with session support
 */
const TutorChatWindow = ({ onClose, onQuizRequest, isFloating = false, sessionId }) => {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isDocked, setIsDocked] = useState(!isFloating);
  const [position, setPosition] = useState({ x: 100, y: 100 });
  const [size, setSize] = useState({ width: 600, height: 700 });
  const [initialized, setInitialized] = useState(false);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Load messages for current session
  useEffect(() => {
    loadSessionMessages();
    inputRef.current?.focus();
  }, [sessionId]);

  const loadSessionMessages = async () => {
    try {
      const sessionKey = `colearn_session_${sessionId}`;
      const savedMessages = localStorage.getItem(sessionKey);

      if (savedMessages) {
        setMessages(JSON.parse(savedMessages));
        setInitialized(true);
      } else {
        // New session - initialize with greeting
        await initializeChat();
      }
    } catch (e) {
      console.error('Error loading session messages:', e);
      await initializeChat();
    }
  };

  const initializeChat = async () => {
    if (initialized) return; // Prevent re-initialization

    setIsLoading(true);
    try {
      // Send "hello" to backend to trigger greeting
      const response = await lessonController.sendGreeting();

      // Add greeting message from backend
      if (response.success) {
        addMessage('tutor', response.message);
      }
      setInitialized(true);
    } catch (error) {
      console.error('Error initializing chat:', error);
      addMessage('tutor', 'Hey! Ready to learn some AI-native development? Which chapter are you interested in?');
      setInitialized(true);
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    // Scroll to bottom when new messages arrive
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    // Save messages to session-specific storage
    if (messages.length > 0 && sessionId) {
      const sessionKey = `colearn_session_${sessionId}`;
      localStorage.setItem(sessionKey, JSON.stringify(messages.slice(-50)));

      // Update session metadata
      updateSessionMetadata();
    }
  }, [messages, sessionId]);

  const updateSessionMetadata = () => {
    try {
      const sessionsKey = 'colearn_sessions';
      const savedSessions = localStorage.getItem(sessionsKey);
      let sessions = savedSessions ? JSON.parse(savedSessions) : [];

      // Find current session
      const sessionIndex = sessions.findIndex(s => s.id === sessionId);

      // Generate title from first user message
      const firstUserMsg = messages.find(m => m.role === 'user');
      const title = firstUserMsg
        ? firstUserMsg.content.substring(0, 40) + (firstUserMsg.content.length > 40 ? '...' : '')
        : 'New Chat';

      const sessionData = {
        id: sessionId,
        title,
        lastActivity: new Date().toISOString(),
        messageCount: messages.length
      };

      if (sessionIndex >= 0) {
        sessions[sessionIndex] = sessionData;
      } else {
        sessions.unshift(sessionData); // Add to beginning
      }

      localStorage.setItem(sessionsKey, JSON.stringify(sessions));
    } catch (e) {
      console.error('Error updating session metadata:', e);
    }
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const addMessage = (role, content, metadata = {}) => {
    const message = {
      id: Date.now().toString() + Math.random(),
      role,
      content,
      timestamp: new Date().toISOString(),
      ...metadata
    };
    setMessages(prev => [...prev, message]);
    return message;
  };

  const handleSendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = inputMessage.trim();
    setInputMessage('');

    // Add user message
    addMessage('user', userMessage);

    // Show loading
    setIsLoading(true);

    try {
      // Process with lesson controller
      const response = await lessonController.processStudentAnswer(userMessage);

      // Add tutor response
      addMessage('tutor', response.message, {
        adaptiveMode: response.adaptiveMode,
        needsClarification: response.needsClarification,
        correct: response.correct
      });

      // Handle special cases
      if (response.showQuiz) {
        if (onQuizRequest) {
          onQuizRequest();
        }
      }
    } catch (error) {
      console.error('Error processing message:', error);
      addMessage('tutor', 'Sorry, I encountered an error. Please try again!', {
        error: true
      });
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleDock = () => {
    setIsDocked(!isDocked);
  };

  const renderChatContent = () => (
    <div className="tutor-chat-content">
      {/* Header */}
      <div className="chat-header">
        <div className="chat-header-info">
          <div className="tutor-avatar">🤖</div>
          <div className="tutor-info">
            <h3 className="tutor-name">AI Tutor</h3>
            <p className="tutor-status">
              {isLoading ? 'Thinking...' : 'Online'}
            </p>
          </div>
        </div>

        <div className="chat-header-actions">
          {/* Dock/Float toggle */}
          <button
            className="chat-action-btn"
            onClick={toggleDock}
            title={isDocked ? 'Float window' : 'Dock window'}
          >
            {isDocked ? '🔲' : '📌'}
          </button>

          {/* Close button */}
          {onClose && (
            <button
              className="chat-action-btn close"
              onClick={onClose}
              title="Close chat"
            >
              ✕
            </button>
          )}
        </div>
      </div>

      {/* Messages */}
      <div className="chat-messages">
        <AnimatePresence>
          {messages.map((message, index) => (
            <motion.div
              key={message.id}
              className={`chat-message ${message.role}`}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              exit={{ opacity: 0, y: -20 }}
              transition={{ duration: 0.3 }}
            >
              {message.role === 'tutor' && (
                <div className="message-avatar">🤖</div>
              )}

              <div className="message-content">
                {message.adaptiveMode && (
                  <div className="message-badge adaptive">🧠 Simplified Mode</div>
                )}
                {message.correct && (
                  <div className="message-badge success">✅ Correct!</div>
                )}
                {message.needsClarification && (
                  <div className="message-badge warning">💡 Let's clarify</div>
                )}

                <div className="message-text">
                  <ReactMarkdown>{message.content}</ReactMarkdown>
                </div>

                <div className="message-time">
                  {new Date(message.timestamp).toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit'
                  })}
                </div>
              </div>

              {message.role === 'user' && (
                <div className="message-avatar user">👤</div>
              )}
            </motion.div>
          ))}
        </AnimatePresence>

        {/* Loading indicator */}
        {isLoading && (
          <motion.div
            className="chat-message tutor loading"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
          >
            <div className="message-avatar">🤖</div>
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </motion.div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className="chat-input-container">
        <div className="chat-input-wrapper">
          <textarea
            ref={inputRef}
            className="chat-input"
            value={inputMessage}
            onChange={(e) => setInputMessage(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your answer or question..."
            rows={2}
            disabled={isLoading}
          />

          <button
            className="chat-send-btn"
            onClick={handleSendMessage}
            disabled={!inputMessage.trim() || isLoading}
          >
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
              <path
                d="M22 2L11 13M22 2L15 22L11 13M22 2L2 8L11 13"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
          </button>
        </div>

        {/* Hints */}
        <div className="chat-hints">
          <span className="hint">💡 Press Enter to send, Shift+Enter for new line</span>
        </div>
      </div>
    </div>
  );

  // Render docked or floating
  if (isDocked) {
    return (
      <div className="tutor-chat-window docked">
        {renderChatContent()}
      </div>
    );
  }

  // Floating window
  return (
    <Rnd
      default={{
        x: position.x,
        y: position.y,
        width: size.width,
        height: size.height
      }}
      minWidth={400}
      minHeight={500}
      maxWidth={900}
      maxHeight={900}
      bounds="window"
      dragHandleClassName="chat-header"
      onDragStop={(e, d) => setPosition({ x: d.x, y: d.y })}
      onResizeStop={(e, direction, ref, delta, position) => {
        setSize({
          width: parseInt(ref.style.width),
          height: parseInt(ref.style.height)
        });
        setPosition(position);
      }}
      className="tutor-chat-window floating"
    >
      {renderChatContent()}
    </Rnd>
  );
};

export default TutorChatWindow;
