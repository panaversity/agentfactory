import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import './ChatSessions.css';

/**
 * Chat Sessions Sidebar - Manage multiple conversation sessions
 * Similar to Claude's sidebar with session management
 */
const ChatSessions = ({ currentSessionId, onSessionChange, onNewSession }) => {
  const [sessions, setSessions] = useState([]);
  const [isCollapsed, setIsCollapsed] = useState(false);

  useEffect(() => {
    loadSessions();
  }, []);

  const loadSessions = () => {
    try {
      const savedSessions = localStorage.getItem('colearn_sessions');
      if (savedSessions) {
        setSessions(JSON.parse(savedSessions));
      }
    } catch (e) {
      console.error('Error loading sessions:', e);
    }
  };

  const saveSessions = (updatedSessions) => {
    localStorage.setItem('colearn_sessions', JSON.stringify(updatedSessions));
    setSessions(updatedSessions);
  };

  const handleDeleteSession = (sessionId, e) => {
    e.stopPropagation();

    if (confirm('Delete this chat session?')) {
      // Delete session
      const updated = sessions.filter(s => s.id !== sessionId);
      saveSessions(updated);

      // Delete session messages
      localStorage.removeItem(`colearn_session_${sessionId}`);

      // If deleting current session, switch to first available or create new
      if (sessionId === currentSessionId) {
        if (updated.length > 0) {
          onSessionChange(updated[0].id);
        } else {
          onNewSession();
        }
      }
    }
  };

  const handleRenameSession = (sessionId, e) => {
    e.stopPropagation();

    const session = sessions.find(s => s.id === sessionId);
    const newTitle = prompt('Rename session:', session.title);

    if (newTitle && newTitle.trim()) {
      const updated = sessions.map(s =>
        s.id === sessionId ? { ...s, title: newTitle.trim() } : s
      );
      saveSessions(updated);
    }
  };

  const formatTimestamp = (timestamp) => {
    const date = new Date(timestamp);
    const now = new Date();
    const diffMs = now - date;
    const diffMins = Math.floor(diffMs / 60000);
    const diffHours = Math.floor(diffMs / 3600000);
    const diffDays = Math.floor(diffMs / 86400000);

    if (diffMins < 1) return 'Just now';
    if (diffMins < 60) return `${diffMins}m ago`;
    if (diffHours < 24) return `${diffHours}h ago`;
    if (diffDays < 7) return `${diffDays}d ago`;
    return date.toLocaleDateString();
  };

  return (
    <div className={`chat-sessions-sidebar ${isCollapsed ? 'collapsed' : ''}`}>
      {/* Header */}
      <div className="sessions-header">
        <div className="sessions-header-content">
          <h3>Chats</h3>
          <button
            className="btn-collapse"
            onClick={() => setIsCollapsed(!isCollapsed)}
            title={isCollapsed ? 'Expand' : 'Collapse'}
          >
            {isCollapsed ? '→' : '←'}
          </button>
        </div>

        {!isCollapsed && (
          <button
            className="btn-new-chat"
            onClick={onNewSession}
            title="Start new chat"
          >
            <span className="btn-icon">+</span>
            <span className="btn-text">New Chat</span>
          </button>
        )}
      </div>

      {/* Sessions List */}
      {!isCollapsed && (
        <div className="sessions-list">
          <AnimatePresence>
            {sessions.map((session) => (
              <motion.div
                key={session.id}
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                exit={{ opacity: 0, x: -20 }}
                className={`session-item ${session.id === currentSessionId ? 'active' : ''}`}
                onClick={() => onSessionChange(session.id)}
              >
                <div className="session-content">
                  <div className="session-title">{session.title}</div>
                  <div className="session-meta">
                    <span className="session-time">{formatTimestamp(session.lastActivity)}</span>
                    {session.messageCount && (
                      <span className="session-count">{session.messageCount} msgs</span>
                    )}
                  </div>
                </div>

                <div className="session-actions">
                  <button
                    className="btn-action"
                    onClick={(e) => handleRenameSession(session.id, e)}
                    title="Rename"
                  >
                    ✏️
                  </button>
                  <button
                    className="btn-action btn-delete"
                    onClick={(e) => handleDeleteSession(session.id, e)}
                    title="Delete"
                  >
                    🗑️
                  </button>
                </div>
              </motion.div>
            ))}
          </AnimatePresence>

          {sessions.length === 0 && (
            <div className="sessions-empty">
              <p>No chat sessions yet</p>
              <p className="sessions-empty-hint">Click "New Chat" to start</p>
            </div>
          )}
        </div>
      )}

      {/* Collapsed View */}
      {isCollapsed && (
        <div className="sessions-collapsed">
          <button
            className="btn-new-chat-collapsed"
            onClick={onNewSession}
            title="New chat"
          >
            +
          </button>

          {sessions.map((session) => (
            <button
              key={session.id}
              className={`session-collapsed ${session.id === currentSessionId ? 'active' : ''}`}
              onClick={() => onSessionChange(session.id)}
              title={session.title}
            >
              💬
            </button>
          ))}
        </div>
      )}
    </div>
  );
};

export default ChatSessions;
