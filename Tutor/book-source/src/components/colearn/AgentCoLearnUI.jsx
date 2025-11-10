import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import SidebarChapters from './SidebarChapters';
import ChatSessions from './ChatSessions';
import TutorChatWindow from './TutorChatWindow';
import QuizComponent from './QuizComponent';
import lessonController from '../../utils/LessonController';
import * as storage from '../../utils/localStorageService';

/**
 * Generate unique session ID
 */
const generateSessionId = () => {
  return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
};

/**
 * Main Co-Learning UI
 * Chat-focused learning mode with sidebar chapter navigation
 */
const AgentCoLearnUI = () => {
  const [initialized, setInitialized] = useState(false);
  const [currentChapter, setCurrentChapter] = useState(1);
  const [showQuiz, setShowQuiz] = useState(false);
  const [showWelcomeModal, setShowWelcomeModal] = useState(false);
  const [selectedLanguage, setSelectedLanguage] = useState('en');
  const [isLoading, setIsLoading] = useState(true);
  const [currentSessionId, setCurrentSessionId] = useState(() => generateSessionId());

  useEffect(() => {
    checkFirstTime();
  }, []);

  const checkFirstTime = () => {
    const progress = storage.getProgress();
    const language = storage.getLanguage();

    if (!progress.lastTimestamp) {
      // First time user
      setShowWelcomeModal(true);
    } else {
      // Returning user
      setCurrentChapter(progress.currentChapter);
      setSelectedLanguage(language);
      initializeLearning(progress.currentChapter, language);
    }

    setIsLoading(false);
  };

  const initializeLearning = async (chapter = 1, language = 'en') => {
    try {
      await lessonController.startChapter(chapter, language);
      setInitialized(true);
    } catch (error) {
      console.error('Error initializing learning:', error);
    }
  };

  const handleWelcomeSubmit = async (startChapter, language) => {
    setSelectedLanguage(language);
    setCurrentChapter(startChapter);
    storage.saveLanguage(language);

    setShowWelcomeModal(false);
    await initializeLearning(startChapter, language);
  };

  const handleChapterSelect = async (chapterNumber) => {
    setCurrentChapter(chapterNumber);
    setShowQuiz(false);
    await lessonController.startChapter(chapterNumber);
  };

  const handleQuizRequest = () => {
    setShowQuiz(true);
  };

  const handleQuizComplete = (result) => {
    setShowQuiz(false);

    // If passed, offer to move to next chapter
    if (result.percentage >= 50) {
      const nextChapter = currentChapter + 1;
      if (confirm(`Great job! Move to Chapter ${nextChapter}?`)) {
        handleChapterSelect(nextChapter);
      }
    }
  };

  const handleNewSession = () => {
    const newSessionId = generateSessionId();
    setCurrentSessionId(newSessionId);
  };

  const handleSessionChange = (sessionId) => {
    setCurrentSessionId(sessionId);
  };

  if (isLoading) {
    return (
      <div className="colearn-loading">
        <div className="loader-spinner"></div>
        <p>Loading Co-Learning Tutor...</p>
      </div>
    );
  }

  return (
    <div className="colearn-container">
      {/* Chat Sessions Sidebar */}
      <ChatSessions
        currentSessionId={currentSessionId}
        onSessionChange={handleSessionChange}
        onNewSession={handleNewSession}
      />

      {/* Chapter Navigation Sidebar */}
      <SidebarChapters
        currentChapter={currentChapter}
        onChapterSelect={handleChapterSelect}
      />

      {/* Main Content */}
      <div className="colearn-main">
        {showQuiz ? (
          <QuizComponent
            chapterNumber={currentChapter}
            onComplete={handleQuizComplete}
            onClose={() => setShowQuiz(false)}
          />
        ) : (
          <TutorChatWindow
            sessionId={currentSessionId}
            onQuizRequest={handleQuizRequest}
            isDocked={true}
          />
        )}
      </div>

      {/* Welcome Modal */}
      <AnimatePresence>
        {showWelcomeModal && (
          <WelcomeModal
            onSubmit={handleWelcomeSubmit}
            onClose={() => setShowWelcomeModal(false)}
          />
        )}
      </AnimatePresence>
    </div>
  );
};

/**
 * Welcome Modal for first-time users
 */
const WelcomeModal = ({ onSubmit, onClose }) => {
  const [selectedChapter, setSelectedChapter] = useState(1);
  const [selectedLanguage, setSelectedLanguage] = useState('en');

  const languages = [
    { code: 'en', name: 'English', flag: '🇬🇧' },
    { code: 'roman_ur', name: 'Roman Urdu', flag: '🇵🇰' },
    { code: 'es', name: 'Spanish', flag: '🇪🇸' }
  ];

  const handleStart = () => {
    onSubmit(selectedChapter, selectedLanguage);
  };

  return (
    <motion.div
      className="modal-overlay"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      onClick={onClose}
    >
      <motion.div
        className="modal-content welcome-modal"
        initial={{ scale: 0.8, opacity: 0, y: 50 }}
        animate={{ scale: 1, opacity: 1, y: 0 }}
        exit={{ scale: 0.8, opacity: 0, y: 50 }}
        transition={{ type: 'spring', stiffness: 300, damping: 25 }}
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div className="modal-header">
          <h1>👋 Welcome to Co-Learning AI!</h1>
          <p className="modal-subtitle">
            I'm your AI Tutor, and I'll guide you through the entire AI-Native Development course step-by-step!
          </p>
        </div>

        {/* Body */}
        <div className="modal-body">
          {/* Chapter Selection */}
          <div className="form-group">
            <label className="form-label">📚 Where would you like to start?</label>
            <div className="chapter-options">
              <motion.button
                className={`chapter-option ${selectedChapter === 1 ? 'selected' : ''}`}
                onClick={() => setSelectedChapter(1)}
                whileHover={{ scale: 1.02 }}
                whileTap={{ scale: 0.98 }}
              >
                <div className="option-icon">🎯</div>
                <div className="option-content">
                  <strong>Start from Chapter 1</strong>
                  <p>Recommended for beginners</p>
                </div>
              </motion.button>

              <motion.button
                className={`chapter-option ${selectedChapter !== 1 ? 'selected' : ''}`}
                onClick={() => setSelectedChapter(2)}
                whileHover={{ scale: 1.02 }}
                whileTap={{ scale: 0.98 }}
              >
                <div className="option-icon">🚀</div>
                <div className="option-content">
                  <strong>Jump to a specific chapter</strong>
                  <p>If you have some background</p>
                  {selectedChapter !== 1 && (
                    <select
                      className="chapter-select"
                      value={selectedChapter}
                      onChange={(e) => setSelectedChapter(parseInt(e.target.value))}
                      onClick={(e) => e.stopPropagation()}
                    >
                      {[1, 2, 3, 4, 5].map(ch => (
                        <option key={ch} value={ch}>Chapter {ch}</option>
                      ))}
                    </select>
                  )}
                </div>
              </motion.button>
            </div>
          </div>

          {/* Language Selection */}
          <div className="form-group">
            <label className="form-label">🌍 Which language do you prefer?</label>
            <div className="language-options">
              {languages.map(lang => (
                <motion.button
                  key={lang.code}
                  className={`language-option ${selectedLanguage === lang.code ? 'selected' : ''}`}
                  onClick={() => setSelectedLanguage(lang.code)}
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.95 }}
                >
                  <span className="language-flag">{lang.flag}</span>
                  <span className="language-name">{lang.name}</span>
                </motion.button>
              ))}
            </div>
          </div>

          {/* Info Box */}
          <div className="info-box">
            <p><strong>What to expect:</strong></p>
            <ul>
              <li>📝 Short notes + clear explanations for each topic</li>
              <li>💡 Quick reflection questions to test understanding</li>
              <li>🎯 10-question quiz after each chapter</li>
              <li>🧠 Adaptive learning that adjusts to your pace</li>
              <li>⚡ Professional yet fun teaching style</li>
            </ul>
          </div>
        </div>

        {/* Footer */}
        <div className="modal-footer">
          <motion.button
            className="btn-primary btn-large"
            onClick={handleStart}
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            🚀 Let's Start Learning!
          </motion.button>
        </div>
      </motion.div>
    </motion.div>
  );
};

export default AgentCoLearnUI;
