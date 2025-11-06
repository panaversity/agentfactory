/**
 * Learning Hub Context
 * Global state management using React Context + useReducer
 */

import React, { createContext, useContext, useReducer, useEffect, type ReactNode } from 'react';
import type { LearningHubState, LearningHubAction, TabName } from '../types';
import { STORAGE_KEYS, LIMITS } from '../types';
import { storage } from '../services/storageService';
import { debounce } from '../utils/debounce';

// Initial state
const initialState: LearningHubState = {
  isOpen: false,
  activeTab: 'chat',
  chatHistory: [],
  savedHighlights: {},
  progressRecords: {},
  errorLog: [],
};

// Reducer function
function learningHubReducer(
  state: LearningHubState,
  action: LearningHubAction
): LearningHubState {
  switch (action.type) {
    case 'TOGGLE_SIDEBAR':
      return {
        ...state,
        isOpen: !state.isOpen,
      };

    case 'OPEN_SIDEBAR':
      return {
        ...state,
        isOpen: true,
      };

    case 'SET_ACTIVE_TAB':
      return {
        ...state,
        activeTab: action.payload,
      };

    case 'SEND_MESSAGE':
      // This action is handled by the useGeminiChat hook via event listener
      // Just return state unchanged here
      return state;

    case 'ADD_CHAT_MESSAGE':
      const newChatHistory = [...state.chatHistory, action.payload];
      // Maintain max limit (FIFO)
      if (newChatHistory.length > LIMITS.MAX_CHAT_MESSAGES) {
        newChatHistory.shift();
      }
      return {
        ...state,
        chatHistory: newChatHistory,
      };

    case 'UPDATE_CHAT_MESSAGE':
      return {
        ...state,
        chatHistory: state.chatHistory.map(msg =>
          msg.id === action.payload.id ? action.payload : msg
        ),
      };

    case 'CLEAR_CHAT_HISTORY':
      return {
        ...state,
        chatHistory: [],
      };

    case 'CLEAR_PAGE_CHAT':
      // Keep messages from OTHER pages, remove messages for this specific page
      return {
        ...state,
        chatHistory: state.chatHistory.filter(msg => msg.pageUrl !== action.payload),
      };

    case 'ADD_HIGHLIGHT':
      const pageUrl = action.payload.pageUrl;
      const pageHighlights = state.savedHighlights[pageUrl] || [];
      const updatedPageHighlights = [...pageHighlights, action.payload];
      
      // Maintain per-page limit (FIFO)
      if (updatedPageHighlights.length > LIMITS.MAX_HIGHLIGHTS_PER_PAGE) {
        updatedPageHighlights.shift();
      }

      return {
        ...state,
        savedHighlights: {
          ...state.savedHighlights,
          [pageUrl]: updatedPageHighlights,
        },
      };

    case 'DELETE_HIGHLIGHT':
      const newHighlights = { ...state.savedHighlights };
      for (const url in newHighlights) {
        newHighlights[url] = newHighlights[url].filter(h => h.id !== action.payload);
        if (newHighlights[url].length === 0) {
          delete newHighlights[url];
        }
      }
      return {
        ...state,
        savedHighlights: newHighlights,
      };

    case 'UPDATE_PROGRESS':
      return {
        ...state,
        progressRecords: {
          ...state.progressRecords,
          [action.payload.pageUrl]: action.payload,
        },
      };

    case 'LOG_ERROR':
      const newErrorLog = [...state.errorLog, action.payload];
      // Maintain max limit (FIFO)
      if (newErrorLog.length > LIMITS.MAX_ERROR_LOG_ENTRIES) {
        newErrorLog.shift();
      }
      return {
        ...state,
        errorLog: newErrorLog,
      };

    default:
      return state;
  }
}

// Context
interface LearningHubContextValue {
  state: LearningHubState;
  dispatch: React.Dispatch<LearningHubAction>;
}

const LearningHubContext = createContext<LearningHubContextValue | undefined>(undefined);

// Provider Props
interface LearningHubProviderProps {
  children: ReactNode;
}

// Provider Component
export function LearningHubProvider({ children }: LearningHubProviderProps) {
  // Load initial state from localStorage
  const loadInitialState = (): LearningHubState => {
    const savedHighlights = storage.get(STORAGE_KEYS.HIGHLIGHTS, { default: {} });
    const progressRecords = storage.get(STORAGE_KEYS.PROGRESS, { default: {} });
    const errorLog = storage.get(STORAGE_KEYS.ERROR_LOG, { default: [] });
    const sidebarState = storage.get<{ isOpen: boolean; activeTab: TabName }>(
      STORAGE_KEYS.SIDEBAR_STATE,
      { default: { isOpen: false, activeTab: 'chat' } }
    );

    // DO NOT persist chat history - always start fresh
    return {
      ...initialState,
      isOpen: sidebarState?.isOpen ?? false,
      activeTab: sidebarState?.activeTab ?? 'chat',
      chatHistory: [], // Always start with empty chat
      savedHighlights: savedHighlights || {},
      progressRecords: progressRecords || {},
      errorLog: errorLog || [],
    };
  };

  const [state, dispatch] = useReducer(learningHubReducer, initialState, loadInitialState);

  // Persist state to localStorage (debounced to avoid excessive writes)
  // DO NOT persist chatHistory - it should always be per-session
  useEffect(() => {
    const persistState = debounce(() => {
      storage.set(STORAGE_KEYS.HIGHLIGHTS, state.savedHighlights);
      storage.set(STORAGE_KEYS.PROGRESS, state.progressRecords);
      storage.set(STORAGE_KEYS.ERROR_LOG, state.errorLog);
      storage.set(STORAGE_KEYS.SIDEBAR_STATE, {
        isOpen: state.isOpen,
        activeTab: state.activeTab,
      });
      // Note: chatHistory is intentionally NOT persisted
    }, 500);

    persistState();

    // Cleanup
    return () => {
      persistState.cancel();
    };
  }, [state]);

  return (
    <LearningHubContext.Provider value={{ state, dispatch }}>
      {children}
    </LearningHubContext.Provider>
  );
}

// Custom Hook
export function useLearningHub(): LearningHubContextValue {
  const context = useContext(LearningHubContext);
  if (context === undefined) {
    throw new Error('useLearningHub must be used within a LearningHubProvider');
  }
  return context;
}
