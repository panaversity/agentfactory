/**
 * Entity Type Definitions - Learning Hub Sidebar
 * Based on specs/012-learning-hub-sidebar/data-model.md
 */

// ===== ChatMessage =====

export interface ChatMessage {
  id: string; // UUID v4
  role: 'user' | 'assistant';
  content: string; // 1-10000 characters
  timestamp: number; // Unix timestamp in milliseconds
  pageUrl: string; // Relative path starting with '/'
  tokens?: number; // Optional token count for API usage tracking
}

export type ChatHistory = ChatMessage[];

// ===== Highlight =====

export interface TextPosition {
  startContainerPath: string; // XPath to start node
  startOffset: number; // Character offset in start node
  endContainerPath: string; // XPath to end node
  endOffset: number; // Character offset in end node
}

export interface Highlight {
  id: string; // UUID v4
  pageUrl: string; // Page where highlight was created
  selectedText: string; // 10-1000 characters
  textPosition: TextPosition; // Serialized Range for re-highlighting
  explanation: string; // AI-generated explanation
  createdAt: number; // Unix timestamp
  backgroundColor?: string; // Default: '#fff3cd'
}

export type HighlightsStorage = {
  [pageUrl: string]: Highlight[];
};

// ===== QuizQuestion =====

export type QuizDifficulty = 'easy' | 'medium' | 'hard';

export interface QuizQuestion {
  id: string; // UUID v4
  pageUrl: string; // Source page for question
  question: string; // 10-500 characters
  choices: [string, string, string, string]; // Exactly 4 answer options
  correctAnswer: 0 | 1 | 2 | 3; // Index of correct choice
  explanation: string; // Why the answer is correct
  difficulty: QuizDifficulty;
  generatedAt: number; // Unix timestamp
}

export interface QuizSession {
  pageUrl: string;
  questions: QuizQuestion[];
  userAnswers: (number | null)[]; // Parallel array to questions
  score?: { correct: number; total: number };
}

// ===== KeyConcept =====

export interface KeyConcept {
  id: string; // UUID v4
  pageUrl: string; // Source page URL
  title: string; // 5-100 characters
  description: string; // 10-500 characters
  sectionId?: string; // Optional HTML element ID to scroll to
  importance: number; // Ranking 1-10
  contentHash: string; // MD5 hash of source page content
  cachedAt: number; // Unix timestamp
}

export interface CachedKeyConcepts {
  contentHash: string;
  cachedAt: number;
  expiresAt: number; // cachedAt + 7 days
  concepts: KeyConcept[];
}

export type KeyConceptsStorage = {
  [pageUrl: string]: CachedKeyConcepts;
};

// ===== RelatedTopic =====

export interface RelatedTopic {
  id: string; // UUID v4
  pageUrl: string; // Source page URL
  title: string; // 5-200 characters
  url: string; // Relative path starting with '/'
  description: string; // 10-300 characters
  relevanceScore: number; // 0.0-1.0
  contentHash: string; // MD5 hash of source page
  cachedAt: number; // Unix timestamp
}

export interface CachedRelatedTopics {
  contentHash: string;
  cachedAt: number;
  expiresAt: number; // cachedAt + 7 days
  topics: RelatedTopic[];
}

export type RelatedTopicsStorage = {
  [pageUrl: string]: CachedRelatedTopics;
};

// ===== ProgressRecord =====

export interface ProgressRecord {
  pageUrl: string; // Unique identifier (page URL)
  visitCount: number; // Number of times viewed
  totalDuration: number; // Cumulative read time in seconds
  firstVisitedAt: number; // Unix timestamp of first visit
  lastVisitedAt: number; // Unix timestamp of most recent visit
  completed: boolean; // Whether marked as "done"
}

export type ProgressStorage = {
  [pageUrl: string]: ProgressRecord;
};

// Computed statistics (not stored)
export interface ProgressSummary {
  totalPagesVisited: number;
  totalTimeSpent: number; // seconds
  totalHighlights: number;
  completionPercentage: number; // 0-100
  streak: number; // consecutive days with visits
}

// ===== ErrorLogEntry =====

export interface ErrorLogEntry {
  id: string; // UUID v4
  timestamp: number; // Unix timestamp
  message: string;
  stack?: string;
  context?: Record<string, any>; // Additional context
}

// ===== LearningHubState =====

export type TabName = 'chat' | 'highlights' | 'quiz' | 'concepts' | 'topics' | 'progress';

export interface LearningHubState {
  isOpen: boolean;
  activeTab: TabName;
  chatHistory: ChatMessage[]; // Max 50 messages
  savedHighlights: HighlightsStorage;
  progressRecords: ProgressStorage;
  errorLog: ErrorLogEntry[]; // Max 50 entries
}

// Actions for state updates
export type LearningHubAction =
  | { type: 'TOGGLE_SIDEBAR' }
  | { type: 'SET_ACTIVE_TAB'; payload: TabName }
  | { type: 'ADD_CHAT_MESSAGE'; payload: ChatMessage }
  | { type: 'UPDATE_CHAT_MESSAGE'; payload: ChatMessage }
  | { type: 'CLEAR_CHAT_HISTORY' }
  | { type: 'CLEAR_PAGE_CHAT'; payload: string } // pageUrl
  | { type: 'ADD_HIGHLIGHT'; payload: Highlight }
  | { type: 'DELETE_HIGHLIGHT'; payload: string } // highlight ID
  | { type: 'UPDATE_PROGRESS'; payload: ProgressRecord }
  | { type: 'LOG_ERROR'; payload: ErrorLogEntry };

// ===== Storage Keys =====

export const STORAGE_KEYS = {
  CHAT: 'learningHub_chat_v1',
  HIGHLIGHTS: 'learningHub_highlights_v1',
  KEY_CONCEPTS: 'learningHub_concepts_v1',
  RELATED_TOPICS: 'learningHub_topics_v1',
  PROGRESS: 'learningHub_progress_v1',
  ERROR_LOG: 'learningHub_errors_v1',
  SIDEBAR_STATE: 'learningHub_sidebarState_v1',
  SCHEMA_VERSION: 'learningHub_schema_version',
} as const;

// ===== Constants =====

export const LIMITS = {
  MAX_CHAT_MESSAGES: 50,
  MAX_HIGHLIGHTS_PER_PAGE: 100,
  MAX_TOTAL_HIGHLIGHTS: 1000,
  MAX_ERROR_LOG_ENTRIES: 50,
  MAX_HIGHLIGHT_LENGTH: 1000,
  MIN_HIGHLIGHT_LENGTH: 10,
  MAX_CHAT_MESSAGE_LENGTH: 10000,
  CACHE_TTL_DAYS: 7,
  RATE_LIMIT_RPM: 15,
  MAX_CONCURRENT_REQUESTS: 2,
} as const;
