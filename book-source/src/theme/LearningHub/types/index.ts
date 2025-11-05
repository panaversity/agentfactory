/**
 * Learning Hub TypeScript Type Definitions
 * Barrel export for all types
 */

// Re-export all entity types
export type {
  ChatMessage,
  Highlight,
  TextPosition,
  QuizQuestion,
  QuizOption,
  QuizDifficulty,
  KeyConcept,
  CachedKeyConcepts,
  KeyConceptsStorage,
  RelatedTopic,
  CachedRelatedTopics,
  RelatedTopicsStorage,
  ProgressRecord,
  ProgressSummary,
  ProgressStorage,
  ErrorLogEntry,
  HighlightsStorage,
  TabName,
  LearningHubState,
  LearningHubAction,
} from './entities';

// Re-export constants
export { STORAGE_KEYS, LIMITS } from './entities';

// Re-export all API types
export type {
  GeminiRequestBase,
  GeminiError,
  ChatRequest,
  ChatResponse,
  ChatStreamChunk,
  ExplainRequest,
  ExplainResponse,
  QuizRequest,
  QuizResponse,
  KeyConceptsRequest,
  KeyConceptsResponse,
  RelatedTopicsRequest,
  RelatedTopicsResponse,
  RateLimitInfo,
  CacheEntry,
  CacheOptions,
  StorageGetOptions,
  StorageSetOptions,
  StorageQuotaInfo,
} from './api';
