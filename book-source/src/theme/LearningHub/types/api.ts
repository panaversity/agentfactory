/**
 * API Type Definitions - Learning Hub Sidebar
 * Request/Response types for Gemini API interactions
 * Based on specs/012-learning-hub-sidebar/contracts/
 */

import type { ChatMessage, QuizQuestion, KeyConcept, RelatedTopic } from './entities';

// ===== Common API Types =====

export interface GeminiRequestBase {
  pageUrl: string;
  pageTitle: string;
  pageContent: string;
}

export interface GeminiError {
  code: string;
  message: string;
  status?: number;
  retryable: boolean;
}

// ===== Chat API =====

export interface ChatRequest extends GeminiRequestBase {
  userMessage: string; // 1-10000 characters
  conversationHistory: ChatMessage[]; // Last 10 messages
}

export interface ChatResponse {
  message: string;
  tokens?: number;
  cached: boolean;
}

export interface ChatStreamChunk {
  text: string;
  done: boolean;
}

// ===== Explain API =====

export interface ExplainRequest extends GeminiRequestBase {
  selectedText: string; // 10-1000 characters
  surroundingContext?: string; // Optional paragraph context
}

export interface ExplainResponse {
  explanation: string; // 10-5000 characters
  tokens?: number;
  cached: boolean;
}

// ===== Quiz API =====

export interface QuizRequest extends GeminiRequestBase {
  questionCount: number; // 3-5
  difficulty?: 'easy' | 'medium' | 'hard';
}

export interface QuizResponse {
  questions: QuizQuestion[];
  tokens?: number;
}

// ===== Key Concepts API =====

export interface KeyConceptsRequest extends GeminiRequestBase {
  targetCount: number; // 5-7
}

export interface KeyConceptsResponse {
  concepts: KeyConcept[];
  contentHash: string; // MD5 hash of pageContent
  tokens?: number;
  cached: boolean;
}

// ===== Related Topics API =====

export interface RelatedTopicsRequest extends GeminiRequestBase {
  targetCount: number; // 3-5
  availablePages?: string[]; // Optional list of valid page URLs
}

export interface RelatedTopicsResponse {
  topics: RelatedTopic[];
  contentHash: string; // MD5 hash of pageContent
  tokens?: number;
  cached: boolean;
}

// ===== Rate Limiter Types =====

export interface RateLimitInfo {
  remaining: number; // Requests remaining in window
  resetAt: number; // Timestamp when limit resets
  limited: boolean; // Whether currently rate limited
}

// ===== Cache Types =====

export interface CacheEntry<T> {
  data: T;
  timestamp: number;
  expiresAt: number;
  contentHash?: string; // Optional for content-based invalidation
}

export interface CacheOptions {
  ttl?: number; // Time-to-live in milliseconds
  contentHash?: string; // For content-based invalidation
}

// ===== Storage Service Types =====

export interface StorageGetOptions {
  default?: any;
  validate?: (data: any) => boolean;
}

export interface StorageSetOptions {
  expires?: number; // Expiration timestamp
  merge?: boolean; // Merge with existing data
}

export interface StorageQuotaInfo {
  used: number; // Bytes used
  available: number; // Bytes available
  percentage: number; // 0-100
  exceeded: boolean;
}
