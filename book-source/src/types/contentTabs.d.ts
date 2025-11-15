/**
 * Type definitions for Content Tabs feature
 */

export type TabType = 'original' | 'summary' | 'personalized';

/**
 * Tab State - represents the current active tab (UI state, not persisted)
 */
export interface TabState {
  activeTab: TabType;
  pageId: string;
}

/**
 * Summary Cache Entry - stored in sessionStorage
 */
export interface SummaryCacheEntry {
  pageId: string;
  summary: string;
  timestamp: number;
  version?: string;
}

/**
 * Authentication State - stored in sessionStorage
 */
export interface AuthState {
  isAuthenticated: boolean;
  token: string | null;
  expiresAt: number | null;
}

/**
 * Summary API Response (streaming chunk)
 */
export interface SummaryChunk {
  chunk: string;
  done: boolean;
  error?: string | null;
}

/**
 * Auth API Response
 */
export interface AuthResponse {
  token: string;
  expires: string;
  user: {
    id: string;
    name: string;
  };
}

/**
 * Error Response from API
 */
export interface ErrorResponse {
  error: string;
  message: string;
}
