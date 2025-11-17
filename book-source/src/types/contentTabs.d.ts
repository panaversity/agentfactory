/**
 * Type definitions for Content Tabs feature
 */

export type TabType = 'original' | 'summary' | 'personalized';

// Personalization types (T008-T011)

/**
 * User proficiency levels for programming and AI knowledge
 */
export type ProficiencyLevel = 'Novice' | 'Beginner' | 'Intermediate' | 'Expert';

/**
 * User Profile - collected during login
 */
export interface UserProfile {
  name: string;
  email: string;
  programmingExperience: ProficiencyLevel;
  aiProficiency: ProficiencyLevel;
}

/**
 * Authentication Session - stored in sessionStorage
 * Composite object containing token and user profile
 */
export interface AuthSession {
  token: string;
  profile: UserProfile;
  expiresAt?: number;
}

/**
 * Personalization Cache Entry - stored in sessionStorage
 * Profile-specific cached content
 */
export interface PersonalizationCacheEntry {
  pageId: string;
  profileFingerprint: string; // Format: "ProgrammingLevel-AILevel"
  personalizedText: string;
  timestamp: number;
  cached: boolean;
}

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
