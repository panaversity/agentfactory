/**
 * Authentication Service - manages user authentication state
 * 
 * Uses sessionStorage to persist authentication tokens and user profiles
 * across page navigations during a browser session.
 */

import { AuthState, AuthSession, UserProfile } from '../types/contentTabs';
import * as cacheService from './cacheService';

const AUTH_TOKEN_KEY = 'authToken';
const AUTH_EXPIRY_KEY = 'authExpiry';
const AUTH_SESSION_KEY = 'authSession'; // T017: New composite session key

/**
 * Check if user is currently authenticated
 * @returns true if valid authentication token exists
 */
export function isAuthenticated(): boolean {
  const token = cacheService.get<string>(AUTH_TOKEN_KEY);
  
  if (!token) {
    return false;
  }
  
  // For dummy implementation, any token means authenticated
  // Future: Check token expiry
  const expiry = cacheService.get<number>(AUTH_EXPIRY_KEY);
  if (expiry && Date.now() > expiry) {
    clearToken();
    return false;
  }
  
  return true;
}

/**
 * Get current authentication token
 * @returns token string or null if not authenticated
 */
export function getToken(): string | null {
  return cacheService.get<string>(AUTH_TOKEN_KEY);
}

/**
 * Set authentication token
 * @param token - Authentication token
 * @param expiresAt - Optional expiration timestamp (for future SSO)
 */
export function setToken(token: string, expiresAt?: number): void {
  cacheService.set(AUTH_TOKEN_KEY, token);
  if (expiresAt) {
    cacheService.set(AUTH_EXPIRY_KEY, expiresAt);
  }
}

/**
 * Clear authentication token (logout)
 */
export function clearToken(): void {
  cacheService.remove(AUTH_TOKEN_KEY);
  cacheService.remove(AUTH_EXPIRY_KEY);
}

/**
 * Get complete authentication state
 * @returns AuthState object
 */
export function getAuthState(): AuthState {
  const token = getToken();
  const expiresAt = cacheService.get<number>(AUTH_EXPIRY_KEY);
  
  return {
    isAuthenticated: isAuthenticated(),
    token,
    expiresAt,
  };
}

// T017-T022: Enhanced authentication with user profiles

/**
 * Save authentication session with user profile
 * @param token - Authentication token
 * @param profile - User profile with proficiency levels
 */
export function saveSession(token: string, profile: UserProfile): void {
  const session: AuthSession = {
    token,
    profile,
    expiresAt: undefined, // Future: Set actual expiration
  };
  
  cacheService.set(AUTH_SESSION_KEY, session);
  
  // Also save token separately for backward compatibility
  setToken(token);
}

/**
 * Get authentication session with profile
 * @returns AuthSession object or null if not authenticated
 */
export function getSession(): AuthSession | null {
  return cacheService.get<AuthSession>(AUTH_SESSION_KEY);
}

/**
 * Get user profile from current session
 * @returns UserProfile or null if no session exists
 */
export function getProfile(): UserProfile | null {
  const session = getSession();
  return session?.profile ?? null;
}

/**
 * Generate profile fingerprint for cache keys
 * @param profile - User profile
 * @returns Fingerprint string in format "ProgrammingLevel-AILevel"
 */
export function generateProfileFingerprint(profile: UserProfile): string {
  return `${profile.programmingExperience}-${profile.aiProficiency}`;
}

/**
 * Check if user session is expired
 * @returns true if session has expired
 */
export function isSessionExpired(): boolean {
  const session = getSession();
  if (!session || !session.expiresAt) {
    return false; // No expiration set means session-based (no expiry)
  }
  return Date.now() > session.expiresAt;
}
