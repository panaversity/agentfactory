/**
 * Authentication Service - manages user authentication state
 * 
 * Uses sessionStorage to persist authentication tokens across page navigations
 * during a browser session.
 */

import { AuthState } from '../types/contentTabs';
import * as cacheService from './cacheService';

const AUTH_TOKEN_KEY = 'authToken';
const AUTH_EXPIRY_KEY = 'authExpiry';

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
