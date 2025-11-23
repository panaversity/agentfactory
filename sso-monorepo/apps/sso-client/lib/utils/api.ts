/**
 * API Error Codes
 */
export const AUTH_ERROR_CODES = {
  // Sign Up
  EMAIL_ALREADY_EXISTS: 'EMAIL_ALREADY_EXISTS',
  INVALID_EMAIL: 'INVALID_EMAIL',
  WEAK_PASSWORD: 'WEAK_PASSWORD',
  
  // Sign In
  INVALID_CREDENTIALS: 'INVALID_CREDENTIALS',
  EMAIL_NOT_VERIFIED: 'EMAIL_NOT_VERIFIED',
  ACCOUNT_LOCKED: 'ACCOUNT_LOCKED',
  TOO_MANY_ATTEMPTS: 'TOO_MANY_ATTEMPTS',
  
  // Password Reset
  INVALID_TOKEN: 'INVALID_TOKEN',
  TOKEN_EXPIRED: 'TOKEN_EXPIRED',
  
  // General
  NETWORK_ERROR: 'NETWORK_ERROR',
  TIMEOUT: 'TIMEOUT',
  UNKNOWN_ERROR: 'UNKNOWN_ERROR',
} as const;

export type AuthErrorCode = typeof AUTH_ERROR_CODES[keyof typeof AUTH_ERROR_CODES];

/**
 * API Error Interface
 */
export interface ApiError {
  status: number;
  message: string;
  code?: AuthErrorCode;
  fieldErrors?: Record<string, string>;
}

/**
 * Handle Authentication Errors
 * Converts various error types into a standardized ApiError format
 */
export function handleAuthError(error: unknown): ApiError {
  // AbortError from timeout
  if (error instanceof Error) {
    if (error.name === 'AbortError') {
      return {
        status: 408,
        message: 'Request timed out. Please try again.',
        code: AUTH_ERROR_CODES.TIMEOUT,
      };
    }
    
    // Network/fetch errors
    if (error.message.includes('fetch') || error.message.includes('network')) {
      return {
        status: 0,
        message: 'Network error. Please check your connection.',
        code: AUTH_ERROR_CODES.NETWORK_ERROR,
      };
    }
  }
  
  // Already formatted ApiError from backend
  if (typeof error === 'object' && error !== null && 'status' in error) {
    return error as ApiError;
  }
  
  // Unknown error
  return {
    status: 500,
    message: 'An unexpected error occurred. Please try again.',
    code: AUTH_ERROR_CODES.UNKNOWN_ERROR,
  };
}

/**
 * Timeout Wrapper for Promises
 * Races a promise against a timeout, rejecting if timeout occurs first
 * 
 * @param promise - The promise to wrap
 * @param timeoutMs - Timeout in milliseconds (default: 30000 = 30 seconds)
 */
export async function withTimeout<T>(
  promise: Promise<T>,
  timeoutMs: number = 30000
): Promise<T> {
  const timeoutPromise = new Promise<never>((_, reject) => {
    setTimeout(() => {
      const error = new Error('Request timed out');
      error.name = 'AbortError';
      reject(error);
    }, timeoutMs);
  });
  
  return Promise.race([promise, timeoutPromise]);
}
