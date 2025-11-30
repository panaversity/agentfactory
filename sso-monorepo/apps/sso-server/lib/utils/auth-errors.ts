/**
 * Error handling utilities for better-auth client operations
 * Normalizes different error formats from better-auth into consistent messages
 */

export const ERROR_MESSAGES = {
  INVALID_CREDENTIALS: 'Invalid email or password',
  EMAIL_NOT_VERIFIED: 'Please verify your email before signing in',
  EMAIL_ALREADY_EXISTS: 'An account with this email already exists',
  WEAK_PASSWORD: 'Password does not meet security requirements',
  INVALID_TOKEN: 'Invalid or expired verification token',
  NETWORK_ERROR: 'Network error occurred. Please try again',
  UNKNOWN: 'An unexpected error occurred. Please try again',
  PASSWORD_RESET_SENT: 'Password reset email sent successfully',
  VERIFICATION_SENT: 'Verification email sent successfully',
} as const;

interface NormalizedError {
  message: string;
  status?: number;
}

/**
 * Normalize better-auth error responses into consistent format
 * Better-auth can return errors in multiple formats:
 * - { error: "message" }
 * - { message: "message" }
 * - { error: { message: "..." } }
 * - Error object
 */
export function normalizeAuthError(error: any): NormalizedError {
  // Handle network errors and Error objects
  if (error instanceof Error) {
    if (error.message.includes('fetch')) {
      return { message: ERROR_MESSAGES.NETWORK_ERROR };
    }
    return { message: error.message };
  }

  // Handle better-auth error responses
  if (error && typeof error === 'object') {
    const errorMessage =
      error.error?.message || // nested error object
      error.error ||          // error string
      error.message ||        // direct message
      null;

    if (!errorMessage) {
      return { message: ERROR_MESSAGES.UNKNOWN, status: error.status };
    }

    // Map common error messages to user-friendly constants
    const lowerMessage = errorMessage.toLowerCase();

    if (lowerMessage.includes('invalid') ||
        lowerMessage.includes('password') ||
        lowerMessage.includes('credentials')) {
      return { message: ERROR_MESSAGES.INVALID_CREDENTIALS, status: error.status };
    }

    if (lowerMessage.includes('verify') ||
        lowerMessage.includes('verification')) {
      return { message: ERROR_MESSAGES.EMAIL_NOT_VERIFIED, status: error.status };
    }

    if (lowerMessage.includes('already exists') ||
        lowerMessage.includes('already registered')) {
      return { message: ERROR_MESSAGES.EMAIL_ALREADY_EXISTS, status: error.status };
    }

    if (lowerMessage.includes('weak') ||
        lowerMessage.includes('password') && lowerMessage.includes('requirement')) {
      return { message: ERROR_MESSAGES.WEAK_PASSWORD, status: error.status };
    }

    if (lowerMessage.includes('token') ||
        lowerMessage.includes('expired')) {
      return { message: ERROR_MESSAGES.INVALID_TOKEN, status: error.status };
    }

    // Return the original message if no mapping found
    return { message: errorMessage, status: error.status };
  }

  // Fallback for unknown error types
  return { message: ERROR_MESSAGES.UNKNOWN };
}

/**
 * Check if an error indicates invalid credentials
 */
export function isInvalidCredentialsError(error: any): boolean {
  const normalized = normalizeAuthError(error);
  return normalized.message === ERROR_MESSAGES.INVALID_CREDENTIALS;
}

/**
 * Check if an error indicates email not verified
 */
export function isEmailNotVerifiedError(error: any): boolean {
  const normalized = normalizeAuthError(error);
  return normalized.message === ERROR_MESSAGES.EMAIL_NOT_VERIFIED;
}

/**
 * Check if an error indicates email already exists
 */
export function isEmailExistsError(error: any): boolean {
  const normalized = normalizeAuthError(error);
  return normalized.message === ERROR_MESSAGES.EMAIL_ALREADY_EXISTS;
}
