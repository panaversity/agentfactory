/**
 * Application Constants
 */

export const ERROR_MESSAGES = {
  INVALID_CREDENTIALS: 'Invalid email or password',
  EMAIL_ALREADY_EXISTS: 'An account with this email already exists',
  TIMEOUT: 'Request timed out. Please try again.',
  NETWORK: 'Network error. Please check your connection.',
  UNKNOWN: 'An unexpected error occurred. Please try again.',
  EMAIL_NOT_VERIFIED: 'Please verify your email address before signing in',
  ACCOUNT_LOCKED: 'Your account has been locked. Please contact support.',
  TOKEN_EXPIRED: 'This link has expired. Please request a new one.',
  INVALID_TOKEN: 'This link is invalid. Please request a new one.',
  WEAK_PASSWORD: 'Password does not meet security requirements',
} as const;

export const ROUTES = {
  SIGNIN: '/signin',
  SIGNUP: '/signup',
  FORGOT_PASSWORD: '/forgot-password',
  RESET_PASSWORD: '/reset-password',
  VERIFY_EMAIL: '/verify-email',
  DASHBOARD: '/dashboard',
  LOGIN: '/login', // OIDC
  CONSENT: '/consent', // OIDC
} as const;
