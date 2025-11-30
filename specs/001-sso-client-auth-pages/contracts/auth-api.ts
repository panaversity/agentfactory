/**
 * API Contracts for SSO Client Authentication Pages
 * 
 * Defines TypeScript interfaces for all authClient method calls and responses.
 * These contracts match the BetterAuth API from sso-server.
 * 
 * @see packages/auth-config/client.ts for authClient implementation
 * @see apps/sso-server/app/api/auth/[...all]/route.ts for backend endpoints
 */

// ============================================================================
// SIGN UP
// ============================================================================

export interface SignUpRequest {
  email: string;
  password: string;
  name: string;
  callbackURL?: string; // Optional redirect after email verification
}

export interface SignUpResponse {
  data?: {
    user: {
      id: string;
      email: string;
      name: string;
      emailVerified: boolean;
      image?: string | null;
      createdAt: Date;
      updatedAt: Date;
    };
    session: {
      id: string;
      userId: string;
      expiresAt: Date;
      token: string;
    };
  };
  error?: {
    status: number;
    message: string;
    code?: string; // e.g., "EMAIL_ALREADY_EXISTS"
  };
}

/**
 * authClient usage:
 * 
 * const { data, error } = await authClient.signUp.email({
 *   email: formData.email,
 *   password: formData.password,
 *   name: formData.name,
 *   callbackURL: '/dashboard'
 * });
 */

// ============================================================================
// SIGN IN
// ============================================================================

export interface SignInRequest {
  email: string;
  password: string;
  rememberMe?: boolean; // Extends session expiration
  callbackURL?: string; // Post-login redirect
}

export interface SignInResponse {
  data?: {
    user: {
      id: string;
      email: string;
      name: string;
      emailVerified: boolean;
      image?: string | null;
    };
    session: {
      id: string;
      userId: string;
      expiresAt: Date;
      token: string;
    };
  };
  error?: {
    status: number;
    message: string;
    code?: string; // e.g., "INVALID_CREDENTIALS", "EMAIL_NOT_VERIFIED"
  };
}

/**
 * authClient usage:
 * 
 * const { data, error } = await authClient.signIn.email({
 *   email: formData.email,
 *   password: formData.password,
 *   rememberMe: formData.rememberMe,
 *   callbackURL: getRedirectUrl(searchParams)
 * });
 */

// ============================================================================
// FORGOT PASSWORD
// ============================================================================

export interface ForgotPasswordRequest {
  email: string;
  redirectTo?: string; // Where the reset link should redirect to
}

export interface ForgotPasswordResponse {
  data?: {
    message: string; // Generic message to prevent user enumeration
  };
  error?: {
    status: number;
    message: string;
  };
}

/**
 * authClient usage:
 * 
 * const { data, error } = await authClient.forgetPassword({
 *   email: formData.email,
 *   redirectTo: '/reset-password'
 * });
 * 
 * Note: Always returns success to prevent user enumeration
 */

// ============================================================================
// RESET PASSWORD
// ============================================================================

export interface ResetPasswordRequest {
  token: string; // From email link query parameter
  newPassword: string;
}

export interface ResetPasswordResponse {
  data?: {
    message: string;
  };
  error?: {
    status: number;
    message: string;
    code?: string; // e.g., "INVALID_TOKEN", "TOKEN_EXPIRED"
  };
}

/**
 * authClient usage:
 * 
 * const { data, error } = await authClient.resetPassword({
 *   token: searchParams.get('token')!,
 *   newPassword: formData.newPassword
 * });
 */

// ============================================================================
// EMAIL VERIFICATION
// ============================================================================

export interface VerifyEmailRequest {
  token: string; // From email link query parameter
}

export interface VerifyEmailResponse {
  data?: {
    user: {
      id: string;
      email: string;
      emailVerified: boolean;
    };
  };
  error?: {
    status: number;
    message: string;
    code?: string; // e.g., "INVALID_TOKEN", "TOKEN_EXPIRED"
  };
}

/**
 * authClient usage:
 * 
 * const { data, error } = await authClient.verifyEmail({
 *   token: searchParams.get('token')!
 * });
 */

// ============================================================================
// GET SESSION
// ============================================================================

export interface GetSessionResponse {
  data?: {
    session: {
      id: string;
      userId: string;
      expiresAt: Date;
    };
    user: {
      id: string;
      email: string;
      name: string;
      emailVerified: boolean;
      image?: string | null;
    };
  } | null;
  error?: {
    status: number;
    message: string;
  };
}

/**
 * authClient usage:
 * 
 * const { data } = await authClient.getSession();
 * if (data?.session) {
 *   // User is logged in
 * }
 */

// ============================================================================
// SIGN OUT
// ============================================================================

export interface SignOutResponse {
  data?: {
    message: string;
  };
  error?: {
    status: number;
    message: string;
  };
}

/**
 * authClient usage:
 * 
 * await authClient.signOut();
 */

// ============================================================================
// OAUTH SIGN IN (Social Login)
// ============================================================================

export interface OAuthProvider {
  id: 'google' | 'github' | 'microsoft';
  name: string;
  enabled: boolean;
}

/**
 * OAuth flow is redirect-based, not API call.
 * 
 * Usage:
 * 
 * window.location.href = `${process.env.NEXT_PUBLIC_SSO_SERVER_URL}/api/auth/sign-in/google?callbackURL=${encodeURIComponent('/dashboard')}`;
 * 
 * The server handles the OAuth flow and redirects back to sso-client with session cookie.
 */

export interface OAuthCallbackQuery {
  code?: string;
  state?: string;
  error?: string;
  error_description?: string;
}

// ============================================================================
// OIDC AUTHORIZE
// ============================================================================

export interface OIDCAuthorizeQuery {
  client_id: string;
  redirect_uri: string;
  response_type: string; // "code"
  scope: string; // "openid profile email"
  state?: string;
  nonce?: string;
}

export interface OIDCAuthorizeResponse {
  // Redirects to consent page if user not logged in or consent not granted
  // Otherwise redirects to redirect_uri with authorization code
}

/**
 * Usage:
 * 
 * const params = new URLSearchParams(window.location.search);
 * const authorizeUrl = `${process.env.NEXT_PUBLIC_SSO_SERVER_URL}/api/auth/authorize?${params.toString()}`;
 * window.location.href = authorizeUrl;
 */

// ============================================================================
// CONSENT
// ============================================================================

export interface ConsentRequest {
  client_id: string;
  redirect_uri: string;
  scope: string;
  state?: string;
  approved: boolean; // true = grant, false = deny
}

export interface ConsentResponse {
  // Redirects to redirect_uri with authorization code or error
}

/**
 * authClient usage (custom endpoint):
 * 
 * const response = await fetch(`${process.env.NEXT_PUBLIC_SSO_SERVER_URL}/api/auth/consent`, {
 *   method: 'POST',
 *   headers: { 'Content-Type': 'application/json' },
 *   body: JSON.stringify({
 *     client_id: searchParams.get('client_id'),
 *     redirect_uri: searchParams.get('redirect_uri'),
 *     scope: searchParams.get('scope'),
 *     state: searchParams.get('state'),
 *     approved: true
 *   })
 * });
 */

// ============================================================================
// ERROR CODES
// ============================================================================

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

// ============================================================================
// ERROR HANDLER
// ============================================================================

export interface ApiError {
  status: number;
  message: string;
  code?: AuthErrorCode;
  fieldErrors?: Record<string, string>;
}

export function handleAuthError(error: unknown): ApiError {
  if (error instanceof Error) {
    if (error.name === 'AbortError') {
      return {
        status: 408,
        message: 'Request timed out. Please try again.',
        code: AUTH_ERROR_CODES.TIMEOUT,
      };
    }
    
    if (error.message.includes('fetch')) {
      return {
        status: 0,
        message: 'Network error. Please check your connection.',
        code: AUTH_ERROR_CODES.NETWORK_ERROR,
      };
    }
  }
  
  if (typeof error === 'object' && error !== null && 'status' in error) {
    return error as ApiError;
  }
  
  return {
    status: 500,
    message: 'An unexpected error occurred. Please try again.',
    code: AUTH_ERROR_CODES.UNKNOWN_ERROR,
  };
}

// ============================================================================
// TIMEOUT WRAPPER
// ============================================================================

export async function withTimeout<T>(
  promise: Promise<T>,
  timeoutMs: number = 30000
): Promise<T> {
  const timeoutPromise = new Promise<never>((_, reject) => {
    setTimeout(() => {
      reject(new Error('Request timed out'));
    }, timeoutMs);
  });
  
  return Promise.race([promise, timeoutPromise]);
}

/**
 * Usage:
 * 
 * try {
 *   const result = await withTimeout(
 *     authClient.signIn.email(formData),
 *     30000
 *   );
 * } catch (error) {
 *   const apiError = handleAuthError(error);
 *   setError(apiError.message);
 * }
 */
