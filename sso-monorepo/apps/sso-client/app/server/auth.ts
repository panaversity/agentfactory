'use server';

import { cookies } from 'next/headers';

const API_URL = process.env.BETTER_AUTH_URL || 'http://localhost:3000';

type AuthSuccessResponse = {
  token: string;
  user: {
    id: string;
    email: string;
    name: string;
    emailVerified: boolean;
    createdAt: string;
  };
};

type AuthResult<T = AuthSuccessResponse> = {
  data?: T;
  error?: {
    message: string;
    status?: number;
  };
};

/**
 * Server action to sign up a new user
 * Calls POST /api/auth/sign-up/email on the auth server
 */
export async function signUpAction(params: {
  email: string;
  password: string;
  name: string;
  callbackURL?: string;
}): Promise<AuthResult> {
  try {
    const response = await fetch(`${API_URL}/api/auth/sign-up/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: params.email,
        password: params.password,
        name: params.name,
      }),
      credentials: 'include',
    });

    let data;
    try {
      data = await response.json();
    } catch (jsonError) {
      // If JSON parsing fails but response was successful, treat as success
      // This happens when backend requires email verification and returns empty response
      if (response.ok) {
        console.log('[signUpAction] Success with no JSON body - email verification required');
        return {
          data: {
            token: '',
            user: {
              id: '',
              email: params.email,
              name: params.name,
              emailVerified: false,
              createdAt: new Date().toISOString(),
            },
          },
        };
      }
      // If response was not ok, throw the error
      throw new Error('Sign up failed');
    }

    // Handle error response
    if (!response.ok) {
      // Better Auth can return errors in different formats:
      // { error: "message" } or { message: "message" } or { error: { message: "..." } }
      const errorMessage =
        data.error?.message || // nested error object
        data.error ||          // error string
        data.message ||        // direct message
        'Sign up failed';      // fallback

      console.log('[signUpAction] Error response:', { status: response.status, data });

      return {
        error: {
          message: errorMessage,
          status: response.status,
        },
      };
    }

    // Success - set session cookie
    if (data.token) {
      const cookieStore = await cookies();
      cookieStore.set('better-auth.session_token', data.token, {
        httpOnly: true,
        secure: process.env.NODE_ENV === 'production',
        sameSite: 'lax',
        path: '/',
      });
    }

    return {
      data: {
        token: data.token,
        user: data.user,
      },
    };
  } catch (error) {
    console.error('Sign up error:', error);
    return {
      error: {
        message: error instanceof Error ? error.message : 'Network error occurred',
      },
    };
  }
}

/**
 * Server action to sign in an existing user
 * Calls POST /api/auth/sign-in/email on the auth server
 */
export async function signInAction(params: {
  email: string;
  password: string;
  callbackURL?: string;
}): Promise<AuthResult> {
  try {
    const response = await fetch(`${API_URL}/api/auth/sign-in/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: params.email,
        password: params.password,
      }),
      credentials: 'include',
    });

    const data = await response.json();

    // Handle error response
    if (!response.ok) {
      // Better Auth can return errors in different formats
      const errorMessage =
        data.error?.message ||
        data.error ||
        data.message ||
        'Invalid email or password';

      console.log('[signInAction] Error response:', { status: response.status, data });

      return {
        error: {
          message: errorMessage,
          status: response.status,
        },
      };
    }

    // Success - set session cookie
    if (data.token) {
      const cookieStore = await cookies();
      cookieStore.set('better-auth.session_token', data.token, {
        httpOnly: true,
        secure: process.env.NODE_ENV === 'production',
        sameSite: 'lax',
        path: '/',
      });
    }

    return {
      data: {
        token: data.token,
        user: data.user,
      },
    };
  } catch (error) {
    console.error('Sign in error:', error);
    return {
      error: {
        message: error instanceof Error ? error.message : 'Network error occurred',
      },
    };
  }
}

/**
 * Server action to send email verification
 * Calls POST /api/auth/send-verification-email on the auth server
 */
export async function sendVerificationEmailAction(params: {
  email: string;
  callbackURL?: string;
}): Promise<AuthResult<{ success: boolean; message?: string }>> {
  try {
    const response = await fetch(`${API_URL}/api/auth/send-verification-email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: params.email,
        callbackURL: params.callbackURL || '/',
      }),
      credentials: 'include',
    });

    const data = await response.json();

    if (!response.ok) {
      const errorMessage =
        data.error?.message ||
        data.error ||
        data.message ||
        'Failed to send verification email';

      console.log('[sendVerificationEmailAction] Error response:', { status: response.status, data });

      return {
        error: {
          message: errorMessage,
          status: response.status,
        },
      };
    }

    return {
      data: {
        success: true,
        message: data.message || 'Verification email sent successfully',
      },
    };
  } catch (error) {
    console.error('Send verification email error:', error);
    return {
      error: {
        message: error instanceof Error ? error.message : 'Network error occurred',
      },
    };
  }
}

/**
 * Server action to verify email with token
 * Calls GET /api/auth/verify-email on the auth server
 * Note: This endpoint returns a redirect, so we handle it differently
 */
export async function verifyEmailAction(params: {
  token: string;
  callbackURL?: string;
}): Promise<AuthResult<{ success: boolean; redirectUrl?: string }>> {
  try {
    const url = new URL(`${API_URL}/api/auth/verify-email`);
    url.searchParams.append('token', params.token);
    url.searchParams.append('callbackURL', params.callbackURL || '/');

    const response = await fetch(url.toString(), {
      method: 'GET',
      credentials: 'include',
      redirect: 'manual', // Don't follow redirects automatically
    });

    // Verify email endpoint returns 302 redirect on success
    if (response.status === 302 || response.status === 301) {
      const location = response.headers.get('location');
      const setCookie = response.headers.get('set-cookie');

      // Set the session cookie if provided
      if (setCookie) {
        const cookieStore = await cookies();
        // Parse the cookie (simplified - better-auth uses better-auth.session_token)
        const tokenMatch = setCookie.match(/better-auth\.session_token=([^;]+)/);
        if (tokenMatch) {
          cookieStore.set('better-auth.session_token', tokenMatch[1], {
            httpOnly: true,
            secure: process.env.NODE_ENV === 'production',
            sameSite: 'lax',
            path: '/',
          });
        }
      }

      return {
        data: {
          success: true,
          redirectUrl: location || params.callbackURL || '/',
        },
      };
    }

    // If not a redirect, check for error response
    const data = await response.json().catch(() => ({}));
    const errorMessage =
      data.error?.message ||
      data.error ||
      data.message ||
      'Failed to verify email';

    console.log('[verifyEmailAction] Error response:', { status: response.status, data });

    return {
      error: {
        message: errorMessage,
        status: response.status,
      },
    };
  } catch (error) {
    console.error('Verify email error:', error);
    return {
      error: {
        message: error instanceof Error ? error.message : 'Network error occurred',
      },
    };
  }
}

/**
 * Server action to request password reset
 * Calls POST /api/auth/forget-password on the auth server
 */
export async function forgetPasswordAction(params: {
  email: string;
  redirectTo?: string;
}): Promise<AuthResult<{ success: boolean; message: string }>> {
  try {
    const response = await fetch(`${API_URL}/api/auth/forget-password`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: params.email,
        redirectTo: params.redirectTo || 'http://localhost:3001/reset-password',
      }),
      credentials: 'include',
    });

    const data = await response.json();

    if (!response.ok) {
      const errorMessage =
        data.error?.message ||
        data.error ||
        data.message ||
        'Failed to send password reset email';

      console.log('[forgetPasswordAction] Error response:', { status: response.status, data });

      return {
        error: {
          message: errorMessage,
          status: response.status,
        },
      };
    }

    return {
      data: {
        success: data.success || true,
        message: data.message || 'Password reset email sent',
      },
    };
  } catch (error) {
    console.error('Forget password error:', error);
    return {
      error: {
        message: error instanceof Error ? error.message : 'Network error occurred',
      },
    };
  }
}

/**
 * Server action to reset password with token
 * Calls POST /api/auth/reset-password on the auth server
 */
export async function resetPasswordAction(params: {
  newPassword: string;
  token: string;
}): Promise<AuthResult<{ success: boolean; message: string }>> {
  try {
    const response = await fetch(`${API_URL}/api/auth/reset-password`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        newPassword: params.newPassword,
        token: params.token,
      }),
      credentials: 'include',
    });

    const data = await response.json();

    if (!response.ok) {
      const errorMessage =
        data.error?.message ||
        data.error ||
        data.message ||
        'Failed to reset password';

      console.log('[resetPasswordAction] Error response:', { status: response.status, data });

      return {
        error: {
          message: errorMessage,
          status: response.status,
        },
      };
    }

    return {
      data: {
        success: data.success || true,
        message: data.message || 'Password successfully reset',
      },
    };
  } catch (error) {
    console.error('Reset password error:', error);
    return {
      error: {
        message: error instanceof Error ? error.message : 'Network error occurred',
      },
    };
  }
}
