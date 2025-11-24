# Data Model: SSO Client Authentication Pages

**Feature**: 001-sso-client-auth-pages  
**Date**: 2025-11-23  
**Type**: Client-side data structures (no database schema - backend handles persistence)

## Overview

This document defines the client-side data structures, form schemas, and state management models for the authentication pages. The actual user data persistence is handled by the SSO server backend using BetterAuth and the database defined in `@repo/database`.

---

## Form Input Models

### SignUpFormData

**Purpose**: Captures user registration input

**Fields**:
| Field | Type | Validation | Required | Notes |
|-------|------|------------|----------|-------|
| email | string | RFC 5322 email format, max 255 chars | Yes | Unique in backend |
| password | string | Min 8 chars, 1 upper, 1 lower, 1 number, 1 special | Yes | Client validates complexity |
| name | string | Min 1 char, max 100 chars | Yes | Display name |

**Zod Schema**:
```typescript
export const signUpSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email format')
    .max(255, 'Email is too long'),
  password: z.string()
    .min(8, 'Password must be at least 8 characters')
    .regex(/[A-Z]/, 'Must contain an uppercase letter')
    .regex(/[a-z]/, 'Must contain a lowercase letter')
    .regex(/[0-9]/, 'Must contain a number')
    .regex(/[^A-Za-z0-9]/, 'Must contain a special character'),
  name: z.string()
    .min(1, 'Name is required')
    .max(100, 'Name is too long'),
});

export type SignUpFormData = z.infer<typeof signUpSchema>;
```

---

### SignInFormData

**Purpose**: Captures user login credentials

**Fields**:
| Field | Type | Validation | Required | Notes |
|-------|------|------------|----------|-------|
| email | string | RFC 5322 email format | Yes | |
| password | string | Min 1 char (no complexity check on login) | Yes | |
| rememberMe | boolean | N/A | No | Default: false |

**Zod Schema**:
```typescript
export const signInSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email format'),
  password: z.string()
    .min(1, 'Password is required'),
  rememberMe: z.boolean().optional().default(false),
});

export type SignInFormData = z.infer<typeof signInSchema>;
```

---

### ForgotPasswordFormData

**Purpose**: Requests password reset email

**Fields**:
| Field | Type | Validation | Required | Notes |
|-------|------|------------|----------|-------|
| email | string | RFC 5322 email format | Yes | No user enumeration - always show success |

**Zod Schema**:
```typescript
export const forgotPasswordSchema = z.object({
  email: z.string()
    .min(1, 'Email is required')
    .email('Invalid email format'),
});

export type ForgotPasswordFormData = z.infer<typeof forgotPasswordSchema>;
```

---

### ResetPasswordFormData

**Purpose**: Sets new password with token from email

**Fields**:
| Field | Type | Validation | Required | Notes |
|-------|------|------------|----------|-------|
| token | string | Min 1 char | Yes | From query param |
| newPassword | string | Same as SignUpFormData password | Yes | |
| confirmPassword | string | Must match newPassword | Yes | Client-side only |

**Zod Schema**:
```typescript
export const resetPasswordSchema = z.object({
  token: z.string().min(1, 'Reset token is required'),
  newPassword: z.string()
    .min(8, 'Password must be at least 8 characters')
    .regex(/[A-Z]/, 'Must contain an uppercase letter')
    .regex(/[a-z]/, 'Must contain a lowercase letter')
    .regex(/[0-9]/, 'Must contain a number')
    .regex(/[^A-Za-z0-9]/, 'Must contain a special character'),
  confirmPassword: z.string(),
}).refine((data) => data.newPassword === data.confirmPassword, {
  message: 'Passwords do not match',
  path: ['confirmPassword'],
});

export type ResetPasswordFormData = z.infer<typeof resetPasswordSchema>;
```

---

### CaptchaState

**Purpose**: Manages CAPTCHA challenge state

**Fields**:
| Field | Type | Validation | Required | Notes |
|-------|------|------------|----------|-------|
| required | boolean | N/A | Yes | Show CAPTCHA after 5 failed attempts |
| token | string \| null | Min 1 char when present | No | hCaptcha response token |
| verified | boolean | N/A | Yes | Whether CAPTCHA was completed |

**TypeScript Interface**:
```typescript
export interface CaptchaState {
  required: boolean;
  token: string | null;
  verified: boolean;
}
```

---

## Response Models

### AuthResponse

**Purpose**: Standardized API response from BetterAuth endpoints

**Structure**:
```typescript
export interface AuthResponse<T = unknown> {
  data?: T;
  error?: string;
  fieldErrors?: Record<string, string>;
}

export interface SessionData {
  session: {
    id: string;
    userId: string;
    expiresAt: string;
  };
  user: {
    id: string;
    email: string;
    name: string;
    emailVerified: boolean;
    image?: string;
  };
}
```

---

### RedirectParams

**Purpose**: Manages redirect logic after authentication

**Fields**:
| Field | Type | Source | Default |
|-------|------|--------|---------|
| callbackUrl | string \| null | Query parameter | null |
| defaultRedirect | string | Constant | '/dashboard' |

**TypeScript Interface**:
```typescript
export interface RedirectParams {
  callbackUrl: string | null;
  defaultRedirect: string;
}

export function getRedirectUrl(searchParams: URLSearchParams): string {
  const callbackUrl = searchParams.get('callbackUrl');
  return callbackUrl || '/dashboard';
}
```

---

## State Management Models

### Form State

**Managed by**: react-hook-form

**States**:
- `isSubmitting`: boolean - Form submission in progress
- `isValid`: boolean - All validations passed
- `errors`: FieldErrors - Validation error messages
- `dirtyFields`: Record<string, boolean> - Fields that have been modified

---

### API Call State

**Purpose**: Track async operations

**TypeScript Interface**:
```typescript
export interface ApiCallState<T> {
  loading: boolean;
  data: T | null;
  error: string | null;
  timestamp: number | null;
}

export function createApiCallState<T>(): ApiCallState<T> {
  return {
    loading: false,
    data: null,
    error: null,
    timestamp: null,
  };
}
```

---

### Failed Login Attempts Tracking

**Purpose**: Client-side tracking for CAPTCHA trigger

**Note**: Server tracks actual rate limiting. Client tracks locally to show CAPTCHA preemptively.

**TypeScript Interface**:
```typescript
export interface FailedAttempts {
  count: number;
  lastAttempt: number; // timestamp
  resetAt: number; // timestamp
}

// localStorage key: 'auth:failed-attempts'
export function getFailedAttempts(): FailedAttempts {
  const stored = localStorage.getItem('auth:failed-attempts');
  if (!stored) {
    return { count: 0, lastAttempt: 0, resetAt: Date.now() + 15 * 60 * 1000 };
  }
  const attempts = JSON.parse(stored);
  // Reset after 15 minutes
  if (Date.now() > attempts.resetAt) {
    return { count: 0, lastAttempt: 0, resetAt: Date.now() + 15 * 60 * 1000 };
  }
  return attempts;
}

export function incrementFailedAttempts(): FailedAttempts {
  const attempts = getFailedAttempts();
  const updated = {
    count: attempts.count + 1,
    lastAttempt: Date.now(),
    resetAt: attempts.resetAt,
  };
  localStorage.setItem('auth:failed-attempts', JSON.stringify(updated));
  return updated;
}

export function clearFailedAttempts(): void {
  localStorage.removeItem('auth:failed-attempts');
}

export function shouldShowCaptcha(): boolean {
  const attempts = getFailedAttempts();
  return attempts.count >= 5;
}
```

---

## Validation Rules Summary

### Email
- **Format**: RFC 5322 (zod's built-in email validator)
- **Length**: 1-255 characters
- **Case**: Normalized to lowercase on submission
- **Unique**: Validated server-side

### Password
- **Length**: 8-128 characters
- **Complexity**: 
  - At least 1 uppercase letter (A-Z)
  - At least 1 lowercase letter (a-z)
  - At least 1 number (0-9)
  - At least 1 special character (!@#$%^&*, etc.)
- **Common Passwords**: Validated server-side (BetterAuth checks against common password lists)

### Name
- **Length**: 1-100 characters
- **Format**: Any UTF-8 characters allowed
- **Sanitization**: Server-side XSS prevention

### Token
- **Format**: Alphanumeric string from email link
- **Validation**: Server-side verification only
- **Expiration**: 1 hour for password reset tokens

---

## Error Handling

### Field-Level Errors

Display inline below each field with red text and icon:

```typescript
export interface FieldError {
  field: string;
  message: string;
}

// Example usage in component:
{errors.email && (
  <p className="text-sm text-destructive mt-1">
    {errors.email.message}
  </p>
)}
```

### Form-Level Errors

Display at top of form in Alert component:

```typescript
export interface FormError {
  type: 'validation' | 'api' | 'timeout';
  message: string;
}

// Example messages:
const ERROR_MESSAGES = {
  INVALID_CREDENTIALS: 'Invalid email or password',
  EMAIL_EXISTS: 'An account with this email already exists',
  TIMEOUT: 'Request timed out. Please try again.',
  NETWORK: 'Network error. Please check your connection.',
  UNKNOWN: 'An unexpected error occurred. Please try again.',
} as const;
```

---

## Relationships to Backend Entities

The client forms map to the following backend entities (defined in `@repo/database`):

| Client Form | Backend Entity | API Endpoint |
|-------------|----------------|--------------|
| SignUpFormData | User | POST /api/auth/sign-up/email |
| SignInFormData | User + Session | POST /api/auth/sign-in/email |
| ForgotPasswordFormData | Password Reset Token | POST /api/auth/forget-password |
| ResetPasswordFormData | User (password update) | POST /api/auth/reset-password |
| - | Email Verification | GET /api/auth/verify-email |
| - | OAuth Account | GET /api/auth/sign-in/{provider} |

---

## State Transitions

### Registration Flow
```
IDLE → VALIDATING → SUBMITTING → SUCCESS → REDIRECTING
                               → ERROR → IDLE
```

### Login Flow
```
IDLE → VALIDATING → SUBMITTING → SUCCESS → REDIRECTING
                               → ERROR → IDLE
      ↓ (after 5 failures)
    CAPTCHA_REQUIRED → VALIDATING → SUBMITTING...
```

### Password Reset Request Flow
```
IDLE → VALIDATING → SUBMITTING → SUCCESS_MESSAGE (stay on page)
                               → ERROR → IDLE
```

### Password Reset Completion Flow
```
LOADING_TOKEN → VALIDATING_TOKEN → VALID
                                 → EXPIRED → ERROR_MESSAGE
                                 → INVALID → ERROR_MESSAGE

VALID → FORM_DISPLAY → VALIDATING → SUBMITTING → SUCCESS → REDIRECTING
                                                → ERROR → VALID
```

---

## Data Flow Diagram

```
┌─────────────┐
│  User Input │
└──────┬──────┘
       │
       ▼
┌─────────────────┐
│ Client          │
│ Validation      │◄── Zod Schema
│ (react-hook-    │
│  form)          │
└──────┬──────────┘
       │ valid
       ▼
┌─────────────────┐
│ authClient      │◄── @repo/auth-config/client
│ API Call        │
└──────┬──────────┘
       │
       ▼
┌─────────────────┐
│ SSO Server      │
│ (BetterAuth)    │
│ localhost:3000  │
└──────┬──────────┘
       │
       ▼
┌─────────────────┐
│ Database        │◄── @repo/database
│ (Neon/Postgres) │
└─────────────────┘
```

---

## Notes

1. **No client-side data persistence**: All user data stored in backend database via BetterAuth
2. **Session management**: Handled by httpOnly cookies set by backend
3. **Form state**: Temporary, cleared on unmount or successful submission
4. **Failed attempts tracking**: localStorage only, server enforces actual rate limits
5. **Type safety**: All forms use TypeScript + Zod for compile-time and runtime validation
