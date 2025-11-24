# API Contracts Overview

This directory contains TypeScript interface definitions for all API contracts used by the SSO client authentication pages.

## Files

- **auth-api.ts**: Complete API contracts for BetterAuth integration
  - Sign up (email/password)
  - Sign in (email/password)
  - Password reset (request + completion)
  - Email verification
  - Session management
  - OAuth social login redirects
  - OIDC authorization + consent
  - Error handling and timeout utilities

## Usage

Import these types in client components to ensure type safety:

```typescript
import type {
  SignUpRequest,
  SignUpResponse,
  SignInRequest,
  SignInResponse,
  // ... other types
} from '@/specs/001-sso-client-auth-pages/contracts/auth-api';
```

Or reference directly from the authClient wrapper functions.

## Contract Validation

All contracts match the BetterAuth API specification:
- Request/Response shapes align with `@repo/auth-config` package
- Error codes match backend error responses
- Type safety enforced at compile time
- Runtime validation handled by Zod schemas in data-model.md

## Testing

Contract types should be used in:
- Unit tests (mocked API responses)
- Integration tests (actual authClient calls)
- Storybook components (mock data fixtures)
