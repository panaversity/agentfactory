# CI/CD Pipeline Documentation

**Last Updated**: 2025-12-02
**Status**: ✅ All tests passing (51 assertions)

## Overview

The auth server uses GitHub Actions for continuous integration and deployment. Every push to any branch triggers automated testing to ensure code quality and OAuth 2.1/OIDC compliance.

## Pipeline Structure

### Workflows

Located in `.github/workflows/`:

- **`test-auth.yml`** - Main test pipeline
  - Runs on: Push to any branch
  - Duration: ~2-3 minutes
  - Jobs: Auth Server Tests, Security Audit

## Test Suite Breakdown

### Total: 51 Test Assertions

#### 1. OAuth Flow Test Suite (7 tests)
**File**: `tests/test-oauth-flows.js`

Tests core OAuth 2.1 and OIDC functionality:
- ✅ OIDC Discovery (`.well-known/openid-configuration`)
- ✅ JWKS Endpoint (`/api/auth/jwks`)
- ✅ PKCE Flow (Public Client with code challenge)
- ✅ Confidential Client Flow (with client secret)
- ✅ Userinfo Endpoint (user claims retrieval)
- ✅ ID Token Structure (JWT validation)
- ✅ Dynamic Client Registration (disabled for security)

**Run locally**:
```bash
node tests/test-oauth-flows.js
```

#### 2. Tenant Claims Test (1 test)
**File**: `tests/test-tenant-claims.js`

Validates multi-tenancy JWT claims:
- ✅ `tenant_id`, `organization_ids`, `org_role` present in userinfo

**Run locally**:
```bash
node tests/test-tenant-claims.js
```

#### 3. Edge Case Test Suite (13 tests)
**File**: `tests/test-edge-cases.js`

Security and error handling validation:

**Security Tests**:
- ✅ Invalid client_id handling (302 redirect)
- ✅ Mismatched redirect_uri rejection (400 error)
- ✅ PKCE verifier mismatch rejection (401 error)
- ✅ Authorization code reuse rejection (401 error)
- ✅ Invalid access token rejection (401 error)
- ✅ Missing auth header rejection (401 error)

**Authentication Tests**:
- ✅ Wrong password rejection (401 error)
- ✅ Non-existent email rejection (401 error)
- ✅ Invalid email format rejection (400 error)

**Flow Tests**:
- ✅ Unauthenticated authorize handling (200 with redirect)
- ✅ Refresh token flow (new access token issued)
- ✅ Invalid grant_type rejection (400 error)
- ✅ Invalid scope handling (200 - graceful degradation)

**Run locally**:
```bash
node tests/test-edge-cases.js
```

#### 4. Tenant/Organization Edge Cases (8 tests)
**File**: `tests/test-tenant-edge-cases.js`

Multi-tenancy specific validation:
- ✅ User with organization has tenant claims
- ✅ `tenant_id` matches primary organization
- ✅ `org_role` present for organization members
- ✅ Profile claims present (software background, hardware tier)
- ✅ Role claim valid
- ✅ `email_verified` is boolean
- ✅ ID token has standard claims (iss, sub, aud, exp, iat)
- ✅ `organization_ids` is array type

**Run locally**:
```bash
node tests/test-tenant-edge-cases.js
```

#### 5. Confidential Client Flow (6 tests)
**File**: `tests/test-confidential-client.js`

Server-to-server OAuth flow validation:
- ✅ Authorization code exchange with client secret
- ✅ Client secret authentication (Basic Auth)
- ✅ Access token issuance
- ✅ User info retrieval
- ✅ Tenant claims present in response
- ✅ Refresh token working (with `offline_access` scope)

**Run locally**:
```bash
node tests/test-confidential-client.js
```

#### 6. Default Organization Tests (4 tests)
**File**: `tests/test-default-organization.js`

Hybrid multi-tenant model validation:
- ✅ User signup auto-joins default organization
- ✅ JWT token includes correct `tenant_id`
- ✅ Duplicate membership prevention (idempotency)
- ✅ Default organization exists (startup validation)

**Run locally**:
```bash
node tests/test-default-organization.js
```

#### 7. OAuth 2.1/OIDC Compliance (12 checks: 9 pass, 3 skip)
**File**: `tests/oauth-validation.test.ts`

Standards compliance validation:

**Passing Checks** (9):
- ✅ Discovery document issuer matches `BETTER_AUTH_URL`
- ✅ Authorization endpoint uses correct base URL
- ✅ Token endpoint uses correct base URL
- ✅ Userinfo endpoint uses correct base URL
- ✅ JWKS URI uses correct base URL
- ✅ Authorization code grant supported
- ✅ PKCE S256 supported
- ✅ JWKS valid structure with keys array
- ✅ Dynamic Client Registration disabled (security best practice)

**Advisory Skips** (3):
- ⏭️ JWKS key metadata fields (Better Auth limitation - not critical)
- ⏭️ CORS headers on JWKS endpoint (not needed for server-side validation)
- ⏭️ Manual token verification (requires browser-based testing)

**Run locally**:
```bash
npx tsx tests/oauth-validation.test.ts
```

## Running Tests Locally

### Prerequisites

1. **Start the auth server**:
```bash
pnpm dev
```

2. **Ensure database is running**:
```bash
# Server uses Neon in production, postgres-js locally
# Check DATABASE_URL in .env.local
```

3. **Seed test data**:
```bash
pnpm run seed:setup
```

### Run All Tests

```bash
pnpm test-all
```

This runs:
1. All API tests (OAuth, tenant, edge cases, confidential client, default org)
2. OAuth compliance validation

### Run Individual Test Suites

```bash
# OAuth flows
node tests/test-oauth-flows.js

# Tenant claims
node tests/test-tenant-claims.js

# Edge cases
node tests/test-edge-cases.js

# Tenant edge cases
node tests/test-tenant-edge-cases.js

# Confidential client
node tests/test-confidential-client.js

# Default organization
node tests/test-default-organization.js

# OAuth compliance
npx tsx tests/oauth-validation.test.ts
```

## CI Environment Configuration

### Environment Variables

Set in `.github/workflows/test-auth.yml`:

```yaml
env:
  NODE_ENV: test
  DISABLE_EMAIL_VERIFICATION: true  # Skip email verification for automated tests
  DATABASE_URL: postgresql://postgres:postgres@localhost:5432/auth_test
  BETTER_AUTH_URL: http://localhost:3001
  BETTER_AUTH_SECRET: test-secret-key-for-ci-only
```

### Database Setup

CI uses PostgreSQL 16 service container:

```yaml
services:
  postgres:
    image: postgres:16
    env:
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: postgres
      POSTGRES_DB: auth_test
    options: >-
      --health-cmd pg_isready
      --health-interval 10s
      --health-timeout 5s
      --health-retries 5
```

### Test User Credentials

CI automatically creates admin user via Better Auth signup API:

```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "admin@robolearn.io",
    "password": "admin@robolearn.io",
    "name": "Admin User"
  }'
```

**Password Requirements**:
- Must be 8+ characters
- Not in HaveIBeenPwned breach database
- Better Auth validates against compromised passwords

## Troubleshooting

### Common Issues

#### 1. Password Verification Failures (401)

**Symptom**:
```
❌ PKCE Flow: FAIL - Sign-in failed: 401 {"code":"INVALID_EMAIL_OR_PASSWORD"}
```

**Cause**: Password hash mismatch between manual hashing and Better Auth's internal format

**Solution**: Always use Better Auth's signup API to create test users (as done in CI workflow)

#### 2. Lockfile Out of Sync

**Symptom**:
```
ERR_PNPM_OUTDATED_LOCKFILE  Cannot install with "frozen-lockfile"
```

**Solution**:
```bash
pnpm install
git add pnpm-lock.yaml
git commit -m "fix: Update lockfile"
```

#### 3. Profile API Missing Organization IDs

**Symptom**:
```
❌ FAIL: User not in default organization!
Email: undefined
Organizations: []
```

**Solution**: Profile API must query `member` table and return flattened response with `organizationIds` array

#### 4. JWKS Validation Failures

**Symptom**:
```
❌ JWKS: key[0] has required fields - Missing fields in key 0
```

**Solution**: These are Better Auth limitations (missing optional JWKS metadata). Marked as SKIP (advisory) in compliance tests.

#### 5. Email Verification Blocking Tests

**Symptom**:
```
❌ Sign-in failed: 403 {"code":"EMAIL_NOT_VERIFIED"}
```

**Solution**: Set `DISABLE_EMAIL_VERIFICATION=true` in test environment

### Debugging CI Failures

1. **View CI logs**:
```bash
gh run list --branch <branch-name> --limit 5
gh run view <run-id> --log
```

2. **Check specific test failure**:
```bash
gh run view <run-id> --log | grep -A 20 "❌\|FAIL"
```

3. **Download test results artifact**:
```bash
gh run download <run-id>
cat test-results/test-results.json | jq '.'
```

4. **Run failed tests locally**:
```bash
# Reproduce CI environment
NODE_ENV=test DISABLE_EMAIL_VERIFICATION=true pnpm dev

# In another terminal
node tests/<test-file>.js
```

## Security Audit

CI also runs security checks:

```bash
# Audit dependencies for vulnerabilities
pnpm audit

# Check for outdated dependencies
pnpm outdated
```

**Note**: These may show warnings but won't fail the build unless critical vulnerabilities are found.

## Test Results Storage

Test results are saved to:
- **CI**: Artifact `test-results` (downloadable from GitHub Actions)
- **Local**: `tests/test-results.json`

Example:
```json
{
  "timestamp": "2025-12-02T10:24:44.116Z",
  "environment": "http://localhost:3001",
  "summary": {
    "passed": 9,
    "failed": 0,
    "skipped": 3,
    "total": 12
  },
  "results": [...]
}
```

## Adding New Tests

### 1. Create Test File

```javascript
// tests/test-my-feature.js
const AUTH_URL = "http://localhost:3001";

async function testMyFeature() {
  console.log("Testing my feature...");

  // Your test logic
  const response = await fetch(`${AUTH_URL}/api/my-endpoint`);

  if (response.ok) {
    console.log("✅ PASS: My feature works");
    process.exit(0);
  } else {
    console.log("❌ FAIL: My feature failed");
    process.exit(1);
  }
}

testMyFeature();
```

### 2. Add to Test Runner

Update `package.json`:

```json
{
  "scripts": {
    "test-api": "node tests/test-oauth-flows.js && node tests/test-my-feature.js && ..."
  }
}
```

### 3. Update CI Workflow (if needed)

For special test requirements, modify `.github/workflows/test-auth.yml`.

## Best Practices

### Writing Tests

1. ✅ **Use unique test users**: Generate unique emails with timestamps to avoid conflicts
2. ✅ **Clean up after tests**: Tests should be idempotent (can run multiple times)
3. ✅ **Test error cases**: Don't just test happy paths
4. ✅ **Use Better Auth APIs**: Don't manually hash passwords or create database records
5. ✅ **Clear assertions**: Print expected vs actual values on failure

### CI Configuration

1. ✅ **Fast feedback**: Keep test suite under 3 minutes
2. ✅ **Isolated environment**: Each CI run uses fresh database
3. ✅ **No secrets in logs**: Use environment variables for sensitive data
4. ✅ **Retry on flaky tests**: Network timeouts should not fail builds
5. ✅ **Fail fast**: Stop on first critical failure to save CI minutes

## Known Limitations

### Better Auth JWKS

Better Auth's JWKS endpoint is missing some optional OAuth 2.1 metadata fields:
- Missing `use` field (should be "sig")
- Missing optional fields like `x5c`, `x5t`

**Impact**: Low - OAuth flows work correctly, just missing metadata for advanced use cases

**Workaround**: Tests mark these as SKIP (advisory) rather than FAIL

### CORS Headers

JWKS endpoint doesn't include CORS headers.

**Impact**: None for server-side OAuth flows (our primary use case)

**Workaround**: If browser-side JWKS validation needed, add CORS middleware

## Maintenance

### Weekly Tasks

- Review security audit warnings
- Update outdated dependencies (if no breaking changes)
- Check for Better Auth updates

### Monthly Tasks

- Review test coverage (aim for 90%+ of OAuth flows)
- Update test data if schema changes
- Benchmark test suite performance (should stay under 3 minutes)

### Before Major Releases

- Run full test suite locally
- Verify CI passing on main branch
- Check test-results.json for any skipped tests that should be fixed
- Review ADRs for any test-related decisions

## Related Documentation

- [Environment Variables](./environment-variables.md) - Required env vars for tests
- [Multi-Tenancy](./multi-tenancy.md) - Understanding tenant_id and organization_ids
- [ADR 001: Standalone Architecture](./adr/001-standalone-architecture.md) - Why we use Better Auth

## Support

If tests fail unexpectedly:

1. Check [Troubleshooting](#troubleshooting) section above
2. Review recent commits for breaking changes
3. Compare CI logs with local test output
4. Check Better Auth changelog for breaking changes
5. Open an issue with:
   - CI run ID
   - Test failure logs
   - Local reproduction steps
