# Proposal 001: Multi-Tenant Profile & Branding Architecture

**Status**: Draft
**Created**: 2025-12-01
**Author**: Architecture Review

---

## Executive Summary

This proposal addresses four architectural concerns identified during auth server hardening:

1. **Redundant Profile System**: Current `user_profile` table with RoboLearn-specific fields (`softwareBackground`, `hardwareTier`) doesn't belong in a multi-tenant auth server
2. **Hardcoded Branding**: Login/signup pages show "RoboLearn" branding, breaking the multi-tenant model
3. **Non-Standard Claims**: Custom profile fields don't align with OIDC standard claims
4. **No Profile Management**: Users lack a UI to manage their profile, forcing developer intervention for basic updates

**Recommendation**:
- Migrate to Better Auth's `additionalFields` for OIDC-compliant user profiles
- Implement dynamic branding per OAuth client
- **Provide Clerk-style profile management UI** at `/account/profile` that:
  - Allows users to edit OIDC standard claims (name, email, picture, phone, locale, timezone)
  - Dynamically renders up to 3 tenant-defined custom fields per organization
  - Supports multi-organization users with organization switcher
  - Saves ALL tenants from building their own profile management

---

## Current Architecture Problems

### Problem 1: RoboLearn-Specific Profile in Generic Auth Server

```typescript
// Current schema - too specific for multi-tenant
export const userProfile = pgTable("user_profile", {
  userId: text("user_id").primaryKey(),
  softwareBackground: text("software_background"),  // RoboLearn-specific
  hardwareTier: text("hardware_tier"),              // RoboLearn-specific
});
```

**Issues**:
- Hardware tier question makes no sense for non-robotics tenants
- Profile data duplicated between `user` and `user_profile` tables
- Not following OIDC standard claims

### Problem 2: Hardcoded Branding

```tsx
// Current - hardcoded in sign-in-form.tsx
<h2>Welcome to RoboLearn</h2>
<p>Sign in to continue your learning journey</p>
```

**Issues**:
- Users from other tenants see "RoboLearn" branding
- Breaks trust - users don't know which app they're signing into
- No tenant identity during auth flow

### Problem 3: Cross-Tenant User Confusion

**Scenario**: User registers with Tenant A, later tries to access Tenant B's app.

**Current behavior**: Unclear - no defined handling
**Expected behavior**: Clear flow for existing vs. new tenant membership

### Problem 4: No Profile Management

**Current state**: No UI for users to manage their profile

**Industry expectation**: Auth providers (Clerk, Auth0, Keycloak) provide basic profile management UI

**User pain point**: Users can't update their name, email, picture, or tenant-specific fields without developer intervention

---

## Proposed Architecture

### 1. Profile Architecture: OIDC Standard Claims + Tenant Custom Fields

#### 1.1 Better Auth `additionalFields` for OIDC Claims

Extend the `user` table with OIDC-compliant fields:

```typescript
// src/lib/auth.ts - additionalFields configuration
export const auth = betterAuth({
  user: {
    additionalFields: {
      // OIDC Standard Claims
      givenName: { type: "string", required: false },
      familyName: { type: "string", required: false },
      picture: { type: "string", required: false },
      phoneNumber: { type: "string", required: false },
      phoneNumberVerified: { type: "boolean", defaultValue: false },
      locale: { type: "string", required: false },
      zoneinfo: { type: "string", required: false },
    }
  },
  // ... rest of config
});
```

**OIDC Standard Claims Reference** (RFC 7519, OpenID Connect Core 1.0):

| Claim | Type | Description | Source |
|-------|------|-------------|--------|
| `sub` | string | Subject identifier | Better Auth default |
| `name` | string | Full name | Better Auth default |
| `given_name` | string | First name | additionalFields |
| `family_name` | string | Last name | additionalFields |
| `picture` | string | Profile picture URL | additionalFields |
| `email` | string | Email address | Better Auth default |
| `email_verified` | boolean | Email verified flag | Better Auth default |
| `phone_number` | string | Phone number (E.164) | additionalFields |
| `phone_number_verified` | boolean | Phone verified | additionalFields |
| `locale` | string | Locale (BCP47, e.g., "en-US") | additionalFields |
| `zoneinfo` | string | Timezone (e.g., "America/New_York") | additionalFields |

#### 1.2 Tenant-Specific Custom Fields

Tenants can define **up to 3 custom fields** stored in `member.metadata`:

```typescript
// Organization configuration (stored in organization.metadata)
{
  "customFields": [
    {
      "key": "hardwareTier",
      "label": "Hardware Tier",
      "type": "select",
      "options": ["tier1", "tier2", "tier3", "tier4"],
      "required": true
    },
    {
      "key": "softwareBackground",
      "label": "Software Background",
      "type": "select",
      "options": ["beginner", "intermediate", "advanced"],
      "required": false
    }
  ]
}

// Member-specific values (stored in member.metadata)
{
  "hardwareTier": "tier2",
  "softwareBackground": "intermediate"
}
```

**Why 3 fields max?**
- Prevents auth server becoming a CRM
- Keeps sign-up flow fast
- Forces tenants to collect detailed data in their own apps

#### 1.3 Userinfo Response Structure

```json
{
  "sub": "user_abc123",
  "name": "John Doe",
  "given_name": "John",
  "family_name": "Doe",
  "email": "john@example.com",
  "email_verified": true,
  "picture": "https://...",
  "phone_number": "+1234567890",
  "phone_number_verified": false,
  "locale": "en-US",
  "zoneinfo": "America/New_York",

  "tenant_id": "org_robolearn",
  "organization_ids": ["org_robolearn"],
  "org_role": "member",

  "tenant_profile": {
    "hardwareTier": "tier2",
    "softwareBackground": "intermediate"
  }
}
```

#### 1.4 Profile Management UI (Clerk-style Approach)

**Rationale**: Following industry standards (Clerk, Auth0), the auth server provides a profile management page that saves ALL tenants from building the same UI.

**Page Location**: `/account/profile`

**UI Structure**:

```
┌─────────────────────────────────────────────────────┐
│  Profile Settings                                    │
├─────────────────────────────────────────────────────┤
│  [STANDARD OIDC CLAIMS - Always Present]             │
│                                                      │
│  Name:            [John                            ] │
│  Family Name:     [Doe                             ] │
│  Email:           [john@example.com                ] │
│                   (✓ Verified)                       │
│  Picture URL:     [https://...                     ] │
│  Phone:           [+1234567890                     ] │
│                   (○ Not verified) [Verify]          │
│  Locale:          [en-US                 ▼]         │
│  Timezone:        [America/New_York      ▼]         │
│                                                      │
├─────────────────────────────────────────────────────┤
│  [ORGANIZATION PROFILE - Dynamic per Active Org]     │
│                                                      │
│  Active Organization: [RoboLearn            ▼]      │
│                                                      │
│  Hardware Tier:   [GPU                   ▼]         │
│  (Select your available hardware)                    │
│                                                      │
│  Software Experience: [Advanced          ▼]         │
│  (Your programming skill level)                      │
│                                                      │
├─────────────────────────────────────────────────────┤
│  [Save Changes]  [Cancel]                            │
└─────────────────────────────────────────────────────┘
```

**Custom Field Schema**:

Each organization defines up to 3 custom fields in `organization.metadata.customFields`:

```typescript
{
  "key": "string",              // Unique identifier (snake_case)
  "label": "string",            // Display label
  "type": "select|text|number|checkbox",
  "options": ["string[]"],      // For select type (2-10 options)
  "required": boolean,          // Whether field is mandatory
  "description": "string"       // Help text shown below field
}
```

**Supported Field Types**:
- `select` - Dropdown menu (most common, 2-10 options)
- `text` - Short text input (max 255 chars)
- `number` - Numeric input
- `checkbox` - Boolean toggle

**Validation Rules**:
- Max 3 custom fields per organization
- Field keys must be unique within organization
- Select type must have 2-10 options
- Text fields max 255 characters
- Required fields validated before save

**Multi-Organization Handling**:

For users belonging to multiple organizations:

```
┌─────────────────────────────────────────────────────┐
│  Active Organization: [RoboLearn            ▼]      │
│                       • RoboLearn                    │
│                       • DataCorp                     │
│                       • AI Academy                   │
├─────────────────────────────────────────────────────┤
│  [Organization-specific fields for selected org]     │
└─────────────────────────────────────────────────────┘
```

When user switches organization:
1. UI fetches custom fields for new org from `organization.metadata`
2. Loads user's values from `member.metadata` for that org
3. Dynamically renders appropriate form fields

**Why This Approach?**

| Benefit | Impact |
|---------|--------|
| **Industry Standard** | Clerk, Auth0 provide profile UI - we match expectations |
| **Saves Tenant Dev Time** | Each tenant saves 5-10 hours not building profile management |
| **Consistent UX** | All users get same profile experience across tenants |
| **Multi-tenant Ready** | Same UI adapts to each org's custom fields |
| **OIDC Compliant** | Standard claims + extensible custom fields |
| **Bounded Complexity** | Max 3 fields prevents auth server becoming CRM |

### 2. Dynamic Branding Architecture

#### 2.1 Client-to-Tenant Mapping

```typescript
// Schema addition
export const oauth2Client = pgTable("oauth2_client", {
  // ... existing fields
  organizationId: text("organization_id").references(() => organization.id),
  brandName: text("brand_name"),           // Display name for auth pages
  brandLogo: text("brand_logo"),           // Logo URL
  brandPrimaryColor: text("brand_primary_color"), // Hex color
  brandAccentColor: text("brand_accent_color"),   // Hex color
});
```

#### 2.2 Auth Page Flow

```
1. User visits: /auth/sign-in?client_id=robolearn-public-client
2. Server looks up client → gets organizationId → gets branding
3. Page renders with tenant branding:
   - Logo: RoboLearn logo
   - Title: "Sign in to RoboLearn"
   - Colors: RoboLearn brand colors
```

#### 2.3 Fallback for Direct Access

When users access `/auth/sign-in` directly (no `client_id`):
- Show neutral branding: "Sign in to your account"
- Generic auth server logo
- After sign-in, redirect to account management or last-used tenant

### 3. Cross-Tenant User Handling

#### Scenario A: New User to New Tenant
```
1. User clicks "Sign in with [Auth Server]" from Tenant B app
2. User has no account → Sign up flow
3. After sign-up, user auto-joins Tenant B organization
4. Tenant B's custom fields collected during onboarding
```

#### Scenario B: Existing User to New Tenant
```
1. User clicks "Sign in with [Auth Server]" from Tenant B app
2. User has account from Tenant A → Sign in
3. After sign-in, check membership:
   - Not member of Tenant B → Prompt: "Join [Tenant B]?"
   - If tenant requires custom fields → Collect them
   - Create member record with role & custom field values
4. Redirect to Tenant B app with tokens
```

#### Scenario C: Existing User to Existing Tenant
```
1. User clicks "Sign in" from Tenant A app
2. User already member of Tenant A
3. Normal sign-in, redirect with tokens
```

---

## Schema Changes

### Drop Tables

```sql
-- Remove RoboLearn-specific profile table
DROP TABLE IF EXISTS user_profile;
```

### Alter Tables

```sql
-- Add OIDC standard claims to user table
ALTER TABLE "user" ADD COLUMN "given_name" TEXT;
ALTER TABLE "user" ADD COLUMN "family_name" TEXT;
ALTER TABLE "user" ADD COLUMN "picture" TEXT;
ALTER TABLE "user" ADD COLUMN "phone_number" TEXT;
ALTER TABLE "user" ADD COLUMN "phone_number_verified" BOOLEAN DEFAULT FALSE;
ALTER TABLE "user" ADD COLUMN "locale" TEXT;
ALTER TABLE "user" ADD COLUMN "zoneinfo" TEXT;

-- Add branding to oauth2_client
ALTER TABLE "oauth2_client" ADD COLUMN "organization_id" TEXT REFERENCES "organization"("id");
ALTER TABLE "oauth2_client" ADD COLUMN "brand_name" TEXT;
ALTER TABLE "oauth2_client" ADD COLUMN "brand_logo" TEXT;
ALTER TABLE "oauth2_client" ADD COLUMN "brand_primary_color" TEXT;
ALTER TABLE "oauth2_client" ADD COLUMN "brand_accent_color" TEXT;

-- Add custom field definitions to organization
ALTER TABLE "organization" ADD COLUMN "custom_fields" JSONB DEFAULT '[]';

-- Add tenant-specific profile to member
ALTER TABLE "member" ADD COLUMN "metadata" JSONB DEFAULT '{}';
```

---

## Implementation Plan

### Phase 1: Schema Migration (No Breaking Changes Yet)

1. Add new columns to `user`, `oauth2_client`, `organization`, `member`
2. Keep `user_profile` table temporarily
3. Update Drizzle schema
4. Run migration

### Phase 2: Profile System Update

1. Configure Better Auth `additionalFields`
2. Update sign-up form to collect OIDC fields (name, email only - keep it minimal)
3. Update userinfo endpoint to return OIDC claims + tenant_profile
4. **Create `/account/profile` page**:
   - Form for OIDC standard claims (name, email, picture, phone, locale, timezone)
   - Organization switcher dropdown (for multi-org users)
   - Dynamic custom field renderer based on active org
   - API endpoints: `GET/PATCH /api/account/profile`, `GET/PATCH /api/account/member-profile`
5. Add phone verification flow (Better Auth phone plugin)

### Phase 3: Dynamic Branding

1. Create branding lookup API
2. Update auth page layout to accept branding props
3. Update sign-in/sign-up pages to use dynamic branding
4. Add branding configuration to client registration

### Phase 4: Tenant Custom Fields

1. Add custom field configuration UI for organization admins
2. Create dynamic form renderer for tenant fields
3. Update member join flow to collect custom fields
4. Add tenant_profile to userinfo response

### Phase 5: Cross-Tenant Flows

1. Implement "Join Organization" flow
2. Add membership check after authentication
3. Handle custom field collection for new tenants
4. Update redirect logic for multi-tenant scenarios

### Phase 6: Cleanup

1. Migrate any remaining `user_profile` data to appropriate locations
2. Drop `user_profile` table
3. Update RoboLearn client to use new custom fields system
4. Update documentation

---

## API Changes

### Userinfo Response (Updated)

```
GET /api/auth/oauth2/userinfo
Authorization: Bearer <access_token>

Response:
{
  // OIDC Standard Claims
  "sub": "user_abc123",
  "name": "John Doe",
  "given_name": "John",
  "family_name": "Doe",
  "email": "john@example.com",
  "email_verified": true,
  "picture": "https://...",
  "phone_number": "+1234567890",
  "phone_number_verified": false,
  "locale": "en-US",
  "zoneinfo": "America/New_York",

  // Tenant Claims
  "tenant_id": "org_robolearn",
  "organization_ids": ["org_robolearn", "org_other"],
  "org_role": "member",

  // Tenant-Specific Profile (from member.metadata)
  "tenant_profile": {
    "hardwareTier": "tier2",
    "softwareBackground": "intermediate"
  }
}
```

### Branding Lookup

```
GET /api/branding?client_id=robolearn-public-client

Response:
{
  "brandName": "RoboLearn",
  "brandLogo": "https://robolearn.io/logo.png",
  "brandPrimaryColor": "#4F46E5",
  "brandAccentColor": "#818CF8",
  "organizationId": "org_robolearn"
}
```

### Organization Custom Fields

```
GET /api/organizations/{orgId}/custom-fields

Response:
{
  "customFields": [
    {
      "key": "hardwareTier",
      "label": "Hardware Tier",
      "type": "select",
      "options": ["tier1", "tier2", "tier3", "tier4"],
      "required": true
    }
  ]
}
```

### Profile Management APIs (New)

```
GET /api/account/profile
Authorization: Bearer <session_token>

Response:
{
  "sub": "user_abc123",
  "name": "John Doe",
  "given_name": "John",
  "family_name": "Doe",
  "email": "john@example.com",
  "email_verified": true,
  "picture": "https://...",
  "phone_number": "+1234567890",
  "phone_number_verified": false,
  "locale": "en-US",
  "zoneinfo": "America/New_York"
}
```

```
PATCH /api/account/profile
Authorization: Bearer <session_token>
Content-Type: application/json

Request:
{
  "given_name": "John",
  "family_name": "Doe",
  "picture": "https://...",
  "locale": "en-US",
  "zoneinfo": "America/New_York"
}

Response:
{
  "success": true,
  "user": { ...updated user }
}
```

```
GET /api/account/member-profile?organizationId=org_robolearn
Authorization: Bearer <session_token>

Response:
{
  "organizationId": "org_robolearn",
  "customFields": [
    {
      "key": "hardwareTier",
      "label": "Hardware Tier",
      "type": "select",
      "options": ["tier1", "tier2", "tier3", "tier4"],
      "required": true
    }
  ],
  "values": {
    "hardwareTier": "tier2",
    "softwareBackground": "advanced"
  }
}
```

```
PATCH /api/account/member-profile
Authorization: Bearer <session_token>
Content-Type: application/json

Request:
{
  "organizationId": "org_robolearn",
  "values": {
    "hardwareTier": "tier3",
    "softwareBackground": "advanced"
  }
}

Response:
{
  "success": true,
  "member": { ...updated member }
}
```

---

## Migration Guide for Existing Tenants

### RoboLearn Migration

1. **Update OAuth Client**:
   ```sql
   UPDATE oauth2_client
   SET organization_id = 'org_robolearn',
       brand_name = 'RoboLearn',
       brand_logo = 'https://robolearn.io/logo.png',
       brand_primary_color = '#4F46E5'
   WHERE client_id = 'robolearn-public-client';
   ```

2. **Configure Custom Fields**:
   ```sql
   UPDATE organization
   SET custom_fields = '[
     {"key": "hardwareTier", "label": "Hardware Tier", "type": "select", "options": ["tier1", "tier2", "tier3", "tier4"], "required": true},
     {"key": "softwareBackground", "label": "Learning Profile", "type": "select", "options": ["beginner", "intermediate", "advanced"], "required": false}
   ]'
   WHERE id = 'org_robolearn';
   ```

3. **Migrate Existing Profiles**:
   ```sql
   INSERT INTO member (id, user_id, organization_id, role, metadata, created_at)
   SELECT
     gen_random_uuid(),
     up.user_id,
     'org_robolearn',
     'member',
     jsonb_build_object(
       'hardwareTier', up.hardware_tier,
       'softwareBackground', up.software_background
     ),
     NOW()
   FROM user_profile up
   WHERE NOT EXISTS (
     SELECT 1 FROM member m
     WHERE m.user_id = up.user_id
     AND m.organization_id = 'org_robolearn'
   );
   ```

---

## Security Considerations

1. **Tenant Isolation**: Custom fields stored per-membership, not globally
2. **Field Validation**: Server-side validation of custom field types/options
3. **Branding Verification**: Only registered clients can set branding
4. **Cross-Tenant Data**: Users can't see other tenants' custom field definitions
5. **Phone Verification**: Uses separate verified flag, not auto-trusted

---

## Testing Requirements

### Unit Tests
- [ ] OIDC claims correctly populated in userinfo
- [ ] Custom fields saved to member.metadata
- [ ] Branding lookup returns correct data
- [ ] Cross-tenant membership flow works

### Integration Tests
- [ ] Full OAuth flow with new userinfo structure
- [ ] Dynamic branding renders correctly
- [ ] Custom field collection during sign-up
- [ ] Existing user joining new tenant

### Visual Tests
- [ ] Branded sign-in page renders correctly
- [ ] Custom field forms display properly during sign-up
- [ ] Profile management page (`/account/profile`) works
- [ ] OIDC standard claims editable
- [ ] Custom fields dynamically render per organization
- [ ] Organization switcher works for multi-org users
- [ ] Phone verification flow works
- [ ] Form validation displays errors correctly

---

## Success Criteria

1. **No RoboLearn-specific code in auth server** (all tenant-configurable)
2. **OIDC-compliant userinfo response** (passes conformance tests)
3. **Dynamic branding** based on client_id
4. **Profile management UI** at `/account/profile` with:
   - OIDC standard claims editable
   - Dynamic custom field rendering per org
   - Organization switcher for multi-org users
   - Phone verification integration
5. **Tenant custom fields** (max 3) working end-to-end:
   - Schema definition in `organization.metadata`
   - Values stored in `member.metadata`
   - Dynamic form rendering in UI
6. **Clean migration** from user_profile table
7. **All existing tests pass** + new tests for multi-tenant features

---

## Open Questions

1. **Phone verification**: Enable Better Auth's phone plugin in Phase 2 (YES - needed for OIDC compliance)
2. **Profile picture upload**: Start with URL input only, defer file upload to Phase 7 (future enhancement)
3. **Custom field types**: Start with select/text/number/checkbox (date type deferred)
4. **Branding preview**: Add preview in organization settings (Phase 3)
5. **Profile UI framework**: Use existing auth page styling or create new design system?
6. **Multi-org UX**: Should we show all orgs' custom fields on one page or require switching?
7. **Custom field validation**: Should orgs define regex patterns for text fields?
8. **Avatar hosting**: If we add file upload later, use Cloudflare R2, S3, or local storage?

---

## Appendix: OIDC Claims Reference

### Standard Claims (OpenID Connect Core 1.0, Section 5.1)

| Claim | Type | Description |
|-------|------|-------------|
| sub | string | Subject - Identifier for the End-User |
| name | string | End-User's full name |
| given_name | string | Given name(s) or first name(s) |
| family_name | string | Surname(s) or last name(s) |
| middle_name | string | Middle name(s) |
| nickname | string | Casual name |
| preferred_username | string | Shorthand name by which the End-User wishes to be referred to |
| profile | string | URL of the End-User's profile page |
| picture | string | URL of the End-User's profile picture |
| website | string | URL of the End-User's Web page or blog |
| email | string | End-User's preferred e-mail address |
| email_verified | boolean | True if the End-User's e-mail address has been verified |
| gender | string | End-User's gender |
| birthdate | string | End-User's birthday (YYYY-MM-DD or YYYY) |
| zoneinfo | string | String from zoneinfo time zone database (e.g., Europe/Paris) |
| locale | string | End-User's locale (BCP47, e.g., en-US) |
| phone_number | string | End-User's preferred telephone number (E.164) |
| phone_number_verified | boolean | True if the End-User's phone number has been verified |
| address | JSON object | End-User's preferred postal address |
| updated_at | number | Time the End-User's information was last updated |

### Claims Implemented in This Proposal

- [x] sub (Better Auth default)
- [x] name (Better Auth default)
- [x] given_name (additionalFields)
- [x] family_name (additionalFields)
- [x] picture (additionalFields)
- [x] email (Better Auth default)
- [x] email_verified (Better Auth default)
- [x] phone_number (additionalFields)
- [x] phone_number_verified (additionalFields)
- [x] locale (additionalFields)
- [x] zoneinfo (additionalFields)
- [ ] middle_name (not needed for MVP)
- [ ] nickname (not needed for MVP)
- [ ] preferred_username (not needed for MVP)
- [ ] profile (not needed for MVP)
- [ ] website (not needed for MVP)
- [ ] gender (not needed for MVP)
- [ ] birthdate (not needed for MVP)
- [ ] address (not needed for MVP)
- [ ] updated_at (can add later)
