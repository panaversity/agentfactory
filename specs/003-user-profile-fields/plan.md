# Implementation Plan: User Profile Additional Fields

**Branch**: `003-user-profile-fields` | **Date**: 2025-12-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-user-profile-fields/spec.md`

## Summary

Add four new profile fields (gender, father's name, city, country) to the Panaversity SSO user profile, plus expose existing phone number field in UI. Implementation uses Better Auth's official `additionalFields` pattern for schema extension, `authClient.updateUser()` for client-side updates, and `getAdditionalUserInfoClaim` for JWT token claims. UI enhanced with npm packages for international phone input and country/city selection.

## Technical Context

**Language/Version**: TypeScript 5.6, React 18.3, Next.js 15.1
**Primary Dependencies**: Better Auth 1.4.4, Drizzle ORM 0.36, react-phone-number-input, react-country-state-city
**Storage**: Neon PostgreSQL (via Drizzle)
**Testing**: Playwright (e2e), custom API tests (`pnpm test-api`)
**Target Platform**: Web (Next.js App Router)
**Project Type**: Full-stack web application (monolithic)
**Performance Goals**: Profile edit page < 2s load, form save < 1s
**Constraints**: Additive schema only, backward compatible, no signup changes
**Scale/Scope**: 100k users, 4 files modified, 2 npm packages added

## Constitution Check

*GATE: Must pass before implementation. All items verified against `.specify/memory/constitution.md`*

| Principle | Status | Evidence |
|-----------|--------|----------|
| Defense in Depth | ✅ PASS | Data validated at UI, API, and DB layers |
| Least Privilege | ✅ PASS | Only authenticated users can edit own profile |
| Fail Secure | ✅ PASS | Save errors reject changes, don't corrupt data |
| No Secrets in Client | ✅ PASS | No secrets involved in profile fields |
| Standards Compliance | ✅ PASS | Uses Better Auth official patterns |
| Secure Defaults | ✅ PASS | All new fields nullable, no security implications |
| Audit Everything | ✅ PASS | Profile updates logged by Better Auth |
| Token Hygiene | ✅ PASS | New claims follow existing token patterns |

## Project Structure

### Documentation (this feature)

```text
specs/003-user-profile-fields/
├── spec.md              # Feature specification
├── plan.md              # This file
├── checklists/
│   └── requirements.md  # Validation checklist
└── tasks.md             # Task breakdown (created by /sp.tasks)
```

### Source Code (files to modify)

```text
# Schema Layer
auth-schema.ts                              # Add 4 columns to user table

# Configuration Layer
src/lib/auth.ts                             # Add additionalFields + claims

# Type Layer
src/types/profile.ts                        # Add Gender type, update ProfileData

# UI Layer
src/app/account/profile/ProfileForm.tsx     # Add form fields + UI section

# Package Layer
package.json                                # Add npm dependencies
```

**Structure Decision**: Existing Next.js App Router structure. No new directories needed. All changes are modifications to existing files following established patterns.

## Implementation Architecture

### Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        ProfileForm.tsx                          │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────────┐   │
│  │ Phone Input │ │Gender Select│ │ Country/City Selectors  │   │
│  │ (npm pkg)   │ │ (native)    │ │ (npm pkg)               │   │
│  └──────┬──────┘ └──────┬──────┘ └───────────┬─────────────┘   │
│         │               │                     │                 │
│         └───────────────┴─────────────────────┘                 │
│                         │                                       │
│                         ▼                                       │
│              authClient.updateUser({...})                       │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Better Auth Server                         │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ additionalFields: { gender, fatherName, city, country }  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                          │                                      │
│                          ▼                                      │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ getAdditionalUserInfoClaim() → JWT token claims          │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Neon PostgreSQL                              │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ user table: + gender, father_name, city, country columns │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **User edits profile** → ProfileForm captures input
2. **Form submit** → `authClient.updateUser()` sends to Better Auth
3. **Better Auth** → Validates fields, updates database via Drizzle
4. **Database** → Stores in user table columns
5. **OAuth flow** → `getAdditionalUserInfoClaim()` adds to JWT
6. **Client app** → Receives claims in ID token

## Implementation Phases

### Phase 1: Schema & Configuration (Backend)

**Files**: `auth-schema.ts`, `src/lib/auth.ts`

1. Add 4 columns to user table in `auth-schema.ts`:
   ```typescript
   gender: text("gender"),
   fatherName: text("father_name"),
   city: text("city"),
   country: text("country"),
   ```

2. Add 4 additionalFields to Better Auth config in `src/lib/auth.ts`:
   ```typescript
   gender: { type: "string", required: false },
   fatherName: { type: "string", required: false },
   city: { type: "string", required: false },
   country: { type: "string", required: false },
   ```

3. Update `getAdditionalUserInfoClaim` to return new fields:
   ```typescript
   gender: user.gender || null,
   father_name: user.fatherName || null,
   city: user.city || null,
   country: user.country || null,
   ```

4. Run `pnpm db:push` to migrate database

### Phase 2: Type Definitions

**Files**: `src/types/profile.ts`

1. Add Gender type:
   ```typescript
   export type Gender = "male" | "female" | "other" | "prefer_not_to_say";
   ```

2. Update ProfileData interface:
   ```typescript
   export interface ProfileData {
     softwareBackground: SoftwareBackground;
     hardwareTier: HardwareTier;
     gender?: Gender;
     fatherName?: string;
     city?: string;
     country?: string;
     phoneNumber?: string;
   }
   ```

### Phase 3: NPM Dependencies

**Files**: `package.json`

Install packages for enhanced UX:
- `react-phone-number-input` - International phone input with country codes
- `react-country-state-city` - Country/city cascading selectors

```bash
pnpm add react-phone-number-input react-country-state-city
```

### Phase 4: UI Implementation (Frontend)

**Files**: `src/app/account/profile/ProfileForm.tsx`

1. Import npm packages and add to formData state:
   ```typescript
   phoneNumber: user.phoneNumber || "",
   gender: user.gender || "",
   fatherName: user.fatherName || "",
   city: user.city || "",
   country: user.country || "",
   ```

2. Add fields to updateUser call:
   ```typescript
   phoneNumber: formData.phoneNumber,
   gender: formData.gender,
   fatherName: formData.fatherName,
   city: formData.city,
   country: formData.country,
   ```

3. Add "Additional Information" UI section with:
   - Phone Number (react-phone-number-input component)
   - Gender (select dropdown with 4 options)
   - Father's Name (text input)
   - City/Country (react-country-state-city components)

### Phase 5: Testing & Validation

1. Run `pnpm test-api` - Ensure existing tests pass
2. Manual testing:
   - Edit profile with all fields
   - Verify data persists on reload
   - Verify fields appear in JWT token claims
   - Verify signup flow unchanged

## NPM Package Selection

### Phone Input: react-phone-number-input

**Why chosen**:
- Most popular (uses libphonenumber-js for validation)
- Minimalist UI, easy to style
- International format support
- [NPM Package](https://www.npmjs.com/package/react-phone-number-input)

### Country/City: react-country-state-city

**Why chosen**:
- Comprehensive country → state → city data
- Searchable dropdowns
- 9,936 weekly downloads, actively maintained
- [NPM Package](https://www.npmjs.com/package/react-country-state-city)

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Schema migration fails | Low | High | Additive columns only, test on staging first |
| NPM package breaks | Low | Medium | Pin versions, fallback to native inputs |
| Type inference issues | Low | Low | Explicit type annotations |
| Performance impact | Low | Low | Lazy load npm components |

## Dependencies

- **Prerequisite**: Better Auth 1.4.4+ (confirmed in package.json)
- **Prerequisite**: Neon PostgreSQL (confirmed via DATABASE_URL)
- **No blocking dependencies**: All changes are additive

## Test Strategy

| Test Type | Scope | Tool |
|-----------|-------|------|
| API Tests | OAuth flows, token claims | `pnpm test-api` |
| E2E Tests | Profile form interactions | Playwright |
| Manual | Field persistence, UI rendering | Browser |

## Rollback Plan

1. Revert git commit
2. Run `pnpm db:push` (nullable columns persist harmlessly)
3. No data loss (additive schema changes)

## Complexity Tracking

> No violations. Implementation uses framework patterns without workarounds.

| Aspect | Complexity | Justification |
|--------|------------|---------------|
| Schema changes | Low | 4 nullable text columns |
| Auth config | Low | Follows existing additionalFields pattern |
| UI changes | Medium | NPM packages add complexity but improve UX |
| Token claims | Low | Follows existing claims pattern |
