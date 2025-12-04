# Tasks: User Profile Additional Fields

**Input**: Design documents from `/specs/003-user-profile-fields/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests will be run via existing `pnpm test-api` suite - no new test tasks needed.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Next.js App Router**: `src/`, root-level config files
- Schema at root: `auth-schema.ts`
- Auth config: `src/lib/auth.ts`
- Types: `src/types/`
- UI: `src/app/account/profile/`

---

## Phase 1: Setup (Dependencies & Configuration)

**Purpose**: Install npm packages and prepare development environment

- [x] T001 Install react-phone-number-input package via `pnpm add react-phone-number-input`
- [x] T002 [P] Install react-country-state-city package via `pnpm add react-country-state-city`
- [x] T003 [P] Verify packages installed correctly in package.json

**Checkpoint**: NPM packages installed - ready for implementation ✅

---

## Phase 2: Foundational (Schema & Auth Config)

**Purpose**: Database schema and Better Auth configuration - MUST complete before UI work

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Add gender column to user table in auth-schema.ts
- [x] T005 [P] Add fatherName column to user table in auth-schema.ts
- [x] T006 [P] Add city column to user table in auth-schema.ts
- [x] T007 [P] Add country column to user table in auth-schema.ts
- [x] T008 Add gender additionalField to Better Auth config in src/lib/auth.ts
- [x] T009 [P] Add fatherName additionalField to Better Auth config in src/lib/auth.ts
- [x] T010 [P] Add city additionalField to Better Auth config in src/lib/auth.ts
- [x] T011 [P] Add country additionalField to Better Auth config in src/lib/auth.ts
- [x] T012 Add Gender type definition in src/types/profile.ts
- [x] T013 [P] Update ProfileData interface with new fields in src/types/profile.ts
- [x] T014 Run database migration via `pnpm db:push`

**Checkpoint**: Schema migrated, auth config updated - UI implementation can begin ✅

---

## Phase 3: User Story 1 - Edit Profile with Additional Information (Priority: P1) 🎯 MVP

**Goal**: Users can view, edit, and save additional profile fields (phone, gender, father's name, city, country)

**Independent Test**: Log in → Navigate to /account/profile → Fill all 5 new fields → Save → Reload page → Verify all values persist

### Implementation for User Story 1

- [x] T015 [US1] Add phoneNumber to formData state in src/app/account/profile/ProfileForm.tsx
- [x] T016 [P] [US1] Add gender to formData state in src/app/account/profile/ProfileForm.tsx
- [x] T017 [P] [US1] Add fatherName to formData state in src/app/account/profile/ProfileForm.tsx
- [x] T018 [P] [US1] Add city to formData state in src/app/account/profile/ProfileForm.tsx
- [x] T019 [P] [US1] Add country to formData state in src/app/account/profile/ProfileForm.tsx
- [x] T020 [US1] Add all 5 new fields to updateUser call in src/app/account/profile/ProfileForm.tsx
- [x] T021 [US1] Create "Additional Information" section header in ProfileForm.tsx UI
- [x] T022 [US1] Import and implement PhoneInput component from react-phone-number-input in ProfileForm.tsx
- [x] T023 [US1] Add gender select dropdown with 4 options (Male, Female, Other, Prefer not to say) in ProfileForm.tsx
- [x] T024 [US1] Add Father's Name text input field in ProfileForm.tsx
- [x] T025 [US1] Import and implement CountrySelect from react-country-state-city in ProfileForm.tsx
- [x] T026 [US1] City field implemented as text input (CitySelect requires state selection not in spec)
- [x] T027 [US1] Style new fields to match existing Panaversity green theme in ProfileForm.tsx

**Checkpoint**: Profile form displays all new fields, data saves and persists - User Story 1 complete ✅

---

## Phase 4: User Story 2 - Additional Fields in JWT Token Claims (Priority: P1)

**Goal**: New profile fields appear in OAuth/OIDC JWT token claims for client applications

**Independent Test**: Authenticate via OAuth → Decode ID token → Verify gender, father_name, city, country claims exist

### Implementation for User Story 2

- [x] T028 [US2] Add gender claim to getAdditionalUserInfoClaim in src/lib/auth.ts
- [x] T029 [P] [US2] Add father_name claim to getAdditionalUserInfoClaim in src/lib/auth.ts
- [x] T030 [P] [US2] Add city claim to getAdditionalUserInfoClaim in src/lib/auth.ts
- [x] T031 [P] [US2] Add country claim to getAdditionalUserInfoClaim in src/lib/auth.ts

**Checkpoint**: JWT tokens include all new claims - User Story 2 complete ✅

---

## Phase 5: User Story 3 - Phone Number UI (Priority: P2)

**Goal**: Existing phone_number field (already in schema) now has UI in profile edit

**Independent Test**: Edit phone number in profile → Save → Reload → Verify phone number persists

**Note**: Phone number was already in database schema, this story just adds UI (completed as part of US1 implementation)

### Implementation for User Story 3

- [x] T032 [US3] Verify phone number field renders correctly with international format support
- [x] T033 [US3] Verify phone number saves and displays correctly after reload

**Checkpoint**: Phone number field fully functional - User Story 3 complete ✅

---

## Phase 6: User Story 4 - Signup Flow Unchanged (Priority: P1) - VERIFICATION ONLY

**Goal**: Verify signup flow has NOT been modified (constraint validation)

**Independent Test**: Go through signup flow → Verify only Name, Email, Password (Step 1) and Software Background, Hardware Tier (Step 2) appear

### Verification for User Story 4

- [x] T034 [US4] Verify src/components/sign-up-form.tsx was NOT modified (no new fields added)
- [ ] T035 [US4] Test signup flow manually - confirm no new demographic fields appear

**Checkpoint**: Signup flow unchanged - constraint verified ✅

---

## Phase 7: Polish & Validation

**Purpose**: Final testing and quality assurance

- [ ] T036 Run existing API tests via `pnpm test-api` (requires dev server running)
- [x] T037 [P] TypeScript compilation passes (`pnpm exec tsc --noEmit`)
- [x] T038 Next.js build passes (`pnpm build`)
- [ ] T039 [P] Manual test: Verify JWT token claims via OAuth flow
- [x] T040 Extended test-tenant-claims.js to verify new profile claims

**Checkpoint**: Build passes, tests extended - ready for manual validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - US1 and US2 can proceed in parallel after foundational
  - US3 depends on US1 (phone field is part of same form)
  - US4 is verification only - can run anytime
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Can run parallel to US1
- **User Story 3 (P2)**: Depends on US1 (phone field is part of same form implementation)
- **User Story 4 (P1)**: Verification only - no implementation dependencies

### Within Each Phase

- Schema columns can be added in parallel (T004-T007)
- additionalFields can be added in parallel (T008-T011)
- formData state additions can be done in parallel (T015-T019)
- JWT claims can be added in parallel (T028-T031)

### Parallel Opportunities

- T001, T002, T003 can run in parallel (npm installs)
- T004-T007 can run in parallel (schema columns - same file but different lines)
- T008-T011 can run in parallel (additionalFields - same file but different lines)
- T015-T019 can run in parallel (formData additions)
- T028-T031 can run in parallel (JWT claims)

---

## Parallel Example: Foundational Phase

```bash
# Launch all schema column additions together (different lines, same file):
Task T004: "Add gender column to user table in auth-schema.ts"
Task T005: "Add fatherName column to user table in auth-schema.ts"
Task T006: "Add city column to user table in auth-schema.ts"
Task T007: "Add country column to user table in auth-schema.ts"

# Launch all additionalFields additions together:
Task T008: "Add gender additionalField to Better Auth config"
Task T009: "Add fatherName additionalField to Better Auth config"
Task T010: "Add city additionalField to Better Auth config"
Task T011: "Add country additionalField to Better Auth config"
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2)

1. Complete Phase 1: Setup (install npm packages)
2. Complete Phase 2: Foundational (schema + auth config)
3. Complete Phase 3: User Story 1 (profile form UI)
4. Complete Phase 4: User Story 2 (JWT claims)
5. **STOP and VALIDATE**: Run `pnpm test-api`, manual test profile edit
6. Deploy/demo MVP

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Profile form works → Deploy/Demo (MVP!)
3. Add User Story 2 → JWT claims work → Verify with client app
4. Add User Story 3 → Phone number verified → Complete
5. Verify User Story 4 → Signup unchanged → Constraint met

### Estimated Effort

| Phase | Tasks | Parallel Opportunities | Est. Time |
|-------|-------|------------------------|-----------|
| Setup | 3 | 3 parallel | 5 min |
| Foundational | 11 | 8 parallel | 15 min |
| User Story 1 | 13 | 6 parallel | 30 min |
| User Story 2 | 4 | 3 parallel | 10 min |
| User Story 3 | 2 | 0 parallel | 5 min |
| User Story 4 | 2 | 0 parallel | 5 min |
| Polish | 5 | 2 parallel | 15 min |
| **Total** | **40** | | **~85 min** |

---

## Notes

- [P] tasks = different files or different parts of same file, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each phase or logical group
- Stop at any checkpoint to validate independently
- Use `pnpm db:push` after schema changes
- Use `pnpm test-api` to validate no regressions
