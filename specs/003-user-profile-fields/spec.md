# Feature Specification: User Profile Additional Fields

**Feature Branch**: `003-user-profile-fields`
**Created**: 2025-12-04
**Status**: Draft
**Input**: Add additional profile fields (gender, father_name, city, country) to user profiles. Based on UX research, collect in Profile Edit only (not signup) to maximize conversion rates.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Edit Profile with Additional Information (Priority: P1)

As an authenticated user, I want to add additional personal information (phone number, gender, father's name, city, country) to my profile so that my account contains complete demographic data for platform services.

**Why this priority**: Core feature request - users need the ability to provide additional demographic information. This is the primary deliverable.

**Independent Test**: Can be fully tested by logging in, navigating to profile edit page, filling in additional fields, saving, and verifying data persists on page reload.

**Acceptance Scenarios**:

1. **Given** I am logged in and on the profile edit page, **When** I view the form, **Then** I see a new "Additional Information" section with fields for Phone Number, Gender, Father's Name, City, and Country.

2. **Given** I am on the profile edit page, **When** I fill in all additional fields and click save, **Then** all fields are saved successfully and I see updated values when I reload the page.

3. **Given** I am on the profile edit page, **When** I fill in only some additional fields (leaving others empty) and save, **Then** only the filled fields are saved and empty fields remain null (all fields are optional).

4. **Given** I am on the profile edit page, **When** I select a gender option from the dropdown, **Then** I can choose from: Male, Female, Other, or Prefer not to say.

---

### User Story 2 - Additional Fields in JWT Token Claims (Priority: P1)

As a client application consuming SSO tokens, I want the additional profile fields (gender, father_name, city, country) to be included in the JWT token claims so that I can personalize the user experience without additional API calls.

**Why this priority**: Equal priority to P1 - tokens must include new fields for downstream applications to use the data. This enables the business value of collecting the data.

**Independent Test**: Can be tested by authenticating via OAuth flow, decoding the returned ID token, and verifying all new fields appear in claims.

**Acceptance Scenarios**:

1. **Given** a user has filled in additional profile fields, **When** they authenticate via OAuth and receive an ID token, **Then** the token claims include: gender, father_name, city, country (in addition to existing claims).

2. **Given** a user has NOT filled in additional profile fields, **When** they authenticate via OAuth, **Then** the token claims include the new fields with null values.

3. **Given** a user updates their additional profile fields, **When** they re-authenticate or refresh their token, **Then** the new token contains the updated field values.

---

### User Story 3 - Phone Number UI in Profile Edit (Priority: P2)

As a user, I want to be able to view and edit my phone number in the profile edit page so that I can update my contact information.

**Why this priority**: Phone number field already exists in database schema but has no UI. Lower priority than new fields since it's an existing capability gap.

**Independent Test**: Can be tested by editing phone number in profile, saving, and verifying it persists.

**Acceptance Scenarios**:

1. **Given** I am on the profile edit page, **When** I view the Additional Information section, **Then** I see a Phone Number field with telephone input type.

2. **Given** I enter a phone number and save, **When** I reload the page, **Then** the phone number is displayed in the field.

---

### User Story 4 - Signup Flow Unchanged (Priority: P1)

As a new user, I want the signup flow to remain simple (name, email, password, then software background and hardware tier) so that I can quickly create an account without being overwhelmed by fields.

**Why this priority**: Critical constraint - UX research shows fewer signup fields = higher conversion (120% increase by reducing fields). Must NOT add new fields to signup.

**Independent Test**: Can be tested by going through signup flow and verifying no new fields appear.

**Acceptance Scenarios**:

1. **Given** I am a new user on the signup page, **When** I view the signup form, **Then** I see only: Name, Email, Password (Step 1) and Software Background, Hardware Tier (Step 2) - no additional demographic fields.

2. **Given** I complete signup, **When** my account is created, **Then** the additional profile fields (gender, father_name, city, country) are null by default.

---

### Edge Cases

- What happens when a user enters special characters in Father's Name or City? System accepts Unicode characters including apostrophes, hyphens, and international characters.
- What happens when a user clears a previously filled field? System saves null value (fields are optional and can be cleared).
- What happens when phone number contains non-numeric characters? Input field accepts standard phone formats including spaces, dashes, parentheses, and plus sign for international prefix.
- What happens when country field has very long values? System accepts standard country names (up to 100 characters).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide an "Additional Information" section in the profile edit page containing five fields: Phone Number, Gender, Father's Name, City, Country.
- **FR-002**: System MUST allow Gender selection from exactly four options: Male, Female, Other, Prefer not to say.
- **FR-003**: All additional profile fields MUST be optional (nullable) - users can save their profile without filling any of them.
- **FR-004**: System MUST persist additional profile field values when user clicks Save on the profile edit page.
- **FR-005**: System MUST include additional profile fields (gender, father_name, city, country) in OAuth/OIDC token claims via the userinfo endpoint and ID token.
- **FR-006**: System MUST NOT modify the existing signup flow - no additional fields added to signup steps.
- **FR-007**: System MUST return null for unfilled additional fields in token claims (consistent with existing phone_number claim behavior).
- **FR-008**: Phone Number field MUST use telephone input type for mobile keyboard optimization.
- **FR-009**: Father's Name, City, and Country fields MUST be text inputs accepting Unicode characters.
- **FR-010**: System MUST preserve existing profile functionality (name, given name, family name, software background, hardware tier, locale, timezone) without changes.

### Key Entities

- **User Profile**: Extended with four new attributes: gender (enum: male/female/other/prefer_not_to_say), father_name (string), city (string), country (string). Phone number already exists.
- **JWT Token Claims**: Extended to include: gender, father_name, city, country in addition to existing claims (phone_number, software_background, hardware_tier, etc.).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can view, edit, and save all five additional fields (phone, gender, father's name, city, country) in profile edit page within 30 seconds.
- **SC-002**: All additional profile fields appear correctly in decoded JWT token claims after user authentication.
- **SC-003**: Signup conversion rate remains unchanged (no new fields added to signup flow).
- **SC-004**: 100% of existing API tests continue to pass after implementation (non-breaking change).
- **SC-005**: Profile edit page maintains current load time (under 2 seconds) after adding new fields.
- **SC-006**: All field values persist correctly across page reloads and re-authentication.

## Constraints

- **C-001**: Database schema changes must be additive only (new nullable columns) to ensure zero downtime deployment.
- **C-002**: Implementation must use framework's official patterns for extending user data (no custom workarounds).
- **C-003**: No changes to signup components or signup API endpoints.
- **C-004**: Production system - changes must be backward compatible with existing user data.

## Non-Goals

- **NG-001**: Field validation beyond basic type checking (e.g., no phone number format validation, no country dropdown with predefined list).
- **NG-002**: Internationalization of field labels in this phase.
- **NG-003**: Adding these fields to admin user management interface.
- **NG-004**: Email or SMS verification for phone number.
- **NG-005**: Autocomplete or suggestion features for city/country fields.

## Assumptions

- **A-001**: Gender options (male, female, other, prefer_not_to_say) are sufficient for current requirements based on common industry practice.
- **A-002**: Father's Name is a required field for certain regional compliance/identification purposes (common in South Asian contexts).
- **A-003**: Free-text city and country fields are acceptable (vs. dropdown with predefined values) for MVP.
- **A-004**: Existing phone_number database column is of appropriate type (string/text) to store formatted phone numbers.
- **A-005**: Token claim naming convention follows snake_case (father_name, not fatherName) to match existing claims (software_background, hardware_tier).
