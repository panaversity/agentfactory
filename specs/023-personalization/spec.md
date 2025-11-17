# Feature Specification: Personalized Content Generation

**Feature Branch**: `023-personalization`  
**Created**: 2025-11-17  
**Status**: Draft  
**Input**: User description: "Now i want a personalization feature implemented in my app, but for that you also need to understand how summarize is integrated into frontend and keep my backened separte, in future i'll take my backened to other repo, So understand how summarize backened is developed and also its frontend, in personalization tab, and in summarize tab first login appears, and then on login, for now create a dummy endpoint of login, bz in future we will implement sso using clerk, and for now dummy endpoint and in that login page, on login create a dummy auth token in session for a user, and on login there are two questions that helps in our personalization phase . The questions are : two questions about user's programming experience and user's AI Proficiency on some kind of scale (may be from 1 to 10 scale or hardcoded option (Novice, Beginner, Intermediate, Expert)) beside user's name, email (or authorization from 3rd party like google). , Login is just dummy, we will implement sso using clerk in future, and after this willl done, Then on personalization tab(similarly to summarization workflow), first token of user is checked if it is present in session or not, if it is present then the personalized content is generated on the basis of user preferences that he chooses on login phase, same streaming like summarization, and same cached personlization of each content after it is generated first time, bcz in future all our cached content of summary and personalization, we moved that to some db, for now just in cached"

## Clarifications

### Session 2025-11-17

- Q: How should the login form handle incomplete or invalid inputs? → A: Validate all fields required and block submission with inline error messages
- Q: What should happen when a user's session expires while they are viewing personalized content? → A: Show non-intrusive notification and allow continued viewing, require re-login only for new actions
- Q: What should happen when personalized content generation fails partway through streaming? → A: Display partial content received, show error message, and provide retry button
- Q: How should the profile fingerprint be constructed for cache keys? → A: Concatenate with separator: "Novice-Beginner" format (programming-AI)
- Q: How should the system prevent duplicate simultaneous personalization requests? → A: Disable generate button and show "Generating..." state while request is active

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Dummy Login with User Profiling (Priority: P1)

As a reader, I need to provide basic information about my programming experience and AI proficiency during login, so that the content can be tailored to my skill level.

**Why this priority**: Foundation for all personalization features. Without user profiles, content cannot be personalized. This establishes authentication flow and user data collection that all other features depend on.

**Independent Test**: Can be fully tested by navigating to Summary or Personalization tab, completing the login form with name, email, programming experience selection, and AI proficiency selection, and verifying a session token is created and stored.

**Acceptance Scenarios**:

1. **Given** a user visits the Summary or Personalization tab for the first time, **When** they are not authenticated, **Then** they see a login form requesting name, email, programming experience level, and AI proficiency level
2. **Given** a user completes the login form with valid inputs, **When** they submit, **Then** a dummy authentication token is created and stored in session storage
3. **Given** a user has an active session token, **When** they navigate to Summary or Personalization tabs, **Then** they do not see the login form again
4. **Given** a user completes login, **When** the system creates the session, **Then** user preferences (programming experience and AI proficiency) are stored alongside the token

---

### User Story 2 - Personalized Content Streaming (Priority: P2)

As a reader with specific skill levels, I want to receive content explanations tailored to my programming experience and AI proficiency, so that I can learn at an appropriate pace and depth.

**Why this priority**: Core value proposition of personalization. Depends on P1 (user profiling) but delivers the main user benefit. Without this, user profiling would have no purpose.

**Independent Test**: Can be fully tested by logging in with specific proficiency levels (e.g., Novice programming + Beginner AI), clicking "Generate Personalized Content" on a page, and verifying the streamed content matches the expected complexity level.

**Acceptance Scenarios**:

1. **Given** an authenticated user with stored preferences, **When** they navigate to the Personalization tab on any page, **Then** the system retrieves their programming experience and AI proficiency from session
2. **Given** a user requests personalized content, **When** the system generates content, **Then** explanations are streamed progressively (similar to summary streaming)
3. **Given** a Novice programmer requests personalized content, **When** content is generated, **Then** explanations include basic programming concepts and avoid advanced jargon
4. **Given** an Expert-level user requests personalized content, **When** content is generated, **Then** explanations skip fundamentals and focus on advanced insights and nuances

---

### User Story 3 - Personalized Content Caching (Priority: P3)

As a reader, I want previously generated personalized content to load instantly on subsequent visits, so that I don't waste time regenerating the same content.

**Why this priority**: Performance optimization. Depends on P2 (content generation) working correctly. Improves user experience but system is functional without it.

**Independent Test**: Can be fully tested by generating personalized content for a page, navigating away, then returning to the same page and verifying content loads instantly without calling the generation endpoint.

**Acceptance Scenarios**:

1. **Given** personalized content has been generated for a specific page and user profile, **When** the same user returns to that page, **Then** cached content is displayed immediately without regeneration
2. **Given** cached personalized content exists, **When** the user views it, **Then** a cache indicator shows the content was retrieved from cache (not freshly generated)
3. **Given** a user with different proficiency levels than cached content, **When** they request personalized content for the same page, **Then** new content is generated matching their profile (cache is profile-specific)
4. **Given** personalized content is being generated, **When** generation completes successfully, **Then** the final content is cached with the user's profile key

---

### Edge Cases

- **Incomplete Profile Prevention**: Login form validates all required fields (name, email, programming experience, AI proficiency) and blocks submission with inline error messages until complete
- **Session Expiration During Viewing**: System shows non-intrusive notification on session expiration, allows continued viewing of current content, but requires re-login for new personalization requests
- **Streaming Failure Recovery**: On mid-stream generation failure, system preserves partial content, displays error message, and provides retry button
- **Cache Differentiation by Profile**: Cache keys differentiate profiles using hyphen-separated fingerprint format (e.g., "Novice-Beginner"), ensuring unique cache entries per profile-page combination
- **Duplicate Request Prevention**: Generate button is disabled and shows "Generating..." state during active personalization requests to prevent simultaneous duplicates
- What happens when a user clears their session storage during content generation?
- How does the system handle simultaneous requests for the same page from the same user?

## Requirements *(mandatory)*

### Functional Requirements

#### Authentication & Session Management

- **FR-001**: System MUST provide a dummy login form on Summary and Personalization tabs for unauthenticated users
- **FR-002**: Login form MUST collect user name, email, programming experience level, and AI proficiency level
- **FR-002a**: All login form fields (name, email, programming experience, AI proficiency) MUST be validated as required before submission
- **FR-002b**: Login form MUST display inline error messages for missing or invalid fields and prevent submission until all fields are valid
- **FR-003**: Programming experience MUST be selectable from predefined options: Novice, Beginner, Intermediate, Expert
- **FR-004**: AI proficiency MUST be selectable from predefined options: Novice, Beginner, Intermediate, Expert
- **FR-005**: System MUST generate a dummy authentication token upon successful login submission
- **FR-006**: Authentication token and user preferences MUST be stored in session storage (browser session)
- **FR-006a**: System MUST detect session expiration and display non-intrusive notification to user
- **FR-006b**: System MUST allow users to continue viewing current personalized content after session expiration
- **FR-006c**: System MUST require re-login before allowing new personalization requests after session expiration
- **FR-007**: System MUST verify authentication token presence before displaying personalized content
- **FR-008**: System MUST redirect unauthenticated users to the login form when accessing Personalization tab

#### Personalized Content Generation

- **FR-009**: System MUST generate personalized content via streaming (Server-Sent Events) similar to summarization workflow
- **FR-010**: Personalized content generation MUST use user's programming experience and AI proficiency as parameters
- **FR-011**: System MUST adjust content complexity based on programming experience level (Novice = basic concepts, Expert = advanced insights)
- **FR-012**: System MUST adjust AI-related explanations based on AI proficiency level
- **FR-013**: Personalized content MUST stream progressively to the frontend, showing chunks as they are generated
- **FR-014**: System MUST indicate when content is actively being generated (loading state)
- **FR-015**: System MUST handle content generation errors gracefully with user-friendly error messages
- **FR-015a**: When streaming fails mid-generation, system MUST preserve and display partial content already received
- **FR-015b**: System MUST show clear error message indicating generation failure alongside partial content
- **FR-015c**: System MUST provide retry button to allow user to restart generation after streaming failure

#### Content Caching

- **FR-016**: System MUST cache successfully generated personalized content in memory (client-side)
- **FR-017**: Cache key MUST include page ID and user profile fingerprint (programming experience + AI proficiency)
- **FR-017a**: Profile fingerprint MUST be constructed by concatenating programming experience and AI proficiency with hyphen separator (format: "ProgrammingLevel-AILevel", e.g., "Novice-Beginner")
- **FR-018**: System MUST check cache before initiating new personalized content generation
- **FR-019**: System MUST display cached content immediately when available, without calling backend
- **FR-020**: System MUST indicate whether displayed content is cached or freshly generated
- **FR-021**: System MUST prevent duplicate simultaneous generation requests for the same page and user profile
- **FR-021a**: Generate button MUST be disabled while personalization request is active
- **FR-021b**: System MUST display "Generating..." state on button and UI while content generation is in progress

#### Backend Architecture Separation

- **FR-022**: Personalization backend endpoints MUST be separate from summarization endpoints
- **FR-023**: Dummy login endpoint MUST be created as `/api/v1/auth/dummy-login-with-profile`
- **FR-024**: Personalized content generation endpoint MUST be created as `/api/v1/personalize`
- **FR-025**: Backend MUST remain modular and independently deployable (future-proofing for separate repository)

### Key Entities

- **User Profile**: Represents a user's learning preferences
  - Name (string)
  - Email (string, for future SSO integration)
  - Programming Experience Level (enum: Novice, Beginner, Intermediate, Expert)
  - AI Proficiency Level (enum: Novice, Beginner, Intermediate, Expert)
  - Session Token (string, dummy implementation)

- **Personalized Content**: AI-generated content tailored to user profile
  - Page ID (string, identifier for content source)
  - User Profile Fingerprint (string, derived from programming + AI proficiency)
  - Personalized Text (string, generated content)
  - Timestamp (number, when generated)
  - Cached Flag (boolean, whether from cache)

- **Authentication Session**: Temporary session state
  - Token (string, dummy token identifier)
  - User Profile (embedded User Profile data)
  - Expiry (string, "session" for current implementation)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete login and profiling in under 30 seconds
- **SC-002**: Personalized content begins streaming within 2 seconds of request
- **SC-003**: Cached personalized content loads in under 500 milliseconds
- **SC-004**: 100% of users with valid session tokens can access personalized content without re-login during browser session
- **SC-005**: System correctly differentiates personalized content for at least 4 distinct user profiles (Novice/Novice, Novice/Expert, Expert/Novice, Expert/Expert)
- **SC-006**: Personalized content generation completes successfully for content of up to 10,000 characters
- **SC-007**: Cache hit rate reaches 80% for users revisiting previously personalized pages during same session

### Assumptions

- Session storage persists for the duration of the browser session (until tab/window is closed)
- Dummy authentication is acceptable as SSO integration with Clerk is planned for future
- Client-side caching in memory is sufficient for current implementation (database migration planned for future)
- User profiles are session-scoped (no persistent user accounts yet)
- Personalized content quality is evaluated subjectively based on proficiency levels (automated quality metrics are out of scope)
