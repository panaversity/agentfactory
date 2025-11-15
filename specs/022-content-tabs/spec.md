# Feature Specification: Interactive Content Tabs with AI Summarization

**Feature Branch**: `022-content-tabs`  
**Created**: 2025-11-15  
**Status**: Draft  
**Input**: User description: "Interactive content tabs for Docusaurus with Original, Summary (AI-powered streaming), and Personalized views. Includes session-based authentication and FastAPI integration for OpenAI Agents SDK summarization."

## Clarifications

### Session 2025-11-15

- Q: Summary cache scope - how long should generated summaries be cached? → A: Session-scoped - Cache persists across page navigations during one user session (browser tab)
- Q: Summary length control - what is the desired summary length or behavior? → A: Proportional with bounds - Target 20-25% of original, min 150 words, max 500 words
- Q: Login page implementation - what should the login page behavior be? → A: Dummy/stub login for now - Future SSO implementation
- Q: Concurrent user behavior - what happens when the same user has multiple browser tabs open with the same content page? → A: Independent state, shared cache - Each tab has its own active tab selection, but summaries are shared across tabs
- Q: Streaming chunk display behavior - how should the streaming text be displayed to the user? → A: Append with auto-scroll - Chunks appear progressively, viewport auto-scrolls to latest content

## User Scenarios & Testing *(mandatory)*

### User Story 1 - View Original Content (Priority: P1)

Users access book content pages and see three tabs at the top: "Original", "Summary", and "Personalized". By default, the "Original" tab is active, displaying the full unmodified content of the page.

**Why this priority**: This is the foundational UI change that enables all other functionality. Without tab navigation, the feature cannot exist. It's the base upon which Summary and Personalized features are built.

**Independent Test**: Can be fully tested by navigating to any content page and verifying that three tabs are visible with "Original" active by default, showing the complete page content.

**Acceptance Scenarios**:

1. **Given** a user visits any book content page, **When** the page loads, **Then** three tabs appear at the top: "Original", "Summary", "Personalized"
2. **Given** the page has just loaded, **When** the user views the tab bar, **Then** the "Original" tab is visually active/selected
3. **Given** the "Original" tab is active, **When** the user scrolls the page, **Then** all original markdown content is displayed without modification

---

### User Story 2 - Switch Between Tabs (Priority: P1)

Users can click on any of the three tabs to switch the displayed content. The active tab is visually distinct, and content changes immediately without page reload.

**Why this priority**: Tab switching is core to the interactive experience. Without this, users cannot access different content views, making the entire feature non-functional.

**Independent Test**: Can be tested by clicking each tab and verifying that the active state changes visually and content area updates instantly.

**Acceptance Scenarios**:

1. **Given** a user is on the "Original" tab, **When** they click the "Summary" tab, **Then** the "Summary" tab becomes visually active and the content area changes
2. **Given** a user is on the "Summary" tab, **When** they click the "Personalized" tab, **Then** the "Personalized" tab becomes visually active and content switches
3. **Given** a user clicks a tab, **When** the tab switch occurs, **Then** no page reload happens (SPA behavior)
4. **Given** any tab is active, **When** the user clicks on the same tab again, **Then** nothing changes (idempotent)

---

### User Story 3 - Access AI Summary with Authentication (Priority: P2)

When a user clicks the "Summary" tab, the system checks for authentication. If not authenticated, the user is redirected to a login page. If authenticated, the system processes the original content through an AI summarization service and streams the summary back in real-time.

**Why this priority**: This is the primary value-add feature that differentiates this from basic tabs. It provides AI-powered content summarization but requires authentication infrastructure to work correctly.

**Independent Test**: Can be tested independently by: (1) clicking Summary without auth to verify login redirect, (2) authenticating and clicking Summary to verify streaming summary appears.

**Acceptance Scenarios**:

1. **Given** a user is not authenticated (no session token), **When** they click the "Summary" tab, **Then** they are redirected to the login page
2. **Given** a user successfully logs in, **When** they return to the content page and click "Summary", **Then** the system sends the original content to the summarization service
3. **Given** the summarization service begins processing, **When** the request is in progress, **Then** a loading indicator is displayed to the user
4. **Given** the agent begins streaming the response, **When** chunks of summary text arrive, **Then** the summary content is progressively appended with viewport auto-scrolling to the latest content
5. **Given** the summary is being streamed, **When** all chunks are received, **Then** the loading indicator disappears and the complete summary is visible
6. **Given** a user switches away from Summary mid-stream, **When** they switch back to Summary, **Then** the completed/cached summary is shown (no re-fetch)

---

### User Story 4 - View Personalized Content Placeholder (Priority: P3)

When a user clicks the "Personalized" tab, they see a message: "Feature coming soon" or similar placeholder text indicating that personalization features are under development.

**Why this priority**: This is a placeholder for future functionality. While important for UX completeness (showing all three tabs), it delivers no functional value yet and can be implemented last.

**Independent Test**: Can be tested by clicking the "Personalized" tab and verifying placeholder message appears.

**Acceptance Scenarios**:

1. **Given** a user clicks the "Personalized" tab, **When** the tab becomes active, **Then** the content area displays "Feature coming soon" or equivalent placeholder message
2. **Given** the placeholder is displayed, **When** the user reads the message, **Then** it is clear that this feature is not yet available

---

### Edge Cases

- **What happens when the summarization service fails or times out?**  
  Display an error message in the Summary tab: "Unable to generate summary. Please try again later." Allow user to retry or switch to other tabs.

- **What happens when a user's authentication expires while viewing Summary?**  
  For the initial implementation with dummy login, detect expired/missing authentication and redirect to login page. The return behavior will be defined when SSO-based authentication is implemented in the future.

- **How does the system handle extremely long content pages (e.g., 10,000+ words)?**  
  The summarization service should handle large content gracefully. Summaries should target 20-25% of original length with bounds: minimum 150 words, maximum 500 words. If content exceeds reasonable limits, implement a strategy to summarize sections or show a warning to the user.

- **What happens if the user clicks Summary multiple times rapidly?**  
  Prevent multiple simultaneous summarization requests by disabling the tab or showing "Loading..." until the first request completes.

- **What happens when the user navigates to a different page while summary is streaming?**  
  Stop the ongoing summary generation. When returning to the page, allow the user to re-request the summary.

- **How does the system handle network interruptions during summary streaming?**  
  Detect connection errors, display an error message, and provide a "Retry" button to restart the summarization request.

- **What happens when the same user has multiple browser tabs open with the same content page?**  
  Each browser tab maintains independent tab state (which tab is active), but they share the summary cache. If a summary is generated in one tab, it becomes immediately available in other tabs without re-requesting.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display three tabs on every book content page: "Original", "Summary", "Personalized"
- **FR-002**: System MUST show "Original" tab as active by default when a content page loads
- **FR-003**: System MUST allow users to click on any tab to switch the displayed content
- **FR-004**: System MUST visually distinguish the active tab from inactive tabs (e.g., different color, underline, bold)
- **FR-005**: System MUST display the full original markdown content when "Original" tab is active
- **FR-006**: System MUST check for a session token when user clicks the "Summary" tab
- **FR-007**: System MUST redirect unauthenticated users to a login page (dummy/stub implementation) when they attempt to access "Summary". Future implementation will use SSO-based authentication
- **FR-008**: System MUST send the original page content to the summarization service when an authenticated user clicks "Summary"
- **FR-009**: System MUST display a loading indicator while the summary is being generated
- **FR-010**: System MUST stream the summary response progressively, appending chunks as they arrive from the service and auto-scrolling the viewport to keep the latest content visible
- **FR-011**: System MUST complete the streaming display and remove the loading indicator when the agent finishes
- **FR-012**: System MUST cache the generated summary within the user's browser session so that switching away from and back to "Summary" does not trigger a new summarization request. Cache is session-scoped, shared across all browser tabs for the same user, and cleared when the browser session ends
- **FR-013**: System MUST display "Feature coming soon" placeholder text when "Personalized" tab is active
- **FR-014**: System MUST preserve tab state within a single page session (e.g., if user switches to Summary, scrolls down the page, then switches tabs and back, they should return to Summary with scroll position maintained)
- **FR-015**: System MUST handle service errors gracefully and display user-friendly error messages in the Summary tab
- **FR-016**: System MUST prevent multiple simultaneous summary requests for the same page content
- **FR-017**: System MUST generate summaries targeting 20-25% of the original content length, with a minimum of 150 words and maximum of 500 words

### Key Entities *(include if feature involves data)*

- **Authentication State**: Represents whether a user is authenticated. Persisted across page navigation. Used to authorize access to the summarization service.
- **Content Page**: Represents a book chapter or section. Contains content that can be displayed in Original tab or passed to the summarization service.
- **Summary**: Represents the AI-generated summary of a content page. Generated by the summarization service. Cached after first generation to avoid redundant requests.
- **Tab State**: Represents the current active tab (Original, Summary, Personalized) for a given content page.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can switch between tabs on any content page in under 1 second without page reload
- **SC-002**: Authenticated users receive a streaming summary that begins displaying within 3 seconds of clicking the "Summary" tab
- **SC-003**: 95% of summary requests complete successfully without errors under normal network conditions
- **SC-004**: Unauthenticated users are redirected to login within 500ms of clicking "Summary" tab
- **SC-005**: Tab UI is visually consistent across all content pages and clearly indicates the active tab
- **SC-006**: System prevents redundant API calls by caching summaries, reducing backend load by at least 80% for repeat Summary tab visits on the same page
- **SC-007**: Error messages are displayed within 2 seconds of a failed summary request, with clear next steps for the user
- **SC-008**: The feature integrates seamlessly with existing Docusaurus navigation and layout without breaking responsive design on mobile devices
