# Feature Specification: Gemini Agent Integration for E-book AI Dialog

**Feature Branch**: `001-gemini-agent-integration`  
**Created**: 2025-11-12  
**Status**: Draft  
**Input**: User description: "the frontend is not running , use agentic approach for this use openAi agent sdk (python for this ) setup this with google gemini model gemini-2.5-flash for it user configer the its own api key for for the gemini model if it set to default then it limit to only 5 request (for more then 5 request user need to configure it own api key from google-ai-studio (include the step to genrate and configure it to assest user ))"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Configure Gemini API Key (Priority: P1)

As a user, I want to configure my Google Gemini API key so that I can make unlimited AI requests for insights.

**Why this priority**: Essential for enabling personalized AI interactions beyond the default limit and for advanced users.

**Independent Test**: Can be fully tested by navigating to the AI Config page, entering an API key, saving it, and verifying that the key is stored and used for subsequent AI requests.

**Acceptance Scenarios**:

1. **Given** I am on the AI Config page, **When** I enter a valid Gemini API key and click "Save Configuration", **Then** the API key is securely stored and retrieved for future AI interactions.
2. **Given** I have saved an API key, **When** I revisit the AI Config page, **Then** the saved API key field is pre-filled (or indicates a key is present).

---

### User Story 2 - Get AI Insight with Custom API Key (Priority: P1)

As a user with a configured API key, I want to highlight text in the e-book and receive a streamed AI-generated explanation or insight.

**Why this priority**: This is the core functionality of the feature, providing immediate AI assistance to users who have configured their API key.

**Independent Test**: Can be fully tested by highlighting text on any e-book page, clicking "Ask AI", and observing a streamed, relevant response in the AI dialog.

**Acceptance Scenarios**:

1. **Given** I have a valid Gemini API key configured and I am viewing an e-book page, **When** I highlight text and click "Ask AI" in the appearing dialog, **Then** a streamed AI response is displayed in the dialog, providing relevant insight based on the highlighted text.
2. **Given** I have a custom prompt configured, **When** I highlight text and click "Ask AI", **Then** the AI response incorporates the custom prompt along with the highlighted text.

---

### User Story 3 - Get AI Insight with Default Limit (Priority: P2)

As a user without a configured API key, I want to highlight text and receive a limited number of AI-generated explanations or insights.

**Why this priority**: Provides basic functionality and demonstrates value to users before they commit to configuring their own API key.

**Independent Test**: Can be fully tested by highlighting text multiple times without an API key configured, observing responses for the first 5 requests, and then a message indicating the limit has been reached.

**Acceptance Scenarios**:

1. **Given** I do not have a Gemini API key configured and I am viewing an e-book page, **When** I highlight text and click "Ask AI" (up to 5 times), **Then** a streamed AI response is displayed in the dialog.
2. **Given** I have made 5 AI requests without a configured API key, **When** I attempt to make a 6th request, **Then** the AI dialog displays a message indicating the request limit has been reached and prompts me to configure an API key.

---

### User Story 4 - Configure Custom Prompt (Priority: P2)

As a user, I want to define a custom prompt for AI interactions so that I can guide the AI's behavior and focus.

**Why this priority**: Enhances user control and customization over the AI's responses.

**Independent Test**: Can be fully tested by navigating to the AI Config page, entering a custom prompt, saving it, and then verifying that subsequent AI responses reflect the influence of the custom prompt.

**Acceptance Scenarios**:

1. **Given** I am on the AI Config page, **When** I enter a custom prompt in the designated field and click "Save Configuration", **Then** the custom prompt is securely stored and used for future AI interactions.
2. **Given** I have saved a custom prompt, **When** I highlight text and click "Ask AI", **Then** the AI's response is influenced by the custom prompt.

## Edge Cases

- What happens when the configured API key is invalid or revoked? The system should inform the user and prompt them to reconfigure.
- How does the system handle network errors or timeouts during communication with the backend or the Gemini API? The system should display an appropriate error message to the user.
- What happens if the user attempts to make an AI request after exceeding the default 5-request limit without a configured API key? The system should clearly communicate the limit and guide the user to the AI Config page.
- What happens if no text is highlighted when the "Ask AI" button is clicked? The button should be disabled or a message should indicate that text needs to be highlighted.
- What happens if the Gemini API returns an error (e.g., rate limit, content policy violation)? The system should display a user-friendly error message.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to configure and save their Google Gemini API key via a dedicated configuration page.
- **FR-002**: System MUST allow users to configure and save a custom prompt for AI interactions via the same configuration page.
- **FR-003**: System MUST detect highlighted text across all e-book content rendered by Docusaurus.
- **FR-004**: System MUST display an AI dialog box near the highlighted text upon selection.
- **FR-005**: System MUST send the highlighted text, the configured API key (if available), and the custom prompt (if available) to a backend service.
- **FR-006**: The backend service MUST utilize the OpenAI Agent SDK to facilitate interaction with the Google Gemini model (specifically `gemini-2.5-flash`).
- **FR-007**: The backend service MUST stream AI responses back to the frontend in real-time.
- **FR-008**: The frontend MUST display the streamed AI responses incrementally within the AI dialog box.
- **FR-009**: If no API key is configured by the user, the system MUST limit AI requests to a maximum of 5.
- **FR-010**: The AI Config page MUST include clear, step-by-step instructions on how to generate a Google Gemini API key from Google AI Studio.
- **FR-011**: System MUST gracefully handle invalid or missing API keys, providing informative feedback to the user.
- **FR-012**: System MUST gracefully handle network errors or API communication failures, displaying appropriate messages to the user.

### Key Entities *(include if feature involves data)*

- **User Configuration**: Stores the user's Google Gemini API key and custom prompt. This data is stored client-side (e.g., in local storage).
- **Highlighted Text**: The segment of text selected by the user in the e-book content.
- **AI Request Counter**: A client-side mechanism to track the number of AI requests made when no API key is configured.
- **AI Response**: The streamed textual output generated by the Google Gemini model.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully configure and save their Gemini API key and custom prompt on the AI Config page within 30 seconds.
- **SC-002**: AI responses for highlighted text are streamed and fully displayed in the AI dialog within an average of 5 seconds of clicking "Ask AI" (assuming valid API key and stable network conditions).
- **SC-003**: Users without a configured API key can successfully make 5 AI requests, and on the 6th attempt, they are clearly informed of the limit and prompted to configure their key.
- **SC-004**: The AI dialog provides relevant and helpful explanations for 90% of highlighted text queries, as determined by user feedback or internal review.
- **SC-005**: The instructions provided on the AI Config page enable 95% of users to successfully generate and configure their own Gemini API key from Google AI Studio without external assistance.