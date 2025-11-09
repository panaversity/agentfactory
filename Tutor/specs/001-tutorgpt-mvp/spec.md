# Feature Specification: TutorGPT MVP - AI Tutor for Docusaurus Book

**Feature Branch**: `001-tutorgpt-mvp`
**Created**: 2025-11-07
**Status**: Draft
**Input**: User description: "AI tutor that appears in sidebar while students read the book. ChatKit widget integrated into Docusaurus website. Students get stuck reading alone - need instant help. Timeline: 4 weeks to production launch."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First-Time Student Gets Instant Help (Priority: P1)

Sarah, a 24-year-old business major, is reading Chapter 1 of the AI-Native Software Development book late at night. She encounters the term "AI-driven development" and doesn't understand what it means. She clicks the ChatKit widget in the bottom-right corner, types her question "What does 'AI-driven development' mean?", and receives an immediate, contextual explanation that references the specific page she's reading.

**Why this priority**: This is the core value proposition - instant help for confused students. Without this, the entire feature fails its primary purpose. This is the absolute minimum viable product.

**Independent Test**: Can be fully tested by opening any book page, clicking the chat widget, asking a question about content on that page, and verifying that the agent responds with a relevant answer within 3 seconds. Delivers immediate value even without personalization or history.

**Acceptance Scenarios**:

1. **Given** Sarah is reading Chapter 1 Introduction page, **When** she clicks the ChatKit widget for the first time, **Then** the widget expands and displays a welcoming greeting message explaining how to use TutorGPT
2. **Given** Sarah has the chat widget open, **When** she types "What does 'AI-driven development' mean?" and sends the message, **Then** she receives a response within 3 seconds that explains the concept in simple terms with reference to her current page location
3. **Given** Sarah is reading a technical paragraph about Python, **When** she asks a follow-up question like "Can you give me an example?", **Then** the agent provides a relevant code example with explanation
4. **Given** Sarah closes her browser after using TutorGPT, **When** she returns to the book website the next day, **Then** the chat widget appears in the same position ready to help

**TDD Test Requirements (US1)**:

**Unit Tests**:
- `test_agent_responds_within_3_seconds()` - Verify response time SLA
- `test_search_book_content_returns_relevant_results()` - RAG search accuracy
- `test_agent_references_current_page()` - Context awareness
- `test_session_creation()` - Session manager creates valid sessions

**Integration Tests**:
- `test_agent_with_rag_system()` - Agent calls search_book_content tool correctly
- `test_chatkit_backend_integration()` - ChatKit session creation works
- `test_full_question_answer_flow()` - User message → Agent → RAG → Response

**Behavior Tests** (Agent Teaching Quality):
- `test_agent_teaches_from_book()` - Agent uses book content, not generic knowledge
- `test_agent_provides_encouraging_response()` - Response tone is friendly/supportive
- `test_agent_asks_clarifying_question_when_needed()` - Handles ambiguous questions
- `test_agent_provides_example_when_helpful()` - Knows when to give examples

**Scenario Tests** (Full User Journeys):
- `test_first_time_student_gets_help()` - Complete US1 flow works end-to-end
- `test_student_asks_multiple_questions()` - Conversation continuity maintained
- `test_agent_redirects_off_topic_question()` - Agent guides back to book content

**Coverage Target**: ≥80% code coverage, 100% behavior coverage for agent teaching

---

### User Story 2 - Student Highlights Confusing Text for Automatic Explanation (Priority: P2)

While reading Chapter 4 about Python, Sarah encounters the sentence "Python is an interpreted, high-level language" and doesn't understand what that means. She highlights the text with her mouse, and the TutorGPT agent automatically detects the selection and provides an explanation in the chat window without her needing to type a question.

**Why this priority**: This dramatically reduces friction for confused students. Instead of formulating a question, they simply highlight what confuses them. This is high-value but depends on the basic chat working first (P1).

**Independent Test**: Can be tested by opening any book page, highlighting a technical term or sentence, and verifying the agent automatically responds with an explanation. Works independently of other features though requires P1 chat functionality.

**Acceptance Scenarios**:

1. **Given** Sarah is reading a page with technical content, **When** she highlights the text "Python is an interpreted, high-level language", **Then** the agent automatically detects the selection and posts an explanation to the chat within 2 seconds
2. **Given** Sarah has highlighted text and received an explanation, **When** she highlights different text on the same page, **Then** the agent provides a new explanation for the newly selected text
3. **Given** Sarah highlights a long paragraph (over 200 words), **When** the highlight is detected, **Then** the agent asks her to narrow down which part confuses her rather than explaining the entire paragraph
4. **Given** Sarah rapidly highlights and un-highlights text (within 1 second), **When** the system detects this behavior, **Then** it waits 1 second after the last selection before responding to avoid spam

**TDD Test Requirements (US2)**:

**Unit Tests**:
- `test_highlight_detection()` - Frontend captures selection correctly
- `test_debouncing_logic()` - 1-second debounce prevents spam
- `test_highlight_text_validation()` - Min 10 chars, max 200 chars validation
- `test_highlight_endpoint()` - POST /api/highlight processes requests

**Integration Tests**:
- `test_highlight_triggers_agent()` - Highlight → Agent → Explanation flow
- `test_agent_searches_highlighted_text()` - Agent uses search_book_content with highlighted text
- `test_multiple_highlights_same_page()` - Each highlight gets new explanation

**Behavior Tests** (Agent Proactive Teaching):
- `test_agent_explains_proactively()` - Agent provides explanation without being asked
- `test_agent_handles_long_selection()` - Agent asks to narrow down if >200 words
- `test_agent_explains_in_context()` - Explanation references current lesson/chapter
- `test_agent_asks_if_understood()` - Agent follows up: "Does this make sense?"

**Scenario Tests**:
- `test_student_highlights_term()` - Complete US2 flow (highlight → explanation)
- `test_rapid_highlighting_debounced()` - Rapid selections handled gracefully
- `test_highlight_with_no_book_content()` - Agent handles non-technical highlights

**Coverage Target**: ≥80% code coverage, debouncing logic 100% tested

---

### User Story 3 - Returning Student Benefits from Conversation History (Priority: P3)

Alex returns to the book website after completing Chapter 1 yesterday. When he opens the ChatKit widget, the agent greets him with "Welcome back! Last time you finished Chapter 1. Ready to continue with Chapter 2?" His previous questions and the agent's answers are still visible in the chat history, and he can scroll up to review them.

**Why this priority**: Session persistence creates a continuous learning experience and helps students build on previous understanding. However, the feature still works without this - it just means each session starts fresh. Can be deferred if time-constrained.

**Independent Test**: Can be tested by using TutorGPT in one browser session, asking several questions, closing the browser completely, reopening the book website, and verifying that conversation history persists and the agent acknowledges previous progress.

**Acceptance Scenarios**:

1. **Given** Alex used TutorGPT yesterday and asked 5 questions, **When** he returns to the book website today and opens the chat widget, **Then** his previous conversation history is visible in the chat
2. **Given** Alex completed Chapter 1 in his last session, **When** he returns and opens the chat widget, **Then** the agent greets him with a personalized message referencing his progress
3. **Given** Alex is viewing his conversation history from a previous session, **When** he scrolls to the top of the chat, **Then** he can see all messages from his current session and previous sessions in chronological order
4. **Given** Alex has used TutorGPT for several days with multiple sessions, **When** he asks a question about a topic he previously asked about, **Then** the agent references his previous questions to provide context (e.g., "Like we discussed earlier...")

**TDD Test Requirements (US3)**:

**Unit Tests**:
- `test_session_persistence()` - Session data saved to SQLite correctly
- `test_get_or_create_session()` - Existing sessions retrieved correctly
- `test_conversation_history_storage()` - All messages stored chronologically
- `test_session_expiry()` - Sessions expire after 30 days inactivity
- `test_progress_tracking()` - Chapter/lesson progress stored correctly

**Integration Tests**:
- `test_session_restoration_flow()` - Browser restart → session restored → history loaded
- `test_agent_loads_student_profile()` - Agent calls get_student_profile() for returning students
- `test_full_history_retrieval()` - Multiple sessions, all messages retrieved in order
- `test_localStorage_session_sync()` - Frontend localStorage syncs with backend sessions

**Behavior Tests** (Agent Memory & Personalization):
- `test_agent_greets_returning_student()` - Agent says "Welcome back!" with progress reference
- `test_agent_remembers_previous_topics()` - Agent: "Like we discussed earlier about variables..."
- `test_agent_suggests_next_lesson()` - Agent calls suggest_next_lesson() autonomously
- `test_agent_uses_last_5_7_messages_priority()` - Recent context prioritized, full history available

**Scenario Tests**:
- `test_returning_student_full_journey()` - Complete US3 flow (return → greeting → history → continue)
- `test_multiple_sessions_same_student()` - Student across 3 days, all history persists
- `test_agent_references_past_confusion()` - Agent remembers student struggled with async

**Coverage Target**: ≥80% code coverage, session management 100% tested, context window prioritization verified

---

### User Story 4 - Agent Adapts to Student's Learning Pace (Priority: P4)

After Sarah has asked three questions about Python variables over the course of reading Chapter 4, the TutorGPT agent notices this pattern of confusion. It proactively sends a message saying "I noticed you're asking several questions about variables - this is a tricky concept for beginners! Let me give you a clearer explanation..." and provides a more detailed, simplified explanation with multiple examples.

**Why this priority**: Personalization significantly enhances learning effectiveness but requires substantial data collection and analysis. The feature provides value without this - it just means all students get the same quality of help regardless of their individual needs. This is an enhancement, not a core requirement for MVP.

**Independent Test**: Can be tested by simulating a student who repeatedly asks about the same topic (e.g., asking 3 different questions about variables within 10 minutes), and verifying that the agent detects this pattern and proactively offers additional help with simpler explanations.

**Acceptance Scenarios**:

1. **Given** Sarah has asked 3 questions about Python variables within 15 minutes, **When** she asks her third question, **Then** the agent detects the pattern and proactively offers a more comprehensive explanation with simpler language and more examples
2. **Given** Alex consistently asks advanced questions and rarely needs follow-up explanations, **When** he asks a new question, **Then** the agent adjusts its response style to be more concise and technically detailed
3. **Given** Sarah typically spends 5+ minutes reading each page before asking questions, **When** the agent tracks this behavior over multiple sessions, **Then** it identifies her as a "slow learner" and adapts responses to include more examples and simpler language
4. **Given** Alex typically spends 1-2 minutes per page and asks few questions, **When** the agent tracks this behavior, **Then** it identifies him as a "fast learner" and adapts responses to be more advanced and challenging

**TDD Test Requirements (US4)**:

**Unit Tests**:
- `test_confusion_detection_algorithm()` - 3+ questions same topic = confusion
- `test_learning_pace_calculation()` - time/page → slow/medium/fast classification
- `test_struggle_topic_tracking()` - Struggling topics stored in profile
- `test_teaching_pace_adjustment()` - adjust_teaching_pace() logic correct

**Integration Tests**:
- `test_agent_calls_detect_confusion()` - Agent autonomously calls detect_confusion() every 3-5 messages
- `test_agent_calls_adjust_teaching_pace()` - Agent adapts when confusion detected
- `test_profile_updates_with_behavior()` - Student behavior → profile updates → agent uses it
- `test_agent_celebrates_mastery()` - Student masters difficult topic → agent celebrates

**Behavior Tests** (Agent Autonomous Adaptation):
- `test_agent_detects_repeated_confusion()` - Agent: "I notice you're struggling with async - let me explain differently"
- `test_agent_simplifies_for_confused_student()` - Agent uses explain_concept(depth="simple", use_analogy=True)
- `test_agent_deepens_for_advanced_student()` - Agent provides more technical details for fast learners
- `test_agent_proactively_offers_help()` - Agent doesn't wait for explicit request
- `test_agent_generates_quiz_when_ready()` - Agent tests understanding at right moment
- `test_agent_suggests_practice_exercise()` - Agent offers hands-on practice autonomously

**Scenario Tests** (Full Adaptive Teaching):
- `test_confused_student_gets_adapted_teaching()` - Complete US4 flow (3 questions → detection → adaptation)
- `test_advanced_student_gets_challenged()` - Fast learner → agent increases complexity
- `test_agent_tracks_progress_over_days()` - Multi-day learning journey tracked correctly
- `test_agent_celebrates_milestone()` - Chapter completion → celebration → suggestion

**Coverage Target**: ≥80% code coverage, 100% coverage for confusion detection and adaptation logic

---

### Edge Cases

- **What happens when a student highlights text that doesn't contain any technical content** (e.g., "The following example demonstrates this concept")?
  - Agent should politely indicate that the selected text doesn't appear to need explanation and ask if the student meant to highlight something else or has a specific question.

- **What happens when a student asks a question completely unrelated to the book content** (e.g., "What's the weather today?")?
  - Agent should politely redirect the conversation back to the book content: "I'm here to help you understand this book about AI-Native Software Development. Do you have any questions about what you're reading?"

- **What happens when the RAG system fails to find relevant content for a valid question about the book?**
  - Agent should acknowledge the question is valid but explain it may not have found the best answer in the book, and offer to explain in general terms or suggest which chapter might cover that topic.

- **What happens when a student rapidly sends multiple messages in quick succession** (e.g., asking 5 questions in 10 seconds)?
  - System should queue messages and process them in order, providing clear feedback that each message is being processed. Rate limiting may apply after 10 messages per minute.

- **What happens when a student closes the chat widget in the middle of the agent typing a response?**
  - The response should be preserved and visible when the student reopens the widget. The agent should not re-send the same response.

- **What happens when a student navigates to a different page while the agent is still responding?**
  - The response should complete in the chat widget and remain visible. The agent's context should update to the new page for any subsequent questions.

- **What happens when multiple browser tabs are open with the same book on different pages?**
  - Each tab should maintain its own context (page location) but share the same conversation history and session. Conversations from any tab should be visible in all tabs.

- **What happens when a student has been inactive for 30+ minutes with the chat widget open?**
  - Session remains active but the agent may send a gentle prompt: "Still reading? Let me know if you have any questions!"

- **What happens when the OpenAI API is temporarily unavailable or returns an error?**
  - The chat widget should display a clear error message: "I'm temporarily unable to respond. Please try again in a moment." and log the error for monitoring.

- **What happens when a student's question contains inappropriate or offensive language?**
  - Agent should maintain professionalism and focus on the learning content, gently redirecting: "Let's keep our conversation focused on learning. How can I help you understand the book content?"

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST integrate the OpenAI ChatKit widget on all 107 pages of the Docusaurus book website in a fixed position at the bottom-right corner
- **FR-002**: ChatKit widget MUST be expandable and collapsible, starting in minimized state with "Ask me!" text visible
- **FR-003**: System MUST maintain conversation context across page navigation within the same browser session
- **FR-004**: System MUST respond to student questions within 3 seconds (95th percentile) under normal load
- **FR-005**: System MUST implement a 4-level RAG retrieval system that prioritizes: (Level 4) highlighted text, (Level 3) current lesson, (Level 2) current chapter, (Level 1) entire book
- **FR-006**: System MUST detect when a student highlights text on the page and automatically send that text to the agent for explanation
- **FR-007**: System MUST capture page context including: page path, page title, current chapter, current lesson, and section ID
- **FR-008**: System MUST persist conversation history and session data across browser sessions
- **FR-009**: System MUST use an autonomous agent system capable of understanding student questions, planning responses, and adapting teaching approach in real-time
- **FR-010**: Agent MUST greet new students with a welcoming message explaining how to use TutorGPT when they first open the chat widget
- **FR-011**: System MUST support at least 100 concurrent users without performance degradation during MVP phase
- **FR-012**: System MUST log all user interactions (questions asked, highlights made, page visits) for analytics and personalization
- **FR-013**: System MUST implement debouncing for highlight detection to prevent spam from rapid text selections (1 second delay)
- **FR-014**: System MUST provide error messages to users when the AI service is unavailable or encounters errors
- **FR-015**: ChatKit widget MUST not block or interfere with reading the book content in its minimized or expanded state
- **FR-016**: System MUST store and retrieve book content using semantic similarity search to find relevant passages matching student questions
- **FR-017**: System MUST implement a content retrieval pipeline that finds and ranks relevant book content based on the 4-level priority system
- **FR-018**: System MUST provide a backend service that handles student questions and returns agent responses
- **FR-019**: Agent MUST provide explanations that reference the student's current page location and section when relevant
- **FR-020**: System MUST track student behavior (questions per hour, repeated topics, time per page) for personalization engine
- **FR-021**: System MUST support keyboard navigation and be WCAG 2.1 Level AA accessible
- **FR-022**: System MUST work on latest 2 versions of Chrome, Firefox, Safari, and Edge browsers
- **FR-023**: System MUST rate limit students to prevent abuse (maximum 10 messages per minute per session)
- **FR-024**: Agent MUST redirect off-topic questions back to book content politely
- **FR-025**: System MUST assign unique session IDs to each student and persist them across browser sessions

### Key Entities

- **Student Session**: Represents a single student's interaction with the book and TutorGPT. Attributes include: session_id (unique identifier), student_id (anonymous identifier), conversation history (all messages), progress tracking (chapters viewed, lessons completed, current location), personalization data (learning pace, confused topics, strong topics, engagement metrics), timestamps (created, last active)

- **Message**: Represents a single message in the conversation. Attributes include: message_id, session_id (foreign key), sender (student or agent), content (message text), timestamp, context_snapshot (page location at time of message), highlighted_text (if applicable), response_time (for agent messages)

- **Page Context**: Represents the student's current location in the book. Attributes include: page_path, page_title, current_chapter, current_lesson, section_id, scroll_position, time_on_page

- **Book Content Chunk**: Represents a chunk of book content stored in the vector database for RAG. Attributes include: chunk_id, source_file, chapter, lesson, content_text, embedding_vector, metadata (headings, code blocks, etc.)

- **Student Profile**: Represents learned information about a student's learning behavior. Attributes include: student_id, learning_pace (slow/medium/fast), confused_topics (list), strong_topics (list), total_questions_asked, total_highlights_made, total_time_spent (minutes), chapters_completed (list)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can receive answers to their questions within 3 seconds in 95% of interactions
- **SC-002**: The ChatKit widget appears correctly on all 107 book pages without blocking content or causing layout issues
- **SC-003**: System supports at least 100 concurrent users with all responding within performance targets (3 second response time)
- **SC-004**: Students can successfully ask a question, receive an answer, and understand it in their first interaction (measured by follow-up question rate - target: <30% require immediate clarification)
- **SC-005**: Conversation history persists correctly across browser sessions for 100% of returning students
- **SC-006**: Text highlight detection works within 2 seconds for 95% of highlights
- **SC-007**: Content retrieval system finds relevant content from the correct priority level (current page > current chapter > entire book) in 90% of queries
- **SC-008**: System uptime of 99.9% during beta and production phases
- **SC-009**: Error rate below 0.1% for all student interactions and system operations
- **SC-010**: Students can complete a full learning session (reading a chapter and asking questions) without encountering any blocking errors in 98% of sessions
- **SC-011**: Agent responses are accurate to book content (measured by manual review of sample conversations - target: 95% accuracy)
- **SC-012**: Page load time remains under 2 seconds even with ChatKit widget loaded
- **SC-013**: Widget is usable on mobile devices (tested on iOS and Android) though full mobile optimization is deferred to Phase 2
- **SC-014**: Students can ask questions using only keyboard navigation (accessibility requirement)
- **SC-015**: Session data is never lost due to database errors (automatic backup and recovery mechanisms in place)

## Assumptions

- Students have modern web browsers (latest 2 versions of major browsers)
- Students have stable internet connections for real-time chat
- The existing book website can be modified to inject custom interactive components
- AI service access is available and remains stable during development and production
- The 107 book lessons are available in a format suitable for content retrieval and search
- Students are primarily reading in English (internationalization deferred to Phase 2)
- Average question length is under 200 words
- Average conversation contains 5-10 messages per session
- Students accept anonymous tracking for personalization (no PII collected)
- Database system is sufficient for MVP with 10,000+ sessions with option to scale if needed
- Browser persistent storage is available and not disabled
- Students do not intentionally try to abuse or break the system (basic rate limiting provides minimal protection)
- The book website build process can accommodate additional styling and interactive components
- Backend service can be deployed on standard cloud infrastructure
- Content storage and retrieval system can handle 107 lessons with reasonable query performance
- Authentication is not required for MVP - all students are anonymous (authentication deferred to Phase 2)

## Out of Scope (Phase 2+)

- Standalone chat interface (not embedded in book pages)
- Pure conversational learning mode (learning by conversation without reading the book)
- User accounts and authentication
- Progress analytics dashboard for students or instructors
- Support for multiple books
- Mobile app (native iOS/Android)
- Custom frontend design beyond standard chat interface styling
- Advanced personalization using machine learning models
- Community features (student-to-student chat, forums)
- Gamification (badges, points, leaderboards)
- Instructor tools (monitoring student progress, creating custom quizzes)
- Voice input/output for questions and answers
- Internationalization and multi-language support
- Advanced security features (rate limiting is basic in MVP)
- Payment or subscription features
- Integration with Learning Management Systems (LMS)
- Downloadable conversation transcripts
- Email notifications or summaries
- Dark mode theme (uses ChatKit default)
- Advanced code execution or sandbox environments for practice
