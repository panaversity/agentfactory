# ChatKit UI Widget Specification

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-01-XX
**Source**: `robolearn-interface/src/components/ChatKitWidget/index.tsx`, `robolearn-interface/src/components/ChatKitWidget/styles.module.css`

## Problem Statement

The RoboLearn educational platform needs a conversational AI interface that:
- Provides a floating chat widget accessible from any page
- Integrates with ChatKit React components for standardized UX
- Supports text selection with "Ask" functionality for contextual questions
- Maintains page context awareness (current page URL, title, headings)
- Requires user authentication before use
- Provides personalization options for user preferences
- Works seamlessly with Docusaurus static site architecture

**Why this exists**: Students need quick access to AI assistance while reading educational content, with context about what they're currently viewing.

## System Intent

**Target Users**: 
- Students learning robotics on RoboLearn platform
- Content authors (for testing chat functionality)

**Core Value Proposition**: 
- Floating chat button always accessible (bottom-right)
- Context-aware assistance (knows current page)
- Text selection → quick questions
- Personalized responses based on user profile

**Key Capabilities**:
- Floating chat button with open/close functionality
- ChatKit React component integration
- Text selection detection and "Ask" button
- Page context extraction and transmission
- User authentication integration
- Personalization menu (settings button)
- Login prompt for unauthenticated users
- Scroll indicator hiding (UI polish)

## Functional Requirements

### Requirement 1: Floating Chat Button

**What**: Fixed-position button in bottom-right corner that opens/closes chat

**Why**: Standard chat widget pattern - always accessible, doesn't interfere with content

**Inputs**: 
- User click on button
- User authentication status

**Outputs**: 
- Chat widget opens/closes
- Button state changes (visual feedback)

**Side Effects**: 
- ChatKit component mounts/unmounts
- Script loading detection triggers

**Success Criteria**: 
- Button visible on all pages
- Click toggles chat open/closed
- Button positioned correctly (bottom-right, 24px from edges)
- Button styled with brand colors (cyan gradient)

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:655-699`, `styles.module.css:14-36`

### Requirement 2: ChatKit Component Integration

**What**: Integrate OpenAI ChatKit React component with custom backend

**Why**: ChatKit provides standardized conversation UI, but needs custom backend URL

**Inputs**: 
- Backend URL (from props or siteConfig)
- Domain key (from siteConfig or env)
- User authentication state

**Outputs**: 
- Rendered ChatKit component
- Chat interface with message history

**Side Effects**: 
- HTTP requests to `/chatkit` endpoint
- Conversation history loaded
- Messages sent/received

**Success Criteria**: 
- ChatKit component renders correctly
- Custom backend URL used
- Authentication headers included
- Streaming responses work

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:190-271`

### Requirement 3: Text Selection and "Ask" Functionality

**What**: Detect text selection and show "Ask" button to send selected text to chat

**Why**: Students can quickly ask questions about specific content they're reading

**Inputs**: 
- User text selection (mouse selection)
- Current page context

**Outputs**: 
- "Ask" button appears above selection
- Selected text sent to chat with context

**Side Effects**: 
- Chat opens if closed
- Message sent with selected text + page context
- Selection cleared after sending

**Success Criteria**: 
- Text selection detected reliably
- "Ask" button positioned correctly above selection
- Button click sends message with context
- Selection cleared after send

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:153-187`, `273-331`

### Requirement 4: Page Context Extraction

**What**: Extract current page metadata (URL, title, headings, description) and send to backend

**Why**: Agent needs to know what page user is viewing for context-aware responses

**Inputs**: 
- Current page DOM
- Meta tags
- Page headings

**Outputs**: 
- Page context object: `{url, title, path, description, keywords, headings, timestamp}`

**Side Effects**: 
- Context included in ChatKit request metadata
- Agent receives page context in prompt

**Success Criteria**: 
- Page URL extracted correctly
- Page title extracted correctly
- Headings extracted (h1-h3, up to 5)
- Meta description extracted
- Context sent with every message

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:121-151`, `202-222`

### Requirement 5: User Authentication Integration

**What**: Require user login before allowing chat access

**Why**: User ID needed for conversation persistence and personalization

**Inputs**: 
- User session from AuthContext
- Login button click

**Outputs**: 
- Login prompt shown if not authenticated
- Redirect to OAuth flow
- Chat accessible after login

**Side Effects**: 
- OAuth redirect initiated
- Session stored in localStorage
- User ID sent to backend

**Success Criteria**: 
- Chat button click checks authentication
- Login prompt shown if not logged in
- OAuth redirect works correctly
- User ID included in requests after login

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:55-65`, `585-596`, `803-820`

### Requirement 6: User Profile Context Transmission

**What**: Send user profile information (name, email, role, software background, hardware tier) to backend

**Why**: Agent needs user profile for personalized responses

**Inputs**: 
- User session data from AuthContext
- ChatKit request metadata

**Outputs**: 
- User info included in request metadata
- Backend receives user profile

**Side Effects**: 
- Agent receives user context in prompt
- Responses personalized to user profile

**Success Criteria**: 
- User info extracted from session
- Info included in ChatKit metadata
- Backend receives correct user info

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:201-222`

### Requirement 7: Personalization Menu

**What**: Settings button above chat button that opens personalization menu

**Why**: Users need easy access to personalize their learning experience

**Inputs**: 
- Settings button click
- Menu option selection

**Outputs**: 
- Personalization menu displayed
- Chat opens with relevant message

**Side Effects**: 
- Menu toggles open/closed
- Messages sent for personalization actions

**Success Criteria**: 
- Settings button visible when logged in
- Menu appears on click
- Options work correctly (Set Preferences, Update Profile, etc.)
- Menu closes on outside click

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:657-786`, `styles.module.css:31-58`

### Requirement 8: Script Loading Detection

**What**: Detect when ChatKit web component script is loaded

**Why**: ChatKit requires external script; component shouldn't render until script ready

**Inputs**: 
- ChatKit script loading events
- Custom element registration

**Outputs**: 
- Script status: 'pending' | 'ready' | 'error'
- Component renders only when ready

**Side Effects**: 
- ChatKit component mounts when script ready
- Error handling if script fails to load

**Success Criteria**: 
- Script loading detected correctly
- Component waits for script before rendering
- Error state handled gracefully

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:67-113`

### Requirement 9: Scroll Indicator Hiding

**What**: Hide unwanted scroll indicators ("Scroll 0%") that appear in ChatKit component

**Why**: UI polish - unwanted indicators clutter the interface

**Inputs**: 
- ChatKit component DOM
- Dynamically added elements

**Outputs**: 
- Scroll indicators hidden via CSS and JavaScript

**Side Effects**: 
- DOM manipulation (hiding elements)
- CSS rules applied

**Success Criteria**: 
- All scroll indicators hidden
- Works for dynamically added elements
- Works in shadow DOM if accessible

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:400-583`, `styles.module.css:127-216`

### Requirement 10: Custom Fetch with Authentication

**What**: Intercept ChatKit requests to add authentication headers and metadata

**Why**: ChatKit needs user ID and context, but doesn't handle auth natively

**Inputs**: 
- ChatKit API requests
- User session
- Page context

**Outputs**: 
- Modified requests with headers and metadata
- User ID in `X-User-ID` header
- Page context in request metadata

**Side Effects**: 
- Backend receives authenticated requests
- Context included in agent prompt

**Success Criteria**: 
- All requests include user ID header
- Page context included in metadata
- User info included in metadata
- Requests fail gracefully if not authenticated

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:197-240`

## Non-Functional Requirements

### Performance

**Observed Patterns**:
- Lazy script loading (only loads when needed)
- Component mounting only when script ready
- Efficient text selection detection (event listeners)

**Target**: 
- Script load time < 2s
- Chat opens < 300ms after click
- Text selection detection < 50ms

**Evidence**: Script loading detection, component mounting logic

### Security

**Observed Patterns**:
- User authentication required
- User ID from authenticated session (not user input)
- No sensitive data in client-side code

**Standards**: 
- All chat requests authenticated
- User ID validated server-side
- No user data exposed in client

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:197-199` (auth check)

### Accessibility

**Observed Patterns**:
- ARIA labels on buttons
- Keyboard navigation support (via ChatKit)
- Focus management

**Standards**: 
- WCAG 2.1 AA compliance
- Screen reader support
- Keyboard accessible

**Evidence**: `aria-label` attributes, semantic HTML

### Responsive Design

**Observed Patterns**:
- Mobile-responsive button sizes
- Responsive chat container (max-width: calc(100vw - 48px))
- Mobile-specific positioning

**Target**: 
- Works on mobile (320px+)
- Works on tablet (768px+)
- Works on desktop (1024px+)

**Evidence**: `styles.module.css:305-353` (mobile responsive)

### Browser Compatibility

**Observed Patterns**:
- Modern browser APIs (customElements, MutationObserver)
- Fallbacks for older browsers
- Shadow DOM handling

**Target**: 
- Chrome/Edge 90+
- Firefox 88+
- Safari 14+

**Evidence**: Custom elements check, MutationObserver usage

## System Constraints

### External Dependencies

- **ChatKit React**: `@openai/chatkit-react` package
- **ChatKit Web Component**: External script from CDN
- **Docusaurus**: Site framework (for siteConfig access)
- **AuthContext**: Custom auth context provider
- **React**: 18+ for hooks and context

### Data Formats

- **ChatKit Protocol**: JSON payloads
- **Metadata**: JSON objects in request metadata
- **Page Context**: JavaScript object with page info

### Deployment Context

- **Static Site**: Docusaurus generates static HTML
- **Build-time Config**: Environment variables baked into build
- **Runtime Config**: siteConfig.customFields for client-side access

### Browser Requirements

- **JavaScript**: ES6+ required
- **Custom Elements**: Required for ChatKit web component
- **Fetch API**: Required for HTTP requests
- **LocalStorage**: Required for auth state

## Non-Goals & Out of Scope

**Explicitly excluded** (inferred from missing implementation):
- **Offline Support**: Requires network connection
- **Message Encryption**: Not implemented (HTTPS only)
- **Multi-language UI**: English only
- **Custom Themes**: Uses ChatKit default theme
- **Voice Input**: Text-only input
- **File Upload UI**: Handled by ChatKit component, not custom UI

## Known Gaps & Technical Debt

### Gap 1: Error Handling Limited

**Issue**: Errors from ChatKit or backend not displayed to user

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:268-270` (only console.error)

**Impact**: Users don't know when requests fail

**Recommendation**: Add error toast/notification component

### Gap 2: No Loading States

**Issue**: No visual feedback during message sending

**Evidence**: No loading spinner or disabled state

**Impact**: Users don't know if message is being sent

**Recommendation**: Add loading state to chat button and message input

### Gap 3: No Retry Logic

**Issue**: Failed requests don't retry automatically

**Evidence**: No retry mechanism in fetch handler

**Impact**: Network errors require manual retry

**Recommendation**: Add exponential backoff retry for transient failures

### Gap 4: Scroll Indicator Hiding Fragile

**Issue**: Multiple attempts needed to hide scroll indicators (CSS + JS + MutationObserver)

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:400-583` (complex hiding logic)

**Impact**: Maintenance burden, may break with ChatKit updates

**Recommendation**: Request ChatKit team to provide CSS variable for hiding indicators

## Success Criteria

### Functional Success

- [x] Floating chat button works on all pages
- [x] Chat opens/closes correctly
- [x] Text selection "Ask" functionality works
- [x] Page context extracted and sent
- [x] User authentication required
- [x] Personalization menu accessible
- [x] User profile sent to backend

### Non-Functional Success

- [x] Responsive design works on mobile/tablet/desktop
- [x] Script loading detected correctly
- [x] Scroll indicators hidden
- [ ] Error handling improved (Gap 1)
- [ ] Loading states added (Gap 2)
- [ ] Retry logic implemented (Gap 3)

## Acceptance Tests

### Test 1: Chat Button Toggle

**Given**: User on any page, logged in

**When**: User clicks chat button

**Then**: 
- Chat widget opens
- Button remains visible
- Click again closes chat

### Test 2: Text Selection "Ask"

**Given**: User on page with content, logged in

**When**: User selects text, clicks "Ask" button

**Then**: 
- "Ask" button appears above selection
- Chat opens if closed
- Message sent with selected text + page context
- Selection cleared

### Test 3: Authentication Required

**Given**: User not logged in

**When**: User clicks chat button

**Then**: 
- Login prompt appears
- Chat does not open
- "Log In" button redirects to OAuth

### Test 4: Page Context Extraction

**Given**: User on page "/ros2/basics" with title "ROS 2 Basics"

**When**: User sends message

**Then**: 
- Request metadata includes pageContext
- Page context has correct URL, title, path
- Headings extracted if present

### Test 5: User Profile Transmission

**Given**: User logged in with profile: `{name: "Alice", hardwareTier: 2}`

**When**: User sends message

**Then**: 
- Request metadata includes userInfo
- User info has correct name and hardware tier
- Backend receives user context

### Test 6: Personalization Menu

**Given**: User logged in

**When**: User clicks settings button

**Then**: 
- Personalization menu appears
- Options visible (Set Preferences, Update Profile, etc.)
- Clicking option opens chat with relevant message
- Menu closes on outside click

### Test 7: Mobile Responsive

**Given**: User on mobile device (320px width)

**When**: User opens chat

**Then**: 
- Chat container fits screen (max-width: calc(100vw - 48px))
- Buttons appropriately sized
- No horizontal scroll

## Architecture Decisions

### ADR-001: Custom Fetch Interceptor vs ChatKit Auth Plugin

**Status**: Accepted

**Context**: Need to add authentication headers and metadata to ChatKit requests

**Decision**: Use custom `fetch` function in ChatKit config

**Rationale**:
1. **Flexibility**: Full control over request modification
2. **Metadata Support**: Can add page context and user info
3. **No Plugin Needed**: Works with standard ChatKit API

**Consequences**:
- **Positive**: Simple, flexible, works immediately
- **Negative**: Must maintain custom fetch logic

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:197-240`

### ADR-002: Page Context Extraction vs Server-Side Detection

**Status**: Accepted

**Context**: Need to know what page user is viewing

**Decision**: Extract page context client-side and send in metadata

**Rationale**:
1. **Client-Side Available**: DOM and window.location accessible
2. **No Server Round-Trip**: Context available immediately
3. **Rich Metadata**: Can extract headings, meta tags, etc.

**Consequences**:
- **Positive**: Fast, rich context, no extra requests
- **Negative**: Context only available if client sends it

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:121-151`

### ADR-003: Text Selection "Ask" vs Copy-Paste

**Status**: Accepted

**Context**: Users want to ask questions about specific content

**Decision**: Detect text selection and show "Ask" button

**Rationale**:
1. **Better UX**: One-click action vs copy-paste
2. **Context Preserved**: Automatically includes page context
3. **Familiar Pattern**: Similar to browser "Search" functionality

**Consequences**:
- **Positive**: Faster, more intuitive, preserves context
- **Negative**: Requires DOM manipulation, event handling

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:153-187`

### ADR-004: Script Loading Detection vs Eager Loading

**Status**: Accepted

**Context**: ChatKit requires external script to be loaded

**Decision**: Detect script loading and wait before rendering component

**Rationale**:
1. **Performance**: Only loads script when needed
2. **Error Handling**: Can detect script load failures
3. **User Experience**: Shows loading state, handles errors

**Consequences**:
- **Positive**: Better performance, error handling
- **Negative**: Slight delay before chat available

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:67-113`

### ADR-005: CSS + JavaScript for Scroll Hiding vs ChatKit Config

**Status**: Accepted (Temporary)

**Context**: ChatKit shows unwanted scroll indicators

**Decision**: Use aggressive CSS + JavaScript to hide indicators

**Rationale**:
1. **No Config Option**: ChatKit doesn't provide way to hide indicators
2. **Immediate Fix**: Works now, improves UX
3. **Multiple Approaches**: CSS + JS + MutationObserver for reliability

**Consequences**:
- **Positive**: Indicators hidden, UX improved
- **Negative**: Fragile, may break with ChatKit updates, maintenance burden

**Recommendation**: Request ChatKit team to add CSS variable for hiding indicators

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:400-583`, `styles.module.css:127-216`

## Implementation Notes

### Key Files

- **`robolearn-interface/src/components/ChatKitWidget/index.tsx`**: Main component (863 lines)
- **`robolearn-interface/src/components/ChatKitWidget/styles.module.css`**: Styles (482 lines)
- **`robolearn-interface/src/theme/Root.tsx`**: Component registration
- **`robolearn-interface/docusaurus.config.ts`**: Configuration (backendUrl, chatkitDomainKey)

### Integration Points

1. **AuthContext**: `@/contexts/AuthContext` - User session and authentication
2. **Docusaurus Config**: `siteConfig.customFields` - Backend URL, domain key
3. **ChatKit React**: `@openai/chatkit-react` - ChatKit component and hooks
4. **OAuth Client**: `@/lib/auth-client` - Login redirect handling

### Configuration

**Build-time Environment Variables** (via `docusaurus.config.ts`):
- `BACKEND_URL`: Backend API URL
- `CHATKIT_DOMAIN_KEY`: ChatKit domain key (for whitelabeled domains)
- `AUTH_URL`: Authentication server URL
- `OAUTH_CLIENT_ID`: OAuth client ID

**Runtime Configuration** (via `siteConfig.customFields`):
- `backendUrl`: Backend API URL (accessible client-side)
- `chatkitDomainKey`: Domain key (accessible client-side)
- `authUrl`: Auth server URL
- `oauthClientId`: OAuth client ID

### Component Structure

```
ChatKitWidget
├── State Management
│   ├── isOpen (chat open/closed)
│   ├── selectedText (text selection)
│   ├── selectionPosition (Ask button position)
│   ├── showPersonalizeMenu (settings menu)
│   └── scriptStatus (ChatKit script loading)
├── Effects
│   ├── Script loading detection
│   ├── Text selection detection
│   ├── Scroll indicator hiding
│   └── Click outside handlers
├── Handlers
│   ├── handleChatButtonClick
│   ├── handleAskSelectedText
│   ├── handlePersonalize
│   └── handleLogin
└── Render
    ├── Settings button
    ├── Chat button
    ├── Personalization menu
    ├── ChatKit component
    ├── Login prompt
    └── Ask button (when text selected)
```

