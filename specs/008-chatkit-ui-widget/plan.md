# ChatKit UI Widget Implementation Plan

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-01-XX

## Architecture Overview

**Architectural Style**: React Component with Hooks and Context Integration

**Reasoning**: 
- React component integrates ChatKit React library
- Uses hooks for state management and side effects
- Integrates with Docusaurus (static site) and AuthContext
- Client-side context extraction and request interception

**Diagram**:
```
┌─────────────────────────────────────────┐
│         Docusaurus Site                │
│  ┌───────────────────────────────────┐  │
│  │   Root.tsx                       │  │
│  │   - Renders ChatKitWidget        │  │
│  └──────────────┬────────────────────┘  │
└─────────────────┼────────────────────────┘
                  │
┌─────────────────▼────────────────────────┐
│      ChatKitWidget Component             │
│  ┌───────────────────────────────────┐  │
│  │   State Management                │  │
│  │   - isOpen, selectedText, etc.    │  │
│  │   - Script loading detection      │  │
│  └──────────────┬────────────────────┘  │
│  ┌──────────────▼────────────────────┐  │
│  │   useChatKit Hook                 │  │
│  │   - Custom fetch interceptor      │  │
│  │   - Adds auth headers             │  │
│  │   - Adds page context             │  │
│  └──────────────┬────────────────────┘  │
│  ┌──────────────▼────────────────────┐  │
│  │   ChatKit React Component         │  │
│  │   - Renders chat UI               │  │
│  │   - Handles messages              │  │
│  └──────────────┬────────────────────┘  │
└─────────────────┼────────────────────────┘
                  │
┌─────────────────▼────────────────────────┐
│      Custom Fetch Interceptor             │
│  ┌───────────────────────────────────┐  │
│  │   - Inject X-User-ID header       │  │
│  │   - Add page context metadata     │  │
│  │   - Add user info metadata        │  │
│  └──────────────┬────────────────────┘  │
└─────────────────┼────────────────────────┘
                  │
┌─────────────────▼────────────────────────┐
│      Backend /chatkit Endpoint           │
└──────────────────────────────────────────┘
```

## Layer Structure

### Layer 1: Component Layer

**Responsibility**: React component rendering, state management, user interactions

**Components**:
- `ChatKitWidget`: Main component
- State hooks: `useState` for component state
- Effect hooks: `useEffect` for side effects
- Callback hooks: `useCallback` for handlers

**Dependencies**: → ChatKit React, → AuthContext, → Docusaurus Context

**Technology**: React 18+, TypeScript

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:26-863`

### Layer 2: Integration Layer

**Responsibility**: Integrate ChatKit React with custom backend and auth

**Components**:
- `useChatKit` hook: ChatKit React hook with custom config
- Custom fetch: Intercept requests, add headers/metadata
- Script loading: Detect ChatKit web component script

**Dependencies**: → ChatKit React SDK

**Technology**: React hooks, ChatKit React SDK

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:190-271`

### Layer 3: Context Extraction Layer

**Responsibility**: Extract page context and user profile

**Components**:
- `getPageContext()`: Extract page metadata
- User session: Extract from AuthContext
- Metadata building: Combine user info + page context

**Dependencies**: → DOM APIs, → AuthContext

**Technology**: JavaScript DOM APIs, React Context

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:121-151`

### Layer 4: UI Enhancement Layer

**Responsibility**: Text selection, scroll hiding, personalization menu

**Components**:
- Text selection detection: Event listeners
- Scroll indicator hiding: CSS + JavaScript + MutationObserver
- Personalization menu: Settings button + menu component

**Dependencies**: → DOM APIs, → CSS

**Technology**: JavaScript, CSS, DOM APIs

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:153-187`, `400-583`

## Design Patterns Applied

### Pattern 1: Custom Fetch Interceptor

**Location**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:197-240`

**Purpose**: Add authentication and context to ChatKit requests

**Implementation**:
- Provide custom `fetch` function to `useChatKit`
- Intercept all requests
- Add `X-User-ID` header
- Add metadata (userInfo, pageContext) to request body

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:197-240`

### Pattern 2: Script Loading Detection

**Location**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:67-113`

**Purpose**: Wait for ChatKit web component script before rendering

**Implementation**:
- Check for custom element registration
- Listen for script load events
- Set status: 'pending' | 'ready' | 'error'
- Only render ChatKit when 'ready'

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:67-113`

### Pattern 3: Text Selection with "Ask" Button

**Location**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:153-187`

**Purpose**: Enable quick questions about selected text

**Implementation**:
- Listen for `selectionchange` and `mouseup` events
- Extract selected text and position
- Show "Ask" button above selection
- Send message with selected text + page context

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:153-187`, `273-331`

### Pattern 4: Page Context Extraction

**Location**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:121-151`

**Purpose**: Extract rich page metadata for agent context

**Implementation**:
- Query DOM for meta tags (description, keywords)
- Find main content area (article, main, body)
- Extract headings (h1-h3, up to 5)
- Build context object with URL, title, path, headings, description

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:121-151`

### Pattern 5: Aggressive Scroll Hiding

**Location**: `robolearn-interface/src/components/ChatKitWidget/index.tsx:400-583`, `styles.module.css:127-216`

**Purpose**: Hide unwanted scroll indicators from ChatKit component

**Implementation**:
- CSS rules: Hide elements with scroll-related classes/attributes
- JavaScript: MutationObserver to catch dynamically added elements
- TreeWalker: Find text nodes containing "Scroll" and "%"
- Multiple timeouts: Check at different intervals

**Evidence**: Multiple approaches for reliability

## Data Flow

### Chat Open Flow

1. **User Clicks Button**: `handleChatButtonClick()` called
2. **Auth Check**: If not logged in, show login prompt
3. **Script Check**: Verify ChatKit script loaded
4. **State Update**: Set `isOpen = true`
5. **Component Render**: ChatKit component mounts
6. **History Load**: ChatKit loads conversation history
7. **Ready State**: Chat ready for messages

### Message Send Flow

1. **User Types Message**: In ChatKit input
2. **ChatKit Request**: ChatKit calls custom `fetch`
3. **Context Extraction**: Extract page context, user info
4. **Request Modification**: Add headers, add metadata to body
5. **HTTP Request**: POST to `/chatkit` endpoint
6. **Streaming Response**: Receive SSE stream
7. **UI Update**: ChatKit renders streaming response

### Text Selection "Ask" Flow

1. **User Selects Text**: Mouse selection detected
2. **Selection Handler**: Extract text and position
3. **Ask Button Render**: Show button above selection
4. **User Clicks Ask**: `handleAskSelectedText()` called
5. **Chat Open**: Open chat if closed
6. **Message Build**: Build message with selected text + page context
7. **Send Message**: Use `sendUserMessage()` hook
8. **Selection Clear**: Clear selection after send

## Technology Stack

### Frontend Framework

- **Primary**: React 18+
- **Rationale**: Component-based, hooks for state management, good ecosystem

### UI Library

- **Choice**: ChatKit React (`@openai/chatkit-react`)
- **Rationale**: Pre-built chat UI, standardized UX, attachment support

### Site Framework

- **Choice**: Docusaurus 3.x
- **Rationale**: Static site generator, MDX support, plugin system

### Styling

- **Choice**: CSS Modules
- **Rationale**: Scoped styles, no conflicts, works with Docusaurus

### Type Safety

- **Choice**: TypeScript
- **Rationale**: Type safety, better IDE support, catches errors early

### Build System

- **Choice**: Docusaurus build (Webpack)
- **Rationale**: Integrated with Docusaurus, handles React, TypeScript

## Module Breakdown

### Module: ChatKitWidget Component

**Purpose**: Main chat widget component

**Key Functions**:
- `handleChatButtonClick()`: Toggle chat
- `handleAskSelectedText()`: Send selected text
- `handlePersonalize()`: Personalization actions
- `getPageContext()`: Extract page metadata

**Dependencies**: 
- ChatKit React
- AuthContext
- Docusaurus Context

**Complexity**: High (863 lines)

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/index.tsx`

### Module: Styles

**Purpose**: Component styling

**Key Styles**:
- `.chatButton`: Floating button styles
- `.chatKitContainer`: Chat container styles
- `.askButton`: Text selection "Ask" button
- `.personalizeMenu`: Settings menu styles

**Dependencies**: CSS Modules

**Complexity**: Medium (482 lines)

**Evidence**: `robolearn-interface/src/components/ChatKitWidget/styles.module.css`

### Module: Integration Points

**Purpose**: Connect widget to site and auth

**Key Files**:
- `Root.tsx`: Widget registration
- `docusaurus.config.ts`: Configuration

**Dependencies**: Docusaurus, AuthContext

**Complexity**: Low

**Evidence**: `robolearn-interface/src/theme/Root.tsx`, `docusaurus.config.ts`

## Regeneration Strategy

### Option 1: Specification-First Rebuild

1. Start with spec.md (requirements and architecture)
2. Apply extracted patterns (custom fetch, script detection, text selection)
3. Implement with improvements:
   - Add error handling UI
   - Add loading states
   - Add retry logic
   - Improve scroll hiding (request ChatKit config option)
4. Test-driven development using acceptance criteria

**Timeline**: 1-2 weeks (single developer)

### Option 2: Incremental Enhancement

1. **Keep existing implementation** (works well)
2. **Add missing features**:
   - Error toast notifications
   - Loading spinners
   - Retry logic for failed requests
3. **Improve UX**:
   - Better error messages
   - Loading feedback
   - Offline detection

**Timeline**: 3-5 days (single developer)

## Improvement Opportunities

### Technical Improvements

- [ ] **Add Error Handling UI**
  - **Addresses Gap**: Gap 1 (error handling limited)
  - **Effort**: Low (add toast component)
  - **Library**: `react-hot-toast` or custom component

- [ ] **Add Loading States**
  - **Addresses Gap**: Gap 2 (no loading states)
  - **Effort**: Low (add loading spinner)
  - **States**: Message sending, chat opening

- [ ] **Add Retry Logic**
  - **Addresses Gap**: Gap 3 (no retry logic)
  - **Effort**: Medium (add exponential backoff)
  - **Approach**: Retry transient failures (network errors)

- [ ] **Improve Scroll Hiding**
  - **Addresses Gap**: Gap 4 (fragile scroll hiding)
  - **Effort**: Low (request ChatKit team)
  - **Approach**: Request CSS variable or config option from ChatKit

### UX Improvements

- [ ] **Add Offline Detection**
  - **Enables**: Show offline message when network unavailable
  - **Effort**: Low (use navigator.onLine API)
  - **Benefit**: Better user experience

- [ ] **Add Keyboard Shortcuts**
  - **Enables**: Quick access (e.g., Cmd+K to open chat)
  - **Effort**: Low (add keyboard event listeners)
  - **Benefit**: Power user experience

- [ ] **Add Message History Search**
  - **Enables**: Search previous conversations
  - **Effort**: Medium (add search UI, filter messages)
  - **Benefit**: Find past answers

### Accessibility Improvements

- [ ] **Improve ARIA Labels**
  - **Enables**: Better screen reader support
  - **Effort**: Low (add more descriptive labels)
  - **Benefit**: Accessibility compliance

- [ ] **Add Keyboard Navigation**
  - **Enables**: Full keyboard access
  - **Effort**: Medium (add focus management)
  - **Benefit**: Accessibility compliance

