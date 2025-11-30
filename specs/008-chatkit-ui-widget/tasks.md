# ChatKit UI Widget Implementation Tasks

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-01-XX

## Overview

This task breakdown represents how to rebuild the ChatKit UI widget from scratch using the specification and plan.

**Estimated Timeline**: 1-2 weeks (single developer)
**Team Size**: 1 frontend engineer

---

## Phase 1: Component Foundation

**Timeline**: Days 1-2
**Dependencies**: None

### Task 1.1: Project Setup

- [ ] Create component directory: `src/components/ChatKitWidget/`
- [ ] Create `index.tsx` file
- [ ] Create `styles.module.css` file
- [ ] Setup TypeScript interfaces
- [ ] Import required dependencies: React, ChatKit React, AuthContext

### Task 1.2: Basic Component Structure

- [ ] Create `ChatKitWidget` component function
- [ ] Setup component props interface (`ChatKitWidgetProps`)
- [ ] Add basic state management (useState hooks)
- [ ] Add component return JSX structure
- [ ] Test component renders

### Task 1.3: Configuration Setup

- [ ] Read backend URL from props or siteConfig
- [ ] Read domain key from siteConfig
- [ ] Read auth URL and OAuth client ID
- [ ] Add fallback defaults for development
- [ ] Test configuration reading

---

## Phase 2: ChatKit Integration

**Timeline**: Days 3-4
**Dependencies**: Phase 1 complete

### Task 2.1: ChatKit Hook Setup

- [ ] Import `useChatKit` from `@openai/chatkit-react`
- [ ] Configure ChatKit API:
  - Custom backend URL
  - Domain key
  - Custom fetch function
- [ ] Setup theme configuration
- [ ] Setup start screen prompts
- [ ] Test ChatKit initialization

### Task 2.2: Script Loading Detection

- [ ] Check for ChatKit custom element on mount
- [ ] Listen for script load events
- [ ] Handle script load timeout (5 seconds)
- [ ] Set script status: 'pending' | 'ready' | 'error'
- [ ] Only render ChatKit when 'ready'
- [ ] Test script loading detection

### Task 2.3: Custom Fetch Interceptor

- [ ] Implement custom `fetch` function
- [ ] Check user authentication
- [ ] Extract user ID from session
- [ ] Extract page context
- [ ] Extract user info from session
- [ ] Add `X-User-ID` header
- [ ] Add metadata to request body (userInfo, pageContext)
- [ ] Test request interception

---

## Phase 3: UI Components

**Timeline**: Days 5-6
**Dependencies**: Phase 2 complete

### Task 3.1: Floating Chat Button

- [ ] Create chat button component
- [ ] Position fixed (bottom-right, 24px from edges)
- [ ] Style with brand colors (cyan gradient)
- [ ] Add hover and active states
- [ ] Add click handler to toggle chat
- [ ] Add ARIA labels
- [ ] Test button functionality

### Task 3.2: Chat Container

- [ ] Create chat container div
- [ ] Position fixed (above button)
- [ ] Size: 400px width, 600px height
- [ ] Responsive: max-width calc(100vw - 48px)
- [ ] Add slide-up animation
- [ ] Hide scrollbars
- [ ] Test container rendering

### Task 3.3: ChatKit Component Rendering

- [ ] Conditionally render ChatKit component
- [ ] Only render when: isOpen && scriptStatus === 'ready' && isLoggedIn
- [ ] Pass control prop from useChatKit
- [ ] Apply styles (hide scrollbars)
- [ ] Test ChatKit renders correctly

---

## Phase 4: Authentication Integration

**Timeline**: Day 7
**Dependencies**: Phase 3 complete

### Task 4.1: Auth Check

- [ ] Import `useAuth` hook
- [ ] Check if user is logged in (`session?.user?.id`)
- [ ] Handle loading state
- [ ] Show login prompt if not authenticated
- [ ] Test authentication flow

### Task 4.2: Login Prompt

- [ ] Create login prompt overlay
- [ ] Show when: isOpen && !isLoggedIn && !authLoading
- [ ] Display message: "Login Required"
- [ ] Add "Log In" button
- [ ] Add "Close" button
- [ ] Handle login redirect
- [ ] Test login prompt

### Task 4.3: Login Handler

- [ ] Implement `handleLogin()` function
- [ ] Get OAuth authorization URL
- [ ] Store PKCE code verifier in localStorage
- [ ] Redirect to OAuth flow
- [ ] Test login redirect

---

## Phase 5: Text Selection Feature

**Timeline**: Days 8-9
**Dependencies**: Phase 4 complete

### Task 5.1: Selection Detection

- [ ] Add event listeners: `selectionchange`, `mouseup`
- [ ] Extract selected text
- [ ] Calculate selection position
- [ ] Update state: selectedText, selectionPosition
- [ ] Clear selection on click outside
- [ ] Test selection detection

### Task 5.2: "Ask" Button

- [ ] Create "Ask" button component
- [ ] Position above selection (centered)
- [ ] Style with subtle cyan background
- [ ] Show only when: selectedText && selectionPosition && isLoggedIn
- [ ] Add click handler
- [ ] Prevent event propagation
- [ ] Test "Ask" button

### Task 5.3: Send Selected Text

- [ ] Implement `handleAskSelectedText()` function
- [ ] Build message with selected text + page context
- [ ] Open chat if closed
- [ ] Wait for ChatKit ready (300ms)
- [ ] Send message using `sendUserMessage()`
- [ ] Clear selection after send
- [ ] Handle errors gracefully
- [ ] Test message sending

---

## Phase 6: Page Context Extraction

**Timeline**: Day 10
**Dependencies**: Phase 5 complete

### Task 6.1: Page Context Function

- [ ] Implement `getPageContext()` function
- [ ] Extract page URL (`window.location.href`)
- [ ] Extract page title (`document.title`)
- [ ] Extract page path (`window.location.pathname`)
- [ ] Extract meta description
- [ ] Extract meta keywords
- [ ] Find main content area (article, main, body)
- [ ] Extract headings (h1-h3, up to 5)
- [ ] Build context object
- [ ] Test context extraction

### Task 6.2: Context Transmission

- [ ] Include page context in custom fetch
- [ ] Add to request metadata
- [ ] Test context sent to backend

---

## Phase 7: Personalization Menu

**Timeline**: Day 11
**Dependencies**: Phase 6 complete

### Task 7.1: Settings Button

- [ ] Create settings button component
- [ ] Position above chat button
- [ ] Style with subtle background
- [ ] Show only when logged in
- [ ] Add click handler to toggle menu
- [ ] Test settings button

### Task 7.2: Personalization Menu

- [ ] Create menu component
- [ ] Position absolute (above settings button)
- [ ] Add menu header with title and close button
- [ ] Add menu options:
  - Set Learning Preferences
  - Update Profile
  - What Can You Help With?
  - Learning Progress
- [ ] Handle option clicks
- [ ] Close menu on outside click
- [ ] Test menu functionality

### Task 7.3: Personalization Actions

- [ ] Implement `handlePersonalize()` function
- [ ] Close menu
- [ ] Open chat
- [ ] Send relevant message after delay
- [ ] Test personalization actions

---

## Phase 8: UI Polish

**Timeline**: Day 12
**Dependencies**: Phase 7 complete

### Task 8.1: Scroll Indicator Hiding

- [ ] Add CSS rules to hide scroll indicators
- [ ] Target elements with scroll-related classes/attributes
- [ ] Use `:global()` for ChatKit component
- [ ] Add JavaScript MutationObserver
- [ ] Find and hide dynamically added elements
- [ ] Use TreeWalker for text nodes
- [ ] Add multiple timeout checks
- [ ] Test scroll hiding

### Task 8.2: Responsive Design

- [ ] Add mobile styles (max-width: 768px)
- [ ] Adjust button sizes for mobile
- [ ] Adjust container sizes for mobile
- [ ] Test on mobile devices

### Task 8.3: Animations

- [ ] Add slide-up animation for chat container
- [ ] Add slide-down animation for "Ask" button
- [ ] Add hover transitions
- [ ] Test animations

---

## Phase 9: Integration & Testing

**Timeline**: Days 13-14
**Dependencies**: Phase 8 complete

### Task 9.1: Docusaurus Integration

- [ ] Register component in `Root.tsx`
- [ ] Pass backendUrl from siteConfig
- [ ] Test component loads on all pages
- [ ] Verify configuration passed correctly

### Task 9.2: Build Configuration

- [ ] Update `docusaurus.config.ts`:
  - Add `chatkitDomainKey` to customFields
  - Read from `CHATKIT_DOMAIN_KEY` env var
- [ ] Test build process
- [ ] Verify env vars baked into build

### Task 9.3: Component Testing

- [ ] Test chat button toggle
- [ ] Test text selection "Ask"
- [ ] Test authentication flow
- [ ] Test page context extraction
- [ ] Test personalization menu
- [ ] Test scroll hiding
- [ ] Test responsive design

### Task 9.4: Integration Testing

- [ ] Test with real backend
- [ ] Test message sending/receiving
- [ ] Test conversation history
- [ ] Test user context transmission
- [ ] Test page context transmission
- [ ] Test error handling

---

## Phase 10: Improvements & Production Readiness

**Timeline**: Days 15-16
**Dependencies**: Phase 9 complete

### Task 10.1: Error Handling UI

- [ ] Add error toast component
- [ ] Display errors from ChatKit
- [ ] Display errors from backend
- [ ] Test error display

### Task 10.2: Loading States

- [ ] Add loading spinner for message sending
- [ ] Add loading state for chat opening
- [ ] Disable buttons during loading
- [ ] Test loading states

### Task 10.3: Retry Logic

- [ ] Add retry logic for failed requests
- [ ] Exponential backoff
- [ ] Retry transient failures only
- [ ] Test retry behavior

### Task 10.4: Accessibility

- [ ] Improve ARIA labels
- [ ] Add keyboard navigation
- [ ] Test with screen reader
- [ ] Test keyboard-only navigation

### Task 10.5: Documentation

- [ ] Document component props
- [ ] Document configuration
- [ ] Document usage
- [ ] Add code comments

---

## Phase 11: Deployment

**Timeline**: Day 17
**Dependencies**: Phase 10 complete

### Task 11.1: Build & Deploy

- [ ] Test production build
- [ ] Verify environment variables set
- [ ] Deploy to staging
- [ ] Test on staging
- [ ] Deploy to production

### Task 11.2: Monitoring

- [ ] Monitor error rates
- [ ] Monitor user engagement
- [ ] Gather user feedback
- [ ] Iterate on improvements

---

## Post-Launch

**Timeline**: Ongoing

### Task 12.1: Monitoring & Iteration

- [ ] Monitor component performance
- [ ] Track user interactions
- [ ] Gather feedback
- [ ] Prioritize improvements

### Task 12.2: Feature Enhancements

- [ ] Add offline detection
- [ ] Add keyboard shortcuts
- [ ] Add message history search
- [ ] Improve scroll hiding (request ChatKit config)

