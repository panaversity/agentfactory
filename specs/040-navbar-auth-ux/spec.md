# Feature Specification: Navbar Authentication UX Refinement

**Feature Branch**: `040-navbar-auth-ux`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User feedback on navbar button labeling, spacing, and hierarchy issues

## Problem Statement

User feedback identified UX issues with the current navbar authentication controls:

1. **"Get Started" label is confusing** - Does not clearly communicate "create an account"
2. **Insufficient spacing** - Search component too close to authentication buttons
3. **Button hierarchy needs refinement** - Visual distinction between primary/secondary actions could be sharper

### Evidence

- **Source**: User feedback via WhatsApp screenshots + text.md
- **Reference**: [Balsamiq Button Design Best Practices](https://balsamiq.com/blog/button-design-best-practices/)
- **Key Principles Violated**:
  - Principle #8: "Say exactly what happens next" - "Get Started" is vague
  - Principle #13: "Use padding to create comfortable tap zones" - cramped spacing
  - Principle #2: "Primary and secondary buttons should look different" - hierarchy could be clearer

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration Flow (Priority: P1)

A new visitor arrives at the site and wants to create an account. They should immediately understand which button creates a new account versus which one logs them into an existing account.

**Why this priority**: Registration is the primary conversion goal. Confusion at this step loses potential users.

**Independent Test**: Can be verified by showing the navbar to 5 users unfamiliar with the site and asking "Which button would you click to create a new account?"

**Acceptance Scenarios**:

1. **Given** a new visitor viewing the navbar, **When** they see the authentication buttons, **Then** they can identify the "Sign Up" button as the action to create a new account within 2 seconds
2. **Given** a visitor who wants to create an account, **When** they click "Sign Up", **Then** they are directed to the registration flow (no confusion with sign-in)

---

### User Story 2 - Returning User Sign-In (Priority: P1)

An existing user returns to the site and wants to log in. They should quickly locate the sign-in option without visual confusion from the primary CTA.

**Why this priority**: Equal to P1 as returning users are equally important for engagement.

**Independent Test**: Can be verified by showing navbar to existing users and timing how quickly they locate "Sign In"

**Acceptance Scenarios**:

1. **Given** a returning user viewing the navbar, **When** they scan for login, **Then** "Sign In" is clearly visible as a secondary (but not hidden) action
2. **Given** a returning user, **When** they click "Sign In", **Then** they are directed to the authentication flow

---

### User Story 3 - Visual Scanning and Cognitive Load (Priority: P2)

Users scanning the navbar should perceive clear visual separation between functional groups: navigation, search, and authentication.

**Why this priority**: Reduces cognitive load and improves overall UX, but not as critical as P1 conversion paths.

**Independent Test**: Eye-tracking or user feedback on visual grouping clarity

**Acceptance Scenarios**:

1. **Given** any user viewing the navbar, **When** they scan from left to right, **Then** they perceive search and authentication as visually distinct groups with clear spacing
2. **Given** a user on mobile, **When** viewing the navbar, **Then** touch targets remain adequately separated (minimum 8px gap)

---

### Edge Cases

- **Mobile viewport**: Spacing adjustments must scale appropriately for smaller screens
- **Loading state**: Button labels should not shift or cause layout reflow during auth state changes
- **Dark mode**: Visual hierarchy must remain clear in both light and dark themes
- **Long usernames**: Authenticated state dropdown should handle long names gracefully (existing behavior, no change needed)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display "Sign Up" as the label for the primary registration CTA (replacing "Get Started")
- **FR-002**: System MUST display "Sign In" as the label for the secondary authentication action
- **FR-003**: "Sign In" button MUST appear to the LEFT of "Sign Up" button (following web convention)
- **FR-004**: System MUST provide minimum 1.5rem (24px) visual gap between search component and authentication buttons on desktop
- **FR-005**: System MUST provide minimum 0.75rem (12px) visual gap between search and auth on mobile (< 996px)
- **FR-006**: "Sign Up" button MUST be visually styled as primary action (filled, brand color)
- **FR-007**: "Sign In" button MUST be visually styled as secondary action (ghost/text style)
- **FR-008**: Visual hierarchy MUST be maintained in both light mode and dark mode themes

### Non-Functional Requirements

- **NFR-001**: Changes MUST NOT alter existing functionality (click handlers, auth flows)
- **NFR-002**: Changes MUST NOT increase bundle size by more than 1KB
- **NFR-003**: Changes MUST maintain WCAG 2.1 AA compliance for color contrast and touch targets

### Constraints

- Work within existing Polar Night design system (no new colors or fonts)
- Preserve existing component architecture (NavbarAuth, SearchBar)
- CSS-only changes where possible (no new JavaScript logic)

### Non-Goals (Explicitly Excluded)

- Redesigning the entire navbar layout
- Changing authentication flow logic
- Adding new authentication methods
- Modifying the search component functionality
- Changing the user dropdown menu design
- Mobile hamburger menu modifications

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of test users can correctly identify which button creates a new account on first viewing (currently ~70% based on feedback)
- **SC-002**: Button labels match industry-standard terminology ("Sign In" / "Sign Up") used by 95%+ of major web applications
- **SC-003**: Visual spacing between search and auth creates perceptible grouping (validated by 3+ user confirmations)
- **SC-004**: No regression in existing auth flow functionality (all existing tests pass)
- **SC-005**: Light mode and dark mode both display correct visual hierarchy (manual verification)

### Verification Method

- Visual inspection in browser (both themes)
- Build verification (no errors, bundle size check)
- Click-through testing of Sign In and Sign Up flows
- Responsive testing at mobile breakpoint (996px)

## Assumptions

1. **Button order convention**: "Sign In" left, "Sign Up" right follows industry standard (validated against Google, GitHub, Stripe, Notion)
2. **Spacing value**: 1.5rem provides adequate visual separation without excessive whitespace
3. **No A/B testing required**: Change aligns with established UX best practices
4. **Existing styles sufficient**: Current `.signInLink` and `.getStartedButton` class structure can be reused with label change

## Files Affected (Preliminary)

| File | Change Type | Description |
|------|-------------|-------------|
| `book-source/src/components/NavbarAuth/index.tsx` | Modify | Change button label from "Get Started" to "Sign Up" |
| `book-source/src/components/SearchBar/styles.module.css` | Modify | Add margin-right for spacing |
| `book-source/src/components/NavbarAuth/styles.module.css` | Modify (optional) | Minor hover state polish if needed |

## References

- [Balsamiq Button Design Best Practices](https://balsamiq.com/blog/button-design-best-practices/)
- User feedback: `feedback/text.md`
- Screenshots: `feedback/WhatsApp Image 2025-12-04 at 12.27.08 PM.jpeg` (current state)
