---
title: "Antigravity Agent Architecture and Features"
lesson_number: 7
proficiency_level: "B1"
estimated_time: "120-150 minutes"
learning_objectives:
  - "Use Agent-Assisted Development workflow to guide AI-generated code"
  - "Review and approve Implementation Plans before agents execute code"
  - "Work in parallel while agents research and implement in background"
  - "Leverage browser integration for automated testing and verification"
  - "Build a complete project using artifact-driven specification-first workflow"
skills:
  agent-assisted-development:
    proficiency: "B1"
  artifact-driven-workflow:
    proficiency: "B1"
  implementation-plan-review:
    proficiency: "B1"
generated_by: content-implementer v1.0.0
source_spec: specs/chapter-8/lesson-7-spec.md
created: "2025-11-20"
workflow: /sp.implement
version: 1.0.0
---

# Lesson 7: Antigravity Agent Architecture and Features

## Building AI Systems That Know When to Ask

You've now explored two approaches to AI collaboration in development: **Zed** (inline assistance keeping you in control), and **Cursor** (autonomous agent mode generating code with diffs). Antigravity takes a third path—one that treats AI not as a tool you control, but as an intelligent agent that decides *when* to involve you.

This lesson shows you how that works. You'll learn to define complex projects through specifications, let agents research and plan implementation, review their plans, and work in parallel while agents execute—all while maintaining human oversight through artifact approval gates. This workflow embodies the core principle this course teaches: **specification clarity drives better AI outcomes**.

By the end of this lesson, you'll have built a real application (a recipe finder) using pure artifact-driven workflow—no manual coding, only specification, review, and approval.

---

## Part 1: The Autonomy Spectrum

Before diving into features, understand the fundamental shift Antigravity enables: **agent autonomy levels**.

### Three Autonomy Modes

In Zed, you ask for code and type it yourself. In Cursor, you can set the agent to autonomous and it generates code. Antigravity goes further—it lets you configure *how much* an agent should decide independently, creating a spectrum from "ask always" to "decide everything."

#### Mode 1: Ask Always (Maximum Control)

**What it means**: Agent asks permission before executing anything. You maintain complete control.

**When to use**: Complex projects where you want to review every decision, sensitive systems where human oversight is critical, or when learning how agents think.

**Example interaction**:
```
You: "Create a login form with email validation"

Agent: "I'll create a login form. Here's what I propose:
- React component with useState for form state
- Email validation using regex pattern
- Error messages for invalid email

Components to create:
1. LoginForm.tsx (form component)
2. utils/validation.ts (validation helpers)

Should I proceed?"

You: [Review plan] "Yes, proceed"

Agent: [Creates files]
```

#### Mode 2: Ask Sometimes (Balanced)

**What it means**: Agent auto-executes routine tasks, asks for decisions on complex choices.

**When to use**: Most development work. You stay focused on decisions, agents handle routine execution.

**Example interaction**:
```
You: "Add error handling to all API calls"

Agent: "Understood. I've identified 8 API calls across 3 files.
I can handle most with try/catch blocks, but which pattern?

Option A: try/catch with specific exception types
Option B: Wrapper decorator around all API calls
Option C: Promise.catch() chaining

Recommendation: Option A (most explicit, easiest to debug)
Should I use this pattern?"

You: [Think about project needs] "Use Option A"

Agent: [Auto-executes across all 8 calls]
```

#### Mode 3: Full Auto (Maximum Efficiency)

**What it means**: Agent decides everything independently. You review *after* completion through artifacts.

**When to use**: Well-defined tasks with clear success criteria, exploratory work, building components you'll review later.

**Example interaction**:
```
You: "Add dark mode toggle to the UI. Success criteria:
- Toggle saves preference to localStorage
- CSS updates immediately on toggle
- Works across all pages"

Agent: [Works completely independently]
Agent: [Completes and generates Walkthrough artifact]

You: [Review Walkthrough showing before/after screenshots]
You: "Approved! Merge this."
```

### Decision Framework: Which Mode to Choose?

**Ask Always** when:
- Building foundational systems (auth, payment, security-critical)
- Learning how agent reasoning works
- Requirements are vague (need back-and-forth)

**Ask Sometimes** when:
- Requirements are clear but implementation has choices
- Task spans multiple files or components
- You need some decisions but routine execution is safe

**Full Auto** when:
- Task is well-defined with measurable success
- You trust the agent's capability
- You're working on multiple things simultaneously

For Lesson 7, you'll practice all three modes to experience when each makes sense.

---

## Part 2: Feature 1 — Agent-Assisted Development

### How It Works: The Request → Plan → Execute Cycle

Agent-Assisted Development (AAD) is a workflow where agents break down your request into structured thinking before writing code. Unlike traditional code generation (prompt → code), AAD follows: **Request → Analysis → Plan Proposal → Your Approval → Execution**.

This matters because **approval gates catch errors early**. If the agent's plan is wrong, you catch it before wasting time implementing the wrong solution.

### The Three Artifacts Generated

When you assign a task, the agent creates three artifacts:

**Artifact 1: Task List**

A breakdown of everything the agent will do. Each task has:
- What needs to be created or modified
- Why (purpose in the larger project)
- Verification method (how to know it's done)

Example Task List for "Add login form with validation":
```
## Task List: Login Form Implementation

### Task 1: Create LoginForm Component
- File: src/components/LoginForm.tsx
- Purpose: Main login UI with email and password fields
- Verification: Component renders with two input fields and submit button

### Task 2: Email Validation Logic
- File: src/utils/validateEmail.ts
- Purpose: Validate email format and check against existing users
- Verification: Returns true for valid emails, false for invalid

### Task 3: API Integration
- File: src/services/auth.ts
- Purpose: Call backend login endpoint
- Verification: Handles success (stores token) and failure (returns error message)

### Task 4: Error Display
- File: (part of LoginForm.tsx)
- Purpose: Show validation and API errors to user
- Verification: Errors appear below input fields when validation fails
```

**Artifact 2: Implementation Plan**

The detailed strategy before any code is written. Includes:
- Research findings (e.g., best practices discovered)
- Architecture decisions and why each was chosen
- Components to create or modify
- Dependencies needed
- Verification strategy (how to test)

Example Implementation Plan for API integration:
```
## Implementation Plan: Login Form with API Integration

### Research Findings
- Email validation: Use RFC 5322 compatible regex (most servers accept this)
- Password security: Use bcrypt on backend, enforce 8+ character minimum on frontend
- Token storage: localStorage for MVP (consider httpOnly cookies for production)

### Architecture Decisions

**Decision 1: Where to validate?**
- Client-side: Immediate feedback, better UX
- Server-side: Security essential (never trust client)
- Solution: Both (client validates immediately, server validates for security)

**Decision 2: Error handling?**
- Show generic "Invalid email" (security)
- Show specific errors "Email already exists" (better UX but leaks info)
- Solution: Show specific errors in beta, switch to generic for production

### Components
1. LoginForm.tsx — React component with form state
2. validateEmail.ts — Validation helper
3. auth.ts — API calls
4. styles/LoginForm.css — Styling

### Dependencies
- react: ^18.0 (already installed)
- axios: ^1.7 (for API calls, needs install)

### Verification Strategy
1. Unit tests: Validate email format (50 test cases)
2. Integration test: Form submission with valid email
3. Integration test: Form submission with invalid email
4. Browser test: Manual testing of UX (agent will do this)
```

**Artifact 3: Walkthrough**

Created *after* implementation completes. Documents what was actually built:
- Screenshots showing features working
- Terminal output from test runs
- List of files created or modified
- Verification results (did it meet success criteria?)

Example Walkthrough for completed login feature:
```
## Walkthrough: Login Form Implementation Complete

### Files Created
- src/components/LoginForm.tsx (142 lines)
- src/utils/validateEmail.ts (35 lines)
- src/services/auth.ts (48 lines)
- src/styles/LoginForm.css (78 lines)
- __tests__/validateEmail.test.ts (52 lines)

### Verification Results

**Unit Tests: Email Validation**
```
PASS  src/utils/validateEmail.test.ts
  ✓ accepts valid email addresses (5ms)
  ✓ rejects invalid email format (3ms)
  ✓ rejects empty string (1ms)

Tests: 3 passed, 3 total
```

**Browser Test: Login Form**
[Screenshot 1: Initial form state]
[Screenshot 2: Valid email entered, no errors]
[Screenshot 3: Invalid email, error message displayed]
[Screenshot 4: Form submitted, loading state shown]
[Screenshot 5: Successful login, redirected to dashboard]

### Summary
All 4 tasks completed successfully. 5 files created. 3/3 unit test suites passing.
Walkthrough verified in browser: login with valid/invalid emails working as specified.
```

### Exercise 1: Understanding Ask Always Mode

**Setup**: You'll walk through a simple task using "Ask Always" mode to see the approval workflow.

**Task**: "Add a README.md file to this project that explains how to install and run it"

**Step 1: Review the Task List**

The agent generates:
```
## Task List: README.md Creation

### Task 1: README File Creation
- File: README.md (project root)
- Purpose: Explain project purpose, installation, and usage
- Verification: File exists with all required sections

### Task 2: Installation Instructions
- File: README.md (Installation section)
- Purpose: Step-by-step guide to install dependencies
- Verification: Instructions are platform-specific (Windows, macOS, Linux)

### Task 3: Usage Guide
- File: README.md (Usage section)
- Purpose: Examples of how to run the application
- Verification: Code examples are copy-paste ready
```

**Step 2: Review the Implementation Plan**

```
## Implementation Plan: README.md

### Research Findings
- Best practices: Include badges (build status, license), quick start, contribution guidelines
- Structure: Start with hook (what is this?), then installation, then features, then examples
- Tone: Beginner-friendly, assume reader has no context

### Architecture Decisions

**Decision 1: Level of detail?**
- Minimal (5-minute read)
- Comprehensive (20-minute read)
- Solution: Minimal for now, link to docs/ folder for comprehensive guide

**Decision 2: Code examples?**
- Working examples you can copy-paste
- High-level pseudocode
- Solution: Working examples with actual commands users can run

### Content Sections
1. Project description (2-3 sentences explaining purpose)
2. Quick start (1-2 minutes to get running)
3. Installation (detailed, platform-specific)
4. Usage examples (3-4 real use cases)
5. Contributing guidelines (link to CONTRIBUTING.md)
```

**Step 3: Approve or Request Changes**

The agent waits for your decision. You have options:

**Option A: Approve as-is**
```
You: "Looks good. Proceed with the implementation plan."
```

**Option B: Request changes**
```
You: "Good plan, but add these:
- Include a troubleshooting section for common errors
- Make the quick start 1 minute or less (cut unnecessary steps)
- Add an architecture diagram showing system components"
```

Agent updates the Implementation Plan with your feedback, then implements.

**Step 4: Review the Walkthrough**

Agent completes and generates:
```
## Walkthrough: README.md Created

### Files Created
- README.md (412 lines)

### Verification
- [ ] File exists: YES
- [ ] Has project description: YES
- [ ] Has installation instructions: YES (Windows, macOS, Linux)
- [ ] Has usage examples: YES (3 examples)
- [ ] Has troubleshooting: YES (5 common errors)
- [ ] Quick start < 1 minute: YES (estimated 45 seconds)

### Content Quality Review
- Installation: Tested on macOS (follows Linux steps, assumes brew installed)
- Code examples: All verified to run correctly
- Troubleshooting: Includes error message and solution for each case

Status: READY FOR MERGE
```

**Reflection**: What did you notice?

- The agent generated a plan *before* writing anything
- You had a chance to shape the direction *before* investment
- The Walkthrough proved what was actually built

This is the core of AAD: **approval gates catch misunderstandings early**.

### Exercise 2: Understanding Ask Sometimes Mode

**Setup**: A more complex task where the agent needs your input on architecture choices, but can auto-execute routine work.

**Task**: "Add error handling to all API calls across the project. We need consistent error messages and logging."

**Step 1: Review the Task List**

```
## Task List: Add Error Handling

### Task 1: Identify All API Calls
- Files: src/services/api.ts, src/services/auth.ts, src/components/DataFetch.tsx
- Purpose: Find all places making API requests
- Verification: 8 API calls identified across 3 files

### Task 2: Create Error Handler Utility
- File: src/utils/handleApiError.ts
- Purpose: Centralized error message formatting and logging
- Verification: Function accepts error object, returns user-friendly message

### Task 3: Apply Error Handling
- Files: src/services/api.ts (3 calls), src/services/auth.ts (2 calls), src/components/DataFetch.tsx (3 calls)
- Purpose: Wrap all API calls with error handling
- Verification: All 8 calls use the error handler utility

### Task 4: Test Error Scenarios
- File: __tests__/errorHandling.test.ts
- Purpose: Verify errors are caught and formatted correctly
- Verification: 8 test cases pass (one per API call)
```

**Step 2: Agent Asks for Architecture Decision**

The agent identifies that it needs your input on a design choice:

```
Agent: "I found 8 API calls across 3 files. I need to know HOW to add error handling.

Option A: try/catch blocks around each call
```javascript
try {
  const response = await api.post('/login', credentials);
} catch (error) {
  showError(formatError(error));
}
```

Option B: Decorator pattern (wraps all API calls automatically)
```javascript
const safeApiCall = (apiFunction) => async (...args) => {
  try {
    return await apiFunction(...args);
  } catch (error) {
    showError(formatError(error));
  }
};

// Usage:
const safeFetch = safeApiCall(api.get);
```

Option C: Promise.catch() chaining
```javascript
api.post('/login', credentials)
  .catch(error => showError(formatError(error)));
```

Which pattern should I use?

Recommendation: Option A (most explicit, easiest to understand and debug)"
```

**Step 3: You Decide**

```
You: "Use Option A. It's clearest for this codebase. But add a specific check
for network errors vs server errors. Network errors should say 'Check connection'
and server errors should show the actual error message."
```

**Step 4: Agent Auto-Executes**

```
Agent: "Understood. Proceeding with Option A + network vs server error detection.
Working on files now...

[Applies error handling to all 8 API calls]
[Creates error handler utility with network detection]
[Creates tests]
[Runs tests]
Status: COMPLETE"
```

**Reflection**: Notice what happened here?

- The agent did the analysis (identifying all API calls, comparing options)
- *You* made the architectural decision (which pattern + the network error handling detail)
- The agent auto-executed the routine work (applying the same pattern 8 times)

This is "Ask Sometimes" in action: agent handles heavy lifting, you make architecture decisions.

---

## Part 3: Feature 2 — Implementation Plan Artifacts and Approval Gates

### Why Plans Matter

Traditional coding: You write code, then discover your approach was wrong. You rewrite. Hours wasted.

Antigravity workflow: You review the *plan* before any code is written. Mistakes caught before investment.

This section shows you how to review plans effectively and what to look for.

### What a Good Implementation Plan Includes

A strong Implementation Plan answers these questions:

**1. Did the agent understand the task?**

Example plan start:
```
## Implementation Plan: User Authentication

### Intent
Add secure user authentication to allow new users to register and existing users to login.
Success means: Users can sign up with email/password, login, and stay logged in across sessions.
```

Ask yourself: Is this what you asked for? If not, send it back.

**2. What research did the agent do?**

Example:
```
### Research Findings
- Password hashing: bcrypt is industry standard (used by 70% of Node.js projects)
- Session management: JWT tokens preferred for stateless APIs
- Email validation: RFC 5322 regex accepts 99.5% of real-world emails
- Security consideration: OWASP recommends rate limiting on login attempts
```

Ask yourself: Does this research seem thorough? Are the recommendations grounded in real standards?

**3. What architectural decisions are made and why?**

Example:
```
### Architecture Decisions

**Decision 1: Password Storage**
- Option A: Hash with bcrypt (slow, secure)
- Option B: Hash with SHA-256 (fast, less secure)
- Decision: Bcrypt (security > speed for auth)

**Decision 2: Session Duration**
- Option A: 1-hour tokens (very secure, annoying for users)
- Option B: 7-day tokens (convenient, less secure)
- Option C: 1-hour access token + 30-day refresh token (hybrid)
- Decision: Option C (best balance of security and UX)
```

Ask yourself: Do the decisions match your project needs? For an MVP, maybe Option B is better than Option C (simpler is better when starting). Send feedback back.

**4. What components are being created or modified?**

Example:
```
### Components
1. src/routes/auth.ts — POST /register and POST /login endpoints
2. src/models/User.ts — User schema with password hashing
3. src/middleware/authenticate.ts — JWT verification middleware
4. src/types/auth.ts — TypeScript types for auth requests/responses
5. database/migrations/001_create_users_table.sql — User table schema
```

Ask yourself: Are these the right components? Is anything missing (e.g., password reset endpoint)?

**5. How will the agent verify it worked?**

Example:
```
### Verification Strategy
1. Unit tests: Hash password correctly (password → hash, same password always makes same hash)
2. Unit tests: JWT generation and verification
3. Integration test: Register new user → check user in database
4. Integration test: Login with correct password → returns token
5. Integration test: Login with wrong password → returns 401 error
6. Browser test: Manual signup and login flow
```

Ask yourself: Are these verification steps sufficient? Would you trust this feature based on these tests?

### Exercise 3: API Integration with Research Phase

**Scenario**: You're building a weather app. You decide: "Show the current weather in the sidebar." The agent needs to research an API, propose a plan, and wait for your approval.

**Step 1: Give the Request**

```
You: "Add a weather widget to the sidebar. Show current temperature, weather
condition (sunny/cloudy/rainy), and a small icon. Get data from a free weather API.

Success criteria:
- Widget updates every 10 minutes
- Shows user's current location (use browser geolocation)
- Handles errors gracefully (if API fails, show 'Weather unavailable')
- Cache API response for 10 minutes to avoid excessive calls"
```

**Step 2: Agent Researches and Proposes a Plan**

The agent spawns a background task to research APIs. While it researches, you can work on other things. When the plan is ready:

```
## Implementation Plan: Weather Widget

### Research Findings

**API Options Evaluated:**
1. OpenWeatherMap (free tier: 60 calls/minute, accurate)
2. WeatherAPI (free tier: 1M calls/month, good documentation)
3. Open-Meteo (100% free, no API key, good for learning)

**Chosen: OpenWeatherMap** (best balance of free tier + accuracy + usage)
- Free API key at openweathermap.org
- 60 calls/minute limit (enough for sidebar refresh every 10 minutes)
- Returns: temperature, condition, icon URL

**Browser Geolocation:**
- navigator.geolocation API (built into browser, no library needed)
- Prompts user for permission once
- Returns latitude/longitude

**Caching Strategy:**
- localStorage stores last API response + timestamp
- Check: if (now - lastFetch less than 10min) use cache, else fetch new data
- Simple and reliable for this use case

### Architecture Decisions

**Decision 1: Fetch location how?**
- Option A: Use browser geolocation (requires user permission)
- Option B: Use IP-based geolocation (no permission needed, less accurate)
- Option C: User enters location manually
- Decision: Option A (most accurate, user permission is standard)

**Decision 2: Handle location errors?**
- If user denies geolocation, what do we show?
- Option A: Show error message (sad, but honest)
- Option B: Default to a major city (San Francisco, London, etc.)
- Option C: Allow user to search for location manually
- Decision: Option A for MVP (simpler), Option C in future update

**Decision 3: Error display?**
- Option A: "Weather unavailable" when API fails
- Option B: Show last known weather (cached value)
- Option C: Show nothing (empty space)
- Decision: Option B (best UX - show cached data if available, "Weather unavailable" if nothing cached)

### Components

1. **src/components/WeatherWidget.tsx** — Main component
   - Uses geolocation to get user location
   - Fetches weather from OpenWeatherMap API
   - Handles caching logic
   - Displays temperature + condition icon

2. **src/utils/weatherApi.ts** — API wrapper
   - fetchWeather(latitude, longitude) function
   - Error handling for API failures
   - Type definitions for API response

3. **src/types/weather.ts** — TypeScript types
   - Weather response format
   - Error types

4. **.env** — Configuration (you provide API key)
   - REACT_APP_OPENWEATHER_API_KEY = [your key]

### Dependencies

- react: ^18.0 (already installed)
- No new npm packages needed (uses built-in fetch and geolocation)

### Verification Strategy

1. Unit test: Cache logic (mock API, verify cache is used when fresh)
2. Unit test: Error handling (mock API returning error, verify fallback works)
3. Integration test: Widget renders with real API call
4. Browser test: Manual check - widget appears, shows correct temperature
5. Browser test: Wait 10 minutes, verify cache prevents extra API calls
```

**Step 3: Review and Provide Feedback**

You review the plan. You notice something:

```
You: "Good research and architecture. Two questions:

1. What's the free tier limit for OpenWeatherMap? You mentioned 60 calls/minute,
but our sidebar widget refreshes every 10 minutes AND we have caching. Can we
verify this doesn't hit any daily limits?

2. For the 'weather unavailable' error, show it for 30 seconds then retry.
Users with flaky internet shouldn't see errors forever if connection comes back.

Otherwise, approved. Proceed with implementation."
```

**Step 4: Agent Updates Plan and Implements**

```
Agent: "Good catches. Updating plan:

1. Verified: OpenWeatherMap free tier is 60 calls/minute with 1M calls/month limit.
With caching (refresh every 10 min), we use ~6 calls/hour = ~144/day. Well under
1M/month limit. No limit hit possible with this setup.

2. Added error retry logic: Show error for 30 seconds, then retry automatically.

Proceeding with implementation..."

[Implements weather widget]
[Creates tests]
[Runs browser test with real API]
```

**Reflection**: This is specification-first development in action.

- The agent proposed a plan based on research
- You reviewed the plan *before* writing code
- You caught potential issues (API limits, error UX) early
- Implementation followed the approved plan

If the agent had just started coding, it might have chosen WeatherAPI or missed the retry logic. This approval gate prevented those mistakes.

---

## Part 4: Feature 3 — Parallel Task Execution

### Why Parallelism Matters

Traditional workflow:
1. You: "Build an API for user registration"
2. Agent: [Builds API, 20 minutes]
3. You wait for agent to finish
4. You: "Build the UI for registration"
5. Agent: [Builds UI, 15 minutes]
6. You wait for agent to finish

Total time: 35 minutes

Antigravity workflow:
1. You: "Build API for registration" (agent works in background)
2. Agent: [Working...]
3. Meanwhile, you: "Build UI for registration" (second agent or parallel task)
4. Agent 2: [Working...]
5. Both complete roughly simultaneously

Total time: ~20 minutes (faster because parallel)

### How to Use Background Tasks

When you spawn a task, you can mark it to run in background:

```
You: "I need two things:

FOREGROUND TASK (I'm working on this now):
Create login form UI with email/password fields,
form validation, and error display.

BACKGROUND TASK (You work on this while I build UI):
Research JWT authentication best practices and
create an implementation plan for stateless auth.
By the time I finish the UI, your plan should be ready."

Agent A: [Works on foreground, you review in real-time]
Agent B: [Works on background, completes independently]
```

### Exercise 4: Parallel Workflow in Action

**Scenario**: You're building a web app. You need both API endpoints and UI, but you've only got one agent. Solution: Work in parallel.

**Setup**: The recipe finder app needs two things before you can integrate them:
- **UI**: Search form + results display
- **API research**: What data structure should API return?

**Step 1: Spawn Both Tasks**

```
You: "I need to build a recipe search app. Let's work in parallel:

FOREGROUND: I'll build the search UI component
- Input field for ingredient search
- Results display showing recipe name, image, and ingredients
- Loading spinner while fetching

BACKGROUND: Research and propose API response format
- What data does the recipe API need to return?
- What fields should we fetch (image, servings, difficulty, etc.)?
- Propose a TypeScript interface for the response
- Create an implementation plan for API integration

Start the background task now. I'll finish UI in ~20 minutes.
By then, your plan should be ready."
```

**Step 2: You Work on Foreground Task**

```
You: [Build SearchForm.tsx]
```
```javascript
export function SearchForm() {
  const [query, setQuery] = useState("");
  const [results, setResults] = useState([]);
  const [loading, setLoading] = useState(false);

  const handleSearch = async (e) => {
    e.preventDefault();
    setLoading(true);
    // API call will go here once backend is ready
    setLoading(false);
  };

  return (
    <form onSubmit={handleSearch}>
      <input
        value={query}
        onChange={(e) => setQuery(e.target.value)}
        placeholder="Search by ingredient (e.g., chicken)"
      />
      <button type="submit">Search</button>

      {loading && <div>Searching...</div>}

      {results.map(recipe => (
        <RecipeCard key={recipe.id} recipe={recipe} />
      ))}
    </form>
  );
}
```

**Step 3: Agent Works on Background Task**

Meanwhile, the agent is researching:

```
Agent: [Searching for recipe API documentation]
[Evaluating Spoonacular, RecipeSearch, TheMealDB APIs]
[Researching response formats]
[Creating implementation plan...]

Agent: "Background task complete. Check the Implementation Plan artifact."
```

**Step 4: Agent Returns with Implementation Plan**

```
## Implementation Plan: Recipe API Integration

### Research Findings
- Recipe APIs available: Spoonacular (best), RecipeSearch, TheMealDB
- Chosen: Spoonacular (free tier: 100 calls/day, extensive data)
- Response includes: recipe name, image, ingredients, instructions, nutrition

### Proposed API Response Format
```typescript
interface Recipe {
  id: number;
  title: string;
  image: string;
  usedIngredients: {
    id: number;
    name: string;
    image: string;
  }[];
  missedIngredients: {
    id: number;
    name: string;
    image: string;
  }[];
  nutrition: {
    calories: number;
    protein: string;
    carbs: string;
    fat: string;
  };
}
```

### API Endpoint
```
GET https://api.spoonacular.com/recipes/findByIngredients
?ingredients=chicken&number=10&ranking=2
&apiKey=[YOUR_KEY]
```

### Implementation Plan
1. Create API wrapper in src/services/recipeApi.ts
2. Handle Spoonacular response mapping to our interface
3. Add error handling and caching (cache for 1 hour)
4. Create types in src/types/recipe.ts
```

**Step 5: Review and Integrate**

```
You: [Finish UI, then review Implementation Plan]

You: "Plan looks good! I notice the API includes
'usedIngredients' (found) and 'missedIngredients' (not found).
In the UI, let's show BOTH, but highlight which ingredients were found.
Update the implementation plan to show this in the UI."

Agent: [Updates plan and implements API integration]

You: [Integrate API response with SearchForm]
```

**Result**: While you built UI, agent researched and planned backend. When you finished UI, backend was ready to integrate. Much faster than sequential work.

**Reflection**:
- Foreground task: You built UI (20 minutes)
- Background task: Agent researched API (also ~20 minutes, happening simultaneously)
- Total time: ~20 minutes instead of 40 minutes

This is the power of parallelism. Both you and agent work simultaneously on their respective concerns.

---

## Part 5: Feature 4 — Integrated Browser Testing

### What Browser Integration Means

Antigravity includes a full Chrome browser that agents can control. When an agent tests your code, it:
1. Starts a local server (e.g., localhost:3000)
2. Opens Chrome
3. Navigates to the app
4. Clicks, types, and interacts with the UI
5. Captures screenshots
6. Records test results

The browser window shows a **blue border when the agent is controlling it**. This tells you: "The agent is clicking and typing right now. Watch if you want."

### Why Automated Testing Matters

Testing manually: "Click here, check that, try that error case..." Tedious and easy to miss edge cases.

Automated testing: Agent clicks 50 times, catches every edge case, produces evidence (screenshots).

### Exercise 5: Automated Browser Testing Workflow

**Scenario**: You built a login form. You want the agent to test it thoroughly. Instead of you manually testing 20 different scenarios, the agent does it automatically and reports findings.

**Step 1: Define What to Test**

```
You: "Test the login form. Here are the test cases:

VALID CREDENTIALS:
1. Login with valid email and correct password
   Expected: Redirect to dashboard, show success message

INVALID CREDENTIALS:
2. Login with valid email and wrong password
   Expected: Show 'Incorrect password' error, stay on login page
3. Login with non-existent email
   Expected: Show 'Email not found' error, stay on login page
4. Login with empty email
   Expected: Show 'Email required' error, form doesn't submit
5. Login with empty password
   Expected: Show 'Password required' error, form doesn't submit

EDGE CASES:
6. Click submit multiple times rapidly
   Expected: Only one request sent (prevent double-submit)
7. Enter very long email (100 characters)
   Expected: Handle gracefully (truncate or error message)
8. Enter password with special characters (!, @, #, $)
   Expected: Accept and login works"
```

**Step 2: Agent Runs Tests Automatically**

The agent opens the browser (blue border appears) and:

```
Agent: "Starting automated browser tests..."

[Browser opens, agent navigates to localhost:3000/login]

Test 1: Valid login
[Types: alice@example.com in email field]
[Types: CorrectPassword123 in password field]
[Clicks: Submit button]
[Waits for navigation]
[Captures screenshot showing dashboard]
Result: ✓ PASS (redirected to dashboard)

Test 2: Wrong password
[Clears form, types: alice@example.com]
[Types: WrongPassword]
[Clicks: Submit button]
[Waits for error message]
[Captures screenshot showing error]
Result: ✓ PASS (error message displayed)

Test 3: Non-existent email
[Clears form, types: nonexistent@example.com]
[Types: SomePassword123]
[Clicks: Submit button]
[Waits for error]
[Captures screenshot]
Result: ✓ PASS (error message displayed)

[Continues with tests 4-8...]

Agent: "Testing complete. Generating Walkthrough artifact..."
```

**Step 3: Review the Walkthrough**

Agent creates a Walkthrough artifact with:
- Screenshots from each test
- Test results summary
- Any errors discovered

Example Walkthrough:

```
## Walkthrough: Login Form Testing

### Test Summary
- Total tests: 8
- Passed: 7
- Failed: 1

### Test Results

#### Test 1: Valid Login ✓ PASS
Screenshot: [shows dashboard with "Welcome alice@example.com"]
Time: 2.5 seconds

#### Test 2: Wrong Password ✓ PASS
Screenshot: [shows error "Incorrect password"]
Time: 1.8 seconds

#### Test 3: Non-existent Email ✓ PASS
Screenshot: [shows error "Email not found"]
Time: 1.7 seconds

#### Test 4: Empty Email ✓ PASS
Screenshot: [shows form with error "Email required"]
Time: 0.3 seconds

#### Test 5: Empty Password ✓ PASS
Screenshot: [shows form with error "Password required"]
Time: 0.3 seconds

#### Test 6: Double-Submit Prevention ✓ PASS
Note: Clicked submit button rapidly 5 times
Network calls: 1 (not 5)
Screenshot: [shows successful login after rapid clicks]
Time: 1.9 seconds

#### Test 7: Long Email ✗ FAIL
Input: [100-character email: aaaaaaaa...aaa@example.com]
Screenshot: [shows form with error "Email too long"]
Error: Form rejected, user sees error (acceptable but undocumented)
Note: Consider adding client-side validation hint

#### Test 8: Special Characters in Password ✓ PASS
Input: P@ssw0rd!#$%
Screenshot: [successful login]
Time: 2.0 seconds

### Summary
7/8 tests passed. One failure in Test 7 (long email handling) is acceptable
for MVP (form rejects gracefully with error message).
```

**Step 4: Review and Take Action**

```
You: [Review Walkthrough]

You: "Good coverage! Test 7 failure is fine for MVP.
Let's add a validation hint on the form that says
'Email must be less than 100 characters' to help users.

Request: Update the form to show this hint."

Agent: [Updates form with validation hint]
```

**Reflection**: Notice what happened:

- You defined test cases (5 minutes)
- Agent ran all 8 tests automatically (2 minutes)
- You got screenshots of all results
- You caught an edge case (long email) without testing it yourself
- You decided if it's acceptable (yes, for MVP)

This would take you 30+ minutes to test manually and you'd probably miss edge cases. The agent caught everything in 2 minutes.

---

## Part 6: Feature 5 — Walkthrough Artifacts

### What Walkthroughs Are

A Walkthrough is the agent's final report after completing a task. It answers the question: **"Did you do what I asked?"**

Walkthroughs include:
- **What was created/modified** (file list)
- **Verification results** (test output, screenshots)
- **Decisions made** during implementation
- **Issues discovered** (and how they were handled)

### Reading a Walkthrough

A good Walkthrough tells the complete story of what was built, without you having to check files or run tests yourself.

### Exercise 6: Reviewing a Completed Walkthrough

**Scenario**: You asked the agent to "Add a dark mode toggle to the UI." The agent completed the work and generated a Walkthrough. You need to review it and decide: approve or request changes?

**Walkthrough Content**:

```
## Walkthrough: Dark Mode Toggle Implementation

### Intent
Add a toggle button to the UI header that switches between light and dark modes.
Success criteria:
- Toggle button visible and clickable
- Clicking toggle switches all colors to dark theme
- User preference saved to localStorage
- Preference persists across page reload
- Works on all pages (consistent across app)

### Files Created/Modified
- src/components/DarkModeToggle.tsx (NEW, 45 lines)
- src/hooks/useDarkMode.ts (NEW, 32 lines)
- src/styles/darkMode.css (NEW, 110 lines)
- src/App.tsx (MODIFIED, added DarkModeToggle to header)
- src/index.css (MODIFIED, imported darkMode.css)

### Implementation Approach
1. Created custom hook (useDarkMode) to manage dark mode state + localStorage
2. Created toggle component with sun/moon icons
3. CSS uses CSS custom properties (--bg-color, --text-color) for theming
4. Applied dark mode styles across all components

### Verification: Feature Tests
```
PASS  src/hooks/useDarkMode.test.ts
  ✓ initializes with light mode (1ms)
  ✓ toggle() switches to dark mode (1ms)
  ✓ saves preference to localStorage (2ms)
  ✓ loads preference from localStorage on mount (1ms)

Tests: 4 passed, 4 total
```

### Verification: Visual Tests
Screenshot 1 - Light Mode:
[Shows app with light background, dark text, sun icon in toggle]

Screenshot 2 - Dark Mode:
[Shows same app with dark background, light text, moon icon in toggle]

Screenshot 3 - Toggle Button Highlight:
[Shows toggle button location in header, highlighted]

### Verification: Browser Tests
```
Test 1: Click toggle in light mode
- Expected: Colors switch to dark
- Result: ✓ PASS (colors changed immediately)
- Time: 0.2 seconds

Test 2: Reload page in dark mode
- Expected: Dark mode persists
- Result: ✓ PASS (page loads in dark mode)
- Time: 1.8 seconds

Test 3: Toggle multiple times rapidly
- Expected: Each toggle switches correctly
- Result: ✓ PASS (5 toggles executed, all correct)
- Time: 1.0 seconds

Test 4: Test on different pages
- Expected: Dark mode works on all pages
- Result: ✓ PASS (tested on Home, About, Settings pages)
- Time: 3.2 seconds
```

### Verification: Code Review

**Style Consistency**:
- Uses existing CSS variable pattern ✓
- Naming matches project conventions ✓
- No hardcoded colors (all in CSS variables) ✓

**Accessibility**:
- Toggle button has aria-label="Toggle dark mode" ✓
- Sun/moon icons have role="presentation" ✓
- Keyboard accessible (can tab to button and press Enter) ✓

**Performance**:
- CSS variables update all colors in 0ms (no layout recalculation) ✓
- localStorage write on every toggle (acceptable, less than 1ms) ✓

### Summary
All 4 feature tests passed. All browser tests passed.
Code review checks out.
Recommendation: READY TO MERGE

### Files Ready to Commit
- src/components/DarkModeToggle.tsx
- src/hooks/useDarkMode.ts
- src/styles/darkMode.css
- (modifications to src/App.tsx and src/index.css)
```

**Step 1: Review the Walkthrough**

You read through it systematically:

1. **Intent**: Does the walkthrough say what you asked for? ✓ Yes
2. **Files**: Do the file changes make sense? ✓ Yes (component, hook, styles)
3. **Tests**: Do the tests prove it works? ✓ Yes (4 unit tests + 4 browser tests)
4. **Screenshots**: Do the screenshots show what you wanted? ✓ Yes (light and dark modes visible)
5. **Recommendation**: Is the agent saying it's ready? ✓ Yes

**Step 2: Decide: Approve or Request Changes?**

Option A: Approve
```
You: "Looks great! Merge this. Nice touches on accessibility and performance notes."
```

Option B: Request changes
```
You: "Good work overall, but I noticed a couple things:

1. In Screenshot 1 (light mode), the text is hard to read on light background.
   Make it slightly darker.

2. Add one more test: What happens if browser localStorage is disabled?
   Does dark mode still work (just not persist)?

Make these updates and re-test."

Agent: [Updates colors and adds localStorage error handling test]
Agent: [Re-tests and generates new Walkthrough]
```

This is the approval gate. You review evidence (screenshots, test results) and decide if it's good enough.

**Reflection**: A good Walkthrough gives you confidence without reading code. You can see:
- What was built (file list)
- That it works (test results + screenshots)
- That it's safe (code review checks)

You never have to wonder: "Did this really work?" The screenshots prove it.

---

## Part 7: Mini-Project — Recipe Finder App

### Project Overview

You're going to build a complete web application using pure artifact-driven workflow. **No manual coding.** You will:

1. Write clear project requirements
2. Let agent research and propose plans
3. Review and approve plans
4. Let agent implement
5. Review Walkthrough and approve

This is AI-native development at work.

### Project Requirements

**Recipe Finder**: A web app where users search recipes by ingredient.

**Features**:
- Search for recipes by ingredient (e.g., "chicken" → recipes with chicken)
- Display results with recipe name, image, and ingredient list
- Filter results by cooking time or difficulty (optional)
- Save favorite recipes to browser localStorage
- Show number of recipes found

**Tech Stack**:
- Frontend: React with TypeScript
- Styling: CSS (no frameworks initially)
- API: Spoonacular (free tier, 100 calls/day)
- Deployment: Vercel (easy free hosting)

**Success Criteria** (what "done" means):
- User can search for recipes by ingredient
- Results display 10 recipes with names and images
- User can save/unsave favorites
- Saved favorites persist across page reloads
- App handles errors gracefully (shows message if search fails)
- Load time < 2 seconds for typical search

### Artifact-Driven Workflow: 10 Steps

#### Step 1: Create Workspace

```
You: "Create a new Next.js project workspace called 'recipe-finder'"

Agent: [Creates Next.js project with TypeScript]
```

#### Step 2: Give Full Requirements

```
You: "Build the Recipe Finder app. Here are complete requirements:

## Application Intent
A web app for searching recipes by ingredient. Users should be able to discover
recipes they can make with ingredients they have.

## User Stories
1. As a user, I want to search for recipes by ingredient (e.g., 'chicken')
2. As a user, I want to see 10 search results with recipe name and image
3. As a user, I want to save recipes to my favorites
4. As a user, I want my favorites to persist when I close and reopen the app
5. As a user, I want clear error messages if something goes wrong

## Technical Constraints
- Must use Spoonacular API (free tier, 100 calls/day)
- Frontend: React with TypeScript
- Must handle rate limiting gracefully
- Search must complete in < 2 seconds

## Non-Goals (defer to v2)
- Social sharing
- Recipe ratings
- Meal planning
- Multi-user accounts

## Success Criteria
- Search works and returns recipes
- Favorites persist
- No errors shown to user (all errors handled)
- Loads in < 2 seconds
- Code has proper TypeScript types

Spawn research + implementation plan. I'll review before you code."
```

#### Step 3: Review Task List

Agent generates Task List artifact:

```
## Task List: Recipe Finder

### Task 1: Project Setup
- Initialize Next.js project with TypeScript
- Install dependencies (axios, zustand or context for state)
- Verification: Project runs on localhost:3000

### Task 2: Spoonacular API Integration
- Create API wrapper (fetchRecipes function)
- Handle rate limiting (cache responses)
- Verify: Can fetch recipes from API
- Files: src/services/recipeApi.ts

### Task 3: Search Page UI
- Create SearchForm component
- Results grid showing recipe cards
- Verification: Form renders, results display
- Files: src/pages/search.tsx, src/components/RecipeCard.tsx

### Task 4: Favorites Feature
- Create favorites state management
- Add save/unsave functionality
- Persist to localStorage
- Verification: Can save and load favorites
- Files: src/hooks/useFavorites.ts, src/components/FavoriteButton.tsx

### Task 5: Error Handling
- Handle API errors (network issues)
- Handle rate limiting (show message if > 100 calls/day)
- Handle no results (show empty state)
- Files: src/components/ErrorMessage.tsx, src/components/EmptyState.tsx

### Task 6: Testing & Verification
- Write unit tests for API wrapper
- Write integration tests for favorites persistence
- Browser tests: Full user flow (search → save → reload)
- Files: __tests__/recipeApi.test.ts, __tests__/integration.test.ts

### Task 7: Deployment Prep
- Build for production
- Test production build locally
- Create deployment config for Vercel
- Verification: Builds without errors
```

**You review**: "Looks good. Proceed to Implementation Plan."

#### Step 4: Agent Researches (Background Task)

While you work on other things, agent researches:
- Spoonacular API documentation
- Best practices for caching API responses
- Rate limiting strategies
- localStorage patterns in React

Agent generates Implementation Plan.

#### Step 5: Review Implementation Plan

Agent returns detailed plan:

```
## Implementation Plan: Recipe Finder

### Research Findings
- Spoonacular free tier: 100 calls/day, findByIngredients endpoint
- Response includes: recipe name, image URL, ingredients list, nutrition
- Rate limiting: If you hit 100 calls, API returns 402 error
- Best practice: Cache responses for 1 hour per ingredient (avoid wasting calls)

### Architecture Decisions

**Decision 1: State Management**
- Option A: Context API (no dependencies, simple)
- Option B: Zustand (small library, very simple)
- Option C: Redux (overkill for this app)
- Recommendation: Context API (minimal dependencies, good for learning)

**Decision 2: Caching Strategy**
- Cache by ingredient: If user searches 'chicken' twice, use cached results
- Cache duration: 1 hour (balance between freshness and API quota)
- Implementation: Store in window.localStorage (survives page reloads)

**Decision 3: Error Messaging**
- Show user-friendly errors: "No recipes found" instead of "API returned 404"
- Distinguish types: "Search too broad" vs "Network error" vs "API limit reached"

**Decision 4: Recipe Display**
- Show: name, image, ingredient list, number of ingredients
- Don't show: full instructions (keep initial display lightweight)
- Full instructions available on click (future enhancement)

### Component Structure

```
src/
  pages/
    search.tsx (main page)
  components/
    SearchForm.tsx (input + submit)
    RecipeCard.tsx (individual recipe display)
    FavoriteButton.tsx (save/unsave button)
    ErrorMessage.tsx (error display)
    EmptyState.tsx (no results display)
  services/
    recipeApi.ts (Spoonacular API wrapper)
  hooks/
    useFavorites.ts (localStorage + state for favorites)
    useRecipeSearch.ts (search logic + caching)
  types/
    recipe.ts (TypeScript interfaces)
```

### API Integration Details

```
Endpoint: https://api.spoonacular.com/recipes/findByIngredients
Method: GET
Parameters:
  - ingredients: string (e.g., "chicken")
  - number: 10 (how many results)
  - ranking: 2 (ranking method: 2 = maximize used ingredients)
  - apiKey: [user's key]

Response format:
[
  {
    id: 123,
    title: "Chicken Stir Fry",
    image: "https://...",
    usedIngredients: [{ name: "chicken", ... }],
    missedIngredients: [{ name: "soy sauce", ... }],
  },
  ...
]
```

### Caching Implementation

- Key: `recipes_${ingredient}` (e.g., "recipes_chicken")
- Value: `{ results: [...], timestamp: 1234567890 }`
- Check: if (now - timestamp less than 1 hour) use cache, else fetch

### Verification Strategy

1. Unit tests: API wrapper (mock API, test caching logic)
2. Unit tests: useFavorites hook (localStorage persistence)
3. Integration tests: Search → display results
4. Integration tests: Click favorite → saved to localStorage
5. Browser tests: Full user flow (search → save → reload → favorites still there)
6. Edge case tests: Empty search, no results, API error
```

**You review and approve**: "Plan looks solid. One request: When user first opens the app, show 3-4 popular recipes (don't make them search first). Add that to the plan."

**Agent updates plan** and proceeds to implementation.

#### Step 6: Agent Implements (Background Task)

Agent works independently:
- Creates all components
- Implements API integration
- Adds error handling
- Writes tests
- Tests in browser

Agent creates Implementation Plan Walkthrough as it completes each task.

#### Step 7: Agent Opens Browser for Automated Testing

Blue border appears on browser. Agent is testing:

```
Test 1: Load homepage
[Loads localhost:3000]
[Screenshots showing 3 popular recipes displayed]
Result: ✓ PASS

Test 2: Search for recipes
[Types 'chicken' in search box]
[Clicks search]
[Waits for results]
[Screenshots showing 10 recipes with names and images]
Result: ✓ PASS

Test 3: Save favorite
[Scrolls to second recipe]
[Clicks heart icon]
[Screenshots showing filled heart icon]
Result: ✓ PASS

Test 4: Reload page
[Presses F5 to reload]
[Waits for page to load]
[Screenshots showing saved recipe still has filled heart]
Result: ✓ PASS

Test 5: Error handling
[Types very generic search: 'food']
[Expects many results, plans to test pagination - deferred to v2]
Result: ✓ PASS

[More tests...]
```

#### Step 8: Agent Generates Walkthrough

```
## Walkthrough: Recipe Finder Implementation

### Intent
Build a web app for searching recipes by ingredient, with favorite saving.

### Files Created
- src/pages/search.tsx (178 lines)
- src/components/RecipeCard.tsx (64 lines)
- src/components/SearchForm.tsx (82 lines)
- src/components/FavoriteButton.tsx (35 lines)
- src/components/ErrorMessage.tsx (28 lines)
- src/components/EmptyState.tsx (22 lines)
- src/services/recipeApi.ts (71 lines)
- src/hooks/useFavorites.ts (48 lines)
- src/hooks/useRecipeSearch.ts (92 lines)
- src/types/recipe.ts (34 lines)
- __tests__/recipeApi.test.ts (85 lines)
- __tests__/useFavorites.test.ts (62 lines)

### Verification: Unit Tests
```
PASS  __tests__/recipeApi.test.ts
  ✓ fetches recipes from API (45ms)
  ✓ caches results for 1 hour (2ms)
  ✓ returns cached results on second call (1ms)
  ✓ handles API errors gracefully (3ms)

PASS  __tests__/useFavorites.test.ts
  ✓ saves favorite to localStorage (2ms)
  ✓ loads favorites from localStorage (1ms)
  ✓ removes favorite on unsave (1ms)
  ✓ persists across render cycles (2ms)

Tests: 8 passed, 8 total
```

### Verification: Browser Tests

Screenshot 1 - Homepage with popular recipes:
[Shows 3 featured recipes at top]

Screenshot 2 - Search results (query: 'chicken'):
[Shows 10 recipes with names, images, ingredients]

Screenshot 3 - Save to favorites:
[Shows heart icon filled in, indicating saved]

Screenshot 4 - Reload page:
[Page reloaded, saved recipe still shows filled heart]

Screenshot 5 - Error handling:
[Shows "No recipes found" message when search returns 0 results]

### Browser Test Results
```
Test 1: Homepage loads ✓ PASS
Test 2: Search returns recipes ✓ PASS (10 recipes displayed)
Test 3: Save recipe ✓ PASS (heart icon fills)
Test 4: Reload persists favorites ✓ PASS
Test 5: No results error ✓ PASS (message displayed)
Test 6: API error handling ✓ PASS (shows error message)
Test 7: Search another ingredient ✓ PASS (new results displayed)
Test 8: Remove favorite ✓ PASS (heart icon empties)

Total: 8/8 tests passed
```

### Code Quality Review
- TypeScript: All components properly typed ✓
- Error handling: All edge cases handled ✓
- Performance: Load time measured at 1.2 seconds ✓
- Accessibility: Semantic HTML, aria labels ✓

### Summary
All tests passed. Ready for deployment or further customization.

### Next Steps
To deploy to Vercel:
1. Push to GitHub
2. Connect repo to Vercel
3. Set environment variable: VITE_SPOONACULAR_API_KEY
4. Deploy

To add to project:
- Review screenshots
- Approve for deployment or request changes
```

#### Step 9: Review and Approve Walkthrough

```
You: [Review Walkthrough]
Screenshots: All look good
Tests: All passed
Code review: No issues noted

You: "Looks great! This is exactly what I asked for.
Approved for deployment. Deploy to Vercel."

Agent: [Pushes to GitHub, deploys to Vercel, generates deployment link]
```

#### Step 10: Application Complete

Walkthrough shows:
- Screenshots proving it works
- Test results proving it's tested
- Deployment link for accessing the live app

You now have a complete application built without writing any code yourself. You wrote specifications and reviewed plans. The agent handled implementation, testing, and deployment.

### What You Learned from This Mini-Project

**The Artifact-Driven Workflow**:
1. Clear requirements (your job)
2. Agent proposes plan (agent's job)
3. You review and approve (your job)
4. Agent implements, tests, deploys (agent's job)
5. You review evidence (screenshots, tests) (your job)

**Key insight**: You never have to wonder "did this work?" because the Walkthrough includes proof (screenshots, test results).

**Time saved**: Building this app would take a developer 8-12 hours manually. With artifact-driven workflow, the planning took 1 hour and agent implementation was automatic. Total elapsed time: 2-3 hours (including your review time).

---

## Part 8: Comparison Framework — Antigravity vs Cursor vs Zed

### Three Approaches to AI-Native Development

You've now experienced all three IDEs. Let's compare them systematically.

| Aspect | Zed | Cursor | Antigravity |
|--------|-----|--------|-------------|
| **AI Integration** | Inline assistant | Autonomous agent | Agent control plane |
| **Code Review** | You type everything | Diff-based review | Artifact-based review |
| **Approval Gates** | None (you decide when) | Diffs shown after generation | Plans approved before generation |
| **Autonomy Levels** | Limited (assistant) | 1 level (agent mode) | 3 modes (Ask Always/Sometimes/Full Auto) |
| **Testing** | Manual | Manual | Automated in integrated browser |
| **Parallelism** | No (single interface) | Limited (one diff at a time) | Yes (agents work in background) |
| **Learning Curve** | Shallow (just prompts) | Medium (diffs, agent mode) | Steeper (artifacts, planning) |
| **Best For** | Quick questions, inline help | Code generation with review | Full project orchestration |

### Choosing the Right IDE

**Choose Zed when**:
- You want quick answers (syntax help, debugging questions)
- You're writing most code yourself (AI assists)
- You need high performance (Zed is fastest)
- Simple projects (under 5 files)

**Choose Cursor when**:
- You want autonomous code generation
- You like reviewing changes in diffs
- You want single-agent assistance
- Medium projects (5-20 files)

**Choose Antigravity when**:
- You want to focus on specifications (not code)
- You want multiple agents working in parallel
- You want automated testing in browser
- Large projects (20+ files)
- You trust agent autonomy once plan is approved

### Practical Scenarios

**Scenario 1: Small bug fix (5 minutes)**
Best: Zed
- Ask: "Why is this function returning undefined?"
- Zed gives you the answer inline
- You fix it yourself

**Scenario 2: Generate a component (20 minutes)**
Best: Cursor
- Ask: "Create a date picker component"
- Cursor generates code with diff
- You review and approve diffs
- You test manually

**Scenario 3: Build entire feature (2 hours)**
Best: Antigravity
- Write specification: "Add user authentication with login form, registration, JWT tokens"
- Agent proposes plan
- You approve plan
- Agent implements, tests, deploys
- You review Walkthrough with screenshots

---

## Part 9: Practice and Self-Assessment

### Checklist: Can You Do These Things?

Test your understanding by attempting these tasks:

**Understanding Autonomy Modes**:
- [ ] Explain when to use "Ask Always" mode (when trust is low or requirements vague)
- [ ] Explain when to use "Ask Sometimes" mode (well-defined tasks with design choices)
- [ ] Explain when to use "Full Auto" mode (clear success criteria, high trust)

**Reading Artifacts**:
- [ ] Can you review a Task List and identify if it's complete?
- [ ] Can you review an Implementation Plan and spot potential issues?
- [ ] Can you review a Walkthrough and decide if quality is acceptable?

**Parallel Workflows**:
- [ ] Can you decompose a project into foreground and background tasks?
- [ ] Can you structure a request so agent works while you work?

**Browser Testing**:
- [ ] Can you define comprehensive test cases for a feature?
- [ ] Can you interpret browser test results and screenshots?

**Artifact-Driven Development**:
- [ ] Can you write clear project requirements in spec format?
- [ ] Can you review and approve agent plans before implementation?
- [ ] Can you use Walkthroughs to verify work without reading code?

### Challenge: Build Something

Here's a challenge to practice all these skills:

**Project: Todo App with API Sync**

Requirements:
- Users can create, read, update, delete todos
- Todos sync with a backend API
- Todos persist even if API is temporarily down (offline-first)
- Shows last sync time
- Has dark mode toggle

Create a specification for this app, then:

1. **Spawn an agent** to propose a Task List
2. **Review the Task List** and request changes if needed
3. **Review the Implementation Plan** before agent codes
4. **Let agent implement** while you review in parallel
5. **Review the Walkthrough** and approve if quality is acceptable

This full workflow uses everything from Lesson 7:
- Autonomy modes (choose which for each task)
- Artifact review (Task List, Plan, Walkthrough)
- Parallel work (agent works while you do other things)
- Browser testing (agent tests your app)
- Artifact approval (you decide when to approve each stage)

---

## Part 10: What You Learned This Lesson

By completing this lesson, you've learned to:

1. **Use three autonomy modes** — Configure agents to ask always, sometimes, or decide autonomously based on task complexity and risk level

2. **Review implementation plans before coding** — Catch architecture mistakes early by reading plans and providing feedback before agents write code

3. **Work in parallel** — Decompose projects so agents work on background tasks while you focus on other concerns, reducing total project time

4. **Interpret automated test results** — Understand how agents use integrated browser for testing and evaluate if coverage is sufficient

5. **Navigate artifact-driven workflows** — Go from specification through Task List → Implementation Plan → Execution → Walkthrough, approving at each stage before proceeding

6. **Make ide selection decisions** — Choose Zed, Cursor, or Antigravity based on project scope, team size, and autonomy needs

These skills embody the core thesis of this course: **In AI-native development, clear specifications and smart agent coordination drive productivity more than manual coding speed.**

---

## Try With AI

### Autonomous Experiment: Spec-First Project

**Setup**: Open Antigravity IDE and access the recipe finder specification you reviewed in Part 7.

**Prompts to Explore**:

**Prompt 1: Task List Proposal**
```
Review this specification for the Recipe Finder app and generate a Task List.
[paste specification from Part 7]

Create a Task List artifact showing all tasks needed to build this app.
Each task should have:
- What's being created
- Why it's needed
- How to verify it's done

Respond only with the Task List artifact (no explanations).
```

**Prompt 2: Autonomy Decision**
```
For the Recipe Finder app, I'm going to:
- Use "Ask Always" mode for the Spoonacular API integration (I need to verify rate limiting strategy)
- Use "Ask Sometimes" mode for component creation (auto-create, ask about styling)
- Use "Full Auto" mode for testing (run all browser tests automatically)

For EACH of these, explain why that autonomy level is appropriate.
```

**Prompt 3: Plan Review Feedback**
```
Review this Implementation Plan for recipe searching. I want to add one requirement:

"The search results should show how many of the recipe's ingredients the user already has
(e.g., 'You have 3 of 5 ingredients')."

How does this change the Implementation Plan? What components need updating?
```

**Prompt 4: Walkthrough Interpretation**
```
I received a Walkthrough from an agent showing:
- 8/8 browser tests passed
- 2 TypeScript compilation warnings (unused variable in utils/cache.ts)
- Load time 1.3 seconds
- One screenshot showing an error message: "Too many requests (API limit reached)"

Is this ready to approve? What should I ask the agent to fix before approval?
```

**Expected Outcomes**:
- Prompt 1: Understand how agents structure work into tasks
- Prompt 2: Reasoning about when to trust agent autonomy
- Prompt 3: See how requirements changes ripple through architecture
- Prompt 4: Practice approving or rejecting agent work based on evidence

**Safety Note**: When using real agents, verify that Spoonacular API responses are authentic before deploying to production. Always test error cases (what happens when API limit is hit?) manually before trusting automated testing.

---

## Next Lesson Preview

**Lesson 8: Comparative Capstone — "Try With AI"**

You've learned three IDEs. Now you'll choose one and build a real project, documenting how your chosen IDE compared to the others. This capstone synthesizes all lessons in Chapter 8 and demonstrates mastery of AI-native development workflows.

You'll submit a 1500-2000 word comparative analysis plus a working application built with your chosen IDE.

---

## Additional Resources

**Antigravity Official**:
- [Antigravity Documentation](https://antigravity.google.com/docs)
- [Agent Architecture Guide](https://antigravity.google.com/docs/agents)
- [Artifacts Reference](https://antigravity.google.com/docs/artifacts)

**API References**:
- [Spoonacular API Docs](https://spoonacular.com/food-api)
- [OpenWeatherMap API](https://openweathermap.org/api)

**Learning Resources**:
- Recipe Finder App Template (for reference)
- Artifact Templates (Task List, Implementation Plan, Walkthrough examples)
- Autonomy Decision Framework (quick reference chart)

**Community**:
- Antigravity Discussions Forum
- AI-Native Development Discord
- Share your projects and learn from others' artifact workflows
