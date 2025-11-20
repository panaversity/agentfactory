# Chapter 8: Git & GitHub for AI-Driven Development — Task Checklist

**Chapter Type**: Technical/Hybrid (Concepts + Hands-on AIDD)

**Status**: Ready for Development

**Feature Branch**: `012-chapter-8-git-github-aidd`

**Owner**: [To be assigned - content-implementer subagent]

**Estimated Effort**: 29-42 hours (writing 22-31h + review 7-11h)

**Note**: Reduced from original 39-52h due to: (1) Shorter lessons (15-20 min vs 25-30 min), (2) 3 IDE options instead of 4, (3) Installation links instead of screenshots, (4) Simplified capstone for non-coders

**Part**: Part 2 - AI Tool Landscape

**Lesson Duration Standard**: Each lesson 15-20 minutes (total: 175 min = 3 hours)

---

## Implementation Strategy

### MVP Scope (User Story 1 - P1)
**Minimum Viable Chapter**: Lessons 1-4 (Git Safety Basics)
- Install Git, configure locally
- Core workflow: init, status, add, commit, push
- Safety: undo changes, understand risks
- AI prompts: natural language Git operations

**Why this is MVP**: Students can safely work with AI assistants using Git locally. GitHub (US2), IDE (US3), and PRs (US4) are enhancements but not blocking.

### User Story Completion Order
1. **US1** (P1): Git Safety Basics → Lessons 1-4
2. **US2** (P2): GitHub Integration → Lessons 6
3. **US3** (P2): IDE Setup → Lesson 8
4. **US4** (P3): Pull Requests → Lesson 7
5. **Integration**: Branching (Lesson 5) + Capstone (Lesson 9)

### Parallel Execution Opportunities
- **Lessons 1-4** can be written in parallel (independent concepts)
- **Lessons 6, 7, 8** can be written in parallel after Lesson 5 complete (share branching prerequisite)
- Installation guides (Windows, Mac, Linux) can be written in parallel
- IDE sections (VS Code, Cursor, Zed) can be written in parallel

---

## Task List by Phase

### Phase 1: Setup - Content Structure & Chapter Scaffolding

**Goal**: Create chapter directory structure and overview documentation

**Independent Test**: Directory structure matches `specs/book/directory-structure.md`, chapter README explains structure and learning outcomes

- [X] T001 Create chapter directory at `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [X] T002 Create chapter `readme.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/readme.md`
  - Include: Overview (2-3 paragraphs), learning outcomes, 9-lesson structure, estimated time (3 hours / 175 min), prerequisites (Chapters 1-7), tools needed (Git, GitHub, IDE)
  - Reference: `.claude/output-styles/chapters.md` for structure

---

### Phase 2: Foundational - Policy and Style Setup

**Goal**: Establish lesson ending policy for consistency across all lessons

**Independent Test**: All lessons use correct "Try With AI" ending format per policy

- [X] T003 Document lesson ending policy in chapter README
  - Policy: Each lesson MUST end with single "Try With AI" section (no "Key Takeaways" or "What's Next")
  - Before AI tools taught (Part 1): Use ChatGPT web in that section
  - After tool onboarding: Instruct learners to use preferred AI tool (Gemini CLI, Claude Code), optionally provide CLI and web variants

---

### Phase 3: User Story 1 (P1) - Git Safety Basics with AI Assistant

**Story Goal**: Beginner learns Git version control to safely work with AI coding assistants without losing work or making irreversible mistakes

**Independent Test**: Student can initialize Git repo, make changes with AI assistant, commit changes, and successfully undo unwanted AI-generated code using natural language prompts

**Duration**: Lessons 1-4 (75 minutes: 15+20+20+20)

#### Lesson 1: Why Git Matters with AI Tools (15 min)

- [X] T004 [P] [US1] Create `01-why-git-matters-with-ai.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 1, title: "Why Git Matters with AI Tools", description: "Recognize why version control is essential for safe AI-assisted development"
  - Content: Hook (AI rewrote 50 lines - how to check/undo?), problem statement, Git introduction (time machine metaphor), why Git matters for AI, real scenario, connection to Chapters 5-7
  - Conceptual only (no code), follows `.claude/output-styles/lesson.md`
  - Learning objective (Bloom's Remember/Understand): "Recognize why version control is essential"

- [ ] T005 [US1] Write "Try With AI" section for Lesson 1 in `01-why-git-matters-with-ai.md`
  - Tool: ChatGPT web (AI tools not yet taught)
  - 3 prompts: (1) "What is Git?", (2) "Explain risks when working with AI code assistants and how Git helps", (3) "Explain version control as if I've never heard of it"
  - Expected outputs documented
  - No additional closing sections

- [ ] T006 [P] [US1] Create optional ASCII/SVG diagram showing Git as safety net in Lesson 1
  - Simple visual: change → commit → undo capability

#### Lesson 2: Essential Setup (20 min)

- [ ] T007 [P] [US1] Create `02-essential-setup.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 2, title: "Essential Setup", description: "Install Git, create GitHub account, configure locally"
  - Three sections: A) Install Git (platform-specific with links), B) GitHub Account Creation, C) Git Configuration
  - Learning objective: "Successfully install and configure Git, create GitHub account"

- [ ] T008 [P] [US1] Write Windows Git installation guide with links and text descriptions in Lesson 2
  - Link: https://git-scm.com/download/win
  - Step-by-step: Download .exe → Run installer → Select options (defaults OK) → Open Command Prompt → Verify `git --version`
  - Troubleshooting: "git not recognized" → restart Command Prompt or add to PATH
  - **No screenshots required**: Use text descriptions and installation link

- [ ] T009 [P] [US1] Write macOS Git installation guide with links and text descriptions in Lesson 2
  - Link: https://git-scm.com/download/mac
  - Two options: Homebrew (`brew install git`) or Direct Download
  - Step-by-step for each method
  - Troubleshooting: Xcode command-line tools if needed
  - **No screenshots required**: Use text descriptions and installation link

- [ ] T010 [P] [US1] Write Linux Git installation guide with links and text descriptions in Lesson 2
  - Link: https://git-scm.com/download/linux
  - Package manager commands: Ubuntu/Debian (`apt-get install git`), Fedora (`dnf install git`), Arch (`pacman -S git`)
  - Verification command: `git --version`
  - **No screenshots required**: Use text descriptions and installation link

- [ ] T011 [P] [US1] Write GitHub account creation guide with links in Lesson 2
  - Link: https://github.com
  - Step-by-step: Visit github.com → Fill registration form → Verify email → Choose free plan → Complete onboarding
  - Written for absolute beginners
  - **No screenshots required**: Use text descriptions and clear step numbers

- [ ] T012 [US1] Write Git configuration section in Lesson 2
  - Commands: `git config --global user.name "Your Name"`, `git config --global user.email "you@example.com"`
  - Verification: `git config --list`
  - Explanation: Why configuration matters (commits need author info)

- [ ] T013 [US1] Write "Try With AI" section for Lesson 2 in `02-essential-setup.md`
  - Tool: ChatGPT web (AI tools not yet taught)
  - 5 prompts: (1) "Help me install Git on [Windows/Mac/Linux]", (2) "I installed Git but 'git --version' doesn't work", (3) "Walk me through creating a GitHub account", (4) "Help me configure Git with my name and email", (5) "Verify my Git configuration is correct"

#### Lesson 3: The Daily Workflow (20 min)

- [ ] T014 [P] [US1] Create `03-the-daily-workflow.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 3, title: "The Daily Workflow", description: "Master the five core Git commands for everyday work"
  - Five commands: init, status, add, commit, push
  - Real scenario: Creating a simple Python file with AI, tracking changes
  - Learning objective (Bloom's Apply): "Perform core Git workflow"

- [ ] T015 [US1] Write command explanations for each of the 5 core Git commands in Lesson 3
  - Each command: What it does, when to use, example with output, common mistakes
  - Commands: `git init`, `git status`, `git add`, `git commit -m`, `git push`

- [ ] T016 [P] [US1] Create simple Python example file specification for Lesson 3
  - Purpose: Concrete file to commit and track
  - Complexity: Beginner (5-10 lines)
  - Code: Simple function (hello world or basic math)
  - AI generates it via student prompt

- [ ] T017 [US1] Write "Try With AI" section for Lesson 3 in `03-the-daily-workflow.md`
  - Tool: ChatGPT web or student's preferred AI tool from Chapters 5-6
  - 5 prompts: (1) "Initialize a Git repository in my project folder", (2) "Show me the status of my Git repository", (3) "Stage my files for commit", (4) "Create a commit with a meaningful message", (5) "Explain when I should commit my work"

#### Lesson 4: Safety Net - Undoing Changes (20 min)

- [ ] T018 [P] [US1] Create `04-safety-net-undoing-changes.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 4, title: "Safety Net: Undoing Changes", description: "Learn five ways to undo changes safely with Git"
  - Five undo methods: diff, checkout, reset --soft, reset --hard, revert
  - Decision tree: Which undo method for which scenario
  - Learning objective (Bloom's Apply/Analyze): "Safely undo changes at different stages"

- [ ] T019 [US1] Write undo method explanations with safety warnings in Lesson 4
  - Each method: What it does, safety level (safe/dangerous), when to use, example
  - **CRITICAL**: Explicit warnings for destructive commands (reset --hard)
  - Decision tree: "I made changes but didn't commit" → checkout, "I committed but want to undo" → reset --soft, etc.

- [ ] T020 [US1] Write "Try With AI" section for Lesson 4 in `04-safety-net-undoing-changes.md`
  - Tool: Student's preferred AI tool from Chapters 5-6
  - 5 prompts: (1) "Show me all changes since last commit", (2) "Explain what each line changed", (3) "I want to undo changes but keep work to fix it", (4) "Execute the undo command", (5) "Show my commit history to confirm"

---

### Phase 4: Integration - Branching for Experimentation

**Goal**: Students learn to create branches for safe AI experimentation before integrating GitHub, IDE, and PRs

**Independent Test**: Student can create feature branch, make AI-assisted changes, test, and merge or discard

**Duration**: Lesson 5 (20 minutes)

- [ ] T021 [P] Create `05-branches-for-experimentation.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 5, title: "Branches for Experimentation", description: "Create and use Git branches to safely test AI-generated changes"
  - Content: Why branches matter for AI (test refactors safely), branching workflow, merge vs discard decision
  - Learning objective (Bloom's Apply): "Create branches for safe experimentation"
  - **Note**: Students practice with feature branch; all code generation done by AI assistant

- [ ] T022 Write branch command explanations in Lesson 5
  - Commands: `git branch feature-name`, `git checkout feature-name`, `git merge feature-name`, `git branch -d feature-name`
  - Workflow: Create → Switch → Make changes → Test → Decide (merge or discard)

- [ ] T023 Write "Try With AI" section for Lesson 5 in `05-branches-for-experimentation.md`
  - Tool: Student's preferred AI tool
  - 5 prompts: (1) "Create a branch for testing AI-generated refactoring", (2) "Switch to the new branch", (3) "Make changes and commit on this branch", (4) "How do I merge this back to main if it works?", (5) "Show me how to discard this branch if I don't want the changes"

---

### Phase 5: User Story 2 (P2) - GitHub Integration for Backup and Portfolio

**Story Goal**: Student connects local Git repository to GitHub for cloud backup, sharing, and portfolio building

**Independent Test**: Student can create GitHub account, connect local repo, push commits, view project online using natural language prompts

**Duration**: Lesson 6 (20 minutes)

- [ ] T024 [P] [US2] Create `06-github-integration.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 6, title: "GitHub Integration", description: "Connect local repository to GitHub for backup and collaboration"
  - Content: What is GitHub, why cloud backup matters, remote setup, authentication (Personal Access Token recommended for beginners), push/pull workflow
  - Learning objective (Bloom's Apply): "Connect local repo to GitHub"
  - **Note**: Personal Access Token recommended over SSH (simpler for beginners)

- [ ] T025 [US2] Write GitHub repository creation guide in Lesson 6
  - Steps: Log into GitHub → Click "New repository" → Fill name, description → Create → Copy remote URL
  - Commands: `git remote add origin [URL]`, `git push -u origin main`

- [ ] T026 [US2] Write authentication guide (Personal Access Token) in Lesson 6
  - Steps: GitHub Settings → Developer settings → Personal access tokens → Generate new token → Select scopes (repo) → Copy token
  - Usage: Use token as password when pushing
  - **Security note**: Never commit tokens to git

- [ ] T027 [US2] Write push/pull workflow explanation in Lesson 6
  - Push: `git push origin main` (upload to GitHub)
  - Pull: `git pull origin main` (download from GitHub)
  - Real scenario: Working on multiple machines, syncing code

- [ ] T028 [US2] Add "Natural Language Prompts for Git Operations" explanatory paragraph in Lesson 6
  - **Key principle**: "You don't memorize `git merge feature-branch`. You ask your AI: 'Merge the feature branch into main.' AI translates intent into commands and explains."
  - Students practice this pattern in every "Try With AI" exercise
  - By Lesson 9 (Capstone), asking AI for Git help becomes natural

- [ ] T029 [US2] Write "Try With AI" section for Lesson 6 in `06-github-integration.md`
  - Tool: Student's preferred AI tool
  - 5 prompts: (1) "I created a GitHub repository. Help me connect my local code", (2) "What's the difference between SSH and Personal Access Token?", (3) "I'll use a token. Show me how to generate one", (4) "Push my code to GitHub using the token", (5) "Verify my code is now on GitHub"

---

### Phase 6: User Story 4 (P3) - Pull Requests and Code Review with AI

**Story Goal**: Student creates pull requests, reviews AI-generated code, addresses feedback, and merges professionally

**Independent Test**: Student can create feature branch, push to GitHub, open PR with AI assistance documentation, respond to review comments, merge PR

**Duration**: Lesson 7 (20 minutes)

- [ ] T030 [P] [US4] Create `07-pull-requests-and-code-review.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 7, title: "Pull Requests and Code Review", description: "Create pull requests documenting AI assistance and merge professionally"
  - Content: What is PR, why PRs matter (review before merge, document AI assistance), PR workflow (create → review → address feedback → merge)
  - Learning objective (Bloom's Apply/Analyze): "Create and review pull requests professionally"
  - **Note**: Focuses on essential PR workflow - create, review, merge. Optional: extended feedback cycle

- [ ] T031 [US4] Write PR creation workflow in Lesson 7
  - Steps: Create branch → Make changes → Push branch to GitHub → Go to GitHub → Click "Compare & pull request" → Write description (title, what changed, why, how to test, AI assistance used) → Create PR

- [ ] T032 [P] [US4] Create example PR description template with AI assistance documentation in Lesson 7
  - Template includes: Title, description, testing instructions, "What AI Helped With" section, "How to Test" section
  - Example: Calculator module PR showing AI generated code, tests, and docstrings

- [ ] T033 [US4] Write code review and feedback cycle in Lesson 7
  - Review: View diff, comment on specific lines, request changes or approve
  - Address feedback: Make changes locally → Commit → Push → PR auto-updates
  - Merge: Click "Merge Pull Request" → Delete branch → Pull locally

- [ ] T034 [US4] Write "Try With AI" section for Lesson 7 in `07-pull-requests-and-code-review.md`
  - Tool: Student's preferred AI tool
  - 5 prompts: (1) "Create a pull request for the feature branch I just pushed", (2) "Help me write a PR description that explains what I built and what AI helped with", (3) "Show me the diff so I can review what I'm about to merge", (4) "I got feedback: 'Add more error handling.' Make those changes and update the PR", (5) "The PR is approved. Merge it to main and clean up"

---

### Phase 7: User Story 3 (P2) - IDE Setup for AI-Assisted Development

**Story Goal**: Student sets up modern IDE with Git integration, GitHub connection, and AI assistants for professional development environment

**Independent Test**: Student can install IDE (VS Code or alternative), configure Git integration, install AI extension, perform Git operations through IDE interface

**Duration**: Lesson 8 (20 minutes)

- [ ] T035 [P] [US3] Create `08-ide-setup-and-integration.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 8, title: "IDE Setup and Integration", description: "Install modern IDE with Git integration and AI coding extensions"
  - Four sections: A) Choose and install IDE (8 min), B) Configure Git integration (6 min), C) Install AI extensions (6 min), D) Perform Git operations in IDE (5 min total: 25 min → 20 min)
  - Learning objective (Bloom's Apply): "Use IDE with Git integration and AI extensions"
  - **Note**: Focuses on 3 IDE options (VS Code, Cursor, Zed) to respect Tier 1 cognitive load limits

- [ ] T036 [P] [US3] Create IDE comparison table with 3 options in Lesson 8
  - **3 IDE options only**: VS Code (recommended), Cursor, Zed
  - Table columns: IDE, Best For, Download Link, Key Features
  - VS Code: code.visualstudio.com | Beginners, flexible | Most popular, great extensions, free
  - Cursor: cursor.sh | AI-first development | Built-in Claude integration
  - Zed: zed.dev | Modern, fast | Fast, minimal, growing ecosystem
  - **Recommendation**: VS Code for most beginners (most popular, best extension ecosystem)
  - **Note**: PyCharm removed to respect Tier 1 cognitive load (max 3 options)

- [ ] T037 [P] [US3] Write VS Code installation guide with links in Lesson 8
  - Link: https://code.visualstudio.com
  - Steps: Download for OS (Windows/Mac/Linux) → Run installer → Launch → See Welcome screen → Click Source Control icon → Verify Git integration
  - **No screenshots required**: Use text descriptions and installation link

- [ ] T038 [P] [US3] Write Cursor installation guide with links in Lesson 8
  - Link: https://cursor.sh
  - Steps: Download for OS → Install (similar to VS Code) → Launch → Sign in with Anthropic API key → Built-in Claude integration → Git already integrated
  - **No screenshots required**: Use text descriptions and installation link

- [ ] T039 [P] [US3] Write Zed installation guide with links in Lesson 8
  - Link: https://zed.dev
  - Steps: Download for OS → Install → Launch → Configure theme → Git integration available
  - **No screenshots required**: Use text descriptions and installation link

- [ ] T040 [US3] Write Git integration configuration in Lesson 8
  - VS Code: Source Control panel (left sidebar) → Shows repo info, changed files, branch name
  - Diff viewer: Click file → See side-by-side diff
  - Optional extensions: Git Graph (visual commit history), GitLens (blame and history)

- [ ] T041 [P] [US3] Write AI extension installation guide in Lesson 8
  - Option 1: GitHub Copilot (~$10/month or free for students) | VS Code Extensions → Search "GitHub Copilot" → Install → Sign in
  - Option 2: Cursor AI (built-in if using Cursor IDE)
  - Option 3: Continue (free, open-source) | VS Code Extensions → Search "Continue" → Install
  - **Choose 1 for beginners**: Explain tradeoffs (paid vs free, features)

- [ ] T042 [US3] Write IDE Git operations guide in Lesson 8
  - Stage files: Click + icon next to filename
  - Commit: Type message in "Message" box → Click ✓ (checkmark)
  - Push: Click "..." menu → Select "Push"
  - Create branch: Click "..." menu → "Create Branch"
  - Comparison: "You can use CLI OR GUI—both work, use whichever is comfortable"

- [ ] T043 [US3] Write "Try With AI" section for Lesson 8 in `08-ide-setup-and-integration.md`
  - Tool: Claude Code (if using IDE) or Gemini CLI
  - 5 prompts: (1) "I'm installing VS Code for the first time. Walk me through setup", (2) "I opened my Git project in VS Code. How do I see my changes?", (3) "Explain the Source Control panel and how to commit", (4) "Help me install and set up a GitHub Copilot alternative", (5) "Now write a test function in the IDE and watch AI suggestions"

---

### Phase 8: Integration - Capstone Exercise

**Goal**: Students integrate all Git, GitHub, and AI skills in realistic capstone project demonstrating complete workflow

**Independent Test**: Student completes calculator project with AI generating all code, uses Git for safety throughout, pushes to GitHub, creates PR

**Duration**: Lesson 9 (20 minutes)

- [ ] T044 [P] Create `09-capstone-exercise.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
  - YAML frontmatter: sidebar_position: 9, title: "Capstone Exercise: Build with Git & GitHub", description: "Integrate all Git, GitHub, and AI skills in realistic project"
  - Scenario: "Build a small Python calculator with AI assistance. AI writes ALL code; you manage Git for safety. Push to GitHub. Create a pull request showing your work."
  - Learning objective (Bloom's Analyze/Create): "Integrate all Git skills in realistic project"
  - **Note**: AI generates ALL code - students focus on Git workflow and safety practices. No coding knowledge assumed.

- [ ] T045 Write capstone workflow steps in Lesson 9
  - Step 1: Project Initialization (3 min) - mkdir, git init, create README, first commit
  - Step 2: Basic Implementation with AI (5 min) - **You**: "Generate calculator module" | **AI**: Generates code | **You**: Save and commit | **Focus**: Understanding Git workflow, NOT writing code
  - Step 3: Test on Feature Branch (5 min) - **You**: "Add error handling" | Create branch | **AI**: Generates code | **You**: Commit | **Focus**: Branch workflow for safe experimentation
  - Step 4: Review and Decide (3 min) - **You**: "Run the tests" | **AI**: Runs tests | If pass: merge; if fail: AI fixes
  - Step 5: GitHub Integration (3 min) - Create GitHub repo → Add remote → Push → Verify online
  - Step 6: Create Pull Request (4 min) - Create branch → **You**: "Add docstrings" | **AI**: Generates docs | Commit → Push → Create PR with description → Merge → **Success**: Complete project on GitHub!

- [ ] T046 [P] Create calculator module specification for Lesson 9
  - Purpose: Realistic project with multiple functions
  - Complexity: Beginner (20-30 lines)
  - Code: Calculator functions (add, subtract, multiply, divide)
  - AI generates initial code, student tests and improves via AI
  - **Student does NOT write code**: AI generates everything

- [ ] T047 [P] Create unit tests specification for Lesson 9
  - Purpose: Demonstrate testing AI-generated code
  - Complexity: Beginner (15-20 lines)
  - Code: Simple pytest tests for calculator functions
  - AI generates tests, student asks AI to run them
  - **Student does NOT write tests**: AI generates and runs them

- [ ] T048 Create verification checklist for capstone in Lesson 9
  - Checklist items: Git repo initialized, GitHub account with visible repo, Calculator code generated and committed, Used AI to generate code (shown in commits), Created and tested feature branch, Merged changes successfully, Created and merged pull request, Project visible on GitHub with clean history, PR description documents AI assistance, Can explain why each Git step mattered

- [ ] T049 Write "Try With AI" section for Lesson 9 in `09-capstone-exercise.md`
  - Tool: Student's preferred AI tool
  - Scenario: "Let's build a calculator project together. You manage Git; I'll generate all the code."
  - Sequential prompts: (1) "Initialize a Git repo and create a README", (2) "Generate a Python calculator module with add, subtract, multiply, divide functions", (3) "Create a feature branch for adding error handling", (4) "Add error handling to the calculator", (5) "Write unit tests for the calculator", (6) "Run the tests and tell me if they pass", (7) "Merge the branch if tests pass", (8) "Connect this project to GitHub and push", (9) "Create a pull request with improved documentation", (10) "Merge the PR and celebrate!"

---

### Phase 9: Polish & Cross-Cutting Concerns

**Goal**: Final review, validation, and polish across all lessons

**Independent Test**: All lessons meet quality checklist, reading level Grade 7, no broken links, consistent style

- [ ] T050 [P] Validate all lesson YAML frontmatter is complete and correct
  - Check: sidebar_position (1-9), title, description, keywords present
  - Consistency: Titles match plan.md, descriptions are 8-15 words

- [ ] T051 [P] Verify all lessons end with single "Try With AI" section per policy
  - No additional closing sections (no "Key Takeaways" or "What's Next")
  - Each "Try With AI" includes: Tool selection, scenario, prompts (3-5), expected outputs, safety note

- [ ] T052 [P] Validate lesson durations match plan (15-20 min each)
  - Lesson 1: 15 min (conceptual)
  - Lessons 2-9: 20 min each
  - Total: 175 min = 3 hours

- [ ] T053 [P] Verify cognitive load within Tier 1 limits across all lessons
  - Max 5 new concepts per lesson section
  - Max 2-3 options presented (not 4+)
  - IDE options: Exactly 3 (VS Code, Cursor, Zed - no PyCharm)

- [ ] T054 [P] Verify all installation guidance uses links + text descriptions (no screenshot requirements)
  - Git installation: Links to git-scm.com with text-based steps
  - GitHub account: Link to github.com with numbered steps
  - IDE installation: Links to code.visualstudio.com, cursor.sh, zed.dev with text descriptions
  - **Success**: No screenshots mentioned as required

- [ ] T055 [P] Check reading level with Flesch-Kincaid test
  - Target: Grade 7 or below
  - Tool: Online readability checker or Word processor
  - Fix: Simplify jargon, shorten sentences if needed

- [ ] T056 [P] Validate all Git commands are accurate for Git 2.40+
  - Test commands: init, status, add, commit, push, branch, merge, checkout, reset, revert
  - Verify syntax and behavior

- [ ] T057 [P] Verify all external links are valid and current
  - Git downloads: https://git-scm.com/download/win, /mac, /linux
  - GitHub: https://github.com
  - VS Code: https://code.visualstudio.com
  - Cursor: https://cursor.sh
  - Zed: https://zed.dev

- [ ] T058 [P] Validate natural language prompt templates work with Claude Code and Gemini CLI
  - Test sample prompts from each "Try With AI" section
  - Verify AI assistants produce correct Git commands
  - Document any prompt adjustments needed

- [ ] T059 Create `.gitignore` example file for Python projects
  - Content: .env, secrets.txt, __pycache__/, *.pyc, .venv/, node_modules/
  - Explanation: Why each line matters (security, cleanup)

- [ ] T060 [P] Cross-reference validation with related chapters
  - Prerequisites: Chapters 1-7 mentioned correctly
  - Forward references: Later chapters can build on Git knowledge

- [ ] T061 Final editorial polish and consistency pass
  - Voice and tone consistent with AIDD principles
  - Terminology consistent across lessons
  - No duplicate explanations
  - Smooth lesson-to-lesson transitions

- [ ] T062 Create chapter validation report
  - Summary: All lessons complete, quality checks passed, ready for validation-auditor
  - Blockers: None expected
  - Recommendations: Any improvements identified during implementation

---

## Dependencies and Execution Order

### User Story Dependencies
- **US1 (P1)** → No dependencies (can start immediately)
- **US2 (P2)** → Requires US1 complete (need Git basics before GitHub)
- **US3 (P2)** → Requires US1 complete (need Git installed before IDE integration)
- **US4 (P3)** → Requires US2 complete (need GitHub account before PRs)
- **Integration (Lesson 5)** → Requires US1 complete (need Git basics before branching)
- **Capstone (Lesson 9)** → Requires US1, US2, Integration complete (applies all skills)

### Lesson Completion Order
**Critical Path** (must be sequential):
1. Lesson 1 → Lesson 2 → Lesson 3 → Lesson 4 (US1 foundation)
2. Lesson 5 (Integration - branching)
3. Lesson 6 (US2 - GitHub)
4. Lesson 7 (US4 - PRs, depends on Lesson 6)
5. Lesson 9 (Capstone, applies all previous lessons)

**Parallel Path** (can be done independently):
- Lesson 8 (US3 - IDE) can be written in parallel with Lessons 6-7 after Lesson 5 complete

### Parallel Execution Examples

**Phase 1: Foundation (Sequential)**
```
Week 1, Day 1-2: T004-T006 (Lesson 1) → Sequential
Week 1, Day 3-4: T007-T013 (Lesson 2) → T008, T009, T010 parallel (platform guides)
Week 1, Day 5: T014-T017 (Lesson 3) → Sequential
```

**Phase 2: Core Skills (Sequential)**
```
Week 2, Day 1: T018-T020 (Lesson 4) → Sequential
Week 2, Day 2: T021-T023 (Lesson 5 - branching) → Sequential
```

**Phase 3: Advanced Topics (Some Parallel)**
```
Week 2, Day 3: T024-T029 (Lesson 6 - GitHub) → Sequential
Week 2, Day 4-5: PARALLEL:
  - T030-T034 (Lesson 7 - PRs) | AND
  - T035-T043 (Lesson 8 - IDE): T036-T039 parallel (IDE guides)
```

**Phase 4: Integration (Sequential)**
```
Week 3, Day 1: T044-T049 (Lesson 9 - Capstone) → Sequential
```

**Phase 5: Polish (All Parallel)**
```
Week 3, Day 2-3: T050-T062 → All tasks can run in parallel (different files, validation checks)
```

---

## Task Summary

**Total Tasks**: 62 tasks
- Setup: 3 tasks
- US1 (Git Safety Basics): 17 tasks (Lessons 1-4)
- Integration (Branching): 3 tasks (Lesson 5)
- US2 (GitHub Integration): 6 tasks (Lesson 6)
- US4 (Pull Requests): 5 tasks (Lesson 7)
- US3 (IDE Setup): 9 tasks (Lesson 8)
- Capstone: 6 tasks (Lesson 9)
- Polish: 13 tasks

**Parallelizable Tasks**: 33 tasks marked [P]

**MVP Scope**: Tasks T001-T020 (Setup + US1 Lessons 1-4)

**Estimated Effort**: 29-42 hours
- Writing: 22-31 hours (9 lessons × 2-3h avg, reduced from 30-39h)
- Review: 7-11 hours (link verification, testing, polish, reduced from 9-13h)

**Parallel Opportunities**:
- Platform installation guides (Windows, Mac, Linux) - 3 parallel
- IDE guides (VS Code, Cursor, Zed) - 3 parallel
- Lessons 7 and 8 after Lesson 6 complete - 2 parallel
- All polish tasks - 13 parallel

**Success Metrics** (from spec):
- 90% initialize repo, commit, undo via AI prompts within 30 min
- 85% create GitHub account and push project within 1 hour
- 90% install Git successfully on all platforms
- 80% install and configure IDE with Git within 30 min
- 80% use Git branches to test AI changes safely
- 95% identify when to commit before AI changes
- 75% set up IDE with Git and AI extensions
- 70% create PR with AI assistance documented
- Reading level: Grade 7 or below (Flesch-Kincaid)
- Post-chapter confidence: 4.5/5 or higher on "I can safely experiment with AI using Git"

---

## Notes for Lesson Writers

1. **Lesson Ending Policy**: Each lesson MUST end with a single "Try With AI" section. No additional "Key Takeaways" or "What's Next" sections. Before AI tools are taught (Part 1), use ChatGPT web; after tool onboarding, instruct learners to use their preferred AI tool (Gemini CLI, Claude Code).

2. **Duration Constraint**: Each lesson is 15-20 minutes. Be concise. Focus on essential concepts only. Lesson 1 is 15 min (conceptual), Lessons 2-9 are 20 min each.

3. **Cognitive Load (Tier 1)**: Max 5 new concepts per lesson section. Max 2-3 options presented. For IDE, present exactly 3 options (VS Code, Cursor, Zed - no PyCharm).

4. **Installation Guidance**: Use installation links + text descriptions. NO screenshot requirements. Links must be direct and current.

5. **AI Generates Code**: In capstone (Lesson 9), emphasize AI generates ALL code. Students manage Git workflow only. No coding knowledge assumed.

6. **Natural Language Prompts**: Include prompt templates students can use with AI assistants. Test prompts with real AI tools (Claude Code, Gemini CLI).

7. **Safety First**: Explicit warnings for destructive Git commands (reset --hard). Teach "commit before major AI changes" pattern throughout.

8. **Constitution Alignment**: Follow evals-first, spec-first, validation-first principles. All content teaches AI as co-reasoning partner, not just coding assistant.

---

## Ready for Implementation

This task checklist is ready for the content-implementer subagent. All tasks are specific, have clear acceptance criteria, and include exact file paths. Lessons are organized by user story priority for incremental delivery.

**Next Step**: Invoke content-implementer subagent with this task list and plan.md to begin implementation.
