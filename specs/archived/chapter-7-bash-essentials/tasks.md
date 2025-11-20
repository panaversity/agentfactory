# Chapter 7: Bash Essentials for AI-Driven Development — Task Checklist

**Chapter Type**: Technical/Practical (Beginner Tier — AI-Native)

**Status**: Ready for Development (Plan Approved, Waiting for Task Implementation)

**Feature Branch**: `feature/chapter-7-bash-essentials`

**Chapter Director**: To be assigned

**Estimated Total Effort**: 95-110 hours (12-14 story points)

**Complexity Tier**: Beginner (Tier 1)

**Time Per Lesson**: 2-4 hours per lesson (8 hours/lesson × 8 lessons + exercises, validation, integration)

---

## Overview

Chapter 7 teaches absolute beginners how to navigate and use the terminal through **natural language communication with AI**, rather than memorizing bash commands. The chapter follows graduated teaching pattern:
- **Lessons 1-2 (Tier 1)**: Book teaches foundational concepts directly
- **Lessons 3-5 (Tier 2)**: AI companion executes complex operations while students specify and supervise
- **Lessons 6-8 (Tier 3)**: AI orchestrates multi-step workflows while students direct and validate

This task list breaks the chapter into atomic work units with clear acceptance criteria. All content must demonstrate real AI behavior (errors, limitations, clarifying questions) through authentic dialogue transcripts.

---

## Task List by Phase

### Phase 1: Chapter Structure and Foundation (Week 1)

#### Task 1.1: Create Chapter README.md (Foundation)

**Priority**: MUST-HAVE | **Effort**: 1-2h | **Owner**: Writer

**Description**: Create comprehensive chapter overview that explains the AI-native pedagogical approach, learning outcomes, and how this chapter differs from traditional bash tutorials.

**Acceptance Criteria**:
- [ ] File exists at `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/readme.md` (LOWERCASE)
- [ ] Follows `.claude/output-styles/chapter-readme.md` structure exactly
- [ ] Section 1: Overview explains "natural language to bash communication" as core learning approach
- [ ] Section 2: Lists 8 lessons and their learning objectives (referenced from plan.md)
- [ ] Section 3: Prerequisites (Chapters 5-6, AI tool access) clearly stated
- [ ] Section 4: How chapter differs from traditional bash tutorials (safety-first, AI collaboration, no syntax memorization)
- [ ] Section 5: Learning outcomes (measurable skills students will demonstrate)
- [ ] Section 6: Chapter structure and time estimates per lesson
- [ ] Writing is clear, engaging, Grade 7-8 reading level
- [ ] No unexplained jargon; all technical terms defined
- [ ] Matches style and tone of existing chapter 5-6 overviews
- [ ] Passes readability check (0 errors, 0 warnings)

**Reference Documents**:
- `.claude/output-styles/chapter-readme.md` for structure
- `plan.md` Section "Lesson Architecture" for learning objectives
- Existing chapters 5, 6 READMEs for tone and style reference

**Reference Files to Check**:
- `/book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/readme.md`
- `/book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/readme.md`

---

#### Task 1.2: Create Lesson File Structure and Frontmatter Template

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: Writer

**Description**: Create template for all 8 lesson files with correct YAML frontmatter, directory structure, and file naming convention.

**Acceptance Criteria**:
- [ ] Directory exists: `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/`
- [ ] 8 lesson files created with correct naming: `01-finding-your-way.md` through `08-orchestrating-complex-workflows.md`
- [ ] Each file has YAML frontmatter with:
  - [ ] `sidebar_position: [N]` (1-8 for lessons 1-8)
  - [ ] `title: "[Descriptive Lesson Title]"` (matches plan.md)
  - [ ] `description: "[One-sentence summary]"` (8-15 words)
  - [ ] `keywords: [3-5 relevant terms]`
- [ ] All frontmatter is valid YAML (can be parsed)
- [ ] Each file has H1 header matching title in frontmatter
- [ ] Placeholder sections created: Opening, Main Content, Try With AI (empty, to be filled)
- [ ] File structure is ready for content writers

**Files to Create**:
```
01-finding-your-way.md
02-the-safety-pattern.md
03-reading-and-understanding.md
04-file-operations-through-language.md
05-configuration-and-secrets.md
06-exploring-the-file-system.md
07-building-and-running-scripts.md
08-orchestrating-complex-workflows.md
```

**Reference**: `.claude/output-styles/chapters.md` (Frontmatter section)

---

### Phase 2: Content Writing and Real Dialogue Collection (Weeks 2-5)

#### Task 2.1: Lesson 1 — Navigation Fundamentals (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 2-3h | **Owner**: Lesson Writer

**Description**: Write Lesson 1 content covering terminal literacy, file system orientation, and location awareness. Focus on foundational concepts (Tier 1 — book teaches directly).

**Acceptance Criteria**:
- [ ] Content fills approximately 1,500-2,000 words
- [ ] Opening paragraph (200 words) hooks reader and establishes why location awareness matters
- [ ] Section A: Explains what terminal is, why developers use it (3-4 paragraphs, plain language)
- [ ] Section B: Working directory concept explained with relatable analogy (filing cabinet metaphor or similar)
- [ ] Section C: Real dialogue transcript 1 — Student asks "Where am I?" → AI executes `pwd`, explains path
- [ ] Section C: Real dialogue transcript 2 — Student asks "What files are here?" → AI shows `ls`, explains difference between files and directories
- [ ] Section D: Authentic visual aid (file system diagram showing home directory hierarchy)
- [ ] Real-world example integrated: Show error when student tries to find files in wrong directory
- [ ] Engagement element: Opening story about Sarah deleting project (cautionary tale, sets up safety importance)
- [ ] All concepts use plain English; no jargon introduced without definition
- [ ] Reading level: Grade 7-8 (validated by readability tool)
- [ ] Concept count validated: 5 new concepts (terminal, working directory, absolute/relative paths, navigation, AI collaboration)
- [ ] Dialogue transcripts must be REAL (from actual AI tools) or clearly marked as realistic examples
- [ ] No code boxes yet; all concepts explained through text and dialogue

**Content Requirements**:
- Real dialogue transcripts from actual Claude Code, Gemini CLI, or ChatGPT interactions
- Analogies that make sense to beginners (no technical jargon in explanations)
- Visual aids: Simple ASCII diagram or referenced web image showing home directory structure
- Integration with Chapter 5-6 (references to "your AI collaborator from Chapter 5")

**Reference**: plan.md Lesson 1 outline

---

#### Task 2.2: Lesson 1 — Practice Checkpoints and Engagement

**Priority**: SHOULD-HAVE | **Effort**: 1h | **Owner**: Exercise Designer

**Description**: Create practice checkpoints for Lesson 1 that guide students to conduct 3 conversations without typing bash commands.

**Acceptance Criteria**:
- [ ] Checkpoint 1: "Conduct conversation about your current location"
  - [ ] Clear instructions on what to ask AI
  - [ ] Validation criteria: Can explain what directory they're in
  - [ ] Expected outcome: Screenshot of conversation

- [ ] Checkpoint 2: "List files and identify directories vs. files"
  - [ ] Clear instructions on how to ask AI to show files
  - [ ] Validation criteria: Can identify which items are directories
  - [ ] Expected outcome: Screenshot of conversation

- [ ] Checkpoint 3: "Navigate to a specific folder and describe contents"
  - [ ] Clear instructions on navigation language
  - [ ] Validation criteria: Can list what's in the folder they navigated to
  - [ ] Expected outcome: Screenshot showing navigation sequence

- [ ] Engagement element: Opening story (Sarah's data loss) is emotionally resonant and foreshadows safety
- [ ] Confidence builder: "You did this without typing a single bash command!"
- [ ] Real benefits highlighted: "This skill prevents mistakes in destructive operations"

**Tone**: Encouraging, non-judgmental, emphasizing understanding over syntax

---

#### Task 2.3: Lesson 1 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: Exercise Designer

**Description**: Create end-of-lesson "Try With AI" activity for Lesson 1 with 2-3 focused prompts and expected outcomes.

**Acceptance Criteria**:
- [ ] Activity section titled "Try With AI" (not "Key Takeaways" or "What's Next")
- [ ] Prompt 1: "Show me where I am right now..."
  - [ ] Clear, open-ended prompt
  - [ ] Expected outcome documented
  - [ ] AI tool selection: Claude Code or Gemini CLI (mention both as options)

- [ ] Prompt 2: "Navigate to my Documents folder..."
  - [ ] Prompts practical exploration
  - [ ] Expected outcome: Student understands directory path

- [ ] Prompt 3: (Optional) Student's own location question
  - [ ] Opens possibility for student-directed exploration
  - [ ] Expected outcome: Student feels confident asking own questions

- [ ] Safety/Ethics note: Explains why location awareness prevents mistakes in future lessons
- [ ] No additional sections after "Try With AI" (no separate Key Takeaways or What's Next)
- [ ] Activity takes approximately 10-15 minutes to complete

**Tool Selection**: Claude Code preferred (more explanatory), Gemini CLI as alternative

---

#### Task 2.4: Lesson 2 — The Safety Pattern (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 3-4h | **Owner**: Lesson Writer

**Description**: Write Lesson 2 content covering the critical 5-step safety pattern. This is the longest lesson (intentionally) because safety is paramount.

**Acceptance Criteria**:
- [ ] Content fills approximately 2,500-3,500 words
- [ ] Opening section (300-400 words): Real story (anonymized) of developer losing 6 months of work
  - [ ] Story is emotionally resonant and scary (appropriate for emphasizing importance)
  - [ ] Explains why it happened (didn't understand location, didn't verify)
  - [ ] Frames lesson: "This could save your career"

- [ ] Section B: Detailed explanation of 5-step safety pattern with real dialogue for each step
  - [ ] Step 1 (Ask): Dialogue showing student requesting operation
  - [ ] Step 2 (Explain): AI explains what will happen, shows location, lists affected files, explains command
  - [ ] Step 3 (Understand): Student asks clarifying questions ("What does `-rf` mean?" "Can I undo this?")
  - [ ] Step 4 (Verify): Student confirms with explicit "Yes, do it" response
  - [ ] Step 5 (Execute): AI runs command and shows result
  - [ ] Each step has real dialogue transcript from actual AI tool

- [ ] Section C: Red flags explanation
  - [ ] List of dangerous commands: `rm`, `rm -rf`, `sudo`, `chmod 777`
  - [ ] For each red flag: What it does, why it's dangerous, how to respond
  - [ ] Example dialogue: Student sees red flag, asks clarifying questions, AI explains

- [ ] Section D: Common mistakes and how to avoid them
  - [ ] Mistake 1-4 as outlined in plan
  - [ ] For each: Real example, how to catch it, correct approach

- [ ] Visual aid: Flowchart of 5-step pattern with decision points
- [ ] Command breakdown examples: Show commands with plain English translation
- [ ] Concept count validated: 5 concepts (irreversibility, 5 steps, red flags, confirmation, execution)
- [ ] Repetition of key concepts: Safety pattern is emphasized multiple times
- [ ] Real dialogue transcripts from actual tools

**Content Tone**: Serious, professional, emphasizing that this is non-negotiable

---

#### Task 2.5: Lesson 2 — Safety Pattern Practice Checkpoints

**Priority**: MUST-HAVE | **Effort**: 2h | **Owner**: Exercise Designer

**Description**: Create exercises where students identify missing safety steps, recognize red flags, and conduct supervised deletion exercises.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students identify missing steps in flawed dialogues
  - [ ] Provide 2-3 flawed dialogue examples
  - [ ] Each missing a different step (one lacks Step 2: Explain, one lacks Step 4: Verify)
  - [ ] Task: "What step is missing? How would you fix this?"
  - [ ] Validation: Student can identify the gap and suggest correction

- [ ] Checkpoint 2: Students identify red flags
  - [ ] Provide 3-4 commands with red flags
  - [ ] Examples: `sudo rm -rf /var/www/`, `chmod 777 config.txt`, `rm -rf ~/projects/`
  - [ ] Task: "What red flags do you see? How would you respond?"
  - [ ] Validation: Student identifies red flags and asks appropriate clarifying questions

- [ ] Checkpoint 3: Supervised deletion exercise
  - [ ] Instructions for setting up practice folder with known contents
  - [ ] Step-by-step guidance for conducting conversation following all 5 steps
  - [ ] Validation: Student demonstrates all 5 steps in conversation transcript
  - [ ] Expected outcome: Screenshot showing conversation with safety pattern

- [ ] Engagement: Comic or visual showing before/after of developer who skipped safety pattern
- [ ] Tone: Serious about safety, but not alarmist

---

#### Task 2.6: Lesson 2 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1.5h | **Owner**: Exercise Designer

**Description**: Create comprehensive "Try With AI" activity for Lesson 2 with 2-4 hands-on safety pattern practice prompts.

**Acceptance Criteria**:
- [ ] Activity section: "Try With AI"
- [ ] Prompt 1: Hands-on safety pattern for deletion
  - [ ] "Help me delete a test folder following the 5-step safety pattern"
  - [ ] Expected outcome: Student conducts full 5-step conversation

- [ ] Prompt 2: Move operation safety pattern
  - [ ] "Walk me through all 5 steps for moving files to backup location"
  - [ ] Expected outcome: Student understands verification steps

- [ ] Prompt 3: Red flag explanation
  - [ ] "Explain what this command does: `find . -type f -name '*.tmp' -delete`"
  - [ ] Expected outcome: Student recognizes destructiveness, asks clarifying questions

- [ ] Prompt 4: (Optional) Complex scenario
  - [ ] "Create a scenario where I delete 100 files. Show me how you'd approach this safely."
  - [ ] Expected outcome: Student demonstrates orchestrated safety approach

- [ ] Safety/Ethics note: "This is NOT optional. Every file operation must include all 5 steps."
- [ ] No additional sections after activity

---

#### Task 2.7: Lesson 3 — Command Structure Understanding (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 2-3h | **Owner**: Lesson Writer

**Description**: Write Lesson 3 content teaching students to read and understand bash command structure without memorizing syntax.

**Acceptance Criteria**:
- [ ] Content fills approximately 1,800-2,200 words
- [ ] Opening (150 words): Frame "You're not learning syntax; you're learning to read commands like a detective"

- [ ] Section A: Anatomy of bash commands
  - [ ] Real command example: `ls -lah ~/Documents`
  - [ ] Breakdown into parts: command, flags, arguments
  - [ ] Plain English translation: "List all files in Documents with sizes in readable format"
  - [ ] Why structure matters: Helps supervise AI

- [ ] Section B: Understanding flags without memorizing
  - [ ] Common flag patterns: `-l`, `-a`, `-h`, `-r`, `-f`
  - [ ] Plain language explanations (what each does, why used)
  - [ ] Real dialogue: Student sees `find . -name "*.py" -type f`, asks AI to explain each part
  - [ ] Strategy: "Ask 'What does this flag do?' and understand the explanation"

- [ ] Section C: Pipes and redirects
  - [ ] Pipes (`|`) explained with real example: `cat file.txt | grep "error" | wc -l`
  - [ ] Redirects (`>`, `>>`, `<`) explained with use cases
  - [ ] Plain English for each
  - [ ] Real dialogue: AI proposes pipeline, student asks to walk through each step

- [ ] Section D: Asking clarifying questions
  - [ ] Good supervision questions: "What does this flag do?", "Why do we need this part?", "Can you show me the result before executing?"
  - [ ] Bad habit: Running without understanding
  - [ ] Good habit: "I understand what you're proposing, proceed"

- [ ] Content Elements: 5-6 command breakdown examples with plain English translation
- [ ] Real dialogues: 3-4 authentic examples of students asking clarifying questions
- [ ] Visual guide: Color-coded command breakdown
- [ ] Concept count validated: 7 concepts (command structure, flags, arguments, pipes, redirects, supervision, understanding intent)

**Real Dialogue Requirements**: All examples must be from actual AI tools or clearly marked as realistic

---

#### Task 2.8: Lesson 3 — Command Comprehension Checkpoints

**Priority**: SHOULD-HAVE | **Effort**: 1.5h | **Owner**: Exercise Designer

**Description**: Create exercises where students demonstrate command reading comprehension without running commands.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students read and explain commands in plain English
  - [ ] Provide 5-6 real commands (progressively more complex)
  - [ ] Task: "Explain this command in plain English"
  - [ ] Validation method: Compare to AI's explanation

- [ ] Checkpoint 2: Students identify what flags do
  - [ ] Provide commands with different flags: `ls -lah`, `find . -type f -name "*.md"`
  - [ ] Task: "What does each flag do and why would we use it?"
  - [ ] Validation: Student can explain flags' purposes

- [ ] Checkpoint 3: Pipe comprehension
  - [ ] Given: `cat requirements.txt | grep "django" | wc -l`
  - [ ] Task: "Explain this in plain English, step by step"
  - [ ] Expected: "Read file, filter for django, count lines"

- [ ] All checkpoints: NO command execution required; reading comprehension only
- [ ] Engagement: Frame as detective work ("Read the clues")

---

#### Task 2.9: Lesson 3 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: Exercise Designer

**Description**: Create "Try With AI" activity for command comprehension with 2-3 focused prompts.

**Acceptance Criteria**:
- [ ] Prompt 1: "Show me a command that finds all Python files"
  - [ ] Expected: AI explains each part
  - [ ] Student understands structure

- [ ] Prompt 2: "I want to search for a specific word in all my text files"
  - [ ] AI walks through command, explaining flags and pipes
  - [ ] Student asks clarifying questions

- [ ] Prompt 3: "Create a command that finds files modified in last 7 days"
  - [ ] AI explains each part
  - [ ] Student validates understanding

- [ ] Safety/Ethics note: Explains how this comprehension skill enables supervision
- [ ] Time: 10-15 minutes to complete

---

#### Task 2.10: Lesson 4 — File Operations Through Natural Language (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 3h | **Owner**: Lesson Writer

**Description**: Write Lesson 4 content teaching students to direct AI for copy and move operations using natural language while applying safety pattern.

**Acceptance Criteria**:
- [ ] Content fills approximately 2,000-2,500 words
- [ ] Opening (150 words): Frame on low-risk (copy) vs. higher-risk (move) operations

- [ ] Section A: Copy operations
  - [ ] Frame: Copy is safe because original stays intact
  - [ ] When to use: Backups, duplicating, organizing copies
  - [ ] Real dialogue: "Create a backup of my project folder"
  - [ ] Shows all 5 safety steps in context
  - [ ] Validation: Confirms backup exists

- [ ] Section B: Move operations
  - [ ] Frame: Move is higher-risk; requires extra caution
  - [ ] When to use: Reorganizing, archiving
  - [ ] Real dialogue: "Move all my Python files into python-projects folder"
  - [ ] Shows 5 safety steps WITH extra verification
  - [ ] Validation: Confirms files in new location and original empty

- [ ] Section C: Creating and organizing directories
  - [ ] Natural language: "Create a folder structure for my projects"
  - [ ] Common patterns shown: projects/2025/, archive/, active/
  - [ ] Validation: Lists new directories

- [ ] Section D: The backup question
  - [ ] Professional practice: "Should we backup first?"
  - [ ] When backup is critical
  - [ ] Real dialogue showing backup preventing disaster

- [ ] Real dialogues: 3 comprehensive examples (copy, move with catch, backup saves day)
- [ ] File operation decision matrix: Copy vs. Move vs. Backup
- [ ] Concept count validated: 7 concepts

**Real Dialogues**: All from actual AI tools or clearly marked as realistic

---

#### Task 2.11: Lesson 4 — File Operations Practice and Validation

**Priority**: MUST-HAVE | **Effort**: 2h | **Owner**: Exercise Designer

**Description**: Create practice exercises for copy, move, and backup operations with validation.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students direct AI to copy files
  - [ ] Scenario: Back up home directory
  - [ ] Task: Use natural language, apply safety pattern, validate
  - [ ] Validation: Screenshot of conversation showing all 5 steps

- [ ] Checkpoint 2: Students direct AI to move files
  - [ ] Scenario: Organize scattered files
  - [ ] Task: Ask about backup first, apply safety pattern, validate
  - [ ] Validation: Confirm files in new location

- [ ] Checkpoint 3: Scenario analysis
  - [ ] Given: Various file management needs
  - [ ] Task: "Copy or move? Why? Do we backup?"
  - [ ] Validation: Student explains decision

- [ ] All checkpoints: Require safety pattern application
- [ ] Engagement: "After this, you can organize your entire projects folder safely"

---

#### Task 2.12: Lesson 4 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: Exercise Designer

**Description**: Create "Try With AI" for file operations with 2-3 practical hands-on exercises.

**Acceptance Criteria**:
- [ ] Prompt 1: Backup operation following 5-step pattern
- [ ] Prompt 2: File organization with safety considerations
- [ ] Prompt 3: Move operation with verification
- [ ] All include tool selection (Claude Code or Gemini CLI)
- [ ] Expected outcomes documented
- [ ] Safety/Ethics note included
- [ ] Time: 15-20 minutes to complete

---

#### Task 2.13: Lesson 5 — Configuration and Secrets (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 2.5-3h | **Owner**: Lesson Writer

**Description**: Write Lesson 5 on environment variables, `.env` files, `.gitignore`, and secure secret management.

**Acceptance Criteria**:
- [ ] Content fills approximately 2,000-2,500 words
- [ ] Opening section (300 words): Real story of developer's AWS credentials leaked on GitHub
  - [ ] Explains what happened and why
  - [ ] Cost and implications
  - [ ] Frame: "This lesson prevents catastrophic breaches"

- [ ] Section B: Environment variables and `.env` files
  - [ ] Concept: Code reads from environment, not hardcoded
  - [ ] Python example: `api_key = os.getenv('OPENAI_API_KEY')`
  - [ ] How it works: Code asks system, returns value
  - [ ] Real dialogue: "Store my OpenAI API key securely"
  - [ ] AI creates `.env`, explains how code accesses it

- [ ] Section C: `.gitignore` preventing commits
  - [ ] What is `.gitignore`: File telling git what not to commit
  - [ ] Why it exists: Prevents accidental secret commits
  - [ ] Common patterns: `*.env`, `.env`, `*.key`
  - [ ] Real scenario: AI sets up `.gitignore`
  - [ ] Validation: Confirm `.env` won't be committed

- [ ] Section D: Package installation safely
  - [ ] Related concept: Installing dependencies requires environment
  - [ ] Real scenario: "Install project dependencies"
  - [ ] Dialogue: AI explains what's being installed, asks confirmation
  - [ ] Validation: Confirm successful installation

- [ ] Real dialogues: 3 comprehensive examples (API key setup, accidental commit prevention, dependency installation)
- [ ] Secret storage decision tree: When to use `.env` vs. secret manager
- [ ] Concept count validated: 7 concepts

---

#### Task 2.14: Lesson 5 — Secrets Management Practice

**Priority**: MUST-HAVE | **Effort**: 1.5h | **Owner**: Exercise Designer

**Description**: Create exercises for secure `.env` setup and `.gitignore` configuration.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students set up secure `.env` file
  - [ ] Scenario: Create `.env` with sample API key
  - [ ] Task: Have AI create file, update `.gitignore`, verify setup
  - [ ] Validation: Check `.env` is git-ignored

- [ ] Checkpoint 2: Students analyze security risks
  - [ ] Given: Code with hardcoded password
  - [ ] Task: "What's wrong? How would you fix it?"
  - [ ] Expected: Recognize it's a secret, move to environment

- [ ] Checkpoint 3: Students install dependencies
  - [ ] Scenario: Project with `requirements.txt`
  - [ ] Task: Direct AI to install, validate installation
  - [ ] Expected: Dependencies installed successfully

- [ ] All checkpoints: Emphasize safety of environment variables
- [ ] Engagement: "Your secrets are safe because..."

---

#### Task 2.15: Lesson 5 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: Exercise Designer

**Description**: Create "Try With AI" for secure configuration with 2-3 hands-on exercises.

**Acceptance Criteria**:
- [ ] Prompt 1: "Store my API key securely and explain how my code accesses it"
- [ ] Prompt 2: "Set up `.env` and `.gitignore` to prevent accidental commits"
- [ ] Prompt 3: "Show me Python code that reads API keys from environment variables"
- [ ] Tool selection: Claude Code or Gemini CLI
- [ ] Expected outcomes documented
- [ ] Safety/Ethics note: Emphasizes security importance
- [ ] Time: 15-20 minutes to complete

---

#### Task 2.16: Lesson 6 — Exploring the File System Safely (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 2.5-3h | **Owner**: Lesson Writer

**Description**: Write Lesson 6 on safe file system exploration using `find`, `grep`, and pattern analysis.

**Acceptance Criteria**:
- [ ] Content fills approximately 2,000-2,400 words
- [ ] Opening (150 words): Frame on using AI to explore without getting lost

- [ ] Section A: Finding files by name and type
  - [ ] Use case: "How many Python files do I have?"
  - [ ] Real dialogue: Student asks, AI searches, shows results
  - [ ] Explanation: What search looked for and why useful
  - [ ] Example results: Find by type, find by name pattern
  - [ ] Safety: Small searches before large operations

- [ ] Section B: Searching inside files
  - [ ] Use case: "Where did I use `import os`?"
  - [ ] Real dialogue: AI uses `grep`, shows matching files
  - [ ] Practical examples: TODO comments, deprecated library usage
  - [ ] Safety: Search helps understand impact

- [ ] Section C: Understanding patterns
  - [ ] After search, analyze results to identify patterns
  - [ ] Examples: "500 `.tmp` files should be cleaned up", "duplicates exist", "haven't touched old work in 2 years"
  - [ ] Decision making: Based on patterns, what should we do?
  - [ ] Real dialogue: Search → analyze → decide → plan → execute

- [ ] Section D: Validation of search results
  - [ ] Before taking action, validate search is correct
  - [ ] Techniques: Review results, sample files, count reasonableness
  - [ ] Scenario: Found 200 files; really want to delete all of them?

- [ ] Real dialogues: 3-4 comprehensive examples
- [ ] Search examples poster: 5-10 realistic commands students will use
- [ ] Concept count validated: 7 concepts

---

#### Task 2.17: Lesson 6 — Safe Exploration Checkpoints

**Priority**: SHOULD-HAVE | **Effort**: 1.5h | **Owner**: Exercise Designer

**Description**: Create exercises for exploratory searching and pattern analysis.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students conduct exploratory search
  - [ ] Scenario: Find all files of certain type
  - [ ] Task: Ask AI to search, review results, explain findings

- [ ] Checkpoint 2: Students analyze search results
  - [ ] Given: Search results showing many files
  - [ ] Task: "What patterns do you see? What decisions?"
  - [ ] Expected: Identify cleanup opportunities

- [ ] Checkpoint 3: Students validate before action
  - [ ] Scenario: Delete temporary files
  - [ ] Task: Search, review, confirm pattern, proceed
  - [ ] Safety: Sample files first

- [ ] Engagement: Frame as detective work

---

#### Task 2.18: Lesson 6 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: Exercise Designer

**Description**: Create "Try With AI" for safe exploration with 2-3 discovery exercises.

**Acceptance Criteria**:
- [ ] Prompt 1: "Find all Python files and show me where they are. What patterns do you notice?"
- [ ] Prompt 2: "Search for all TODO comments. Where should I focus attention?"
- [ ] Prompt 3: "Find all backup files. Should I delete them?"
- [ ] Tool selection: Claude Code or Gemini CLI
- [ ] Expected outcomes: Pattern recognition, cleanup decisions
- [ ] Safety/Ethics note: Emphasizes exploration before action
- [ ] Time: 15-20 minutes to complete

---

#### Task 2.19: Lesson 7 — Building and Running Scripts (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 3-4h | **Owner**: Lesson Writer

**Description**: Write Lesson 7 on script creation, understanding, testing, and validation. This is Tier 2/3 boundary lesson.

**Acceptance Criteria**:
- [ ] Content fills approximately 2,200-2,800 words
- [ ] Opening (200 words): Frame "Scripts are just commands in a file. You don't write them; you direct AI to write them."

- [ ] Section A: What is a script and why it matters
  - [ ] Script definition: Commands saved in file, executable as `bash script.sh`
  - [ ] Why matter: Reproducibility, version control, automation, error prevention
  - [ ] Real scenario: Backup projects every morning as a script
  - [ ] Frame: "You need to know WHEN to ask for help, not HOW to code"

- [ ] Section B: Describing what you want (specification)
  - [ ] Key skill: Tell AI what, not how
  - [ ] Good specification: "Back up my projects to cloud storage every morning"
  - [ ] Bad specification: "Use bash loops and rsync"
  - [ ] Real dialogue: Specification → AI writes script → you understand it

- [ ] Section C: Understanding AI-generated scripts
  - [ ] AI writes script; your job is to read and verify
  - [ ] Breaking down script: AI explains what each line does
  - [ ] Common patterns: Loops, conditionals, variables
  - [ ] You verify intent, not memorize syntax

- [ ] Section D: Testing before production
  - [ ] Always test on small data first
  - [ ] Real scenario: Test backup script on one small folder
  - [ ] Validation: Did script produce expected results?
  - [ ] Only test successful on small data → run on production

- [ ] Real dialogues: 3 comprehensive (backup script, script with parameters, test and validation)
- [ ] Script example annotated: Line-by-line plain English explanations
- [ ] Concept count validated: 7 concepts (boundary between A2 and B1)

---

#### Task 2.20: Lesson 7 — Script Understanding and Testing Checkpoints

**Priority**: MUST-HAVE | **Effort**: 2h | **Owner**: Exercise Designer

**Description**: Create exercises for directing AI to write scripts, understanding them, and testing.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students direct AI to write script
  - [ ] Scenario: Multi-step task (find, copy, rename)
  - [ ] Task: Specify what script should do, have AI write it, review
  - [ ] Validation: Can explain what script does

- [ ] Checkpoint 2: Students understand AI-written script
  - [ ] Given: AI-generated script
  - [ ] Task: "Explain each section. Does it match what you asked?"
  - [ ] Expected: Explain purpose of main sections

- [ ] Checkpoint 3: Students test script
  - [ ] Scenario: File-modifying script
  - [ ] Task: Test on small sample, validate, then use on full data
  - [ ] Expected: Script tested before production use

- [ ] All checkpoints: Emphasize validation before execution
- [ ] Engagement: "You're not writing code; you're directing AI"

---

#### Task 2.21: Lesson 7 — "Try With AI" Activity

**Priority**: MUST-HAVE | **Effort**: 1.5h | **Owner**: Exercise Designer

**Description**: Create "Try With AI" for script creation and validation with 2-3 hands-on exercises.

**Acceptance Criteria**:
- [ ] Prompt 1: "Create a backup script. Test it first with a small folder."
- [ ] Prompt 2: "Create a script that finds old files and moves them to archive. Explain what it will do before running."
- [ ] Prompt 3: "I have a morning task I repeat. Let's automate it with a script."
- [ ] Tool selection: Claude Code or Gemini CLI
- [ ] Expected outcomes: Script created, tested, validated
- [ ] Safety/Ethics note: "Always test first, always understand what script will do, never run untested script on production data"
- [ ] Time: 20-25 minutes to complete

---

#### Task 2.22: Lesson 8 — Orchestrating Complex Workflows (Content Writing)

**Priority**: MUST-HAVE | **Effort**: 4-5h | **Owner**: Lesson Writer

**Description**: Write Lesson 8 (capstone) on decomposing complexity, directing AI through multi-step workflows, and batch orchestration. This is Tier 3 (orchestration) lesson.

**Acceptance Criteria**:
- [ ] Content fills approximately 2,500-3,200 words
- [ ] Opening (200 words): Frame "Orchestration is about directing strategy, not executing tactics"

- [ ] Section A: When orchestration makes sense
  - [ ] Manual: 1-2 commands (too much overhead)
  - [ ] AI companion: Complex single operation
  - [ ] Orchestration: 5+ coordinated steps with decisions
  - [ ] Real examples: 10 git worktrees, batch process 200 images, deploy application
  - [ ] Frame: "Directing strategy, not executing tactics"

- [ ] Section B: Decomposing complexity into steps
  - [ ] Technique: "What would I do myself?"
  - [ ] Example workflow: Clone, create branches, setup environments
    - Step 1, 2, 3, etc. with decision points
  - [ ] AI's role: Execute steps, handle syntax
  - [ ] Your role: Define steps, validate, approve

- [ ] Section C: Directing AI through multi-step workflow
  - [ ] Technique: Execute one step, validate, move to next
  - [ ] Never: "Do all 5 steps without checking"
  - [ ] Always: "Do step 1, show what happened, then step 2"
  - [ ] Real dialogue: Step-by-step with validations
  - [ ] Checkpoints: Confirm before proceeding

- [ ] Section D: Parallel execution and batch operations
  - [ ] When you have 10+: Orchestration becomes powerful
  - [ ] Example: Create 10 git worktrees
  - [ ] Rather than: Manual 1 at a time
  - [ ] Instead: "Create all 10 with loop, validate"
  - [ ] Validation critical: Count results, spot-check, ensure all correct

- [ ] Real dialogues: 3 comprehensive (deployment, batch operation, orchestration with pauses)
- [ ] Workflow decomposition template: Goal, steps, validations
- [ ] Concept count validated: 10 concepts (B1 max for proficiency level)

**Real Dialogues**: All from actual AI tools or clearly marked as realistic

---

#### Task 2.23: Lesson 8 — Orchestration Checkpoints

**Priority**: MUST-HAVE | **Effort**: 2-2.5h | **Owner**: Exercise Designer

**Description**: Create exercises for decomposing tasks, directing multi-step workflows, and batch operations.

**Acceptance Criteria**:
- [ ] Checkpoint 1: Students decompose complex task
  - [ ] Scenario: Multi-step task (backup, organize, archive)
  - [ ] Task: Break into steps, identify validations
  - [ ] Expected: 5-8 clear steps with validation points

- [ ] Checkpoint 2: Students direct workflow step-by-step
  - [ ] Scenario: Task requiring step-by-step execution
  - [ ] Task: Don't hand off entire task; direct step-by-step
  - [ ] Expected: Student asks for step-by-step, validates between steps

- [ ] Checkpoint 3: Students orchestrate batch operation
  - [ ] Scenario: 10+ similar items
  - [ ] Task: Ask AI to batch-execute, then validate
  - [ ] Expected: AI executes efficiently, student validates all items correct

- [ ] All checkpoints: Progress from guided to autonomous
- [ ] Engagement: "You're learning to think like a project manager"

---

#### Task 2.24: Lesson 8 — "Try With AI" Activity (Capstone)

**Priority**: MUST-HAVE | **Effort**: 1.5-2h | **Owner**: Exercise Designer

**Description**: Create comprehensive "Try With AI" capstone activity with 2-4 orchestration exercises.

**Acceptance Criteria**:
- [ ] Prompt 1: "Clone, create 5 branches, setup environments. Execute step-by-step with validation."
- [ ] Prompt 2: "Batch-process 50 files: find, organize, create summary. Use orchestration efficiently."
- [ ] Prompt 3: "Deploy application: build, test, backup, deploy, verify. Direct me safely through this."
- [ ] Prompt 4: (Optional) "Complex workflow: back up, archive old, generate summary."
- [ ] Tool selection: Claude Code or Gemini CLI
- [ ] Expected outcomes: Complex task completed with validation
- [ ] Safety/Ethics note: "Validate between steps. More complexity = more validation."
- [ ] Time: 25-35 minutes to complete

---

### Phase 3: Real Dialogue Collection and Validation (Weeks 4-5)

#### Task 3.1: Collect Real Dialogue Transcripts from AI Tools

**Priority**: MUST-HAVE | **Effort**: 4-6h | **Owner**: Content Collector

**Description**: Conduct actual conversations with Claude Code, Gemini CLI, and ChatGPT to create authentic dialogue transcripts for all lessons. These transcripts are essential to demonstrate real AI behavior.

**Acceptance Criteria**:
- [ ] Lesson 1: 2 real dialogues collected
  - [ ] Dialogue 1: "Where am I?" → AI executes `pwd`, explains path
  - [ ] Dialogue 2: "What files are here?" → AI executes `ls`, explains files vs. directories
  - [ ] Tool: Claude Code preferred
  - [ ] Format: Clean transcript with student prompts and AI responses

- [ ] Lesson 2: 4 real dialogues (showing all 5 steps)
  - [ ] Dialogue 1: Step 1-5 for deletion operation
  - [ ] Dialogue 2: Step 1-5 for move operation
  - [ ] Dialogue 3: Red flag recognition
  - [ ] Dialogue 4: Backup preventing disaster
  - [ ] Tools: Mix of Claude Code and Gemini CLI to show variability

- [ ] Lesson 3: 3 real dialogues
  - [ ] Dialogue 1: Explaining command structure
  - [ ] Dialogue 2: Asking about flags
  - [ ] Dialogue 3: Understanding pipes
  - [ ] Tool: Claude Code

- [ ] Lesson 4: 3 real dialogues (copy, move, backup)
  - [ ] Each demonstrates safety pattern in context
  - [ ] Tools: Mix of Claude Code and Gemini CLI

- [ ] Lesson 5: 3 real dialogues (API key, `.gitignore`, dependencies)
  - [ ] Focus on security aspects
  - [ ] Tool: Claude Code

- [ ] Lesson 6: 3 real dialogues (search, pattern analysis, validation)
  - [ ] Show actual search results
  - [ ] Tool: Gemini CLI preferred (shows shell output)

- [ ] Lesson 7: 3 real dialogues (script creation, testing, validation)
  - [ ] Show actual script execution
  - [ ] Tool: Claude Code

- [ ] Lesson 8: 3 real dialogues (orchestration examples)
  - [ ] Show multi-step execution
  - [ ] Tool: Claude Code

- [ ] Total: 25-27 real dialogue transcripts
- [ ] Format: Clean markdown with clear prompt/response structure
- [ ] Authenticity: Show real errors, limitations, clarifying questions from actual AI
- [ ] No synthetic examples; all real or clearly marked as "realistic example"

**Tools to Use**:
- Claude Code (Web)
- Gemini CLI
- ChatGPT Code Interpreter (if available)

**Documentation Requirements**:
- [ ] Each transcript includes: Tool used, date/time, student intent, AI response
- [ ] Clear formatting distinguishing user prompts from AI responses
- [ ] Dialogue is complete (not truncated)
- [ ] Shows authentic AI behavior (errors, questions, explanations)

---

#### Task 3.2: Validate Authenticity and Relevance of Dialogue Transcripts

**Priority**: MUST-HAVE | **Effort**: 2h | **Owner**: Technical Reviewer

**Description**: Review collected dialogue transcripts to ensure they demonstrate real AI behavior and support learning objectives.

**Acceptance Criteria**:
- [ ] All 25-27 transcripts validated as authentic (from real AI tools or clearly marked)
- [ ] Each dialogue directly supports the lesson's learning objective
- [ ] Transcripts show real AI behavior: errors, clarifications, explanations
- [ ] No idealized/perfect scenarios; messiness is okay and educational
- [ ] Formatting is consistent across all dialogues
- [ ] Dialogues are complete (not truncated or edited to remove important parts)
- [ ] At least one dialogue per lesson shows an error or limitation
- [ ] At least one dialogue per lesson shows student asking clarifying questions
- [ ] Validation report: List of all dialogues, their lesson, and how they support learning

---

### Phase 4: Code Examples and Practical Demonstrations (Week 5-6)

#### Task 4.1: Create Code Examples for Each Lesson

**Priority**: SHOULD-HAVE | **Effort**: 2-3h | **Owner**: Code Example Writer

**Description**: Create practical, runnable code examples for lessons that require them (primarily Lessons 3-8).

**Acceptance Criteria**:
- [ ] Lesson 1: No code required (navigation concepts)
- [ ] Lesson 2: No code examples (safety pattern shown through dialogue)
- [ ] Lesson 3: 5-6 command examples with annotations
  - [ ] Examples: `ls -lah`, `find . -name "*.py"`, `grep -r "import"`, pipes, redirects
  - [ ] Each with plain English translation and output example
  - [ ] Format: Bash code block with comments

- [ ] Lesson 4: 3 command examples (copy, move, mkdir)
  - [ ] Real commands with context
  - [ ] Output examples
  - [ ] Format: Bash code block

- [ ] Lesson 5: 2 code examples
  - [ ] Python code reading from environment variable
  - [ ] `.env` file example structure
  - [ ] `.gitignore` example structure

- [ ] Lesson 6: 5 search examples
  - [ ] `find` commands
  - [ ] `grep` commands
  - [ ] Example output shown

- [ ] Lesson 7: 2 script examples
  - [ ] Simple backup script (annotated)
  - [ ] Script with parameters
  - [ ] Format: Full bash script with comments

- [ ] Lesson 8: 2 orchestration examples
  - [ ] Multi-step workflow outline
  - [ ] Batch operation example
  - [ ] Format: Pseudocode or bash script

- [ ] All examples: Cross-platform (work on macOS, Linux, WSL Windows)
- [ ] All examples: Type hints/comments for clarity
- [ ] All examples: Runnable (not pseudocode) where applicable
- [ ] All examples: Include expected output or validation step

---

#### Task 4.2: Create Visual Aids and Diagrams

**Priority**: SHOULD-HAVE | **Effort**: 2h | **Owner**: Visual Designer

**Description**: Create diagrams and visual aids to clarify concepts, especially for abstract ideas.

**Acceptance Criteria**:
- [ ] Lesson 1: File system hierarchy diagram (home directory structure)
- [ ] Lesson 2: 5-step safety pattern flowchart with decision points
- [ ] Lesson 2: Command breakdown annotation (showing command parts with colors)
- [ ] Lesson 3: Command structure diagram (command + arguments + flags labeled)
- [ ] Lesson 3: Pipe visualization (showing data flow through commands)
- [ ] Lesson 4: Copy vs. Move decision tree
- [ ] Lesson 6: Search result analysis flowchart
- [ ] Lesson 7: Script development workflow (specify → write → understand → test → execute)
- [ ] Lesson 8: Orchestration decomposition template (visual)

- [ ] Format: SVG, ASCII art, or web-friendly images
- [ ] All diagrams: Accessible (alt text, clear labels)
- [ ] All diagrams: Consistent style with rest of chapter
- [ ] Grade 7-8 reading level on any text in diagrams

---

### Phase 5: Review and Integration (Week 6-7)

#### Task 5.1: Pedagogical Coherence Review

**Priority**: MUST-HAVE | **Effort**: 2h | **Owner**: Pedagogical Reviewer

**Description**: Review chapter for pedagogical coherence, learning objective alignment, and scaffolding appropriateness.

**Acceptance Criteria**:
- [ ] All learning objectives are measurable and Bloom's-level appropriate
- [ ] Lesson sequence is logical and builds progressively
- [ ] Prerequisites for each lesson are satisfied by prior lessons
- [ ] Safety pattern is reinforced in every applicable lesson
- [ ] Tier 1/2/3 teaching progression is clear and maintained
- [ ] Cognitive load per lesson is within limits (5 concepts A1, 7 concepts A2, 10 concepts B1)
- [ ] Real dialogues support stated learning objectives
- [ ] Practice checkpoints align with objectives
- [ ] "Try With AI" activities test understanding of lesson concepts
- [ ] Chapter maintains consistent voice and tone
- [ ] Reading level is Grade 7-8 throughout
- [ ] Peer review comments addressed

**Validation Method**:
- [ ] Pedagogical review checklist completed
- [ ] Reviewer confirms or requests changes
- [ ] All changes tracked and approved

---

#### Task 5.2: Content Style and Standards Review

**Priority**: MUST-HAVE | **Effort**: 1.5h | **Owner**: Technical Editor

**Description**: Review for consistency with project style guides and output standards.

**Acceptance Criteria**:
- [ ] All YAML frontmatter matches `.claude/output-styles/chapters.md` exactly
- [ ] All lesson files follow naming convention: `NN-descriptive-name.md`
- [ ] All Markdown formatting is consistent (headings, code blocks, lists, links)
- [ ] All code blocks have language identifier (bash, python, etc.)
- [ ] All real dialogue transcripts are consistently formatted
- [ ] No orphaned sections or incomplete content
- [ ] Internal links use correct relative paths (if cross-referencing)
- [ ] All images/diagrams referenced are in appropriate location
- [ ] Spelling and grammar: Zero errors (automated checker + manual review)
- [ ] Terminology consistent with rest of book

**Tools Used**:
- [ ] Automated: Markdown linter, spell checker
- [ ] Manual: Line-by-line review for consistency

---

#### Task 5.3: Technical Accuracy Review

**Priority**: MUST-HAVE | **Effort**: 2-2.5h | **Owner**: Technical Reviewer

**Description**: Verify all bash commands, concepts, and AI behavior descriptions are technically accurate and current.

**Acceptance Criteria**:
- [ ] All bash commands work on macOS, Linux, and WSL Windows
- [ ] All command syntax is accurate for bash 4.0+
- [ ] All file system examples use correct path conventions
- [ ] All `.env` and `.gitignore` examples follow best practices
- [ ] Security recommendations (secrets management) are current best practices
- [ ] AI behavior descriptions match actual behavior of tools mentioned
- [ ] Python code examples work and follow best practices
- [ ] All prerequisite chapters (5, 6) are accurately referenced
- [ ] All dependent chapters (8+) dependencies are accurate
- [ ] Cross-platform compatibility notes are correct (Windows WSL, macOS, Linux)

**Validation Method**:
- [ ] Run all commands on actual system (macOS + Linux/WSL)
- [ ] Verify AI tool behavior with actual tools
- [ ] Check best practices against official documentation
- [ ] Document any assumptions or system-specific notes

---

#### Task 5.4: Accessibility and Inclusivity Review

**Priority**: SHOULD-HAVE | **Effort**: 1.5h | **Owner**: Accessibility Reviewer

**Description**: Ensure content is accessible to diverse learners and uses inclusive language.

**Acceptance Criteria**:
- [ ] All images/diagrams have alt text
- [ ] Text explanations don't rely solely on images
- [ ] Code examples are readable (font size, contrast)
- [ ] No color-only distinctions (for color-blind users)
- [ ] Language is gender-neutral and inclusive
- [ ] No jargon without definition
- [ ] Examples include diverse names and contexts
- [ ] Tone is welcoming and non-judgmental
- [ ] No gatekeeping language ("real developers", "just use")

---

#### Task 5.5: Integration with Chapter Index and Directory

**Priority**: MUST-HAVE | **Effort**: 0.5h | **Owner**: Project Coordinator

**Description**: Verify chapter is correctly integrated into book structure and chapter index.

**Acceptance Criteria**:
- [ ] All 8 lesson files exist in correct directory: `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/`
- [ ] Chapter README exists at: `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/readme.md`
- [ ] All lesson YAML frontmatter has correct `sidebar_position` (1-8)
- [ ] Chapter appears correctly in `specs/book/chapter-index.md`
- [ ] All internal links are valid (relative paths)
- [ ] Cross-chapter references to Chapters 5-6 and 8 are valid
- [ ] Docusaurus build succeeds with no warnings about this chapter

---

### Phase 6: Final Polish and Publication Preparation (Week 7)

#### Task 6.1: Proofread and Final Copy Edit

**Priority**: MUST-HAVE | **Effort**: 2-3h | **Owner**: Copy Editor

**Description**: Final proofread for grammar, spelling, consistency, and polishing prose.

**Acceptance Criteria**:
- [ ] Zero spelling errors (verified with automated checker)
- [ ] Zero grammatical errors
- [ ] Consistent tone and voice throughout
- [ ] Sentence variety and pacing maintained
- [ ] Active voice preferred (minimal passive)
- [ ] No redundant phrasing
- [ ] Transitions between sections smooth
- [ ] Dialogue transcripts have consistent formatting
- [ ] All code formatted consistently
- [ ] Hyperlinks tested and working

**Tools**:
- [ ] Automated: Grammarly or similar
- [ ] Manual: Line-by-line reading by experienced editor

---

#### Task 6.2: Build and Visual Validation

**Priority**: MUST-HAVE | **Effort**: 1h | **Owner**: QA

**Description**: Build Docusaurus locally and validate visual appearance of chapter.

**Acceptance Criteria**:
- [ ] Docusaurus build succeeds with no errors
- [ ] No build warnings specific to this chapter
- [ ] Chapter renders correctly in web view
- [ ] Sidebar navigation shows all 8 lessons correctly ordered
- [ ] Images/diagrams display correctly
- [ ] Code blocks are readable and syntax-highlighted
- [ ] Links render correctly (both internal and external)
- [ ] Mobile view is readable and usable
- [ ] Page load time is acceptable

**Tools**:
- [ ] Docusaurus local build: `npm run start`
- [ ] Browser inspection: Chrome DevTools
- [ ] Mobile testing: Chrome DevTools responsive mode

---

#### Task 6.3: Final Approval and Sign-Off

**Priority**: MUST-HAVE | **Effort**: 0.5h | **Owner**: Chapter Director

**Description**: Final review and approval for publication.

**Acceptance Criteria**:
- [ ] All MUST-HAVE tasks completed
- [ ] All critical reviews passed (pedagogical, technical, accessibility)
- [ ] No critical issues remaining
- [ ] All non-critical feedback addressed or deferred
- [ ] Content ready for publication
- [ ] Sign-off documented with approval date

---

## Acceptance Criteria (Definition of Done)

**All Chapters**:
- [ ] All MUST-HAVE tasks completed (100% of critical items)
- [ ] All 8 lesson files written with YAML frontmatter correct
- [ ] Learning objectives are measurable and Bloom's-aligned
- [ ] Content demonstrates Tier 1/2/3 graduated teaching progression
- [ ] Chapter integrates core domain skills contextually (natural language direction, supervision, validation)
- [ ] Output style matches chapter type requirements (technical/practical, beginner tier)
- [ ] Accessibility requirements met (clear language, inclusive examples, alternative text)
- [ ] Each lesson ends with "Try With AI" section ONLY (no separate Key Takeaways or What's Next)
- [ ] "Try With AI" tool selection documented (Claude Code, Gemini CLI, ChatGPT Code Interpreter)

**This Chapter Specifically**:
- [ ] 25-27 real dialogue transcripts included showing authentic AI behavior
- [ ] Safety pattern (5 steps) integrated into all relevant lessons
- [ ] Natural language-first pedagogy maintained throughout
- [ ] No command memorization encouraged; reading/supervision skills taught instead
- [ ] All bash commands verified as cross-platform (macOS, Linux, WSL)
- [ ] Real-world scenarios and error cases demonstrated
- [ ] Graduated complexity progression is clear and maintainable
- [ ] All practice checkpoints provide measurable validation

---

## Effort Estimation Summary

| Phase | Tasks | Hours | Story Points |
|-------|-------|-------|--------------|
| Phase 1: Foundation | 2 tasks | 4-6 | 1 |
| Phase 2: Content Writing (Lessons) | 14 content tasks | 35-48 | 5-6 |
| Phase 2: Practice & AI Activities (Lessons) | 8 exercise tasks | 12-16 | 2-3 |
| Phase 3: Dialogue Collection & Validation | 2 tasks | 6-8 | 1 |
| Phase 4: Code Examples & Visuals | 2 tasks | 4-5 | 1 |
| Phase 5: Review & Integration | 5 tasks | 8-11 | 1-2 |
| Phase 6: Polish & Publication | 3 tasks | 3-5 | 1 |
| **TOTAL** | **36 tasks** | **72-99 hours** | **12-14 SP** |

**Schedule**: 2-3 weeks (assuming one full-time writer + supporting reviewers)

---

## Risks and Mitigations

### Risk 1: Real Dialogue Transcripts Take Longer Than Expected

**Likelihood**: Medium | **Impact**: Medium (delays publication)

**Mitigation**:
- Collect dialogues in parallel with content writing
- Use multiple AI tools to create variety efficiently
- If real transcripts delayed, use "realistic example" label temporarily; replace with real transcripts later

---

### Risk 2: Cognitive Load Validation Fails (Content Teaches More Than Specified)

**Likelihood**: Low | **Impact**: High (breaks beginner accessibility)

**Mitigation**:
- Pedagogical reviewer explicitly counts new concepts in each lesson
- Ruthlessly cut out non-essential content
- If concept count exceeds limit, move to appendix or split into additional lesson

---

### Risk 3: Cross-Platform Bash Compatibility Issues

**Likelihood**: Medium | **Impact**: Medium (breaks for Windows/macOS users)

**Mitigation**:
- Test all commands on macOS, Linux, and WSL Windows before finalizing
- Use only bash 4.0+ compatible commands
- Note any OS-specific differences in content

---

### Risk 4: AI Tool Behavior Varies Over Time

**Likelihood**: High | **Impact**: Low (content shows behavior, not promises)

**Mitigation**:
- Clearly label dialogues as "from [Tool Name] on [Date]"
- Frame content to acknowledge AI variability ("AI may respond differently")
- Teach students to expect and handle variation

---

## Follow-Up Actions (After Implementation)

1. **Docusaurus Deployment**: Deploy chapter to live book site after approval
2. **User Testing**: Optional: Collect feedback from beta readers in target audience
3. **Maintenance**: Monitor for tool updates that affect dialogue transcripts
4. **Expansion**: Consider adding Appendix with troubleshooting guide (common errors)

---

## Notes

**AI-Native Authenticity**: This chapter is intentionally different from traditional bash tutorials because it teaches students to **communicate intent and supervise AI** rather than **memorize syntax**. This aligns with the AI-native development philosophy outlined in the project constitution.

**Safety is Non-Negotiable**: The 5-step safety pattern appears in Lesson 2 and is reinforced in every lesson involving file operations. This is intentional and critical to prevent catastrophic mistakes.

**Real Dialogues are Essential**: Synthetic examples don't demonstrate real AI behavior (errors, limitations, clarifying questions). Real transcripts show students what to expect and how to supervise effectively.

**Graduated Progression**: Lessons 1-2 (Tier 1) → Lessons 3-5 (Tier 2) → Lessons 6-8 (Tier 3) mirrors how students' autonomy and responsibility increases. Early lessons are more scaffolded; later lessons shift responsibility to students.
