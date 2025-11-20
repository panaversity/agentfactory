# Chapter 7: Bash Essentials for AI-Driven Development — Lesson Plan

**Chapter Type**: Technical/Practical (AI-Native Beginner Tier — Tier 1 & 2 Graduated Teaching)

**Chapter Title**: Bash Essentials for AI-Driven Development

**Chapter Number**: 7 | **Part**: 2 (AI Tool Landscape) | **Complexity Tier**: Beginner (Tier 1)

**Target Audience**: Absolute beginners with zero terminal/bash experience, having completed Chapters 5-6 (Claude Code and Gemini CLI)

**Estimated Total Time**: ~5 hours (40 minutes per lesson × 8 lessons, plus exercises and AI practice)

**Prerequisites**:
- Chapter 5: Claude Code features and workflows
- Chapter 6: Gemini CLI installation and basics
- Students have access to a terminal and one AI tool (Claude Code, Gemini CLI, or ChatGPT Code Interpreter)
- Basic comfort with conversations and copy-pasting text

**Pedagogical Approach**:
- **Tier 1 (Foundational)**: Lessons 1-2 teach concepts directly through book explanations, dialogues, and safe exploration
- **Tier 2 (AI Companion)**: Lessons 3-5 use AI to execute complex syntax while students specify intent and observe
- **Tier 3 (Orchestration)**: Lessons 6-8 use AI to automate multi-step workflows while students supervise and validate
- **Safety-First**: Every lesson involving file operations includes the 5-step safety pattern (Ask → Explain → Understand → Verify → Execute)
- **Natural Language First**: Students communicate bash needs in plain English; AI translates to commands

---

## Section/Lesson Architecture

### Lesson 1: Finding Your Way — Terminal Navigation Fundamentals

**Duration**: 35 minutes

**Sidebar Position**: 1

**Learning Objectives** (Bloom's: Understand/Remember):
- Understand what a terminal is and why it matters for AI developers
- Recognize what file system location means and why it matters before any operation
- Identify key navigation concepts: working directory, absolute vs. relative paths
- Explain in plain English what "I'm in this directory" means in practical terms
- Predict how changing directories affects what files are accessible

**Skills Taught** (using skills-proficiency-mapper):
1. **Terminal Literacy** — A1 — Technical — Student can define "terminal," explain why it's used, and recognize bash is a tool AI collaborators use
2. **File System Orientation** — A1 — Conceptual — Student understands that files are organized in directories (folders) with a current location (working directory)
3. **Location Awareness** — A1 — Technical — Student can ask "Where am I?" in conversations and understand directory paths as locations, not abstract concepts

**Core Concepts** (≤5 for A1):
1. What is a terminal and why developers use it
2. File systems have a current location (working directory)
3. Absolute paths start from root (`/`); relative paths start from current location
4. Navigation changes what files are accessible
5. AI tools can help navigate through natural language conversation

**Teaching Tier**: 1 (Book teaches directly)

**Content Outline**:
- **Section A: The Terminal — What It Is and Why It Matters**
  - Explain terminal as a text-based interface to the file system
  - Why developers use terminals: speed, precision, scripting, AI collaboration
  - Real analogy: Terminal is like standing in a filing cabinet; you can see and access only what's in your current location
  - Frame: "You're learning to navigate with AI help, not memorizing commands"

- **Section B: Your Starting Point — Understanding Working Directory**
  - Concept: You always start somewhere (home directory `/Users/username` on macOS, `/home/username` on Linux)
  - Why it matters: You can't move/copy/delete files if you don't know where you are
  - Real example dialogue: Show authentic conversation where student asks AI "Where am I right now?" and AI executes `pwd`, explains the path

- **Section C: Asking the Right Questions**
  - Teach students natural language questions to ask AI:
    - "Show me where I am right now"
    - "What files are in this directory?"
    - "How do I get to my Documents folder?"
    - "Navigate to my project folder and show me what's there"
  - Emphasize: No command syntax needed; students describe intent, AI translates

- **Section D: Reading File Listings with Understanding**
  - When AI executes `ls` or `ls -l`, what does the output mean?
  - Explain columns: permissions, owner, size, date, name
  - How to identify files vs. directories (directories have `/` suffix or specific indicators)
  - Real dialogue showing AI explaining file listing in plain English

**Content Elements**:
- **Real Dialogue Transcript 1**: Student asks "Where am I?" → AI executes `pwd`, shows `/Users/alice/Documents`, explains what this path means (home directory → Documents folder)
- **Real Dialogue Transcript 2**: Student asks "What files are in my project folder?" → AI shows `ls` output with explanation of each item (directories vs. files)
- **Visual Aid**: Simple file system diagram showing home directory hierarchy (home → Documents → projects)

**Practice Approach**:
- **Checkpoint 1**: Student conducts 3 back-and-forth conversations with AI about their own file system without typing any bash commands
  - Conversation 1: Locate current working directory
  - Conversation 2: List files in home directory and identify directories vs. files
  - Conversation 3: Navigate to a specific folder (e.g., Documents) and describe what files are there
- **Validation**: Student takes a screenshot of each conversation to document learning

**Prerequisites**: None (foundational lesson)

**Cognitive Load Validation**:
- Concepts taught: 5 (terminal, working directory, absolute/relative paths, navigation, AI collaboration)
- New vocabulary: 4 terms (terminal, directory, path, bash)
- Fits within A1 limit (max 5 concepts)

**Engagement Elements**:
- Opening story: "Sarah opened the terminal for the first time and deleted her entire project folder. Here's what went wrong..." (foreshadows importance of understanding location)
- Real error examples: Show what happens when student tries to find files in wrong directory
- Relatable analogy: "Terminal is like talking to a chef in a restaurant. The chef needs to know which kitchen they're working in before they start cooking."

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-3 focused conversation starters):
1. "Show me where I am right now in the file system. Explain what that path means."
2. "Navigate to my Documents folder and tell me what files and folders are there. How do you know which items are folders vs. files?"
3. "I want to go back to my home directory. What does 'home directory' mean and how would you navigate there?"

**Expected Outcomes**:
- Student can explain in their own words what directory they're currently in
- Student can identify files vs. directories in a listing
- Student understands why location matters before any file operations

**Safety/Ethics Note**: This lesson builds foundation for safe file operations. Understanding location prevents catastrophic mistakes in future lessons.

---

### Lesson 2: The Safety Pattern — Never Lose Important Files Again

**Duration**: 40 minutes (intentionally longest lesson — safety is critical)

**Sidebar Position**: 2

**Learning Objectives** (Bloom's: Understand/Apply):
- Understand why file operations are dangerous and require a safety pattern
- Identify the 5-step safety pattern: Ask → Explain → Understand → Verify → Execute
- Apply the safety pattern to real file operations (copy, move, delete)
- Recognize red-flag commands (`rm -rf`, destructive flags) that require extra caution
- Explain to others why the safety pattern prevents catastrophic mistakes

**Skills Taught** (using skills-proficiency-mapper):
1. **Safety-First Mindset** — A1 — Soft — Student recognizes that understanding before execution is non-negotiable for file operations
2. **5-Step Safety Pattern** — A1 — Technical — Student can identify all 5 steps and apply them to any file operation
3. **Red Flag Recognition** — A1 — Conceptual — Student can recognize dangerous commands (`rm -rf`, `sudo`, `chmod 777`) and ask clarifying questions

**Core Concepts** (≤5 for A1):
1. File operations are irreversible (deleted files cannot be recovered)
2. The 5-step safety pattern: Ask → Explain → Understand → Verify → Execute
3. Each step has a specific purpose and cannot be skipped
4. Red-flag commands signal potential data loss
5. Professional developers ALWAYS verify before destructive operations

**Teaching Tier**: 1 (Book teaches directly)

**Content Outline**:
- **Section A: Why This Lesson Matters — Real Cost of Mistakes**
  - Real story (anonymized): Developer deleted 6 months of work in 5 seconds by running wrong command
  - Why it happened: Didn't understand location, didn't verify what would be deleted
  - Frame: "This lesson could save your career. It absolutely will prevent heartbreak."

- **Section B: The 5-Step Safety Pattern (With Real Dialogues)**
  - **Step 1: Ask** — Student asks AI to perform an operation (e.g., "Delete my temp folder")
    - Dialogue example showing this step
  - **Step 2: Explain** — AI explains exactly what will happen before executing
    - Shows current location
    - Lists files/folders that will be deleted
    - Explains what command will be used
    - Example dialogue transcript
  - **Step 3: Understand** — Student asks clarifying questions
    - "What does `-rf` mean?" (recursive, force)
    - "Can I undo this if something goes wrong?" (no, deleted is gone)
    - "Are you sure you're deleting the right folder?" (confirmation)
    - Example dialogue showing student asking good questions
  - **Step 4: Verify** — Student confirms they're ready
    - Explicit "Yes, do it" or "Wait, I'm not sure" response
    - Example showing student saying "I see you're in `/Users/alice/project`, and you're deleting only the `temp` folder. That's correct, proceed."
  - **Step 5: Execute** — AI runs the command only after explicit confirmation
    - Shows command executed
    - Shows result (confirmation that operation succeeded)
    - Example dialogue

- **Section C: Red Flags That Require Extra Caution**
  - `rm` (remove) — permanently deletes files
  - `rm -rf` — recursively deletes entire directories (DANGEROUS)
  - `sudo` — runs as administrator (can damage system)
  - `chmod 777` — opens files to everyone (security risk)
  - How to respond when you see red flags: "What does this do? Why do we need it? What could go wrong?"

- **Section D: Common Mistakes and How to Avoid Them**
  - Mistake 1: Assuming AI knows what you want to delete (IT DOESN'T)
    - How to catch: "Are you sure you're deleting the right thing?"
  - Mistake 2: Not understanding the difference between `-r` and `-rf`
    - How to catch: "What's the difference between these flags?"
  - Mistake 3: Skipping verification because operation feels small
    - How to catch: "Apply safety pattern to EVERY operation, no exceptions"
  - Mistake 4: Deleting from wrong directory
    - How to catch: "Confirm current location BEFORE executing"

**Content Elements**:
- **Real Dialogue Transcript 1** (Full 5-Step Example): Student wants to delete old project → Dialogue showing all 5 steps → Result (successful deletion)
- **Real Dialogue Transcript 2** (Caught Mistake): Student almost deletes wrong folder → Step 3 catches it → Operation cancelled → Crisis averted
- **Red Flag Dialog**: Student sees `rm -rf` command → Asks clarifying questions → AI explains each part → Student feels confident proceeding
- **Visual Aid**: Flowchart of 5-step safety pattern with real decision points
- **Example Command Breakdown**: `rm -rf ~/projects/old_project/` broken down into plain English:
  - `rm` = remove (delete)
  - `-rf` = recursive (entire folder) and force (no confirmations)
  - `~/projects/old_project/` = this specific folder
  - Plain English: "Delete the entire 'old_project' folder and everything inside it, immediately"

**Practice Approach**:
- **Checkpoint 1**: Students identify missing safety steps in flawed dialogues
  - Scenario: Dialogue where AI suggests deletion without explaining what will be deleted
  - Task: "What step is missing? How would you fix this conversation?"
  - Expected answer: Step 2 (Explain) — need to show location and list files

- **Checkpoint 2**: Students identify red flags in commands
  - Prompt: "I want to execute `sudo rm -rf /var/www/old_site`. What red flags do you see?"
  - Expected answer: 2 red flags (`sudo` and `rm -rf`) — requires extra verification

- **Checkpoint 3**: Supervised deletion exercise
  - Create a practice folder with known contents
  - Direct AI to delete it following all 5 steps
  - Validate that correct folder was deleted
  - Screenshot conversation showing all 5 steps

**Prerequisites**: Lesson 1 (location awareness)

**Cognitive Load Validation**:
- Concepts: 5 (irreversibility, 5 steps, red flags, confirmation, execution)
- New vocabulary: 6 terms (destructive, recursive, force, flag, execute, verify)
- Fits within A1 limit

**Engagement Elements**:
- Opening: Real (anonymized) story of data loss
- Emotional appeal: "Your future self will thank you for being careful now"
- Practical motivation: "This is the single most important lesson in this chapter"
- Humor: Comic strip showing before/after of developer who skipped safety pattern
- Relatable: "Even experts use this pattern — it's not paranoia, it's professionalism"

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-4 hands-on safety pattern practice):
1. "Help me delete a test folder following the 5-step safety pattern. Show me exactly what will happen before you execute the command."
2. "I want to move some files from my Documents folder to a backup location. Walk me through all 5 steps so I'm confident this is safe."
3. "Explain what this command does: `find . -type f -name '*.tmp' -delete`. Are there any red flags? What could go wrong?"
4. "Create a practice scenario: I have a folder with 100 files and I want to delete files older than 30 days. Show me how you'd approach this safely."

**Expected Outcomes**:
- Student can identify all 5 steps of the safety pattern
- Student can recognize red-flag commands and ask clarifying questions
- Student can conduct a full safety-pattern conversation for any destructive operation
- Student understands why safety pattern is non-negotiable

**Safety/Ethics Note**: This is NOT optional. Every file operation in future lessons reinforces this pattern. Skipping steps is never acceptable.

---

### Lesson 3: Reading and Understanding Bash Commands

**Duration**: 35 minutes

**Sidebar Position**: 3

**Learning Objectives** (Bloom's: Understand/Apply):
- Recognize basic bash command structure (command, arguments, flags, pipes, redirects)
- Interpret flags (`-l`, `-a`, `-h`) without memorizing them
- Understand pipes (`|`) as connecting commands together
- Explain what any command will do by reading its structure
- Ask clarifying questions when command is unclear

**Skills Taught** (using skills-proficiency-mapper):
1. **Command Structure Recognition** — A2 — Technical — Student can read command syntax and predict what will happen, asking for clarification on unfamiliar flags
2. **Pipe and Redirect Understanding** — A2 — Technical — Student can explain what pipes (`|`) do and how they chain commands
3. **Supervision Skills** — A2 — Soft — Student can read command before execution and confidently say "yes, that's what I want" or "wait, explain X first"

**Core Concepts** (≤7 for A2):
1. Commands have a structure: command + arguments + flags
2. Flags modify behavior (usually start with `-` or `--`)
3. Arguments specify what the command operates on
4. Pipes (`|`) connect output of one command to input of another
5. Redirects (`>`, `>>`, `<`) send output to files or read from files
6. Understanding intent matters more than memorizing syntax
7. Asking clarifying questions is a supervision skill

**Teaching Tier**: 2 (AI Companion)

**Content Outline**:
- **Section A: Anatomy of a Bash Command**
  - Real command example: `ls -lah ~/Documents`
  - Break down into parts:
    - `ls` = command (list files)
    - `-lah` = flags (long format, all files, human-readable sizes)
    - `~/Documents` = argument (which directory)
  - Plain English translation: "List all files in my Documents folder with sizes shown in readable format"
  - Why structure matters: AI expects specific order; understanding structure helps supervise

- **Section B: Understanding Flags (Without Memorizing)**
  - Key insight: You don't need to memorize flags; you need to understand what they do
  - Common patterns:
    - `-l` = long format (show more details)
    - `-a` = all (include hidden files)
    - `-h` = human-readable (show sizes as KB, MB, not bytes)
    - `-r` = recursive (apply to all subdirectories)
    - `-f` = force (do it without asking)
  - How to supervise: When AI suggests command, ask "What does `-rf` mean?" and understand explanation
  - Real dialogue: Student sees `find . -name "*.py" -type f` and asks AI to explain each part

- **Section C: Pipes and Redirects**
  - Pipes (`|`) connect commands: output of first command becomes input to second
  - Example: `cat file.txt | grep "error" | wc -l`
    - Plain English: "Read file, filter for 'error', count matching lines"
  - Redirects:
    - `>` sends output to file (creates or overwrites)
    - `>>` appends output to file (adds to end)
    - `<` reads input from file
  - When to use redirects: Save results to file for later use
  - Real dialogue: AI proposes pipeline, student asks "Walk me through each step"

- **Section D: Asking Clarifying Questions (Supervision Strategy)**
  - Good questions when you don't understand:
    - "What does this flag do?"
    - "Why do we need this part?"
    - "What happens if we remove `-r`?"
    - "What will this command output?"
    - "Can you show me the result before executing?"
  - Bad habit: Just running commands without understanding
  - Good habit: "I understand what you're proposing, proceed"

**Content Elements**:
- **Command Breakdown Examples** (5-6 real commands):
  - `ls -lah` (list files in detail)
  - `find . -name "*.py"` (find Python files)
  - `grep -r "import" .` (search for text in files)
  - `cat file.txt | sort` (read and sort)
  - `mkdir -p ~/projects/new_project` (create nested directories)
- **Real Dialogue**: Student sees command → Asks "What does this do?" → AI explains each part → Student confirms understanding → Execution
- **Visual Guide**: Color-coded command breakdown showing command/args/flags
- **Common Patterns Poster**: Show 10 common command patterns students will encounter

**Practice Approach**:
- **Checkpoint 1**: Students read commands and explain in plain English
  - Given: `find . -type f -name "*.md" | wc -l`
  - Expected: "This finds all markdown files and counts how many there are"

- **Checkpoint 2**: Students identify what flags do
  - Given: `ls -lah`
  - Task: "Without looking anything up, explain what each flag does and why we'd use it"
  - Validation: Ask AI to confirm understanding

- **Checkpoint 3**: Pipe comprehension
  - Given: `cat requirements.txt | grep "django" | wc -l`
  - Task: "Explain this command in plain English, step by step"
  - Expected: "Read requirements file, filter for 'django' lines, count how many"

**Prerequisites**: Lessons 1-2

**Cognitive Load Validation**:
- Concepts: 7 (command structure, flags, arguments, pipes, redirects, supervision, understanding intent)
- Fits within A2 limit (max 7 concepts)

**Engagement Elements**:
- Frame: "You're not learning bash syntax; you're learning to read commands like a detective"
- Relatable: "Professional developers don't memorize command syntax either. They understand structure and ask clarifying questions"
- Confidence builder: "If you can read this command, you can supervise AI execution safely"

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-3 focused comprehension exercises):
1. "Show me a command that finds all Python files in my current directory. Explain each part so I understand exactly what it will do."
2. "I want to search for a specific word in all my text files. Walk me through the command you'd use, explaining flags and pipes."
3. "Create a command that lists all files modified in the last 7 days. Explain what each part does and why."

**Expected Outcomes**:
- Student can read a bash command and explain what it does in plain English
- Student can identify flags and ask "What does this do?"
- Student understands pipes as connecting commands
- Student feels confident supervising command execution

**Safety/Ethics Note**: This skill directly enables supervision. Students who can read commands can catch mistakes before execution.

---

### Lesson 4: File Operations Through Natural Language — Copy, Move, and Organize

**Duration**: 38 minutes

**Sidebar Position**: 4

**Learning Objectives** (Bloom's: Apply/Analyze):
- Direct AI to perform file operations (copy, move) using natural language
- Apply the 5-step safety pattern to each operation
- Analyze which operation is appropriate for different scenarios
- Validate that operations completed successfully
- Explain why certain operations require backups or extra verification

**Skills Taught** (using skills-proficiency-mapper):
1. **File Organization** — A2 — Technical — Student can direct AI to organize files into folders using natural language and understand what `cp`, `mv`, `mkdir` do
2. **Backup Thinking** — A2 — Conceptual — Student understands why backups matter and proactively asks "Should we create a backup first?"
3. **Operation Validation** — A2 — Technical — Student can verify operations completed correctly by listing directories, checking file counts, etc.

**Core Concepts** (≤7 for A2):
1. Copy operations preserve originals (safe to experiment)
2. Move operations remove originals (more dangerous, requires extra caution)
3. Directories can be organized hierarchically
4. File operations should always have a backup plan
5. Validation means checking that operation succeeded
6. Understanding intent matters more than memorizing `cp` vs `mv` syntax
7. Asking "Should we backup first?" is professional practice

**Teaching Tier**: 2 (AI Companion)

**Content Outline**:
- **Section A: Copy Operations — The Safe Experiment**
  - Frame: Copy is low-risk; original stays intact
  - When to use: Creating backups, duplicating files, organizing copies
  - Real scenario: Student has project folder, wants to copy to backup location
  - Dialogue: "Create a backup copy of my project folder in a safe location"
  - AI explains what will happen (shows source and destination)
  - Apply 5-step safety pattern
  - Validate: List directory to confirm backup exists

- **Section B: Move Operations — Requires Caution**
  - Frame: Move is higher-risk; removes original, so requires verification
  - When to use: Reorganizing files into proper locations, archiving old work
  - Real scenario: Student has scattered project files, wants to organize
  - Dialogue: "Move all my Python files into a 'python-projects' folder"
  - AI explains what will happen (shows what will be moved)
  - Apply 5-step safety pattern (EXTRA caution here)
  - Validate: Confirm files are in new location, check original location is empty

- **Section C: Creating and Organizing Directories**
  - Using natural language: "Create a folder structure for my projects"
  - AI suggests structure and creates it
  - Common patterns: `~/projects/2025-projects/`, `~/archive/old_work/`, `~/active/feature-branches/`
  - Validation: List new directories to confirm structure created correctly

- **Section D: The Backup Question**
  - Professional practice: Always ask "Should we backup first?"
  - When backup is critical: Before moving important files, before any large-scale operation
  - How backups work: Simple copy to separate location
  - Real dialogue showing student asking "What if we make a mistake? Should we backup first?" and AI responding positively

**Content Elements**:
- **Real Dialogue 1**: Copy operation
  - Student: "Create a backup of my project folder"
  - AI: Lists source, proposes destination, explains command
  - Student: Asks backup questions
  - Result: Backup created, validated

- **Real Dialogue 2**: Move operation with catch
  - Student: "Move all my old Python scripts to an archive folder"
  - AI: Asks clarifying question "How do I know which scripts are 'old'?"
  - Student: Refines request
  - AI: Explains operation and validates before executing
  - Result: Move completes successfully

- **Real Dialogue 3**: Backup saves the day
  - Student: "Let me move these files" (with no backup first)
  - AI: Suggests creating backup first
  - Student: Agrees and follows advice
  - Later: Student realizes mistake, but backup exists
  - Message: "Backup thinking prevents disasters"

- **File Operation Decision Matrix**:
  - Is original needed after operation? If YES → Copy. If NO → Move.
  - Is operation easily reversible? If YES → Can skip backup. If NO → Create backup first.
  - Are files critical/irreplaceable? If YES → Always backup. If NO → Probably safe.

**Practice Approach**:
- **Checkpoint 1**: Students direct AI to copy files
  - Scenario: Create a backup of home directory
  - Task: Use natural language, apply safety pattern, validate result

- **Checkpoint 2**: Students direct AI to move files (with caution)
  - Scenario: Organize scattered files into folders
  - Task: Ask about backups first, apply safety pattern, validate

- **Checkpoint 3**: Students analyze which operation fits scenario
  - Given: Various file management needs
  - Task: "Would you copy or move for this scenario? Why? Do we need a backup?"

**Prerequisites**: Lessons 1-3 (location, safety, command understanding)

**Cognitive Load Validation**:
- Concepts: 7 (copy, move, backup, hierarchy, validation, intent, professional practice)
- Fits within A2 limit

**Engagement Elements**:
- Story: "Marcus didn't backup before reorganizing. One mistake, and 3 months of work was scattered."
- Confidence builder: "You're not memorizing `cp` and `mv`. You're learning to think like a professional."
- Empowerment: "After this lesson, you can organize your entire projects folder safely with AI help."

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-3 practical file operations):
1. "Help me create a backup of my documents folder. Walk me through the 5-step safety pattern so I'm confident this is correct."
2. "I want to organize my projects folder. Create a folder structure and move projects into it. Should we backup first?"
3. "Show me how to move all .txt files from my Desktop to a Documents/text-files folder. Verify it worked correctly."

**Expected Outcomes**:
- Student can direct AI to copy/move files using natural language
- Student applies safety pattern to every operation
- Student validates operations completed correctly
- Student proactively asks about backups for important files
- Student can explain why copy vs. move matters

**Safety/Ethics Note**: Every file operation in this lesson must include safety pattern and validation. Reinforce: "Fast and reckless loses data. Careful and thorough keeps data safe."

---

### Lesson 5: Configuration and Secrets — Environment Files and Sensitive Data

**Duration**: 37 minutes

**Sidebar Position**: 5

**Learning Objectives** (Bloom's: Apply/Analyze):
- Create and configure environment files (`.env`) for sensitive data
- Understand why secrets (API keys) must never be committed to git
- Apply `.gitignore` patterns to prevent accidental commits
- Analyze security implications of different secret storage approaches
- Explain to others why environment variables exist and how they work

**Skills Taught** (using skills-proficiency-mapper):
1. **Secrets Management Awareness** — A2 — Conceptual — Student understands that API keys, passwords, and tokens are secrets that must be protected and never committed
2. **Environment Configuration** — A2 — Technical — Student can direct AI to create `.env` files, understand dotenv patterns, and recognize environment variables
3. **Git Security** — A2 — Conceptual — Student understands why `.gitignore` exists and proactively uses it to prevent leaking secrets

**Core Concepts** (≤7 for A2):
1. Secrets (API keys, passwords, tokens) must be kept private, not in code
2. Environment variables allow code to access secrets without hardcoding them
3. `.env` files store local secrets and are excluded from git
4. `.gitignore` prevents accidental commits of sensitive files
5. Public repositories are truly public (GitHub search can find leaked keys)
6. Professional workflows always separate code from configuration
7. AI can help set up secure configuration patterns safely

**Teaching Tier**: 2 (AI Companion)

**Content Outline**:
- **Section A: The Secret Disaster**
  - Real story: Developer hardcoded AWS credentials in code, pushed to GitHub, credentials stolen within hours
  - Why it happened: Didn't know secrets belong in environment variables
  - Cost: Thousands in fraudulent charges
  - Frame: "This lesson prevents catastrophic breaches"

- **Section B: Environment Variables and `.env` Files**
  - Concept: Code reads from environment, not from hardcoded values
  - Python example: `api_key = os.getenv('OPENAI_API_KEY')`
  - How it works: Code asks "What's my OPENAI_API_KEY?" → System looks in environment → Returns value
  - Local development: `.env` file stores values locally (never committed)
  - Real dialogue: Student asks "Store my OpenAI API key securely" → AI creates `.env` file, adds key, explains how code accesses it

- **Section C: `.gitignore` — Preventing Accidental Commits**
  - What is `.gitignore`: File that tells git "Never commit these files"
  - Why it exists: Prevents accidental commits of `.env`, credentials, passwords
  - Common patterns: `*.env`, `.env`, `*.key`, `secrets/`, etc.
  - Real scenario: AI sets up `.gitignore` to exclude `.env` file
  - Validation: Student confirms `.env` is listed in `.gitignore` and won't be committed

- **Section D: Package Installation and Dependencies**
  - Related concept: Installing packages requires setting up environment
  - Real scenario: Student wants to install dependencies from `requirements.txt`
  - Dialogue: "Install the dependencies for this Python project"
  - AI explains what packages are being installed, asks for confirmation
  - Safety check: Validate that `requirements.txt` exists and hasn't been tampered with

**Content Elements**:
- **Real Dialogue 1**: Setting up secure API key storage
  - Student: "I have an OpenAI API key I need to use in my project. How do I keep it safe?"
  - AI: Creates `.env` file, adds key, updates `.gitignore`
  - AI: Shows code example accessing key via environment variable
  - Student: Understands why this is safer than hardcoding

- **Real Dialogue 2**: Preventing accidental commits
  - Student: "I accidentally added my `.env` file to git. How do I remove it?"
  - AI: Explains how to undo commit, remove from history, update `.gitignore`
  - Teaching point: Even if removed, committed secrets might be exposed (regenerate keys)

- **Real Dialogue 3**: Installing dependencies safely
  - Student: "Install the packages this project needs"
  - AI: Shows what will be installed, asks for confirmation
  - Student: Reviews list and approves
  - AI: Installs using `uv sync` or `pip install -r requirements.txt`
  - Result: All dependencies installed, project ready to run

- **Secret Storage Decision Tree**:
  - Is secret used locally during development? → Store in `.env` (excluded from git)
  - Is secret needed in production? → Use secure secret manager (AWS Secrets, CloudFlare, etc.)
  - Is secret hardcoded in source? → EMERGENCY: Regenerate key immediately

**Practice Approach**:
- **Checkpoint 1**: Students set up secure `.env` file
  - Scenario: Create `.env` file with sample API key
  - Task: Ask AI to create file, update `.gitignore`, verify setup
  - Validation: Check that `.env` is git-ignored

- **Checkpoint 2**: Students analyze security risk
  - Given: Code with hardcoded password
  - Task: "What's wrong with this? How would you fix it?"
  - Expected: Recognize it's a secret, move to environment, use `.env`

- **Checkpoint 3**: Students install dependencies
  - Scenario: Project with `requirements.txt`
  - Task: Direct AI to install dependencies, validate installation

**Prerequisites**: Lessons 1-4 (navigation, safety, commands, file operations)

**Cognitive Load Validation**:
- Concepts: 7 (secrets, environment variables, `.env`, `.gitignore`, hardcoding, configuration, package management)
- Fits within A2 limit

**Engagement Elements**:
- Fear appeal (appropriate): "Every day, attackers scan GitHub looking for exposed API keys"
- Confidence builder: "After this lesson, your secrets are safe and you understand why"
- Real impact: "This one habit prevents the catastrophic breach that ends careers"

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-3 secure configuration exercises):
1. "I have an API key I need to use in my Python project. Walk me through how to store it securely in a `.env` file. How does my code access it?"
2. "Set up my `.env` file for this project and update `.gitignore` to prevent accidental commits. Show me why this setup is important."
3. "Show me a Python project that reads API keys from environment variables. Explain why this approach is safer than hardcoding secrets."

**Expected Outcomes**:
- Student can create and configure `.env` files
- Student understands why secrets must be environment variables
- Student can update `.gitignore` to prevent commits of sensitive files
- Student recognizes hardcoded secrets as a security emergency
- Student explains to others "My secrets are safe because..."

**Safety/Ethics Note**: This is security education. The choices students make in this lesson determine whether their projects are secure or vulnerable to compromise.

---

### Lesson 6: Exploring the File System Safely — Using AI to Search and Understand

**Duration**: 36 minutes

**Sidebar Position**: 6

**Learning Objectives** (Bloom's: Apply/Analyze):
- Use AI to search for files across the file system safely
- Understand what `find`, `grep`, and `ls` commands reveal about your project
- Analyze file organization to identify what's missing or what should be archived
- Validate search results to ensure operations are safe before scaling up
- Explain why exploration before action prevents mistakes

**Skills Taught** (using skills-proficiency-mapper):
1. **Safe File Discovery** — A2 — Technical — Student can ask AI to find files (by name, type, content) and understand results without getting lost
2. **Pattern Recognition** — A2 — Conceptual — Student can analyze search results and identify patterns (e.g., "I have 200 `.bak` files I should clean up")
3. **Exploration-Before-Action** — A2 — Soft — Student proactively explores before attempting large-scale operations, preventing mistakes

**Core Concepts** (≤7 for A2):
1. File systems can be large; you need AI to search efficiently
2. `find` command searches by name, type, modification date
3. `grep` command searches inside files for specific text
4. `ls` with options shows different views of directory contents
5. Search results reveal patterns that guide cleanup/organization decisions
6. Always explore first, then act (small scale before large scale)
7. Validation of search results prevents mistaken deletions

**Teaching Tier**: 2 (AI Companion)

**Content Outline**:
- **Section A: Finding Files by Name and Type**
  - Use case: "How many Python files do I have in my projects folder?"
  - Dialogue: Student asks, AI searches using `find`, shows results
  - Breaking it down: AI explains what search looked for and why it's useful
  - Example results: Find all `.py` files, all `.bak` backup files, all directories with 'old' in name
  - Safety angle: Small searches before large operations (e.g., find 20 files before deleting all)

- **Section B: Searching Inside Files**
  - Use case: "Where did I use `import os` in my projects?"
  - Dialogue: Student asks, AI searches using `grep`, shows files matching pattern
  - Breaking it down: AI shows which files contain the pattern, counts matches
  - Practical example: Find all TODO comments, find all references to deprecated library, find configuration values
  - Safety: Search helps understand impact before making changes

- **Section C: Understanding Patterns**
  - After search, students analyze results to identify patterns
  - Pattern examples: "I have 500 `.tmp` files that should be cleaned up", "I have duplicates of projects", "I haven't touched old-work/ in 2 years"
  - Decision making: Based on patterns, what should we do? (archive, backup, delete, ignore)
  - Dialogue showing: Search → analyze → decide → plan → execute

- **Section D: Validation of Search Results**
  - Before taking action (especially deletion), validate search caught right things
  - Validation techniques: Review results, sample a few files, count to check reasonableness
  - Real scenario: Found 200 files matching pattern. Are these really all the files we want to delete? Double-check.

**Content Elements**:
- **Real Dialogue 1**: Finding all Python files
  - Student: "How many Python files do I have in my projects?"
  - AI: Executes `find`, shows results, explains pattern
  - Result: "You have 42 Python files across 8 projects"

- **Real Dialogue 2**: Searching for patterns inside files
  - Student: "Where do I use the 'requests' library in my code?"
  - AI: Searches with `grep`, shows matching files
  - Result: "Found in 12 files across 3 projects; might be worth consolidating"

- **Real Dialogue 3**: Cleanup decision based on patterns
  - Student: "I have a lot of `.bak` and `.tmp` files. Should I delete them?"
  - AI: Searches to count them (1,200 files found)
  - Student: Surprised by count
  - AI: Shows sample to confirm they're safe to delete
  - Result: After validation, proceed with cleanup

- **Search Examples Poster**: Show 5-10 realistic search commands students will use

**Practice Approach**:
- **Checkpoint 1**: Students conduct exploratory search
  - Scenario: Find all files of certain type in home directory
  - Task: Ask AI to search, review results, explain what was found

- **Checkpoint 2**: Students analyze search results
  - Given: Results of search showing many files
  - Task: "What patterns do you see? What decisions does this suggest?"
  - Expected: Identify cleanup opportunities, archives needed, etc.

- **Checkpoint 3**: Students validate before deletion
  - Scenario: Want to delete temporary files
  - Task: Search first, review results, confirm pattern, then proceed
  - Safety check: Sample a few files to confirm they're really temporary

**Prerequisites**: Lessons 1-5 (navigation, safety, commands, file operations, configuration)

**Cognitive Load Validation**:
- Concepts: 7 (find, grep, ls, patterns, exploration, validation, decision-making)
- Fits within A2 limit

**Engagement Elements**:
- Discovery frame: "You're learning to explore your own file system like a detective"
- Practical benefit: "After this, you'll know exactly what's on your computer and be able to find things instantly"
- Empowerment: "Organized file system = faster development"

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-3 safe exploration exercises):
1. "Help me understand my file system. Find all Python files I have and show me where they are. What patterns do you notice?"
2. "Search for all TODO comments in my code. Where should I focus attention? What's missing or incomplete?"
3. "I want to clean up old backup files. Find all `.bak` and `.backup` files and show me where they are. Should I delete them?"

**Expected Outcomes**:
- Student can ask AI to search for files and understand results
- Student can analyze search results to identify patterns
- Student validates search results before taking action
- Student uses exploration to inform decisions (cleanup, archival, reorganization)
- Student feels confident supervising searches

**Safety/Ethics Note**: Exploration prevents mistakes. Encouraging "look before you act" throughout the lesson.

---

### Lesson 7: Building and Running Scripts — From Natural Language to Automation

**Duration**: 38 minutes

**Sidebar Position**: 7

**Learning Objectives** (Bloom's: Apply/Analyze/Evaluate):
- Direct AI to create simple bash scripts from natural language specifications
- Understand what scripts do and validate they work before running
- Identify when a script is useful (repetitive tasks, multi-step workflows)
- Evaluate whether AI-generated script is correct and safe
- Explain why automation through scripts saves time and prevents errors

**Skills Taught** (using skills-proficiency-mapper):
1. **Automation Thinking** — A2/B1 — Conceptual — Student recognizes when task is repetitive and asks "Could we automate this?" with AI's help
2. **Script Creation** — A2/B1 — Technical — Student can specify what a script should do and have AI write it; understands script is just commands in a file
3. **Script Validation** — A2/B1 — Technical — Student can test script, verify it works, and understand what it's doing before running on important data

**Core Concepts** (≤7 for A2/B1 boundary):
1. Scripts are just bash commands saved in a file (demystification)
2. Scripts run when you ask them to; they're reproducible and version-controllable
3. Multi-step workflows are good candidates for scripts
4. Scripts can have parameters (input) to make them flexible
5. Always test scripts on small data before running on production data
6. AI can write scripts; your job is to understand and validate them
7. Scripts prevent manual errors and save time

**Teaching Tier**: 2-3 (AI Companion for writing, Orchestration for validation and execution)

**Content Outline**:
- **Section A: What Is a Script and Why It Matters**
  - Script = commands saved in a file, executable with `bash script.sh`
  - Why scripts matter: Reproducibility, version control, automation, error prevention
  - Real scenario: Backing up projects every morning could be a script
  - Frame: "You don't need to know how to write bash. You need to know when to ask AI for help."

- **Section B: Describing What You Want (Specification)**
  - The key skill: Tell AI what you want the script to do, not how to code it
  - Good specification: "Back up my projects folder to cloud storage every morning"
  - Bad specification: "Use bash loops and rsync"
  - Dialogue showing specification → AI writes script → You understand it

- **Section C: Understanding AI-Generated Scripts**
  - AI writes script; your job is to read and verify it does what you asked
  - Breaking down script: AI explains what each line does
  - Common patterns: Loops (repeat commands), conditions (if/then), variables (store values)
  - You don't memorize syntax; you verify intent matches specification

- **Section D: Testing Before Production**
  - Always run script on test data first
  - Real scenario: Test backup script by backing up one small folder, not entire project
  - Validation: Did script produce expected results? Is output correct?
  - Only after successful test run on small data → run on production data

**Content Elements**:
- **Real Dialogue 1**: Creating a backup script
  - Student: "I want to backup my projects folder every day automatically"
  - AI: Asks clarifying questions (What format? Where to store? How many backups to keep?)
  - AI: Writes script, explains what it does
  - Student: Reviews and validates script
  - Result: Script created and tested

- **Real Dialogue 2**: Script with parameters
  - Student: "Create a script that processes all .csv files in a folder"
  - AI: Writes script that takes folder name as parameter
  - Student: Understands how to call script with different folders
  - Result: Reusable script for different data folders

- **Real Dialogue 3**: Script test and validation
  - Student: "Let's test this script first before running on all my data"
  - AI: Creates test data, runs script, shows results
  - Student: Confirms results are correct
  - Result: Script validated, ready for production use

- **Script Example Annotated**: Show simple script line-by-line with plain English explanations

**Practice Approach**:
- **Checkpoint 1**: Students direct AI to write a script
  - Scenario: Task that involves multiple steps (e.g., find files, copy them, rename them)
  - Task: Specify what script should do, have AI write it, review result

- **Checkpoint 2**: Students understand AI-generated script
  - Given: AI-written script
  - Task: "Explain what each section does. Does it match what you asked for?"
  - Expected: Can explain purpose of main sections

- **Checkpoint 3**: Students test script
  - Scenario: Script that modifies files
  - Task: Test on small sample first, validate results, then use on full data

**Prerequisites**: Lessons 1-6 (foundation through exploration)

**Cognitive Load Validation**:
- Concepts: 7 (scripts, automation, specification, validation, testing, parameters, reproducibility)
- Boundary between A2 and B1; ensure concepts are accessible but challenging

**Engagement Elements**:
- Empowerment: "After this lesson, you can have AI write complex workflows for you"
- Time savings: "Automation saves hours of repetitive work"
- Safety: "Scripts are more reliable than doing things manually over and over"
- Confidence: "You're not writing code; you're directing AI to write it for you"

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-3 script creation and testing exercises):
1. "I want to back up my important folders every day. Help me create a script that does this. Test it first with a small folder."
2. "Create a script that finds all .txt files older than 30 days and moves them to an archive folder. Walk me through what it will do before running."
3. "I have a repetitive task I do every morning: check for new files, organize them by type, and send me a summary. Could we automate this with a script?"

**Expected Outcomes**:
- Student can specify what a script should do in natural language
- Student can understand AI-generated script and verify it matches specification
- Student tests scripts on small data before production use
- Student recognizes when manual tasks should be automated
- Student uses scripts to save time and prevent errors

**Safety/Ethics Note**: Script validation is critical. Emphasize: "Always test first, always understand what script will do, never run untested script on production data."

---

### Lesson 8: Orchestrating Complex Workflows — From Planning to AI-Driven Execution

**Duration**: 40 minutes

**Sidebar Position**: 8

**Learning Objectives** (Bloom's: Analyze/Evaluate/Create):
- Plan multi-step workflows and break them down for AI execution
- Evaluate when to orchestrate vs. execute manually
- Create step-by-step specifications for complex tasks
- Supervise AI through multi-step workflows, validating each step
- Explain and teach others how to decompose complexity into manageable steps

**Skills Taught** (using skills-proficiency-mapper):
1. **Workflow Decomposition** — B1 — Conceptual — Student can take complex goal and break into steps AI can execute one at a time
2. **Strategic Supervision** — B1 — Soft — Student directs AI strategically ("Do step 1, then show me results before step 2"), not tactically
3. **Orchestration Thinking** — B1 — Technical — Student understands when to ask AI to execute multiple steps vs. pausing between steps for validation

**Core Concepts** (≤10 for B1):
1. Complex tasks can be decomposed into steps
2. Each step should be validated before moving to next
3. AI can execute steps but needs clear direction
4. Strategic supervision (high-level decisions) differs from tactical execution (command details)
5. Pausing between steps prevents cascading errors
6. Documentation of multi-step workflows prevents confusion
7. Parallel execution (10+ independent tasks) is where orchestration shines
8. Questions guide execution: "What's step 1? Do it. Validate. What's step 2?"
9. Professional workflows always have rollback plans
10. Orchestration is a key skill in AI-native development

**Teaching Tier**: 3 (AI Orchestration)

**Content Outline**:
- **Section A: When Orchestration Makes Sense**
  - Manual approach: Run 1-2 commands (too much overhead)
  - AI companion approach: Complex single operation (copy, move, search)
  - Orchestration approach: 5+ coordinated steps with decisions between steps
  - Real examples: Set up git worktrees for 10 features, batch process 200 images, build and deploy application
  - Frame: "Orchestration is about directing strategy, not executing tactics"

- **Section B: Decomposing Complexity into Steps**
  - Technique: "What would I do if I had to do this myself?"
  - Example workflow: "Clone repo, create 5 branches, set up virtual environment in each"
    - Step 1: Clone repository
    - Step 2: Create first branch
    - Step 3: Set up Python environment in first branch
    - Step 4-5: Repeat for remaining branches
    - Step 6: Validate all branches exist and environments work
  - AI's role: Execute steps, handle syntax, manage complexity
  - Your role: Define steps, validate between steps, approve execution

- **Section C: Directing AI Through Multi-Step Workflow**
  - Technique: Execute one step, validate, then move to next
  - Never: "Do all 5 steps without checking"
  - Always: "Do step 1, show me what happened, then we'll decide on step 2"
  - Real dialogue: Student → AI (step 1) → validate → Student → AI (step 2) → validate → ...
  - Checkpoints: After each step, confirm results before proceeding

- **Section D: Parallel Execution and Batch Operations**
  - When you have 10+ similar tasks, orchestration becomes powerful
  - Example: "Create git worktrees for features 1-10"
  - Rather than: Create 1 manually, repeat 10 times
  - Instead: "Create all 10 with a loop, then validate they exist"
  - Validation is critical: Count results, spot-check a few, ensure all are correct

**Content Elements**:
- **Real Dialogue 1**: Multi-step deployment
  - Student: "I want to deploy my app. Here's what needs to happen: build, test, create backup, deploy, validate"
  - AI: Asks clarifying questions for each step
  - AI: Proposes sequence with decision points
  - Student: Approves plan, AI executes step by step
  - Student: Validates each step before approving next
  - Result: App deployed successfully, with safety checkpoints throughout

- **Real Dialogue 2**: Batch operation (10+ tasks)
  - Student: "Create 10 git worktrees for features 1-10"
  - AI: Proposes loop-based approach
  - AI: Creates all 10 in one orchestrated command
  - AI: Shows validation (lists all branches, confirms all exist)
  - Student: Spot-checks a few, confirms success
  - Result: 10 worktrees created with validation, saved hours of manual work

- **Real Dialogue 3**: Orchestration with pauses
  - Student: "Process 200 image files: resize, compress, upload"
  - AI: Suggests running on small batch first (5 files)
  - Student: Agrees, AI processes 5
  - Student: Reviews results, confirms quality
  - AI: Processes remaining 195
  - Result: Batch operation completed safely with validation checkpoints

- **Workflow Decomposition Template**:
  - Goal: [What you want to accomplish]
  - Step 1: [First action] → Validation: [How to confirm success]
  - Step 2: [Second action] → Validation: [How to confirm success]
  - ...
  - Final Validation: [Overall check]

**Practice Approach**:
- **Checkpoint 1**: Students decompose complex task
  - Scenario: Describe a multi-step task (e.g., backup, organize, and archive old projects)
  - Task: Break into steps, identify validation points
  - Expected: 5-8 clear steps with validation between each

- **Checkpoint 2**: Students direct orchestrated workflow
  - Scenario: Have AI execute one step, validate, then execute next
  - Task: Don't hand off the entire task; direct it step-by-step
  - Expected: Student asks for step-by-step execution, validates results

- **Checkpoint 3**: Students orchestrate batch operation
  - Scenario: Task with 10+ similar items (files, branches, tasks)
  - Task: Ask AI to batch-execute, then validate results
  - Expected: AI executes efficiently, student validates that all items processed correctly

**Prerequisites**: Lessons 1-7 (full foundation)

**Cognitive Load Validation**:
- Concepts: 10 (decomposition, steps, validation, strategic vs. tactical, parallel execution, orchestration, checkpoints, rollback, supervision, AI-native thinking)
- Boundary between A2 and B1; introduces professional-level orchestration thinking
- Fits within B1 limit (max 10 concepts)

**Engagement Elements**:
- Empowerment: "After this lesson, you can direct AI to handle complex multi-step tasks"
- Professional frame: "This is how professional teams use AI — strategic direction, not micromanagement"
- Impact: "Orchestration saves hours on repetitive, multi-step tasks"
- Confidence builder: "You're learning to think like a project manager, not a typist"

**End-of-Lesson: Try With AI** — Claude Code or Gemini CLI

**Prompts** (2-4 orchestration and batch operation exercises):
1. "I have a complex task: clone a repository, create 5 feature branches, set up a Python environment in each. Break this into steps and execute them with validation between steps."
2. "Help me batch-process 50 files: find them, organize them by type, create a summary. Use AI to orchestrate this efficiently. Validate the results."
3. "I want to deploy my application. Here are the steps: build, run tests, create backup, deploy to production, verify it's working. Direct me through this safely."
4. "Create a workflow that backs up my important folders, archives files older than 1 year, and generates a summary of what was done. Show me how you'd break this down into steps."

**Expected Outcomes**:
- Student can decompose complex tasks into manageable steps
- Student directs AI through multi-step workflows with validation between steps
- Student understands when orchestration is more efficient than manual execution
- Student supervises batch operations and validates results
- Student can explain to others how to orchestrate complex workflows with AI

**Safety/Ethics Note**: Orchestration is powerful, so validation is critical. Emphasize: "The more steps you orchestrate, the more important validation becomes. Always validate between steps."

---

## Content Flow & Dependencies

**Progressive Difficulty Progression**:
- **Lessons 1-2** (Tier 1): Foundation — Students learn concepts directly through book explanations and authentic dialogues
  - Lesson 1 builds location awareness (essential before any file operation)
  - Lesson 2 teaches 5-step safety pattern (foundation for safe operations throughout chapter)

- **Lessons 3-5** (Tier 2): AI Companion — Students learn to work with AI on complex operations
  - Lesson 3 teaches reading/understanding bash commands (supervision skill)
  - Lesson 4 applies supervision to file operations with safety pattern
  - Lesson 5 adds security consciousness (secrets and configuration)

- **Lessons 6-8** (Tier 3): Orchestration — Students direct AI on complex, multi-step workflows
  - Lesson 6 adds exploration and pattern recognition (prerequisite for scaling)
  - Lesson 7 introduces automation through scripts (Tier 2/3 boundary)
  - Lesson 8 orchestrates complex workflows at scale (Tier 3 capstone)

**Lesson Dependencies**:
```
Lesson 1 (Navigation)
    ↓
Lesson 2 (Safety Pattern) ← MUST learn before Lessons 3-8
    ↓
Lesson 3 (Command Understanding)
    ↓
Lesson 4 (File Operations) — requires Lessons 1-3
Lesson 5 (Configuration) — requires Lessons 1-3
    ↓
Lesson 6 (Safe Exploration) — requires Lessons 1-5
    ↓
Lesson 7 (Scripts) — requires Lessons 1-6
    ↓
Lesson 8 (Orchestration) — requires Lessons 1-7
```

**Concepts That Recur**:
- **5-Step Safety Pattern**: Introduced in Lesson 2, applied in Lessons 4-8 (reinforcement)
- **Command Understanding**: Introduced in Lesson 3, applied in Lessons 4-8 (building mastery)
- **Natural Language Direction**: Introduced in Lesson 1, reinforced in all lessons (core AI-native skill)
- **Validation**: Introduced in Lesson 4, emphasized in Lessons 5-8 (professional practice)

---

## Scaffolding Strategy

**Cognitive Load Management**:
- Each lesson introduces exactly 5-7 new concepts (A1/A2 limits)
- Concepts are introduced in increasing complexity, but foundational (Lesson 1-2) concepts enable later lessons
- Tier 1 lessons are longer because they require more explanation
- Tier 2 lessons balance explanation with AI assistance
- Tier 3 lessons shift responsibility to student to direct AI (less scaffolding, more autonomy)

**Graduated Teaching Pattern Applied**:
1. **Book Teaches** (Lessons 1-2, 5): Foundational concepts about terminal, safety, and security explained directly
2. **AI Companion** (Lessons 3-4, 6-7): Complex operations where AI executes, student specifies and supervises
3. **Orchestration** (Lesson 8): Student orchestrates multi-step workflows; AI handles execution complexity

**Metacognitive Scaffolding**:
- Each lesson includes concept count validation (helping students track cognitive load)
- Engagement elements explicitly connect concepts to real-world value
- Practice checkpoints are progressively more autonomous (Lesson 1 = heavily guided; Lesson 8 = student-directed)

**Error Prevention**:
- Safety pattern reinforced in every lesson involving file operations
- Command understanding checkpoint in Lesson 3 prevents supervision mistakes later
- Validation emphasis prevents students from trusting AI blindly

---

## Integration Points

**Prerequisite Chapters**:
- **Chapter 5** (Claude Code): Students familiar with AI tool interface and conversation flow
- **Chapter 6** (Gemini CLI): Students understand CLI tools and terminal basics

**Dependent Chapters**:
- **Chapter 8** (Git & GitHub): Requires bash navigation and file operations from this chapter
- **Chapter 9** (Markdown): May reference bash examples
- **Chapter 12** (Python UV): Requires terminal confidence and environment setup skills
- **All Future Chapters**: Terminal and bash skills are prerequisite for all subsequent development work

**Cross-Chapter Skills**:
- **Specification Skills** (Chapters 30-33): This chapter teaches specifying intent in natural language (foundation for specification-driven development)
- **Validation Skills** (taught here): Applied throughout book for validating AI-generated code
- **AI Collaboration** (Chapter 5-6): Extended here to demonstrate supervision and orchestration
- **Error Literacy**: Foundation for troubleshooting in all technical chapters

---

## Validation Strategy

**How Learners Demonstrate Understanding** (by lesson):

**Lesson 1 (Navigation)**:
- Independent skill: Student conducts 3+ conversations with AI about their file system without typing bash commands
- Success criteria: Can ask for location, list files, identify directories, navigate to specific folders

**Lesson 2 (Safety Pattern)**:
- Independent skill: Student directs AI to perform file operation following all 5 steps
- Success criteria: Demonstrates Ask, Explain, Understand, Verify, Execute in conversation

**Lesson 3 (Command Understanding)**:
- Independent skill: Student reads command and explains in plain English what it does
- Success criteria: Can explain flags, pipes, redirects without memorizing syntax

**Lesson 4 (File Operations)**:
- Independent skill: Student uses AI to copy/move files safely, validates results
- Success criteria: Asks about backups, applies safety pattern, validates operation completed

**Lesson 5 (Configuration)**:
- Independent skill: Student sets up `.env` file with API key and verifies it's git-ignored
- Success criteria: Can explain why secrets must be environment variables

**Lesson 6 (Safe Exploration)**:
- Independent skill: Student asks AI to search for files, analyzes results, identifies patterns
- Success criteria: Can find files, understand patterns, make decisions based on results

**Lesson 7 (Scripts)**:
- Independent skill: Student directs AI to create script, understands it, tests it on small data
- Success criteria: Script does what was specified, works correctly on test data

**Lesson 8 (Orchestration)**:
- Independent skill: Student decomposes multi-step task, directs AI through steps with validation
- Success criteria: Task completed safely with checkpoints between steps; can explain approach to others

**Assessment Methods** (Chapter-Level):
- **Conversation Transcripts**: Students document AI conversations showing skills in action
- **File Validation**: Screenshots showing successful completion of operations
- **Comprehension Checks**: Questions about why safety pattern matters, why certain approaches are used
- **Real-World Application**: Students apply chapter concepts to their own projects

---

## Notes on Chapter Type and Pedagogical Approach

**Chapter Type**: Technical/Practical (Beginner Tier)
- Not conceptual/narrative (no essay-style sections; this is hands-on learning)
- Not code-heavy scripting (not teaching students to write bash scripts from scratch)
- Hybrid approach: Foundational concepts (book teaches) + AI-assisted execution (Tier 2/3)

**AI Tool Onboarding**:
- This is Part 2, after Chapter 6 (Gemini CLI)
- Students have already learned to use Claude Code (Chapter 5) and Gemini CLI (Chapter 6)
- "Try With AI" sections use learner's preferred AI tool from Chapters 5-6 (Claude Code, Gemini CLI, or ChatGPT Code Interpreter)
- This chapter teaches students to direct AI toward specific bash operations (natural language → commands)

**Cognitive Load Per Lesson**:
- Lesson 1: 5 concepts (A1 max)
- Lesson 2: 5 concepts (A1 max)
- Lesson 3: 7 concepts (A2 max)
- Lesson 4: 7 concepts (A2 max)
- Lesson 5: 7 concepts (A2 max)
- Lesson 6: 7 concepts (A2 max)
- Lesson 7: 7 concepts (A2/B1 boundary)
- Lesson 8: 10 concepts (B1 max)

All lessons fit within proficiency limits with room for practice and reinforcement.

---

## Quality Standards

**Amazon Book Quality**: Content will be written at professional publishing level (O'Reilly, Manning, Pragmatic Bookshelf standards)
- Clear, engaging writing with varied pacing
- Real dialogues from actual AI tools (not synthesized examples)
- Practical examples students can immediately apply
- Professional polish (zero typos, consistent formatting, proper grammar)

**Pedagogical Rigor**:
- Every learning objective has measurable criteria and validation method
- Lesson progression scaffolds complexity systematically
- Safety is emphasized throughout (not added as afterthought)
- Skills proficiency uses international standards (CEFR A1/A2/B1, Bloom's taxonomy, DigComp)

**AI-Native Authenticity**:
- Shows real AI behavior (errors, limitations, clarifying questions)
- Uses authentic dialogue transcripts from actual tools
- Teaches supervision and validation alongside generation
- Demonstrates that AI is a tool students direct, not magic that always works

---

## Conclusion

This lesson plan transforms Chapter 7 from a traditional bash tutorial into an AI-native learning experience where students become confident terminal users by learning to **communicate intent in natural language** and **supervise AI execution** rather than memorizing commands.

The progression from Lesson 1 (location awareness) through Lesson 8 (orchestration) takes students from absolute beginners to users who can direct AI to handle complex, multi-step workflows safely and effectively. Every lesson reinforces the safety-first mindset that prevents catastrophic mistakes while building confidence in terminal work.
