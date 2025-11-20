# Chapter 8: Git & GitHub for AI-Driven Development — Implementation Plan

**Chapter Type**: Technical/Hybrid (Core concepts + hands-on conversational AIDD)

**Chapter Objective(s)** (from approved spec):
- Learners understand why Git is essential for safe AI-assisted development
- Learners can perform core Git workflows (init, commit, branch, undo) using natural language prompts
- Learners can connect local repositories to GitHub for backup and collaboration
- Learners can use modern IDEs with Git integration and AI coding extensions
- Learners demonstrate safety-first practices when working with AI code assistants

**Estimated Total Time**: 3 hours (175 minutes total: Part I 75 min + Part II 100 min)

**Part**: Part 2 - AI Tool Landscape (follows Chapters 5, 6, 7)

**Complexity Tier**: Tier 1 - Beginner
- Max 2 options per section (GitHub only; VS Code primary + alternatives)
- Max 5 new concepts per lesson section
- AI-as-partner pattern: students don't memorize commands
- Concept-first approach: WHAT → WHY → HOW (via AI) → PRACTICE

**Target Audience**: Complete beginners (no prior Git/version control experience)

**Prerequisites**:
- Part 1 completed (understand AIDD paradigm)
- Chapter 5 (Claude Code) or Chapter 6 (Gemini CLI) completed
- Chapter 7 (Bash Essentials) completed
- Basic computer literacy (file management, terminal navigation)

---

## Lesson Architecture

Chapter 8 follows the **two-part structure** defined in the specification:
- **Part I** (Lessons 1–4): Git concepts and commands (75 minutes: 15+20+20+20)
- **Part II** (Lessons 5–9): AI workflows, GitHub, IDE setup, and capstone (100 minutes: 20+20+20+20+20)

**Lesson Duration Standard**: Each lesson is 15-20 minutes to maintain engagement without cognitive overload. Content is focused on essential concepts only.

---

### Lesson 1: Why Git Matters with AI Tools (15 minutes)

**Objective**: Recognize why version control is essential for safe AI-assisted development and understand Git as a foundational safety mechanism.

**Skills Taught**:
- Conceptual Understanding of Version Control — A1 — Conceptual — Student can explain "What is Git?" and "Why does it matter for AI development?" in their own words without jargon
- Safety-First Mindset for AI Development — A1 — Soft — Student recognizes when to commit before AI makes changes and why

**Key Concepts** (max 5):
1. Git as a "time machine" for code (non-technical metaphor)
2. Why AI-assisted code needs safety nets (risk of breaking changes)
3. Commits as snapshots (save points)
4. Undo capability (reversibility)
5. Backup via GitHub (remote storage)

**Prerequisites**: None (opening lesson)

**Duration**: 15 minutes

**Note**: This lesson is conceptual only (no hands-on), keeping it short and focused on motivation.

**Content Outline**:
- Hook: "Your AI assistant just rewrote 50 lines of code. How do you check if it's correct AND undo it if broken?"
- Problem statement: AI can be powerful but risky without safety mechanisms
- Git introduction: A tool that tracks changes and lets you travel back in time
- Why Git matters for AI: Commit before major changes, review diffs, discard bad changes
- Real scenario: Student using Claude Code to refactor a function; Git lets them experiment safely
- Connection to chapters 5–7: Now that you know how to work WITH AI tools, Git keeps you safe

**Content Elements**:
- No code yet; focus on concepts and narrative
- Conversational example: "Before I let my AI assistant refactor my code, I should..."
- Emphasis: AI is powerful; Git is the safety harness

**Practice Approach**:
- Reflection prompt: "What changes might an AI assistant make that could break your code?"
- Real-world scenario walkthrough (no hands-on yet)

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI (student's preference from Ch. 5–6)
- **Scenario**: "Explain to me why version control matters for AI-assisted development"
- **Prompts**:
  1. "What is Git and why do software developers use it?"
  2. "Explain the main risks when working with AI code assistants, and how Git helps mitigate them"
  3. "I'm a complete beginner. Explain version control as if I've never heard of it before"
- **Expected Outputs**: AI explains Git concepts in beginner language; student captures key points
- **Safety Note**: This is conceptual; no commands yet

---

### Lesson 2: Essential Setup (25 minutes)

**Objective**: Successfully install and configure Git, create a GitHub account, and set up initial Git configuration for the student's machine.

**Skills Taught**:
- Install Git on Windows/Mac/Linux — A1 — Technical — Student can install Git on their specific OS and verify installation with `git --version`
- Create GitHub Account — A1 — Technical — Student can create GitHub account, verify email, and navigate GitHub interface
- Configure Git Locally — A1 — Technical — Student can set git username and email globally on their machine

**Key Concepts** (max 5):
1. Git installation (platform-specific binary)
2. GitHub account setup (cloud backup service)
3. Git configuration (identity: name + email)
4. Verification (testing installation)
5. First-time setup (one-time configuration)

**Prerequisites**: Lesson 1

**Duration**: 20 minutes (10 min install, 10 min GitHub + config)

**Note**: Installation is guided via links and AI assistance. Students follow installation links while AI helps troubleshoot issues.

**Content Outline**:

#### Part A: Install Git (Platform-Specific) — 10 min

**Installation Links** (students follow these):

**For Windows**:
- Visit: https://git-scm.com/download/win
- Download the installer (64-bit recommended)
- Run the `.exe` file → Click "Next" through installer (defaults are fine)
- Important option: "Use Git from the Windows Command Prompt" (should be selected)
- After install: Open Command Prompt → Type `git --version` → Should see version number
- **Troubleshooting**: If "git not recognized", restart Command Prompt or add Git to PATH (ask AI assistant for help)

**For Mac**:
- **Option 1 (Homebrew)**: If you have Homebrew installed, run `brew install git` in Terminal
- **Option 2 (Direct Download)**: Visit https://git-scm.com/download/mac → Download → Run installer
- After install: Open Terminal → Type `git --version` → Should see version number
- **Troubleshooting**: If command not found, may need Xcode command line tools (ask AI assistant)

**For Linux**:
- **Ubuntu/Debian**: Open Terminal → Run `sudo apt update && sudo apt install git`
- **Fedora/RHEL**: Run `sudo dnf install git`
- **Arch**: Run `sudo pacman -S git`
- After install: Type `git --version` → Should see version number
- **Troubleshooting**: Permission denied? Need `sudo` before command

**Teaching Approach**: Provide installation links and text descriptions. Students ask their AI assistant (Claude Code or Gemini CLI) for help if they encounter issues. AI can troubleshoot platform-specific problems in real-time.

#### Part B: GitHub Account Creation — 5 min

**Step-by-Step** (text guide):
1. Visit https://github.com
2. Click "Sign up" button (top right)
3. Enter your email address (use real email - you'll need to verify it)
4. Create strong password (at least 15 characters or 8+ with number and lowercase letter)
5. Choose username (alphanumeric, hyphens OK; no spaces; will be part of your portfolio URL)
6. Verify email: Check inbox/spam for verification email from GitHub → Click link
7. Choose "Free" plan (completely sufficient for this book and beyond)
8. Welcome survey is optional (skip or fill as you prefer)
9. After signup: You'll see your GitHub dashboard

**Key Pages to Know**:
- Profile: `github.com/your-username` (this is your portfolio)
- Repositories: Where your projects live
- Settings: Account configuration

**Teaching Approach**: Provide direct link and step-by-step text. If student encounters issues (email not received, username taken), they ask AI assistant for immediate help.

#### Part C: Configure Git Locally — 5 min

- Why: Git needs your identity for commits
- Commands (via natural language to AI):
  - `git config --global user.name "Your Full Name"`
  - `git config --global user.email "your-email@example.com"`
- Verify configuration: `git config --list` (shows all settings)
- Explanation: `--global` applies to all repositories on this machine
- Screenshot: Terminal showing config commands and output

**Content Elements**:
- Direct download links for all platforms
- Step-by-step screenshots for each OS
- Command-line examples (but students can ask AI to help run them)
- Troubleshooting section for common issues

**Practice Approach**:
- Guided exercise: Student installs Git on their machine (support for each OS)
- Verification checkpoint: Student runs `git --version` and sees version
- GitHub account creation: Student follows steps, shows they can log in
- Config verification: Student runs `git config --list`

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI
- **Scenario**: "Help me install and set up Git on my machine"
- **Prompts**:
  1. "I'm on [Windows/Mac/Linux]. Help me install Git step-by-step"
  2. "After installing Git, how do I verify it's working?"
  3. "Help me configure Git with my name and email"
  4. "I ran `git config --list`. What does each setting mean?"
- **Expected Outputs**: AI provides platform-specific installation steps; student executes under AI guidance
- **Safety Note**: Include prompt like "I'm a complete beginner—talk me through each step slowly"

**Troubleshooting** (integrated into lesson):
- Git not found after Windows install: Add Git to PATH (step-by-step)
- GitHub email verification not received: Check spam, resend email
- GitHub username taken: Try variations with numbers/hyphens
- Terminal can't find git command on Mac: May need Xcode command-line tools

---

### Lesson 3: The Daily Workflow (20 minutes)

**Objective**: Perform the five core Git operations (init, status, add, commit, push) using both direct commands AND natural language prompts to AI assistants.

**Note**: Focus on the 5 core commands only. Students practice with a simple file (hello.py or README.md) - AI can generate it.

**Skills Taught**:
- Initialize Git Repository — A1 — Technical — Student can create new Git repository in a project folder
- Track File Changes — A1 — Technical — Student can stage files and create commits with descriptive messages
- Push to Remote — A1 — Technical — Student can upload commits to GitHub
- Understand Git Workflow Conceptually — A1 — Conceptual — Student can explain the stages: working directory → staging area → commit history

**Key Concepts** (max 5):
1. Repository (project with version history)
2. Working directory (files you're editing)
3. Staging area (files ready to commit)
4. Commit (snapshot of changes)
5. Push (upload to GitHub)

**Prerequisites**: Lesson 2 (Git installed and configured)

**Duration**: 25 minutes

**Content Outline**:

#### The Five Core Commands:

1. **git init** — Create a new repository
   - What: Initialize Git tracking in a folder
   - Why: Every project needs a `.git` folder to track history
   - How: `git init` (run in project folder)
   - Visual: Shows `.git` folder creation

2. **git status** — Check what changed
   - What: See which files changed since last commit
   - Why: Helps you review before committing
   - How: `git status` (shows changed files)
   - Visual: Color-coded output (red=unstaged, green=staged)

3. **git add** — Stage files for commit
   - What: Mark files you want to save in next commit
   - Why: Lets you choose WHICH changes to include
   - How: `git add filename` or `git add .` (all files)
   - Visual: Shows moving from "Changes not staged" to "Changes to be committed"

4. **git commit** — Save a snapshot
   - What: Create a permanent record of changes with a message
   - Why: Messages explain WHY changes were made
   - How: `git commit -m "Clear message describing changes"`
   - Visual: Shows message format and history

5. **git push** — Upload to GitHub
   - What: Send commits to GitHub (remote)
   - Why: Backup, collaboration, portfolio
   - How: `git push origin main` (after setup in Lesson 6)
   - Visual: Shows local commits moving to GitHub

#### Workflow Diagram:
```
[working directory] → git add → [staging area] → git commit → [commit history] → git push → [GitHub]
```

**Content Elements**:
- Five mini-demonstrations (text descriptions, could include screenshots)
- Real example: Create simple Python file, make changes, commit
- Emphasis: These five commands handle 90% of daily Git work
- NO advanced features yet (branching, merging come later)

**Practice Approach**:

*Guided Exercise*:
1. Create a new folder: `mkdir my-project`
2. Navigate into it: `cd my-project`
3. Initialize Git: `git init` (shows `.git` folder created)
4. Create a file: Add `hello.py` with simple Python code
5. Check status: `git status` (shows `hello.py` untracked)
6. Stage file: `git add hello.py`
7. Check status again: `git status` (shows staged for commit)
8. Create commit: `git commit -m "Add hello world program"`
9. View history: `git log` (shows your commit)

*Conversational Practice*:
- Student asks AI: "Initialize a Git repository for my Python project"
- AI runs `git init`
- Student asks AI: "I made changes to my file. Show me what changed"
- AI runs `git status` and `git diff`
- Student asks AI: "Save my work with a commit message"
- AI stages files and commits with message

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI
- **Scenario**: "I want to learn Git by actually using it. Help me initialize a repository and make my first commit"
- **Prompts**:
  1. "Create a new folder for a practice project and initialize Git"
  2. "Create a simple Python file (hello world is fine) and show me the status"
  3. "Show me what's in the staging area and explain each file"
  4. "Commit my changes with a descriptive message explaining what I added"
  5. "Show me my commit history with `git log`"
- **Expected Outputs**: Student completes all five workflow steps through AI guidance
- **Safety Note**: Emphasize "commit before making big changes" pattern

---

### Lesson 4: Safety Net – Undoing Changes (20 minutes)

**Objective**: Safely undo changes at different stages (uncommitted, committed, pushed) and understand when to use each recovery method.

**Note**: Focus on practical undo methods students will actually use. Skip esoteric cases.

**Skills Taught**:
- View File Changes — A1 — Technical — Student can use `git diff` and `git log` to understand what changed
- Undo Uncommitted Work — A1 — Technical — Student can discard changes to files before staging
- Undo Committed Work Safely — A2 — Technical — Student can use `git reset --soft` to undo commits while preserving work
- Understand Danger Zones — A1 — Soft — Student recognizes commands that permanently delete work and when to use them

**Key Concepts** (max 5):
1. Diff (show what changed in a file)
2. Log (commit history)
3. Reset soft (undo commit, keep files)
4. Reset hard (undo commit, DELETE files—DANGER)
5. Revert (create new commit that undoes changes—SAFE)

**Prerequisites**: Lesson 3 (understand commits and history)

**Duration**: 30 minutes

**Content Outline**:

#### View Changes:

**git diff**:
- What: Show exact changes in files since last commit
- Why: Review changes before staging
- How: `git diff` (shows all unstaged changes in detail)
- Visual: Side-by-side diff output with + and - lines
- Use case: "The AI changed my function. Let me see exactly what changed."

**git log**:
- What: Show commit history
- Why: Understand what was changed and when
- How: `git log` (shows recent commits) or `git log --oneline` (condensed)
- Visual: Commit list with dates and messages
- Use case: "I need to find which commit broke the code."

#### Undo Uncommitted Changes (Safe):

**git checkout -- filename** (or git restore filename):
- What: Discard changes to a file, revert to last commit
- Why: "I made a mess. Start over."
- How: `git checkout -- myfile.py`
- **⚠️ WARNING**: Deletes changes permanently (but they were never committed)
- Use case: "The AI made bad changes. Throw them away."
- Recovery: If you didn't commit, it's gone. BUT you can always ask AI to recreate.

#### Undo Committed Work (Safe Methods):

**git reset --soft HEAD~1** (SAFE):
- What: Undo last commit but keep changes staged
- Why: "I committed too early. Let me adjust before committing again."
- How: `git reset --soft HEAD~1`
- Safe? YES—your changes are preserved (still staged)
- Use case: "I committed but forgot to add a line. Reset and fix it."
- Visual: Shows changes moving back to staging area

**git revert HEAD** (SAFEST):
- What: Create a NEW commit that undoes last commit
- Why: "I want to undo changes but keep a record of why"
- How: `git revert HEAD`
- Safe? YES—creates new commit (doesn't delete history)
- Use case: "I committed AI changes that broke something. Create a new commit that reverses them."
- Visual: New commit appears that undoes previous changes

**git reset --hard HEAD~1** (⚠️ DANGER ZONE):
- What: Permanently DELETE last commit and all changes
- Why: "This commit is completely wrong. Erase it."
- How: `git reset --hard HEAD~1`
- Safe? NO—deletes work permanently (cannot be recovered from history)
- ⚠️ WARNING: Only use if you're 100% sure
- Use case: Rare; usually `revert` is safer
- Visual: Commit disappears from history completely

#### Decision Tree (When to Use What):

```
I made changes but haven't committed yet:
  → Use: git checkout -- filename (or git restore)
  → Effect: Discard changes

I committed but want to fix it (changes still needed):
  → Use: git reset --soft HEAD~1
  → Effect: Undo commit, keep changes staged for re-commit

I committed but it's wrong (want to keep history):
  → Use: git revert HEAD
  → Effect: Create new commit that reverses changes

I committed and want to completely erase it (dangerous):
  → Use: git reset --hard HEAD~1
  → Effect: Permanently delete (cannot recover from history)
```

**Content Elements**:
- Detailed explanation of each undo method
- Visual diffs showing before/after
- Step-by-step walkthroughs with examples
- **CRITICAL**: Explicit warnings before dangerous commands
- Real scenario: "AI changed a core function. How do I review and undo if needed?"

**Practice Approach**:

*Hands-On Exercise* (safe progression):
1. Create file with initial content
2. Commit it: `git commit -m "Initial version"`
3. Make changes to file
4. Use `git diff` to see changes
5. **Practice 1**: Use `git checkout -- filename` to discard changes (safe)
6. Make new changes, stage them
7. Commit: `git commit -m "Version 2"`
8. **Practice 2**: Use `git reset --soft HEAD~1` to undo commit (safe)
9. Adjust files, re-commit
10. **Practice 3** (safe): Use `git revert HEAD` to create undo commit
11. View log to see both original and revert commits

*Conversational Practice*:
- Student: "The AI made changes I don't like. Show me what changed"
- AI: Runs `git diff`
- Student: "Can I undo this?"
- AI: Explains options and suggests `git reset --soft` for safety
- Student: "Undo it"
- AI: Runs command, shows changes preserved

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI
- **Scenario**: "I made some changes with AI help, but I'm not sure about them. Help me review and undo if needed."
- **Prompts**:
  1. "Show me all the changes I made since my last commit"
  2. "Explain in detail what each line changed"
  3. "I want to undo these changes but keep my work so I can fix it. What command do I use?"
  4. "Execute the undo command and show me the result"
  5. "Now show me my commit history to confirm the undo worked"
- **Expected Outputs**: Student uses `diff`, `log`, `reset --soft`, and understands the safety difference
- **Safety Note**: "Reset --hard deletes forever. Only use when certain. Revert is safer for commits you want to remove from history."

---

## Part I Summary (Lessons 1–4)

**Time**: 75 minutes (15+20+20+20)

**Learning Outcomes**:
- Understand Git as safety net for AI-assisted development
- Install Git on their specific OS
- Create GitHub account and configure Git locally
- Perform core workflow: init, status, add, commit, push
- Safely undo changes at different stages
- Recognize when each undo command is appropriate

**Cognitive Load**: Within Tier 1 limits (max 5 concepts per lesson, one skill per lesson)

---

### Lesson 5: Branches for Experimentation (20 minutes)

**Objective**: Create and use Git branches to safely test AI-generated changes before merging to main code.

**Note**: Students practice with a feature branch for testing. All code generation done by AI assistant.

**Skills Taught**:
- Create Feature Branch — A2 — Technical — Student can create branch, make changes, and merge back to main
- Test Before Merging — A2 — Technical — Student can test changes on branch and decide to merge or discard
- Understand Branching Metaphor — A2 — Conceptual — Student can explain branching as "parallel versions of code"

**Key Concepts** (max 5):
1. Branch (parallel version of code)
2. Main/master branch (production code)
3. Feature branch (temporary experimental version)
4. Merge (combine changes back to main)
5. Delete branch (clean up after merge)

**Prerequisites**: Lesson 4 (understand commits and history)

**Duration**: 20 minutes

**Content Outline**:

#### Why Branches Matter for AI:
- Scenario: "Your AI assistant suggests a major refactor. You want to test it without risking main code."
- Solution: Create a branch, let AI make changes, test thoroughly, then decide: merge (good) or discard (bad)
- Safety: If experiments fail, main branch is untouched

#### The Branching Workflow:

1. **git branch feature-name** — Create a branch
   - Creates a parallel version of code
   - Don't push changes to main yet

2. **git checkout feature-name** (or git switch feature-name) — Switch to branch
   - Move to the branch
   - Now commits go to this branch, not main

3. **git push origin feature-name** — Push branch to GitHub
   - Backs up your experiment
   - Allows team collaboration on the branch

4. **git merge feature-name** (on main branch) — Combine changes back
   - After testing, merge successful experiments to main
   - If bad, just delete the branch

5. **git branch -d feature-name** — Delete branch
   - Clean up after merging
   - Branch no longer needed

#### Real Example: AI-Assisted Refactor

```
Start on main (production code)
  ↓
Ask AI: "Create a branch called 'optimize-database'"
  ↓
AI creates branch and switches to it
  ↓
AI makes refactoring changes (10-15 commits)
  ↓
You test: "Does the code still work? Is it faster?"
  ↓
If YES: Ask AI to merge branch back to main
  ↓
If NO: Ask AI to delete branch (discard experiment)
  ↓
Back to main, production code unchanged
```

**Content Elements**:
- Step-by-step branch creation and usage
- Diagram showing branch divergence and merge
- Real conversational examples with AI
- Emphasis: Branching is FREE (doesn't cost anything); use branches liberally

**Practice Approach**:

*Hands-On Exercise*:
1. On main branch, create a file with initial code
2. Commit: `git commit -m "Initial version on main"`
3. Create branch: `git branch experiment`
4. Switch to branch: `git checkout experiment` or `git switch experiment`
5. Make changes on branch (simulate AI refactor)
6. Commit changes: `git commit -m "Refactor function"`
7. View log on branch: `git log`
8. Switch back to main: `git checkout main`
9. View log on main (no refactor commits): `git log`
10. Merge branch: `git merge experiment`
11. View log on main (now includes refactor commits): `git log`
12. Delete branch: `git branch -d experiment`

*Conversational Practice*:
- Student: "I want to test an AI-generated refactoring safely"
- AI: "Let me create a branch for experimentation"
- AI: Creates branch, makes changes
- Student: "Test the code" (verify it works)
- Student: "Merge it" or "Discard it"
- AI: Executes merge or delete

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI
- **Scenario**: "Help me create a safe testing branch, make changes, and decide whether to keep them"
- **Prompts**:
  1. "Create a Git branch called 'improve-ui' for testing UI improvements"
  2. "Make some example changes to a sample file on this branch"
  3. "Show me what commits are on this branch that aren't on main"
  4. "Switch back to main and show me that the changes aren't there"
  5. "Merge the branch back to main since the improvements looked good"
- **Expected Outputs**: Student creates branch, tests changes, decides to merge, and verifies main branch updated
- **Safety Note**: "Branches are free. Create a new branch for every experiment. Protects main from accidents."

---

### Lesson 6: GitHub Integration (20 minutes)

**Objective**: Connect local Git repository to GitHub, authenticate securely, and push/pull to keep GitHub updated.

**Note**: Personal Access Token recommended for beginners (simpler than SSH). SSH is mentioned but not required.

### Natural Language Prompts for Git Operations

Throughout Part II (Lessons 5-8), students learn to ask their AI assistant to perform Git operations using natural language instead of memorizing commands. This is the core of the AIDD approach: **understand concepts, let AI handle syntax**.

**Key principle**: "You don't memorize `git merge feature-branch`. You ask your AI: 'Merge the feature branch into main.' The AI translates your intent into correct commands and explains what's happening."

Students practice this pattern in every "Try With AI" exercise. By Lesson 9 (Capstone), asking AI for Git help becomes natural.

**Skills Taught**:
- Add Remote Repository — A1 — Technical — Student can connect local repo to GitHub repository
- Authenticate Securely — A2 — Technical — Student can use SSH or Personal Access Token for authentication
- Push and Pull — A1 — Technical — Student can upload commits to GitHub and download updates

**Key Concepts** (max 5):
1. Remote (GitHub repository connected to local)
2. Origin (default name for GitHub remote)
3. SSH key (secure authentication)
4. Personal Access Token (alternative authentication)
5. Push/Pull (sync local and GitHub)

**Prerequisites**: Lesson 3 (understand commits), GitHub account created (Lesson 2)

**Duration**: 20 minutes

**Content Outline**:

#### Adding Remote to GitHub:

**Step 1: Create Repository on GitHub**
- Go to github.com
- Click "New" button (or + icon)
- Enter repo name (e.g., "my-project")
- Choose "Public" (default for learning)
- Click "Create repository"
- GitHub shows setup instructions

**Step 2: Add Remote (Connect GitHub to Local)**
- GitHub provides command: `git remote add origin https://github.com/username/repo-name.git`
- "origin" = the name of this GitHub repository
- Copy-paste command from GitHub page
- Verify: `git remote -v` (shows GitHub URL)

**Step 3: Authenticate**

**Option A: Personal Access Token (Simpler for Beginners)**
- Create token on GitHub (Settings → Developer Settings → Personal Access Tokens)
- Use token as password when pushing
- Simpler but less secure (token visible in history)
- Good for learning

**Option B: SSH Key (More Secure)**
- Generate SSH key on local machine: `ssh-keygen -t ed25519` (AI can help)
- Add public key to GitHub (Settings → SSH Keys)
- Git uses SSH automatically (no password needed)
- More secure but slightly complex to set up

**Recommendation for Beginners**: Start with Personal Access Token. Can switch to SSH later.

**Step 4: Push to GitHub**
- `git push -u origin main` (first push; `-u` sets upstream)
- Future pushes: `git push origin main` (or just `git push`)
- GitHub receives your commits

**Step 5: Verify on GitHub**
- Refresh github.com/username/repo
- See your files and commit history online
- Now have cloud backup!

#### Push and Pull Workflow:

**Push** (upload to GitHub):
- `git push origin main`
- Sends your local commits to GitHub
- When: After committing locally, want to backup

**Pull** (download from GitHub):
- `git pull origin main`
- Gets updates from GitHub to local
- When: Working on multiple machines, want latest code

**Real Scenario**:
```
Local Machine A: Make changes → Commit → Push to GitHub
                                              ↓
Local Machine B: Pull from GitHub → Get updates locally
```

**Content Elements**:
- Step-by-step repository creation on GitHub (with screenshots)
- Two authentication options (token and SSH); recommend token for beginners
- Push and pull commands explained
- Verification: "You should see your code on GitHub"

**Practice Approach**:

*Hands-On Exercise*:
1. Create repository on GitHub (follow web UI)
2. Copy remote add command from GitHub page
3. Run: `git remote add origin https://...`
4. Verify: `git remote -v`
5. Create Personal Access Token on GitHub
6. Push: `git push -u origin main`
7. Enter username and use token as password
8. Verify: Go to github.com/username/repo and see your files
9. Make local changes, commit
10. Push again: `git push origin main`
11. Verify on GitHub (show new commit)

*Conversational Practice*:
- Student: "I want to upload my project to GitHub"
- AI: "First, create a repository on GitHub"
- Student: (Creates repo, gets instructions)
- Student: "Now connect my local code to GitHub"
- AI: Provides `git remote add` command
- Student: "Push my code to GitHub"
- AI: Guides authentication (token or SSH) and `git push`
- Student: Verifies on GitHub website

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI
- **Scenario**: "Help me push my project to GitHub for backup and sharing"
- **Prompts**:
  1. "I created a GitHub repository. Help me connect my local code to GitHub"
  2. "What's the difference between SSH and Personal Access Token authentication?"
  3. "I'll use a Personal Access Token. Show me how to generate one"
  4. "Push my code to GitHub using the token"
  5. "Verify that my code is now on GitHub"
- **Expected Outputs**: Student creates GitHub repo, authenticates, pushes code, and sees it online
- **Safety Note**: "Personal Access Tokens are sensitive. Don't commit them to git; use .gitignore for config files with secrets."

---

### Lesson 7: Pull Requests and Code Review (20 minutes)

**Objective**: Create pull requests documenting AI assistance, review changes, address feedback, and merge changes professionally.

**Note**: Focuses on essential PR workflow - create, review, merge. Optional: extended feedback cycle.

**Skills Taught**:
- Create Pull Request — A2 — Technical — Student can create PR describing changes and AI assistance
- Review Changes — A2 — Technical — Student can review code diffs, understand changes, and approve/request changes
- Address Feedback — A2 — Technical — Student can update PR based on review feedback and push updates
- Merge Safely — A1 — Technical — Student can merge PR and clean up branch

**Key Concepts** (max 5):
1. Pull Request (proposed changes for review)
2. Diff (code changes visible in PR)
3. Code Review (feedback on changes)
4. Discussion (comments on specific lines)
5. Merge (combine PR to main after approval)

**Prerequisites**: Lesson 5 (branching), Lesson 6 (GitHub integration)

**Duration**: 20 minutes

**Content Outline**:

#### What is a Pull Request?

**Definition**: A formal request to merge a branch into main, with discussion and review.

**Why PRs Matter**:
- In teams: Allows code review before merging
- In AI-assisted development: Document what AI helped build
- For safety: "4-eyes" check before merging to main
- For learning: Explain your decisions

#### Pull Request Workflow:

**Step 1: Create Branch and Make Changes**
- Create feature branch (e.g., `improve-login`)
- Make changes (AI-assisted or manual)
- Commit with descriptive messages
- Push branch to GitHub

**Step 2: Open Pull Request on GitHub**
- Go to repository on GitHub
- Click "Compare & pull request" (GitHub suggests after pushing new branch)
- Or: Click "Pull Requests" tab → "New Pull Request"
- Select: Compare branch (`improve-login`) to base branch (`main`)
- Write PR title: "Improve login error messages"
- Write PR description:
  - What changes are included
  - Why are they needed
  - How to test
  - **NEW**: What AI assistance was used (tools, prompts)
- Click "Create Pull Request"

**Step 3: Code Review and Discussion**
- GitHub shows diff of all changes
- Reviewers can comment on specific lines
- AI and student discuss feedback
- Request changes if needed, or approve

**Step 4: Address Feedback**
- Student makes requested changes locally
- Commits: `git commit -m "Address review feedback"`
- Pushes: `git push origin improve-login`
- PR automatically updates (shows new commits)
- Reviewer re-reviews and approves

**Step 5: Merge Pull Request**
- Click "Merge Pull Request" button on GitHub
- Choose merge strategy (merge commit, squash, rebase—default is fine)
- Click "Confirm merge"
- Branch is merged to main
- GitHub suggests deleting feature branch (click delete)

**Step 6: Pull Latest Locally**
- Switch to main: `git checkout main`
- Pull updates: `git pull origin main`
- Feature branch no longer needed

#### Example PR Description (with AI assistance documented):

```
## Title: Add calculator with error handling

## Description
Added a calculator module that supports basic math operations (+, -, *, /) with proper error handling for division by zero.

## What AI Helped With
- Generated initial function implementations from docstrings
- Suggested error handling patterns
- Generated unit tests using pytest

## How to Test
Run the tests: `python -m pytest test_calculator.py`
Or test manually:
  python
  from calculator import add
  print(add(2, 3))  # Should print 5

## Related Issues
Closes #15
```

**Content Elements**:
- Step-by-step PR creation on GitHub (with screenshots)
- Example PR description showing AI assistance documentation
- Code review cycle (request changes, update, re-review)
- Merge options explained (merge commit vs squash)
- Best practices: Small PRs, descriptive messages, document AI help

**Practice Approach**:

*Hands-On Exercise*:
1. Create feature branch: `git checkout -b add-feature`
2. Make code changes
3. Commit: `git commit -m "Add feature X"`
4. Push: `git push origin add-feature`
5. Go to GitHub, click "Compare & pull request"
6. Write PR description with AI assistance documented
7. Create PR
8. (Optional/Simulated) Create another local branch as "reviewer"
9. Review changes in PR (click "Files changed" tab)
10. Leave comment: "Looks good, but can you add a test?"
11. Go back to `add-feature` branch, make changes
12. Commit and push again
13. PR auto-updates with new commits
14. Merge PR on GitHub
15. Delete feature branch
16. Pull updates locally

*Conversational Practice*:
- Student: "I completed a feature with AI help. Now what?"
- AI: "Create a pull request to get feedback"
- Student: (Creates PR with description)
- Student: "How do I document that AI helped?"
- AI: Shows PR description template with AI assistance section
- Student: "I got feedback. How do I update my PR?"
- AI: Shows how to make changes, commit, push (PR auto-updates)

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code or Gemini CLI
- **Scenario**: "I built a feature with your help. Let's create a pull request to document the changes"
- **Prompts**:
  1. "Create a pull request for the feature branch I just pushed"
  2. "Help me write a PR description that explains what I built and what AI helped with"
  3. "Show me the diff so I can review what I'm about to merge"
  4. "I got feedback: 'Add more error handling.' Make those changes and update the PR"
  5. "The PR is approved. Merge it to main and clean up"
- **Expected Outputs**: Student creates PR with clear description of AI assistance, handles feedback, and merges
- **Safety Note**: "PRs are a safety mechanism. Before merging to main, always review the diff. Make sure AI generated what you asked for."

---

### Lesson 8: IDE Setup and Integration (20 minutes)

**Objective**: Install a modern IDE (VS Code or alternative), configure Git integration, and integrate AI coding extensions for professional development environment.

**Note**: Focuses on 3 IDE options (VS Code, Cursor, Zed) to respect Tier 1 cognitive load limits. VS Code recommended.

**Skills Taught**:
- Choose and Install IDE — A1 — Technical — Student can select appropriate IDE, install on their OS, and launch successfully
- Configure Git in IDE — A1 — Technical — Student can enable Git integration and see visual diff/commit tools
- Install AI Extensions — A2 — Technical — Student can install and activate GitHub Copilot, Cursor AI, or Continue extension
- Use IDE Git Tools — A2 — Technical — Student can perform Git operations through IDE interface (stage, commit, push)

**Key Concepts** (max 5):
1. IDE (Integrated Development Environment - editor + tools)
2. Source Control panel (built-in Git visualization)
3. Diff viewer (side-by-side change visualization)
4. Git extensions (GitLens, Git Graph for enhanced features)
5. AI coding assistant (built-in or extension)

**Prerequisites**: Lesson 2 (Git installed), Lesson 6 (GitHub connected)

**Duration**: 20 minutes

**Content Outline**:

#### Part A: Choose and Install IDE (8 min)

**IDE Comparison Table** (3 options for Tier 1 cognitive load):

| IDE | Best For | Download | Key Features |
|-----|----------|----------|--------------|
| **VS Code** (Recommended) | Beginners, flexible | code.visualstudio.com | Most popular, great extensions, free, lightweight |
| **Cursor** | AI-first development | cursor.sh | Built-in Claude integration, AI-native editor |
| **Zed** | Modern, fast | zed.dev | Fast, minimal, growing ecosystem |

**Recommendation**: VS Code for most beginners (most popular, best extension ecosystem)

**Installation Steps** (Tier 1: max 2 options, so focus on VS Code primary):

**For VS Code**:
- Download: code.visualstudio.com → Select OS (Windows/Mac/Linux)
- Windows: Run .exe installer, choose installation options (defaults OK), finish
- Mac: Download .dmg, drag to Applications folder
- Linux: Download .deb/.rpm or use package manager
- Launch: Open VS Code, see Welcome screen
- First steps: Choose theme, install Git extension if needed (usually auto-enabled)
- Verification: Click Source Control icon (left sidebar) and see "Initialize Repository"

**For Cursor** (AI-first alternative):
- Download: cursor.sh → Select OS
- Installation similar to VS Code
- Built-in Claude integration (sign in with Anthropic API key)
- Git already integrated

**Screenshot**: Each step of installation process, final welcome screen

#### Part B: Configure Git Integration (6 min)

**Automatic Git Detection**:
- When you open a folder with a `.git` directory, VS Code automatically recognizes it
- Shows Source Control icon in left sidebar (highlighted if changes exist)

**Visual Git Tools**:
1. **Source Control Panel** (left sidebar)
   - Click Source Control icon (branching symbol)
   - Shows current branch
   - Shows changed files
   - Can stage files with checkboxes
   - Can write commit message
   - Can commit from GUI (no command line needed)

2. **Diff Viewer**
   - Click on a file in Source Control panel
   - See side-by-side diff (before → after)
   - Understand exactly what changed

3. **Git Graph Extension** (optional but helpful)
   - Install "Git Graph" extension from VS Code marketplace
   - Shows visual commit history with branches
   - Can merge, checkout, delete branches graphically

4. **GitLens Extension** (optional)
   - Shows blame (which commit last modified each line)
   - Helpful for understanding code history

**Setup**:
- Open VS Code
- Click Source Control icon (left sidebar)
- If repo already initialized: Shows changed files
- If not: Click "Initialize Repository" button
- Verify Git is detected (shows branch name, file list)

**Content Elements**:
- Screenshots of each IDE UI element
- Step-by-step extension installation (search, install, reload)
- Visual diff example (showing +/- lines side-by-side)
- Commit through GUI (no command line needed)

#### Part C: Install AI Coding Extensions (6 min)

**Option 1: GitHub Copilot** (Popular, paid)
- Cost: ~$10/month or free for students
- How: VS Code Extensions → Search "GitHub Copilot" → Install → Sign in with GitHub
- Usage: Start typing code, see AI suggestions (press Tab to accept)
- Pro: Very capable, lots of examples online

**Option 2: Cursor AI** (If using Cursor IDE)
- Built-in, no installation needed
- Sign in with Anthropic account
- Inline code suggestions as you type

**Option 3: Continue** (Free, open-source)
- VS Code Extensions → Search "Continue" → Install
- Works with open-source models or paid APIs
- Good middle ground: free to try, flexible

**Installation Steps** (Choose 1 for beginner):
- VS Code → Extensions icon (left sidebar)
- Search extension name (e.g., "GitHub Copilot")
- Click "Install"
- Click "Sign in" and authenticate
- Reload VS Code
- Create new file and start typing (see AI suggestions)
- Test: Write a comment like `# Function to add two numbers`, see AI generate code

**Setup Verification**:
- Create test Python file
- Write function comment
- See AI suggestion appear
- Accept suggestion with Tab key
- AI extension is working!

**Content Elements**:
- Three AI extension options (prices, capabilities)
- Installation screenshots
- Example: Comment → AI suggestion workflow
- Safety: "Always review AI-generated code. Don't blindly accept suggestions."

#### Part D: Perform Git Operations in IDE (5 min)

**Common Workflows**:

1. **Stage and Commit Through GUI**:
   - Edit file in editor
   - Source Control panel shows file as changed
   - Click file → see diff
   - Click + icon next to filename to stage
   - Type message in "Message" box
   - Click ✓ (checkmark) to commit
   - No command line needed!

2. **Push Through GUI**:
   - Click "..." menu in Source Control panel
   - Select "Push"
   - GitHub receives commits
   - No command line needed!

3. **Create Branch Through GUI**:
   - Click "..." menu → "Create Branch"
   - Type branch name
   - Switch between branches easily

4. **View Diff Before Committing**:
   - Click filename in Source Control panel
   - See side-by-side diff
   - Understand exactly what will be committed

**Content Elements**:
- Screenshots of each operation
- Comparison: "You can use CLI OR GUI—both work, use whichever is comfortable"
- Emphasis: IDE tools make Git visual and easier for beginners

**Practice Approach**:

*Hands-On Exercise*:
1. Download and install VS Code (or chosen IDE)
2. Launch and configure theme
3. Open existing Git repository folder
4. Verify Source Control panel shows repo and branch
5. Make changes to a file
6. Click Source Control → see file listed as changed
7. Click file → see diff
8. Click + icon to stage
9. Type commit message
10. Click ✓ to commit
11. View commit in Source Control history
12. (Optional) Install Git Graph extension, see visual history
13. (Optional) Install GitHub Copilot, test AI suggestions

*Conversational Practice*:
- Student: "I have VS Code but don't know how to use Git in it"
- AI: "Let me show you the Source Control panel"
- AI: Screenshots and walkthrough
- Student: "Can AI help me code in the IDE?"
- AI: "Install GitHub Copilot or Continue extension"
- AI: Shows installation steps and how to use

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (if using IDE) or Gemini CLI
- **Scenario**: "Help me set up my IDE with Git integration and AI coding assistance"
- **Prompts**:
  1. "I'm installing VS Code for the first time. Walk me through the setup"
  2. "I opened my Git project in VS Code. How do I see my changes?"
  3. "Explain the Source Control panel and how to commit"
  4. "Help me install and set up a GitHub Copilot alternative (GitHub Copilot, Cursor AI, or Continue)"
  5. "Now write a test function in the IDE and watch AI suggestions"
- **Expected Outputs**: Student installs IDE, configures Git integration, installs AI extension, and uses it
- **Safety Note**: "IDE tools are visual and friendly, but they're doing the same Git commands we learned. Understanding the command line helps you recover if something goes wrong."

---

## Part II Summary (Lessons 5–9)

**Time**: 100 minutes (20+20+20+20+20)

**Learning Outcomes**:
- Create and manage Git branches safely
- Connect local repository to GitHub and authenticate
- Create pull requests with clear documentation
- Review and merge changes professionally
- Use modern IDE with Git integration
- Install and use AI coding extensions
- Perform Git operations through IDE GUI or CLI interchangeably
- Complete capstone project demonstrating full Git workflow with AI

**Cognitive Load**: A2 level (some complexity, but still manageable for beginners)

---

### Lesson 9: Capstone Exercise – Build with Git & GitHub (20 minutes)

**Objective**: Integrate all Git, GitHub, and AI skills in a realistic capstone project demonstrating complete workflow.

**Note**: AI generates ALL code - students focus on Git workflow and safety practices. No coding knowledge assumed.

**Skills Applied**:
- Project Setup — A2 — Technical — Student can initialize project with Git
- AI-Assisted Development — A2 — Technical — Student uses AI to generate code while Git tracks progress
- Branch-Based Testing — A2 — Technical — Student creates branches for AI experiments
- Safety Checkpoints — A2 — Soft — Student commits before major changes and reviews AI output
- GitHub Portfolio — A2 — Technical — Student connects project to GitHub and creates PR

**Scenario**: "Build a small Python calculator with AI assistance. AI writes ALL code; you manage Git for safety. Push to GitHub. Create a pull request showing your work."

**Duration**: 20 minutes (guided exercise, can be extended for homework)

**Content Outline**:

#### Capstone Workflow:

**Step 1: Project Initialization** (3 min)
- Create new folder: `mkdir my-calculator`
- Navigate: `cd my-calculator`
- Initialize Git: `git init`
- Create README: `touch README.md` (explain project briefly)
- Commit: `git commit -m "Initial project setup"`
- Verification: `git log` shows initial commit

**Step 2: Basic Implementation with AI** (5 min)
- **You**: "Generate a Python calculator module with functions for add, subtract, multiply, divide"
- **AI**: Generates complete code (students don't write code, just review it)
- **You**: Save AI output to `calculator.py`
- **You**: Commit: `git commit -m "Add calculator module with basic operations"`
- **Focus**: Understanding Git workflow, NOT writing code

**Step 3: Test on Feature Branch** (5 min)
- **You**: "Add error handling for division by zero"
- Create branch: `git branch feature/error-handling`
- Switch: `git checkout feature/error-handling`
- **AI**: Generates error handling code (student just reviews)
- Commit: `git commit -m "Add error handling to calculator"`
- **You**: "Write unit tests for calculator"
- **AI**: Generates tests (student commits)
- Commit: `git commit -m "Add unit tests"`
- **Focus**: Branch workflow for safe experimentation

**Step 4: Review and Decide** (3 min)
- **You**: "Run the tests. Do they pass?"
- **AI**: Runs tests and reports results
- If tests pass:
  - Switch to main: `git checkout main`
  - Merge: `git merge feature/error-handling`
  - Delete branch: `git branch -d feature/error-handling`
- If tests fail:
  - Ask AI to fix issues, commit, try again

**Step 5: GitHub Integration** (3 min)
- Create repository on GitHub (visit github.com, click "New repository")
- Add remote: `git remote add origin [GitHub URL]`
- Push to GitHub: `git push -u origin main`
- Verify on GitHub website (see your code online!)

**Step 6: Create Pull Request** (4 min)
- Create branch: `git checkout -b feature/final-improvements`
- **You**: "Add docstrings and improve code comments"
- **AI**: Generates improved documentation (student commits)
- Commit: `git commit -m "Improve documentation"`
- Push: `git push origin feature/final-improvements`
- Go to GitHub, create PR
- Write description: "AI generated calculator code, tests, and docs. Used Git for safe workflow."
- Merge PR
- **Success**: Complete project visible on GitHub with professional commit history!

#### Verification Checklist:

Student demonstrates understanding by completing:
- [ ] Git repository initialized and commits created
- [ ] GitHub account with visible repository
- [ ] Python calculator code generated and committed
- [ ] Used AI to generate code (shown in commit messages)
- [ ] Created and tested feature branch safely
- [ ] Merged changes successfully
- [ ] Created and merged pull request
- [ ] Project visible on GitHub with clean commit history
- [ ] PR description documents AI assistance
- [ ] Can explain why each Git step mattered

**Content Elements**:
- Step-by-step walkthrough (can be followed in 30 min or extended)
- Checkpoints to verify success
- All Git operations reviewed and practiced
- AI collaboration integrated throughout
- Real GitHub portfolio piece created

**Practice Approach**:

*Guided Execution*:
- Follow each step sequentially
- Use AI to help with code generation and testing
- Commit frequently (before major changes)
- Review Git log to see commit history
- Verify on GitHub website at end

*Conversational Approach*:
- Student: "Help me build a calculator project with Git tracking"
- AI: Guides through each step
- Student: "Generate the calculator functions"
- AI: Creates code
- Student: "Create a test branch for error handling"
- AI: Sets up branch workflow
- Student: "Push to GitHub"
- AI: Guides authentication and push
- Student: "Create a PR"
- AI: Helps write clear PR description

**End-of-Exercise: Reflection**

After completing capstone, student reflects:

1. **Why Git?**: "Before I committed, I could always undo mistakes. That made me confident experimenting with AI code."

2. **Why Branches?**: "I tested new features on a branch before merging. If something broke, I could discard the branch without affecting main."

3. **Why GitHub?**: "My code is backed up on GitHub. I can show it to others or access it from any computer."

4. **Why Document AI Help?**: "In the PR, I explained what AI did. This helps my future self (and teammates) understand the code's origin."

5. **What's Next?**: "Now I have a foundation. Future chapters will teach more advanced Git (rebasing, conflict resolution) and Python (to improve the calculator code)."

---

## Content Flow & Dependencies

**Lesson Progression** (each builds on previous):

1. **Lesson 1** (Why Git) → Establishes safety motivation
   ↓
2. **Lesson 2** (Setup) → Installs tools needed for Lesson 3
   ↓
3. **Lesson 3** (Daily Workflow) → Core commands needed for Lesson 4
   ↓
4. **Lesson 4** (Safety Net) → Advanced undo patterns; builds safety confidence
   ↓
5. **Lesson 5** (Branches) → Uses Lesson 3 knowledge; prepares for Lesson 6
   ↓
6. **Lesson 6** (GitHub Integration) → Uses Lesson 3 commits; adds remote
   ↓
7. **Lesson 7** (Pull Requests) → Uses Lesson 5 branches and Lesson 6 GitHub
   ↓
8. **Lesson 8** (IDE Setup) → Optional but synergistic; ties all commands to visual tools
   ↓
9. **Lesson 9** (Capstone) → Integrates all skills in realistic project

**Prerequisite Chapters**:
- Chapter 1–4: Understand AIDD mindset
- Chapter 5: Claude Code (AI assistant for commands)
- Chapter 6: Gemini CLI (alternative AI assistant)
- Chapter 7: Bash Essentials (command-line basics)

**Forward References** (mentioned but not fully taught):
- Chapter 9 (Markdown): Used in PR descriptions and README
- Chapter 10 (Prompt Engineering): How to ask AI for Git help effectively
- Chapter 11 (Context Engineering): How to provide Git history as context to AI

---

## Scaffolding Strategy

**Tier 1 (Beginner) Constraints Enforced**:

### Constraint 1: Max 2 Options Per Decision
- Authentication: Personal Access Token OR SSH (not 3+ methods)
- IDE: VS Code primary, mention alternatives but don't overwhelm
- Branching strategy: Simple feature branches only (no rebase, cherry-pick)
- Remote: GitHub only (not GitLab, Bitbucket)

### Constraint 2: Max 5 Concepts Per Lesson
- Lesson 1: Git, time machine, commits, undo, backup (5 concepts)
- Lesson 2: Install, GitHub account, username/email, verification, first-time (5 concepts)
- Lesson 3: Init, status, add, commit, push (5 core operations)
- Lesson 4: Diff, log, reset soft, reset hard, revert (5 undo methods)
- Lesson 5: Branch, main, feature, merge, delete (5 branching concepts)
- Lesson 6: Remote, origin, SSH/token, push, pull (5 GitHub concepts)
- Lesson 7: PR, diff viewer, code review, discussion, merge (5 PR concepts)
- Lesson 8: IDE, source control, diff viewer, extension, AI assistant (5 IDE concepts)
- Lesson 9: Project, integration, workflow, safety, portfolio (integrated review)

### Constraint 3: One Skill Per Lesson
- Lesson 1: Conceptual understanding (no hands-on)
- Lesson 2: Installation and setup (first hands-on)
- Lesson 3: Core workflow (first substantive skill)
- Lesson 4: Safety and undo (builds confidence)
- Lesson 5: Branching (new feature)
- Lesson 6: Remote storage (new feature)
- Lesson 7: Professional workflow (new pattern)
- Lesson 8: IDE integration (new environment)
- Lesson 9: Integration (application of all)

### Constraint 4: Concept-First Pattern (WHAT → WHY → HOW → PRACTICE)

Example from Lesson 3 (git add):

1. **WHAT**: "The staging area is a holding zone for changes you're ready to save"
2. **WHY**: "You can choose WHICH files to include in a commit (maybe you fixed 3 files but only 1 is ready)"
3. **HOW**: "Ask AI: 'Stage the calculator.py file for my next commit' → AI runs `git add calculator.py`"
4. **PRACTICE**: Student uses AI assistant to stage specific files for commit

### Constraint 5: AI-as-Partner Pattern

Emphasis throughout:
- Students don't memorize commands; AI does
- Focus on understanding WHAT Git does, not HOW to type commands
- Natural language interface: "Show me what changed" → AI runs `git diff`
- Students learn to ask questions: "Is this safe?" "Why would I use this?" "Explain the risk"

### Constraint 6: Error Literacy (Safety Focus)

Integrated throughout:
- Explicit warnings before destructive commands (e.g., `git reset --hard`)
- Recovery procedures for common mistakes
- Decision trees ("When should I use...?")
- Safe defaults (e.g., "revert is safer than reset --hard")

---

## Integration Points

### Backward References (Prerequisites):
- **Chapter 1**: AIDD paradigm shift (why safety matters)
- **Chapter 5**: Claude Code (AI assistant for commands)
- **Chapter 6**: Gemini CLI (alternative AI assistant)
- **Chapter 7**: Bash Essentials (terminal navigation: `pwd`, `cd`, `ls`)

### Forward References (Future Chapters):
- **Chapter 9 (Markdown)**: README files, PR descriptions, documentation
- **Chapter 10 (Prompt Engineering)**: How to ask AI to perform Git operations effectively
- **Chapter 11 (Context Engineering)**: Providing commit history as context to AI
- **Chapter 12+ (Python)**: Applying Git to Python projects
- **Chapter 30+ (SDD)**: Using Git within Spec-Driven Development workflow

### Horizontal References (Same Part):
- **Chapter 5–7**: Familiarization with AI tools; Git is the safety mechanism for experiments
- **Chapter 8 (THIS)**: Central to Part 2; every tool (Claude Code, Gemini CLI, Bash) interacts with Git
- Together: "You have AI tools (5–7), now you have the safety net (8) to use them confidently"

---

## Validation Strategy

### For Technical Chapters, Assessments Include:

**Skill Demonstrations** (what students show they can do):

1. **Installation Verification**:
   - [ ] Student shows `git --version` output on their machine
   - [ ] Student shows GitHub account created and login successful
   - [ ] Student shows IDE installed and Git detected

2. **Workflow Demonstration**:
   - [ ] Student initializes repository, creates file, stages, commits, views log
   - [ ] Student creates GitHub repository, adds remote, pushes, verifies on GitHub

3. **Safety Demonstration**:
   - [ ] Student uses `git diff` to review changes before committing
   - [ ] Student uses `git reset --soft` to undo a commit safely (reversible)
   - [ ] Student uses `git branch` to test changes before merging
   - [ ] Student recognizes which commands are dangerous (reset --hard) and when to use them

4. **Branching Demonstration**:
   - [ ] Student creates feature branch, makes changes, merges back to main
   - [ ] Student discards unwanted changes by deleting branch without merging

5. **GitHub/PR Demonstration**:
   - [ ] Student creates pull request with clear description
   - [ ] Student documents AI assistance in PR description
   - [ ] Student handles simulated code review feedback

6. **IDE Integration**:
   - [ ] Student uses Source Control panel to view changes
   - [ ] Student stages and commits through IDE GUI (no CLI required)
   - [ ] Student installs and activates AI extension

### Comprehension Checks (quizzes):

1. **Concept Questions** (identify knowledge):
   - "What does a commit do?"
   - "What's the difference between staging and committing?"
   - "When would you use a branch?"
   - "Why is Git important for AI-assisted development?"

2. **Scenario Questions** (apply knowledge):
   - "You asked AI to refactor your code. AI made 20 changes, but you're not sure they're correct. What's the first thing you do?"
     - *Answer*: `git diff` to review changes
   - "You committed to the wrong branch. You want to undo the commit but keep the changes. What do you do?"
     - *Answer*: `git reset --soft HEAD~1`
   - "You want to test a risky AI refactoring without affecting main. What do you do?"
     - *Answer*: Create feature branch, test, decide merge or discard

3. **Safety Questions** (recognize risks):
   - "You accidentally ran `git reset --hard`. Is your work recoverable?"
     - *Answer*: No, it's permanently deleted. But the committed version is still in GitHub.
   - "You pushed sensitive API keys to GitHub. What should you do?"
     - *Answer*: Revoke the key, never reuse it, and remove from history (advanced topic)

### Success Metrics (from spec):
- [ ] 90% successfully initialize repo, commit, and undo via AI prompts within 30 min
- [ ] 85% create GitHub account and push project within 1 hour
- [ ] 90% install Git successfully on all platforms without external help
- [ ] 80% install and configure IDE with Git integration within 30 min
- [ ] 80% use Git branches to test AI changes safely
- [ ] 95% identify when to commit before AI changes (safety awareness)
- [ ] 75% set up IDE with Git and AI extensions
- [ ] 70% create pull request with AI assistance documented
- [ ] Reading level: Grade 7 or below (Flesch-Kincaid)
- [ ] Post-chapter confidence: 4.5/5 or higher on "I can safely experiment with AI using Git"

---

## Code Examples Needed

Chapter does NOT include extensive code examples (focuses on concepts and commands), but the following specifications are needed for development:

### Example 1: Simple Python File (Lesson 3 - Daily Workflow)
- **Purpose**: Concrete file to commit and track changes
- **Complexity**: Beginner (5-10 lines)
- **Code**: Simple function (hello world or basic math)
- **AI Involvement**: Student asks AI to generate it
- **Validation**: Should be syntactically correct and runnable

### Example 2: Calculator Module (Capstone Exercise)
- **Purpose**: Realistic project with multiple functions
- **Complexity**: Beginner (20-30 lines)
- **Code**: Calculator class/functions with add, subtract, multiply, divide
- **AI Involvement**: AI generates initial code, student tests and improves
- **Validation**: Should pass unit tests

### Example 3: Unit Tests for Calculator (Capstone Exercise)
- **Purpose**: Demonstrate testing AI-generated code
- **Complexity**: Beginner (15-20 lines)
- **Code**: Simple pytest tests for calculator functions
- **AI Involvement**: AI generates tests, student runs them
- **Validation**: Should pass 100% on correct code, fail on broken code

### Example 4: .gitignore File (Implicit in Lesson 2/6)
- **Purpose**: Prevent committing secrets and temp files
- **Complexity**: Beginner (5-10 lines)
- **Code**: Common patterns for Python projects
- **Content**:
  ```
  .env
  secrets.txt
  __pycache__/
  *.pyc
  .venv/
  node_modules/
  ```
- **Explanation**: Each line and why it matters

### Example 5: Sample PR Description (Lesson 7)
- **Purpose**: Show professional documentation
- **Complexity**: Beginner (15-20 lines)
- **Content**: Title, description, testing instructions, AI assistance documented
- **Real example provided in lesson content**

---

## Book Gaps Checklist Coverage

**Per Constitution Section II.C**, the following gaps are addressed:

### Factual Accuracy Requirements:
- [ ] Git installation links verified (git-scm.com, github.com active)
- [ ] GitHub UI screenshots taken from current interface (UI may change)
- [ ] Command syntax correct for Git 2.40+ (current standard)
- [ ] Platform-specific instructions tested on Windows 10+, macOS 10.15+, recent Linux
- [ ] IDE installation links verified and current
- [ ] Python syntax examples compatible with Python 3.13+

### Field Volatility / Maintenance Triggers:
- [ ] **GitHub UI changes**: Screenshots may need updates if GitHub redesigns interface
  - *Trigger*: Major GitHub UI redesign (check annually)
  - *Maintenance*: Re-screenshot key workflows
- [ ] **Git versions**: Command syntax stable, but new flags added over time
  - *Trigger*: Git release with breaking changes (rare)
  - *Maintenance*: Test commands on latest Git version
- [ ] **IDE versions**: VS Code updates monthly; extensions change frequently
  - *Trigger*: Major VS Code UI redesign or extension compatibility break
  - *Maintenance*: Re-screenshot IDE and extension setup annually
- [ ] **Python compatibility**: Chapter doesn't teach Python, but examples use Python 3.13+
  - *Trigger*: Python 3.14 release (2025-10)
  - *Maintenance*: Verify code examples still work with new Python

### Inclusivity / Accessibility Requirements:
- [ ] Three OS platforms covered (Windows, Mac, Linux) — avoids Windows-only bias
- [ ] Three IDE options presented (VS Code, Cursor, Zed) — avoids single-tool dependence while respecting Tier 1 cognitive load
- [ ] AI assistants flexible (Claude Code OR Gemini CLI) — avoids single-vendor lock-in
- [ ] Jargon minimized; "time machine" metaphor used for Git concept
- [ ] Installation links with text descriptions for all steps — accessible without screenshots
- [ ] Platform-specific instructions for each OS — respects OS choice
- [ ] Reading level: Grade 7 or below (verified with Flesch-Kincaid)

### Security Considerations:
- [ ] SSH authentication option explained (not just Personal Access Token)
- [ ] `.gitignore` section prevents accidental API key commits
- [ ] Warning: "Never commit secrets; use environment variables"
- [ ] GitHub security best practices mentioned (strong passwords, 2FA)
- [ ] Advice on revoking leaked credentials (if student commits API keys)

### Technical Depth & Appropriateness for Tier 1:
- [ ] Advanced Git avoided (no rebase, cherry-pick, interactive rebase)
- [ ] Merge conflicts not covered (taught in later chapters)
- [ ] Git internals (plumbing, object database) not discussed
- [ ] Focus on most-used 80/20 workflows, not comprehensive reference

### Real-World Context:
- [ ] All examples reflect actual AI-assisted development (not synthetic scenarios)
- [ ] Branching strategy matches professional practice (feature branches)
- [ ] PR workflow reflects team collaboration norms
- [ ] IDE setup matches production-ready development environments
- [ ] Capstone project is portfolio-ready (can be shown to employers/collaborators)

---

## Pedagogical Approach: AI-Driven Teaching

This chapter embodies the **AI-Driven Development teaching philosophy**:

### Learning WITH AI, Not ABOUT AI
- Students use AI assistants (Claude Code, Gemini CLI) to LEARN Git, not learn Git to eventually use AI
- AI is their co-pilot from Lesson 2 forward
- Every "Try With AI" section leverages AI for interactive learning

### Conversational, Not Imperative
- Students ask AI: "Show me what changed" instead of memorizing "`git diff`"
- Natural language is the primary interface
- Command syntax is secondary (AI handles it)
- Students learn INTENT, not SYNTAX

### Safety-First Methodology
- Every Git operation explained in context of safety for AI development
- Destructive operations (git reset --hard) explicitly warned with recovery advice
- Branching pattern (test before merge) emphasized as safety mechanism
- Capstone reinforces: "Commit before major changes, always review AI diffs, decide merge or discard"

### Progressive Disclosure
- Lesson 1: Why Git matters (motivation)
- Lessons 2–4: Core 5 commands and safety (foundation)
- Lessons 5–8: Advanced patterns (branching, GitHub, IDE, PRs)
- Lesson 9: Integrated project (application)
- Students never feel overwhelmed; complexity increases gradually

### Real Workflows, Not Toy Examples
- All exercises reflect authentic AI-assisted development patterns
- Capstone project (calculator) is realistic and portfolio-worthy
- PR workflow matches professional practice
- IDE setup matches what professionals use

### Two-Part Structure (Concepts + Prompts)
- **Part I (Lessons 1–4)**: Understand Git concepts, perform operations (75 min)
- **Part II (Lessons 5–9)**: AI workflows, GitHub, IDE setup, and capstone (100 min)
- Students learn WHAT (Part I) and HOW TO ASK AI + INTEGRATE (Part II)

---

## Assessment & Learning Objectives Alignment

### Bloom's Taxonomy Mapping:

| Lesson | Learning Objective | Bloom's Level | Assessment |
|--------|-------------------|---------------|----|
| 1 | Recognize why Git matters for AI development | Remember/Understand | Quiz: "What does Git help prevent in AI-assisted development?" |
| 2 | Install Git and create GitHub account | Remember | Demo: Show `git --version` and GitHub profile |
| 3 | Perform core Git workflow (init, status, add, commit) | Apply | Exercise: Create repo, make changes, commit |
| 4 | Safely undo changes at different stages | Apply/Analyze | Scenario: Given broken code, choose correct undo method |
| 5 | Create branches for safe experimentation | Apply | Exercise: Create branch, test changes, merge/discard |
| 6 | Connect local repo to GitHub | Apply | Demo: Push code, verify on GitHub |
| 7 | Create and review pull requests professionally | Apply/Analyze | Exercise: Create PR, address feedback, merge |
| 8 | Use IDE with Git integration and AI extensions | Apply | Demo: Perform Git operations through IDE GUI |
| 9 | Integrate all Git skills in realistic project | Analyze/Create | Capstone: Build project with Git, GitHub, and AI |

### CEFR Proficiency Levels:

| Lesson | Skill | CEFR Level | Measurable Outcome |
|--------|-------|-----------|-------------------|
| 1 | Conceptual Understanding of Version Control | A1 | Student explains "What is Git?" in own words |
| 2 | Install Git on Windows/Mac/Linux | A1 | Student shows `git --version` on their machine |
| 2 | Create GitHub Account | A1 | Student successfully logs into GitHub account |
| 2 | Configure Git Locally | A1 | Student runs `git config --list` and sees settings |
| 3 | Initialize Git Repository | A1 | Student creates `.git` directory in project folder |
| 3 | Track File Changes | A1 | Student stages files and creates commits |
| 3 | Push to Remote | A1 | Student uploads commits to GitHub |
| 4 | View File Changes | A1 | Student uses `git diff` to understand what changed |
| 4 | Undo Uncommitted Work | A1 | Student discards changes with `git checkout` |
| 4 | Undo Committed Work Safely | A2 | Student uses `git reset --soft` and explains why it's safe |
| 5 | Create Feature Branch | A2 | Student branches, makes changes, merges back |
| 5 | Test Before Merging | A2 | Student tests changes on branch before production |
| 6 | Add Remote Repository | A1 | Student connects GitHub repository to local |
| 6 | Authenticate Securely | A2 | Student understands SSH vs Token authentication |
| 6 | Push and Pull | A1 | Student uploads and downloads commits |
| 7 | Create Pull Request | A2 | Student opens PR with clear description |
| 7 | Review Code Changes | A2 | Student understands code diffs in PR |
| 7 | Address Feedback | A2 | Student updates PR based on review comments |
| 8 | Choose and Install IDE | A1 | Student downloads and launches VS Code (or alternative) |
| 8 | Configure Git in IDE | A1 | Student enables Source Control panel and sees repo info |
| 8 | Install AI Extensions | A2 | Student activates GitHub Copilot or Continue |
| 8 | Use IDE Git Tools | A2 | Student stages, commits, and pushes through GUI |
| 9 | Project Integration | A2 | Student builds project with Git tracking end-to-end |

---

## File Organization & Naming Conventions

Following `specs/book/directory-structure.md`:

```
book-source/docs/
├── 02-AI-Tool-Landscape/                    # Part 2 (Title Case with Hyphens)
│   ├── README.md                            # Part intro (UPPERCASE)
│   ├── 08-git-and-github/                   # Chapter 8 directory (lowercase-with-hyphens)
│   │   ├── readme.md                        # Chapter overview (LOWERCASE)
│   │   ├── 01-why-git-matters-with-ai.md   # Lesson 1 (descriptive)
│   │   ├── 02-essential-setup.md            # Lesson 2
│   │   ├── 03-the-daily-workflow.md         # Lesson 3
│   │   ├── 04-safety-net-undoing-changes.md # Lesson 4
│   │   ├── 05-branches-for-experimentation.md # Lesson 5
│   │   ├── 06-github-integration.md         # Lesson 6
│   │   ├── 07-pull-requests-and-code-review.md # Lesson 7
│   │   ├── 08-ide-setup-and-integration.md  # Lesson 8
│   │   └── 09-capstone-exercise.md          # Lesson 9
```

### Lesson File Frontmatter (YAML):

Each lesson `.md` file starts with:

```yaml
---
sidebar_position: [N]                        # Lesson number (1, 2, 3, etc.)
title: "[Descriptive Lesson Title]"          # Clear, specific title
description: "[One-sentence summary]"        # 8-15 words for search/sidebar
keywords: [key, words, for, search]          # Optional: 3-5 keywords
---
```

### Chapter README Structure:

`readme.md` should include:

```markdown
# Chapter 8: Git & GitHub for AI-Driven Development

## Overview
[2-3 paragraphs explaining chapter purpose and learning outcomes]

## Learning Outcomes
[Bullet list of what students can do after this chapter]

## Chapter Structure
[Brief description of lessons and flow]

## Estimated Time
[3 hours total: 175 minutes]

## Prerequisites
[Reference to chapters 1-7]

## Tools You'll Need
[Git, GitHub account, IDE]
```

---

## Estimated Implementation Effort

### Writing Tasks (by lesson):

| Lesson | Type | Effort | Notes |
|--------|------|--------|-------|
| 1 | Conceptual | 2-3h | Hook, narrative, Why Git matters (15 min lesson) |
| 2 | Installation | 3-4h | Platform-specific instructions, links (not screenshots), troubleshooting (20 min) |
| 3 | Technical | 2-3h | Five commands, diagram, examples (20 min) |
| 4 | Technical | 3-4h | Five undo methods, decision tree, safety warnings (20 min) |
| 5 | Technical | 2-3h | Branching concept, workflow, example (20 min) |
| 6 | Technical | 2-3h | Remote setup, authentication, push/pull (20 min) |
| 7 | Technical | 3-4h | PR creation, review, feedback cycle (20 min) |
| 8 | Setup | 3-4h | 3 IDE options only, Git integration, extensions (20 min) |
| 9 | Capstone | 2-3h | Streamlined exercise, AI generates all code (20 min) |

**Total Estimated Effort**: 22-31 hours (3-5 days full-time development)

**Note**: Reduced from original 30-39 hours due to: (1) Shorter lessons (15-20 min vs 25-30 min), (2) 3 IDE options instead of 4, (3) Installation links instead of screenshots, (4) Simplified capstone for non-coders

### Review & Validation Tasks:

- Technical review (Git commands tested): 2-3h
- Installation link verification (all platforms): 1-2h (reduced: no screenshots to capture)
- Peer review (pedagogy and clarity): 2-3h
- Final editing and polish: 2-3h

**Total Review Effort**: 7-11 hours

**Grand Total**: 29-42 hours (realistic estimate for complete chapter)

---

## Quality Assurance Checklist

Before finalizing, verify:

- [ ] All 9 lessons written and follow lesson.md output style
- [ ] YAML frontmatter complete for each lesson
- [ ] Chapter readme.md created and explains structure
- [ ] Lesson objectives are measurable and use Bloom's taxonomy
- [ ] Cognitive load within Tier 1 limits (max 5 concepts/lesson, max 2 options)
- [ ] Each lesson ends with single "Try With AI" section (no additional Key Takeaways)
- [ ] All Git commands tested to ensure accuracy
- [ ] Installation links verified and current for all platforms (Windows, Mac, Linux)
- [ ] Text-based installation descriptions clear without screenshots
- [ ] Troubleshooting section covers edge cases from spec
- [ ] Code examples (calculator, tests) provided with specifications
- [ ] PR description template includes AI assistance documentation
- [ ] IDE setup covers exactly 3 options (VS Code, Cursor, Zed) and recommends VS Code
- [ ] AI prompt templates provided for all Git operations
- [ ] Reading level verified at Grade 7 or below (Flesch-Kincaid)
- [ ] Capstone exercise is realistic and portfolio-ready
- [ ] Accessibility checks: inclusive examples, jargon minimized, images described
- [ ] Book gaps checklist addressed: factual accuracy, field volatility, security, inclusivity
- [ ] Forward references documented (Chapters 9, 10, 11, etc.)
- [ ] Backward prerequisites verified (Chapters 1-7)
- [ ] Success metrics from spec addressed in assessment section
- [ ] All 14 domain skills applied contextually (if applicable—Git chapter may not use all)
- [ ] Constitution v3.0.1 principles honored (evals-first, spec-first, validation-first)

---

## Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| **GitHub UI Changes** | Screenshots become outdated | Create maintenance trigger; annual review; focus instructions on concept over exact steps |
| **Installation Issues Across Platforms** | Students blocked on setup | Comprehensive troubleshooting; multiple installation methods per OS; offer synchronous support |
| **Students Confuse Commands** | Mistakes in Git operations | Emphasize decision trees; clear warnings for destructive commands; practice exercises with recovery |
| **IDE Installation Blocker** | Students can't complete Lesson 8 | Lesson 8 optional (can skip to CLI); IDE has system requirements (RAM, OS version) noted upfront |
| **Authentication Complexity** | Students stuck on SSH/Token setup | Recommend Personal Access Token first (simpler); SSH as optional advanced step |
| **Merge Conflicts Not Covered** | Student assumes PRs are simple | Note in forward references: "Merge conflicts taught in later chapters" |
| **Reading Level Too High** | Beginners struggle with jargon | Use simple words, metaphors (time machine), define technical terms clearly |
| **Capstone Too Long** | Students don't complete within 30 min | Scaffold with checkpoints; offer extended homework version; AI assistance speeds execution |

---

## Next Steps After Planning

1. **Lesson Writing** (content-implementer subagent)
   - Implement each lesson following this plan
   - Create code examples and test
   - Capture screenshots for installation steps
   - Validate "Try With AI" prompts with real AI assistants

2. **Content Review**
   - validation-auditor: Verify Git commands accuracy
   - factual-verifier: Check grammar, style, reading level
   - pedagogical-reviewer: Ensure learning objectives met

3. **Testing & Validation**
   - Platform testing: Verify instructions on Windows, Mac, Linux
   - User testing: Beginner user follows lesson; note friction points
   - AI testing: Verify prompt templates work with Claude Code and Gemini CLI
   - Technical testing: All code examples run successfully

4. **Publication**
   - Docusaurus build test
   - Visual inspection (formatting, images, code blocks)
   - Cross-reference validation with related chapters
   - SEO optimization (keywords, descriptions)

---

## Conclusion

This plan breaks Chapter 8 into 9 manageable lessons appropriate for complete beginners (Tier 1) learning Git and GitHub in the context of AI-assisted development. The chapter emphasizes **safety, concepts over syntax, and AI-as-partner**, following AIDD principles throughout.

Key differentiators:
- **Conversational approach**: Students ask AI to do Git; don't memorize commands
- **Safety-first**: Extensive coverage of undo methods and branch-based testing
- **Real workflows**: Capstone project is portfolio-ready, not toy example
- **Two-part structure**: Part I teaches concepts; Part II teaches AI prompts
- **Accessibility**: Screenshots, multiple OS platforms, jargon minimized, Grade 7 reading level

The lesson sequence respects cognitive load limits (max 5 concepts, max 2 options per lesson) while progressively building toward professional GitHub workflows and IDE integration.

Ready for content-implementer subagent implementation.
