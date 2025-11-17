---
title: "GitHub Integration"
chapter: 8
lesson: 6
sidebar_position: 6
duration_minutes: 20
description: Connect your local Git repository to GitHub, authenticate securely, and keep your code backed up online.

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Add Remote Repository"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can connect a local Git repository to a GitHub repository using git remote commands"

  - name: "Authenticate Securely"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student understands the difference between SSH and Personal Access Token authentication and can choose appropriate method"

  - name: "Push and Pull"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can upload commits to GitHub (push) and download updates (pull) to keep local and remote in sync"

learning_objectives:
  - objective: "Connect a local Git repository to a GitHub repository"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Exercise demonstrating successful remote connection"

  - objective: "Authenticate to GitHub using a secure method (Personal Access Token recommended)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of authentication choice and successful authentication"

  - objective: "Push commits to GitHub and verify they appear online"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Exercise showing code on GitHub after push"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (remote, origin, SSH, Personal Access Token, push/pull) at A1-A2 limit ✓"

differentiation:
  extension_for_advanced: "Explore SSH key setup and compare security models between PAT and SSH; research GitHub security best practices"
  remedial_for_struggling: "Use AI assistant to guide each step of remote setup and authentication; practice with simple test files before pushing real projects"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# GitHub Integration

Your code is only on your computer. What if:
- Computer breaks → code is lost
- Work from different computer → code is trapped
- Share your work → can't access easily

**Solution**: GitHub - cloud backup for your code.

**Time**: 20 minutes

---

## What Is GitHub?

GitHub is like Google Drive for code.

**Simple explanation**:
- Code on computer = local
- Upload to GitHub = cloud backup
- Access anywhere = work from any device

**Why this matters**:
- Safe backup (computer dies, code survives)
- Work from anywhere (laptop, desktop, phone)
- Professional portfolio (employers check GitHub)
- Collaboration (work with teams)

---

## Step 1: Create Repository on GitHub

Do this on the GitHub website:

1. Go to **github.com**
2. Click **"New repository"** (green button or "+" in top right)
3. Fill in:
   - **Name**: `my-project`
   - **Description**: "My first project" (optional)
   - **Public** (visible to everyone)
4. Click **"Create repository"**

GitHub shows you a page with setup commands. Don't worry - Gemini CLI will handle it.

---

## Step 2: Connect Local Code to GitHub

Link your local repository to the GitHub repository.

**You ask Gemini CLI**: "Connect my local code to GitHub repo https://github.com/username/my-project"

Gemini runs: `git remote add origin https://github.com/username/my-project.git`

Creates connection between your local Git and GitHub.

**Check it worked**:

**Ask Gemini CLI**: "Show me my remote repositories"

Gemini runs: `git remote -v`

You should see `origin` pointing to your GitHub URL.

---

## Step 3: Set Up Authentication

GitHub needs to verify it's really you uploading code.

**Method: Personal Access Token** (recommended for beginners)

### Creating Your Token

1. Go to **github.com → Your profile picture → Settings**
2. Scroll down → **Developer settings**
3. **Personal access tokens → Tokens (classic)**
4. Click **"Generate new token"**
5. Name it: "My computer"
6. Check **"repo"** (gives access to repositories)
7. Click **"Generate token"**
8. **Copy the token immediately** (starts with `ghp_...`)

⚠️ **Save it now** - you won't see it again!

**Where to save**: Password manager, secure note file, or write it down safely.

---

**Security notes**:
- ⚠️ Never share your token (like a password)
- ⚠️ If exposed, delete it on GitHub immediately
- ✅ Create new tokens for different computers
- ✅ Delete tokens when you stop using a device

---

## Step 4: Push Code to GitHub

Upload your commits to GitHub.

**First time push**:

**You ask Gemini CLI**: "Push my code to GitHub for the first time"

Gemini runs: `git push -u origin main`

GitHub prompts for authentication:
- **Username**: Your GitHub username
- **Password**: Paste your token (not your GitHub password!)

Code uploads to GitHub.

**Check it worked**: Visit `github.com/username/my-project` - you should see your files and commits.

---

**Subsequent pushes** (after first time):

**Ask Gemini CLI**: "Push my latest commits to GitHub"

Gemini runs: `git push`

No authentication prompt needed (Git remembers).

---

## Pull: Download from GitHub

Get the latest code from GitHub to your local computer.

**You ask Gemini CLI**: "Pull the latest code from GitHub"

Gemini runs: `git pull`

Downloads any changes from GitHub to your local repository.

**When to pull**:
- Start of work session
- Before making new changes
- Working from different computer
- Someone else pushed changes

---

## Working Across Multiple Computers

**First computer (laptop)**:

1. Create code locally
2. Commit changes
3. **Push to GitHub**: `git push`

**Second computer (desktop)**:

1. Clone the repository:

**Ask Gemini CLI**: "Download my GitHub repo https://github.com/username/my-project"

Gemini runs: `git clone https://github.com/username/my-project.git`

2. Make changes
3. Commit
4. **Push to GitHub**: `git push`

**Back to first computer (laptop)**:

1. **Pull from GitHub**: `git pull`
2. All desktop changes are now on laptop
3. Continue working

---

## The Complete Workflow

```
Day 1 (Laptop):
  → Write code
  → Commit: git commit -m "Add feature"
  → Push: git push

Day 2 (Desktop):
  → Pull: git pull (get laptop's work)
  → Write more code
  → Commit: git commit -m "Fix bug"
  → Push: git push

Day 3 (Laptop):
  → Pull: git pull (get desktop's work)
  → All changes are here!
```

No USB drives, no email, no confusion. Your code follows you everywhere.

---

#### 💬 AI Colearning Prompt
> "Why does GitHub require authentication (tokens or SSH) instead of just using my GitHub username and password like a normal website?"

---

## Key Commands Reference

| Task | Command |
|------|---------|
| Connect to GitHub | `git remote add origin [URL]` |
| Check remote connection | `git remote -v` |
| First push | `git push -u origin main` |
| Regular push | `git push` |
| Pull from GitHub | `git pull` |
| Clone repository | `git clone [URL]` |
| Check push status | `git status` |

Ask Gemini CLI in natural language - you don't need to memorize these.

---

## Troubleshooting Common Issues

**"Authentication failed"**:
- Double-check you're using token (not password)
- Token must have "repo" permission
- Try creating a new token

**"Repository not found"**:
- Check URL spelling
- Repository must exist on GitHub first
- Your GitHub username is correct

**"Updates were rejected"**:
- Someone else pushed first
- Pull first: `git pull`
- Then push: `git push`

**Token lost**:
- Create a new token on GitHub
- Delete the old one (security)
- Use new token when pushing

---

## Safety Guidelines

**Always**:
- Push at end of work sessions
- Pull before starting work
- Commit before pushing
- Keep tokens secure

**Never**:
- Share your tokens
- Commit your token to code
- Use GitHub password for Git commands (use token)
- Force push without understanding consequences

---

#### 🎓 Expert Insight
> In AI-native development, you don't memorize GitHub connection commands—you understand cloud backup intent. When you want to sync with GitHub, you specify "connect to my repo and push" and AI handles `git remote add origin / git push -u origin main`. Your value is understanding WHEN to sync (end of session, before switching computers), not memorizing remote/origin/upstream syntax.

---

#### 🤝 Practice Exercise

> **Ask your AI**: "Walk me through connecting a local repository to GitHub: create the remote connection, do the first push, then pull to verify synchronization. Explain what 'origin' and 'main' mean."

**Expected Outcome**: You'll understand GitHub integration isn't about commands—it's about the relationship between local (your computer) and remote (GitHub cloud). Push = upload your commits. Pull = download others' commits. Origin = the GitHub copy. Main = the primary branch.

---

## Try With AI

Practice GitHub integration.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: Connect Repository

```
I created a repository on GitHub: https://github.com/myusername/test-project
Connect my local Git repository to this GitHub repository.
Then verify the connection worked.
```

### Exercise 2: Token Setup

```
Walk me through creating a Personal Access Token on GitHub.
What permissions do I need?
Where should I save it securely?
```

### Exercise 3: First Push

```
I have my token ready.
Push my local commits to GitHub for the first time.
Guide me through authentication if GitHub asks.
Then verify my code appears on GitHub.
```

### Exercise 4: Push and Pull Practice

```
Simulate this workflow:
1. Make a small change locally
2. Commit it
3. Push to GitHub
4. Pull from GitHub (should say "already up to date")
5. Verify everything is in sync
```

### Exercise 5: Clone to New Location

```
Pretend I'm on a new computer.
How do I download my repository from GitHub?
Walk me through the clone process.
```

### Exercise 6: Troubleshooting

```
What should I do if:
1. Push says "authentication failed"
2. Push says "rejected - fetch first"
3. I lost my token

Explain each solution.
```
