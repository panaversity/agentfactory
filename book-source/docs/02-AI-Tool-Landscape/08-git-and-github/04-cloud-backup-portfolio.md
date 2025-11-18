---
sidebar_position: 4
title: "Cloud Backup & Portfolio - GitHub Integration"
description: "Connect your local repository to GitHub for cloud backup and portfolio building through AI collaboration with Three Roles framework"
duration_minutes: 40

# HIDDEN SKILLS METADATA
skills:
  - name: "Create GitHub Account"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can create free GitHub account and navigate web interface"

  - name: "Understand Remote Repositories"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain GitHub as cloud copy of local repository"

  - name: "Connect Local to Remote Repository"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can execute git remote add origin <URL> and verify connection"

  - name: "Push Code to GitHub"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can push commits to GitHub using git push with authentication"

learning_objectives:
  - objective: "Connect local repository to GitHub remote using git remote add"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student executes command and verifies with git remote -v"

  - objective: "Push commits to GitHub using git push -u origin main"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student successfully pushes and verifies code appears on GitHub"

  - objective: "Clone repository from GitHub using git clone"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student clones project from different directory and verifies recovery"

  - objective: "Explain GitHub as cloud backup and portfolio platform"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Student articulates dual value (catastrophe prevention + career showcase)"

cognitive_load:
  new_concepts: 4
  concepts_list:
    - "GitHub accounts (cloud platform for hosting repositories)"
    - "Remote repositories (cloud-hosted copy of local repository)"
    - "Push (uploading commits to cloud)"
    - "Clone (downloading repository from cloud)"
  assessment: "4 concepts (within A1 limit of 5-7) ✓"

teaching_approach: "AI Collaboration with Three Roles (Teacher/Student/Co-Worker)"
modality: "Three Roles demonstration with hands-on activities"
# stage: "2 (AI Collaboration with bidirectional learning)" # Internal scaffolding - hidden from students
# ai_involvement: "AI as collaborator demonstrating all three roles (teacher, student, co-worker)" # Internal scaffolding
# three_roles_validation: "✓ AI teaches backup+portfolio dual value (Role 1), Student teaches constraints (Role 2), Convergence through recovery testing (Role 3)" # Internal scaffolding

# Generation metadata
generated_by: "content-implementer v1.0.0 (reasoning-activated)"
source_spec: "specs/028-chapter-8-git-redesign/spec.md (US2 Priority P2)"
source_plan: "specs/028-chapter-8-git-redesign/plan.md (Lesson 4 details, lines 265-320)"
source_tasks: "specs/028-chapter-8-git-redesign/tasks.md (T044-T057)"
created: "2025-01-17"
last_modified: "2025-01-17"
version: "1.0.0"
constitution_version: "6.0.0"
---

# Cloud Backup & Portfolio - GitHub Integration

## Backup Your Work + Build Your Portfolio

What if your computer crashes tomorrow? All your projects, gone.

**GitHub prevents this.** It's a cloud platform that stores your code online—safe from hardware failures.

**Bonus:** GitHub is also your portfolio. Share your GitHub profile with employers and they see:
- Your actual projects (not just claims on a resume)
- Your commit history (proof you build things)
- How you solve problems

In this lesson, you'll:
1. Create a free GitHub account
2. Connect your local Git repository to GitHub's cloud
3. Push your commits to the cloud (backup)
4. Clone a project to verify recovery works
5. Experience **Three Roles** of AI collaboration: AI teaching you backup strategies, you correcting AI's approach, and together validating the system works

**By the end**, your project will be:
- ✅ Protected in the cloud (catastrophe prevention)
- ✅ Shareable on your resume (career showcase)
- ✅ Recoverable from any computer (tested and verified)

---

## Part 1: Manual Foundation — Create GitHub Account

Before involving AI, you'll set up GitHub manually so you understand the platform.

### Activity 1.1: Create Free GitHub Account

1. Open your browser and visit **github.com**
2. Click the green "Sign up" button (top right)
3. Enter your email address
4. Create a password (GitHub will check strength—make it secure)
5. Choose a username (this will be in your portfolio URL: github.com/yourname)
6. Complete email verification

**What You're Creating**: A cloud identity where your projects will live. Your GitHub username becomes your public developer identity.

### Activity 1.2: Create New Repository on GitHub

1. Click the **+** icon (top right of GitHub) → Select "New repository"
2. **Repository name**: Use same name as your local project (e.g., `my-first-project`)
3. **Description**: Optional, but helpful (e.g., "First Git project with AI experimentation")
4. **Public**: Select this so your portfolio is visible (private is option for sensitive projects)
5. **Initialize repository**: Uncheck "Add a README" (we already have files locally)
6. Click **"Create repository"**

**What You're Creating**: An empty GitHub repository ready to receive your local code.

---

## Connecting Your Repository with AI

Now you'll work with AI to handle GitHub setup complexity (authentication, remote configuration, potential errors). You have a local Git repository with commits—let's connect it to GitHub and push the code with AI guidance.

---

## Learning Dual Backup+Portfolio Value

**Your Question to ChatGPT/Claude**:

```
I have a local Git repository on my computer and just created a GitHub
repository. I want to connect them. How do I back up my code to GitHub?
```

**What AI Might Suggest**:

> "You can push your local repository to GitHub as a backup. But here's
> something useful: GitHub isn't just backup—it's also your portfolio.
> When you share your GitHub profile link on your resume or LinkedIn,
> employers see all your projects, commits, and collaboration history.
> It demonstrates your real development skills."

#### 💬 AI Colearning Prompt
> "Explain why GitHub serves as both backup and portfolio. What specific features make it useful for each purpose?"

**What You Learned**: AI taught you that backup and portfolio are dual values of GitHub. You asked about backup, but AI added the career dimension you hadn't considered.

**This is Role 1: AI as Teacher**—It suggests patterns beyond what you explicitly asked.

---

## Role 2: AI as Student — You Teach AI Your Constraints

**Your Refinement to AI**:

```
Wait—before you help me push to GitHub, I need to be careful. My project
has a config.json file with API secrets. I don't want to push that to GitHub.
How do I avoid accidentally sharing secrets?
```

**What AI Should Adapt**:

> "Good catch—that's a critical security concern. Before pushing, you should
> create a `.gitignore` file that tells Git to ignore `config.json`. Here's
> the approach:
>
> 1. Create `.gitignore` file in your project root
> 2. Add `config.json` to it
> 3. Run `git add .gitignore && git commit -m "Add gitignore for secrets"`
> 4. Then push to GitHub
>
> Now your secrets stay local, and only the public code goes to GitHub."

#### 🎓 Expert Insight
> In AI-native development, you don't memorize `.gitignore` patterns—you understand that secrets and credentials must never be committed. AI can suggest patterns, but you verify nothing sensitive gets pushed.

**What AI Learned**: Your security constraint (secrets must stay local). AI adapted its approach from naive "push everything" to "protect secrets first."

**This is Role 2: AI as Student**—It learns your constraints and refines its output.

---

## Part 3: Hands-On Activities — Connect and Push

Now execute the GitHub connection with AI guidance. Follow these activities while AI helps verify each step.

### Activity 3.1: Create .gitignore (If Needed)

If your project has configuration files with secrets, protect them:

```bash
# In your project root, create .gitignore
echo "config.json" > .gitignore
echo ".env" >> .gitignore

# Commit the gitignore
git add .gitignore
git commit -m "Add gitignore for secrets"
```

**What these commands do:**
- `echo "config.json" > .gitignore` = create `.gitignore` file with "config.json" as first line
- `echo ".env" >> .gitignore` = append ".env" as second line (the `>>` adds without replacing)
- `.gitignore` file format: one filename per line (any files matching these names won't be uploaded)

**Why**: `.gitignore` tells Git which files to ignore. Secrets stay on your computer, never uploaded to GitHub.

### Activity 3.2: Connect Local to Remote Repository

Copy your GitHub repository URL (from the green "Code" button on GitHub), then:

```bash
# Add GitHub as your remote repository
git remote add origin https://github.com/yourname/your-repo.git
```

Replace `yourname` and `your-repo` with your actual GitHub username and repository name.

**Verify the connection**:

```bash
git remote -v
```

**You should see**:

```
origin  https://github.com/yourname/your-repo.git (fetch)
origin  https://github.com/yourname/your-repo.git (push)
```

**What This Means**: GitHub is now registered as your remote repository. The name "origin" is convention for your main remote. You're connected but haven't uploaded anything yet.

### Activity 3.3: Push Your Code to GitHub

Now upload your commits:

```bash
git push -u origin main
```

**Important**: GitHub may ask for authentication. Two options:

**Option A: Personal Access Token (Recommended)**

1. On GitHub, go to Settings → Developer settings → Personal access tokens → "Generate new token (classic)"
2. Select "repo" scope (allows pushing code)
3. Copy the token
4. When Git asks for password, paste the token
5. You'll be pushed successfully

**Option B: SSH Key (Advanced)**

If you've set up SSH, it works automatically.

**Prompt to ChatGPT if Stuck**:

```
I'm getting "fatal: Authentication failed" when pushing to GitHub.
What should I do?
```

### Activity 3.4: Verify Code Appears on GitHub

1. Refresh your GitHub repository page (browser)
2. You should see your files (hello.txt, README.md, etc.) in the cloud
3. Click on "Commits" to see your entire history pushed

**Discovery**: Your entire project—files AND commit history—is now on GitHub.

---

## Role 3: AI as Co-Worker — Convergence Through Recovery Testing

This is where AI becomes your peer, not just a guide. You iterate together to validate the system works.

### Scenario: Test the Backup System

**Your Request to AI**:

```
Now I want to verify that GitHub is really a backup. Can we test
recovering the project on a different machine? How would that work?
```

**What AI Suggests (Iteration 1)**:

> "Easy: Go to a different directory and run `git clone <URL>` to download
> the repository. Then verify all files are there."

**Your Response (Teaching AI)**:

```
Let me test that. But wait—if something goes wrong in the clone, I want
to make sure my original local repository isn't affected. Should I clone
to a completely different folder to test in isolation?
```

**What AI Learns & Refines (Iteration 2)**:

> "Exactly right! Good thinking. Clone to a temporary test folder:
>
> 1. Create a new folder outside your project
> 2. Clone there: `git clone <URL>`
> 3. Verify files and history
> 4. Delete the test clone
> 5. Your original local repo stays safe
>
> This tests recovery without risking your working project."

**Your Convergence (Iteration 3)**:

Together, you've designed a safe recovery verification process. Neither of you initially planned it this thoroughly:
- You focused on testing
- AI focused on the clone command
- Together: **safe test in isolation** + **original project protected**

#### 🤝 Practice Exercise

> **Ask your AI**: "Show me how to clone a GitHub repository to verify my backup works. Then explain the difference between cloning and pulling, and when I'd use each command."

**Expected Outcome**: You'll understand that `git clone` creates a new copy of the entire repository (useful for recovery testing), while `git pull` updates an existing local repository.

### Activity 3.5: Test Recovery (Optional but Recommended)

```bash
# Create a test directory outside your project
cd ..
mkdir test-clone
cd test-clone

# Clone from GitHub
git clone https://github.com/yourname/your-repo.git

# Navigate to cloned project
cd your-repo

# Verify files exist
ls

# View commit history
git log

# Success? Delete test directory
cd ../..
rm -rf test-clone
```

**What This Validates**: Your code is truly recoverable from GitHub. Backup verified.

---

## Troubleshooting: Common Errors

### Error 1: "fatal: Authentication failed"

**Cause**: GitHub doesn't recognize your credentials.

**Solution**:
1. Verify you're using your GitHub username (not email) in the URL
2. Use Personal Access Token (not GitHub password)
3. Check token hasn't expired (GitHub tokens expire)
4. Try SSH key if token isn't working

### Error 2: "fatal: 'origin' already exists"

**Cause**: You already added a remote called "origin" (possibly from an earlier attempt).

**Solution**:
```bash
# Remove the old remote
git remote remove origin

# Add the correct one
git remote add origin https://github.com/yourname/your-repo.git
```

### Error 3: "fatal: Pathspec is in submodule" or branch name mismatch

**Cause**: Your local branch is named `master` but GitHub defaults to `main`.

**Solution**:
```bash
# Check your branch name
git branch

# If it says "master", rename it
git branch -M main

# Then push
git push -u origin main
```

### Error 4: "The file is too large"

**Cause**: GitHub has file size limits (100MB+ files). Large files shouldn't be in Git anyway (videos, datasets, binaries).

**Solution**:
```bash
# Add large files to .gitignore
echo "*.mp4" >> .gitignore
echo "*.zip" >> .gitignore
git add .gitignore
git commit -m "Ignore large files"
```

---

## Your Portfolio is Live

**Right now, your GitHub profile is visible to the world.**

When you share `github.com/yourname` with:
- **Employers**: They see your real projects, not just resume claims
- **Collaborators**: They can review your code and contribution style
- **Community**: They can clone your work or contribute to it

This is the power of GitHub as portfolio. You've transformed from "I did Git projects" to "See my Git projects."

---

## Try With AI

You've now connected your project to GitHub and experienced AI collaboration across three roles. Let's explore GitHub features through AI assistance.

**Setup**: Open ChatGPT (chat.openai.com) or Claude (claude.com) in your browser.

**Prompt 1: GitHub Profile Enhancement**:

```
I just pushed my first project to GitHub. I want to make my GitHub profile
look professional for employers. What should I include in my GitHub profile?
```

**Expected Outcome**: AI suggests:
- Profile picture (professional)
- Bio (2-3 sentences about you)
- Featured repositories (highlight best projects)
- README.md on your profile (GitHub feature)
- Links to portfolio/blog/LinkedIn

**Prompt 2: Portfolio Building**:

```
I'm building a portfolio of AI-assisted projects. What projects should
I build to demonstrate my skills to employers?
```

**Expected Outcome**: AI suggests project ideas that demonstrate:
- Your ability to work with AI assistants
- Problem-solving with modern tools
- Real-world applications

**Prompt 3: GitHub Best Practices**:

```
What are some GitHub best practices I should follow as I push more projects?
```

**Expected Outcome**: AI mentions:
- Meaningful commit messages
- .gitignore for secrets
- README.md in projects
- Documentation
- Clear project descriptions