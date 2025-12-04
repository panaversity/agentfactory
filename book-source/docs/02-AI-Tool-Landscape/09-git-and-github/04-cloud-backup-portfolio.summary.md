### Core Concept
GitHub serves dual purposes: catastrophe prevention (cloud backup) and career showcase (portfolio). When you push code to GitHub, your projects become recoverable from any computer AND visible to employers who can see your actual work, commit history, and problem-solving approach.

### Key Mental Models
- **Remote Repository**: A cloud-hosted copy of your local repository—"origin" is the conventional name for your main remote
- **Push vs. Clone vs. Pull**: Push uploads commits to cloud, clone downloads entire repository (recovery), pull updates existing local copy
- **Portfolio as Proof**: GitHub transforms "I did projects" into "see my projects"—employers view real code, not resume claims

### Critical Patterns
- Create `.gitignore` BEFORE first push—secrets must never reach GitHub (config.json, .env, API keys)
- Use Personal Access Token for authentication (not GitHub password)—tokens need "repo" scope
- Test recovery in isolated directory: clone to temp folder, verify files and history, delete test clone
- `git remote add origin <URL>` connects local to remote; `git push -u origin main` uploads commits
- Verify connection with `git remote -v` before pushing

### Common Mistakes
- Pushing secrets to public repositories—always check `.gitignore` first
- Using GitHub password instead of Personal Access Token—authentication will fail
- Branch name mismatch: local "master" vs. GitHub "main"—rename with `git branch -M main`
- Adding remote when "origin" already exists—remove first with `git remote remove origin`
- Committing large files (100MB+)—add file patterns to `.gitignore` (*.mp4, *.zip)

### Connections
- **Builds on**: Local Git repository creation and commits (Lessons 2-3)
- **Leads to**: GitHub collaboration workflows, pull requests, and team development (future chapters)
