---
id: 0002
title: Update Spec Installation Guidance
stage: spec
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 012-chapter-8-git-github-aidd
branch: 012-chapter-8-git-github-aidd
user: HP
command: /sp.specify (iteration)
labels: ["chapter-8", "git", "github", "ide-setup", "installation", "beginner-guidance", "spec-update"]
links:
  spec: specs/012-chapter-8-git-github-aidd/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-chapter-8-git-github-aidd/spec.md (updated)
  - specs/012-chapter-8-git-github-aidd/checklists/requirements.md (updated)
tests:
  - Installation guidance completeness check (PASS)
  - Beginner accessibility verification (PASS)
  - Updated success criteria validation (PASS)
---

## Prompt

yes but also include how to install IDE it could be any and provide installation link and for github also tell how to create an account. This chapter target audience is very beginners who never used it before

## Response snapshot

Updated Chapter 8 specification to include comprehensive installation and setup guidance for absolute beginners:

**Git Installation** (Section 2 - Essential Setup):
- Platform-specific instructions with direct download links:
  - Windows: git-scm.com (direct link)
  - Mac: Homebrew or installer download
  - Linux: Package manager instructions (apt, yum, pacman)
- Installation verification: `git --version`
- First-time configuration (username and email)

**GitHub Account Creation** (Section 2 - Essential Setup):
- Step-by-step guide with screenshots:
  1. Visit github.com (direct link)
  2. Sign up form: username, email, password
  3. Verify email address (check inbox/spam)
  4. Choose Free plan
  5. Complete welcome survey (optional)
  6. Tour of GitHub interface
- Troubleshooting: What if verification email doesn't arrive

**IDE Installation** (Section 8 - IDE Setup, expanded from 15 to 25 minutes):
- **Multiple IDE Options** with beginner guidance:
  - VS Code (recommended): Most popular, free, beginner-friendly
    - Download: code.visualstudio.com
  - Cursor: AI-first editor with built-in AI assistant
    - Download: cursor.sh
  - PyCharm Community: Python-focused IDE
    - Download: jetbrains.com/pycharm/download
  - Zed: Lightweight, fast modern editor
    - Download: zed.dev
  - Brief comparison table to help students choose
- **Step-by-step Installation** with screenshots:
  - Download installer for operating system
  - Run installer (platform-specific: .exe, .dmg, .deb/.rpm)
  - First launch walkthrough
  - Verify installation by creating test file
- **Git Integration Configuration**:
  - Source Control panel setup
  - GitLens extension installation
  - Git Graph for visual history
- **AI Coding Extensions**:
  - GitHub Copilot (if available)
  - Cursor AI (if using Cursor)
  - Continue extension (free alternative)
  - Activation and testing

**Updated Functional Requirements**:
- **FR-004**: GitHub account creation with screenshots for absolute beginners
- **FR-004a**: Repository setup and authentication with troubleshooting
- **FR-006**: Comprehensive IDE installation with download links
- **FR-006a**: IDE options with comparison to help students choose
- **FR-006b**: Detailed extension installation (Git + AI tools)
- **FR-015**: Troubleshooting section for common beginner issues (permissions, email verification, Git detection, path configuration)

**New Success Criteria**:
- **SC-002a**: 90% successfully install Git and verify (all platforms)
- **SC-002b**: 80% successfully install and configure IDE within 30 minutes

**Updated Edge Cases**:
- Git installation fails due to administrator permissions
- GitHub verification email not received
- IDE doesn't detect Git after installation
- Machine doesn't meet minimum IDE requirements
- Student confused by multiple IDE options

**Time Adjustment**:
- Part I: 90 min → 100 min (added 5 min for detailed Git installation, 5 min for GitHub account creation)
- Part II: 90 min → 100 min (IDE setup expanded from 15 min to 25 min)
- Total: 3-4 hours → 3.5-4 hours (to accommodate beginner installation guidance)

**Conversational Examples Updated**:
- "Set up Git and GitHub for my machine - I've never done this before"
- "I'm a complete beginner - help me choose and install an IDE, then connect it to Git"

## Outcome

- ✅ Impact: Specification now fully addresses absolute beginner needs with explicit installation guidance, direct download links, and troubleshooting. No assumptions about prior tool installation experience.
- 🧪 Tests: Installation completeness check PASS, Beginner accessibility verification PASS, Success criteria validation PASS
- 📁 Files: Updated spec.md (+4 functional requirements, +3 success criteria, +5 edge cases) and checklist with installation-specific validation
- 🔁 Next prompts: Specification is validated and ready for `/sp.plan` to create detailed lesson plan with installation steps properly sequenced
- 🧠 Reflection: Critical improvement for true beginners. Original spec assumed some tool installation experience. Updated version provides complete hand-holding with screenshots, download links, and troubleshooting - essential for students who have never installed developer tools.

## Evaluation notes (flywheel)

- Failure modes observed: None. Update addresses user feedback completely.
- Graders run and results (PASS/FAIL): Installation guidance completeness PASS, Beginner accessibility PASS, Time estimate adjustment appropriate PASS
- Prompt variant (if applicable): User feedback iteration - adding explicit installation steps
- Next experiment (smallest change to try): Proceed to planning phase. Monitor if 25 minutes for IDE setup is sufficient for absolute beginners (may need adjustment during implementation based on user testing)
