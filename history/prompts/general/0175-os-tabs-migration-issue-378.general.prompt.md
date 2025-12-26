---
id: "0175"
title: "OS-Tabs Migration for Installation Lessons"
stage: general
date: 2025-12-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: none
branch: feat/os-tabs-migration-issue-378
user: mjs
command: /sp.git.commit_pr
labels: [docs, ux, docusaurus, os-tabs]
links:
  spec: null
  ticket: https://github.com/panaversity/ai-native-software-development/issues/378
  adr: null
  pr: https://github.com/panaversity/ai-native-software-development/pull/405
files:
  - apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/02-installing-zed.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/04-installing-cursor.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/06-installing-antigravity.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/09-git-and-github/01-your-first-git-repository.md
  - apps/learn-app/docs/05-Python-Fundamentals/15-python-uv-package-manager/02-installing-uv-with-ai-collaboration.md
  - apps/learn-app/docs/05-Python-Fundamentals/16-introduction-to-python/02-installing-python.md
  - apps/learn-app/docs/05-Python-Fundamentals/32-cpython-gil/04-free-threaded-python.md
  - apps/learn-app/docs/06-AI-Native-Software-Development/40-fastapi-for-agents/01-hello-fastapi.md
  - apps/learn-app/docs/07-AI-Cloud-Native-Development/49-docker-for-ai-services/01-docker-installation-and-setup.md
  - apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/02-setting-up-minikube.md
  - apps/learn-app/docs/07-AI-Cloud-Native-Development/50-kubernetes-for-ai-services/21-helm-charts-ai-agent-packaging.md
  - apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/06-argocd-architecture-installation.md
tests:
  - null
---

## Prompt

User requested migration of OS-specific installation instructions to `::::os-tabs` Docusaurus component across the book (GitHub Issue #378). Initial scope identified 8 files, but comprehensive search found 13 files total requiring migration across Chapters 5, 8, 9, 15, 16, 32, 40, 49, 50, and 55.

## Response snapshot

Executed full book audit using Grep patterns (`### macOS Installation`, `## Windows`, `On macOS:`, etc.) to identify all platform-specific installation sections. Migrated 13 files to use `::::os-tabs` syntax with `::macos`, `::windows`, and `::linux` tabs. Created feature branch, committed with conventional commit format, pushed to origin, and created PR #405.

## Outcome

- ✅ Impact: Improved UX for readers selecting their OS during installation steps
- 🧪 Tests: Docusaurus build verification pending in PR
- 📁 Files: 13 files modified
- 🔁 Next prompts: None - task complete
- 🧠 Reflection: Initial scope underestimated; comprehensive Grep search found 5 additional files beyond original 8

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - docs-only change
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
