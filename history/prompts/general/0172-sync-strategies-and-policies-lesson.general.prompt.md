---
id: 0172
title: "Write Lesson 8: Sync Strategies and Policies"
stage: general
date: 2025-12-23
surface: claude-code
model: claude-haiku-4-5-20251001
feature: chapter-55-lesson-8
branch: 014-docker-networking-volumes
user: mjs
command: write-lesson
labels: [content, chapter-55, argocd, layer-1, gitops]
links:
  spec: /specs/014-docker-networking-volumes/spec.md
  ticket: null
  adr: null
  pr: null
files:
  created:
    - path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/08-sync-strategies-policies.md
      lines: 698
      focus: sync policies configuration
tests:
  validation:
    - framework-labels: PASS (no "AI as Teacher", "What to notice" found)
    - code-output-sections: PASS (11 Output sections for production-relevant examples)
    - proper-ending: PASS (ends with "## Try With AI")
    - deprecated-metadata: PASS (no cefr_level found)
    - concept-coverage: PASS (all 6 required concepts present)
---

## Prompt

Write Lesson 8: Sync Strategies and Policies for Chapter 55 (CI/CD & GitOps with ArgoCD)

**Requirements**:
- Layer 1 (Manual Foundation) - B1-B2 proficiency tier
- 6 core concepts: manual sync, auto-sync, auto-prune, self-heal, replace vs apply, sync windows
- Every code block MUST have **Output:** section
- NO meta-commentary ("What to notice", "AI teaches you", etc.)
- End ONLY with "## Try With AI" section
- Production-relevant examples (why you'd choose each strategy)
- 40-minute duration lesson

**Output file**: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-AI-Cloud-Native-Development/55-cicd-gitops-argocd/08-sync-strategies-policies.md

## Response snapshot

Created 698-line lesson with complete coverage of sync strategies:

1. **Manual Sync** - Default behavior, when you control everything, triggers via CLI and UI
2. **Auto-Sync** - Automated reconciliation, understanding polling behavior, fine-tuning settings
3. **Auto-Prune** - Resource cleanup when removed from Git, tradeoffs, safety progression
4. **Self-Heal** - Drift correction, automatic reversion of manual changes, risks and timing
5. **Replace vs Apply** - Handling immutable fields (Service selectors, etc.), Apply vs Replace tradeoff
6. **Sync Windows** - Time-based restrictions, cron syntax, maintenance windows and blackout windows

**Key design decisions**:
- Production scenarios throughout (why each strategy matters)
- Comparison table for common deployment scenarios
- 4 complete Application YAML configurations (learning, staging, production-safe, conservative)
- Practical examples: Service deletion, manual deployment change, immutable field updates
- Sync window examples with real use cases (maintenance windows, business hours blackout)
- Try With AI section with 3 prompts addressing key tradeoffs

## Outcome

- ✅ Impact: Complete Layer 1 lesson enabling students to configure ArgoCD for their environment
- 🧪 Tests: All validation checks passed (framework invisibility, code output coverage, structure)
- 📁 Files: 1 new lesson file (698 lines)
- 🔁 Next prompts: Write Lesson 7 (Your First ArgoCD Application), then Lesson 9 (Sync Waves)
- 🧠 Reflection: Lesson emphasizes decision-making (when to use each strategy) rather than just configuration syntax

## Evaluation notes

- Failure modes observed: None - followed constitutional requirements precisely
- Graders run and results: All validation passed (no exposed framework, code output coverage adequate)
- Prompt variant: N/A
- Next experiment: Ensure Lesson 7 is written before this goes live (students reference Lesson 7 concepts)
