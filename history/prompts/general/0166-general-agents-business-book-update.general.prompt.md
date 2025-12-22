---
id: "0166"
title: "General Agents and Business Content Book Update"
stage: general
date: 2025-12-21
surface: claude-code
model: claude-opus-4-5-20251101
feature: none
branch: feat/general-agents-business-content
user: mjs
command: manual
labels: [book-content, general-agents, business-monetization, chapter-5]
links:
  spec: N/A
  ticket: N/A
  adr: N/A
  pr: https://github.com/panaversity/ai-native-software-development/pull/353
files:
  - apps/learn-app/docs/preface-agent-native.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/README.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/README.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/01-origin-story.md
  - apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/14-from-skills-to-business.md
  - apps/learn-app/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/01-what-is-an-ai-agent.md
  - apps/learn-app/docs/14-Capstone-Building-AI-Native-Books/README.md
tests:
  - N/A (content update)
---

## Prompt

User requested integration of "AI Agents for Business Automation" presentation content into the book, focusing on:
1. General Agents explanation (what they are, OODA loop, Trojan Horse insight)
2. Business monetization aspects (Digital FTE, 4 revenue models, Unicorn roadmap)
3. Making Chapter 5 actionable (students can monetize Skills immediately)

User emphasized: "after ch 5 students can actually do something" and questioned why all business content was going to Chapter 33 instead of being front-loaded.

## Response snapshot

Implemented comprehensive book updates:

**Preface**: Added OODA loop explanation, Agent Factory Paradigm, Business Opportunity section (Digital FTE, 4 models, Unicorn math, OpenAI Apps)

**Part 2 & Chapter 5 READMEs**: Renamed Chapter 5 to "Your First General Agent", updated learning objectives

**Chapter 5 Lesson 1**: Added "Understanding General Agents: The Trojan Horse", Habitat comparison table, OODA Loop section, Prediction vs Reasoning table, expanded "Code as Universal Interface" with business example

**Chapter 5 Lesson 14 (NEW)**: "From Skills to Business" - What students CAN do after Ch5 (License packs, Done-for-you services, Consulting), What they CAN'T do yet (Custom Agents - Part 6), 30-day roadmap, case study

**Chapter 33 & Part 14**: Business angle sections linking back to Ch5 L14

## Outcome

- ✅ Impact: Major content addition - 709 lines added, new lesson created
- 🧪 Tests: N/A (content update, needs MDX build verification)
- 📁 Files: 7 files modified/created
- 🔁 Next prompts: Update Google Slides link in preface if presentation changed
- 🧠 Reflection: Front-loading business content in Preface and Ch5 provides motivation early; distinguishing what students CAN do now vs later prevents false expectations

## Evaluation notes (flywheel)

- Failure modes observed: Initially put all business content in Ch33 (too late in book); fixed by distributing to Preface, Ch5 L1, Ch5 L14
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Verify MDX builds without errors
