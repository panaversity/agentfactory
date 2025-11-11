---
id: 010
title: Streamline CLAUDE.md - Remove Constitutional Duplication
stage: constitution
date: 2025-11-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: constitution-v3.1.2-alignment
branch: main
user: mjs
command: manual
labels: ["constitution", "CLAUDE.md", "performance", "DRY-principle"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - CLAUDE.md
  - CLAUDE.md.backup
tests:
  - File size verification (under 40KB limit)
  - Content references constitution (not duplicates)
  - All operational workflows retained
---

## Prompt

I also see some code snippets in CLAUDE.md let's remove them and finalize CLAUDE.md no need for a separate STREAMLINED thing.

(User identified CLAUDE.md was 67.3KB > 40KB limit with verbose duplication of constitutional content and code snippets)

## Response snapshot

**Problem Identified**:
- Constitution: 85KB (source of truth)
- CLAUDE.md: 68KB (operational guide) ⚠️ Over 40KB performance limit
- Total: 153KB of documentation
- Issue: CLAUDE.md duplicated constitutional content verbatim instead of referencing

**Root Cause**: 
During v3.1.2 alignment updates, we added comprehensive sections (LLMs to LAMs, Nine Pillars, 10x-99x Multiplier, AI Development Spectrum, Target Audience) with full details rather than references. This created ~55KB of duplication.

**Solution Applied**:
Created streamlined CLAUDE.md that **references** constitution instead of duplicating:

**Removed**:
- ❌ Verbose duplications of constitutional content
- ❌ Code snippets and examples (22 code blocks)
- ❌ Long explanatory sections already in constitution
- ❌ Detailed framework documentation

**Kept**:
- ✅ References to constitution (not copies)
- ✅ Operational workflow guidance
- ✅ Subagent invocation instructions
- ✅ PHR/ADR creation guidelines
- ✅ Quick reference to key principles
- ✅ Execution contracts and checklists

**Results**:
- Before: 68,150 chars (67.3KB) ⚠️ Over limit
- After: 13,210 chars (13.2KB) ✅ Well under limit
- Reduction: **80.6% smaller** (54.9KB removed)
- Backup: CLAUDE.md.backup (original preserved)

**Benefits**:
1. Performance: No more 67KB warning, faster agent loading
2. Maintainability: Single source of truth (constitution)
3. Clarity: Operational guide focused on workflows
4. DRY Principle: Reference, don't duplicate

## Outcome

- ✅ Impact: CLAUDE.md streamlined to 13.2KB (80.6% reduction); performance warning eliminated; single source of truth maintained
- 🧪 Tests: File size verified under 40KB limit; content references constitution appropriately
- 📁 Files: CLAUDE.md (replaced), CLAUDE.md.backup (original preserved)
- 🔁 Next prompts: Create PHRs documenting all constitutional work
- 🧠 Reflection: CLAUDE.md had grown too verbose during alignment updates; streamlining to reference-based approach maintains clarity while improving performance

## Evaluation notes (flywheel)

- Failure modes observed: Initial alignment approach duplicated content instead of referencing; created performance issue (67KB > 40KB limit)
- Graders run and results (PASS/FAIL): PASS - CLAUDE.md now 13.2KB (well under 40KB limit); all operational guidance retained
- Prompt variant (if applicable): Created reference-based operational guide instead of comprehensive documentation
- Next experiment (smallest change to try): Monitor agent performance with streamlined CLAUDE.md; verify no critical information lost
