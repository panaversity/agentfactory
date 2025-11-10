---
id: 0004
title: unified-cc-by-nc-nd-licensing
stage: plan
date: 2025-01-10
surface: conversational
model: claude-sonnet-4-5-20250929
feature: 017-usage-data-collection
branch: 017-usage-data-collection
user: mjs
command: N/A (licensing clarification)
labels: [licensing, cc-by-nc-nd, no-mit, unified-license, non-commercial]
links:
  spec: specs/017-usage-data-collection/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: LICENSE
    status: modified
    lines: 236
  - path: NOTICE
    status: modified
    lines: 237
  - path: specs/017-usage-data-collection/plan.md
    status: modified
    lines: 437
tests:
  - name: unified-licensing-validation
    status: passed
    notes: All components now under CC BY-NC-ND 4.0, MIT removed
---

## Prompt

User clarification on licensing strategy:

> "Let's no MIT anything"

**Intent**: User wants to remove MIT licensing completely and apply protective non-commercial licensing uniformly across the entire repository (book content AND infrastructure code).

**Previous State**: Dual licensing with CC BY-NC-ND 4.0 for book content and MIT for infrastructure code.

**Required Change**: Unified CC BY-NC-ND 4.0 licensing for ALL repository content.

## Response snapshot

**Unified Licensing Implemented**:

Removed dual licensing structure and applied **CC BY-NC-ND 4.0 to ALL content** in the repository:

**LICENSE File Updated** (236 lines):
- Single license section (removed "Part 1/Part 2" structure)
- Applies CC BY-NC-ND 4.0 to:
  - Book content (`book-source/docs/`)
  - Infrastructure code (`.claude/`, `.specify/`, `telemetry-server/`)
  - Documentation (README, guides, specifications)
  - Configuration templates (`.env.template`, `docker-compose.yml`, etc.)
  - Scripts and tools
  - All other repository materials

- Clarifications for special cases:
  - Code examples: Personal learning allowed, commercial requires permission
  - Development tools: Personal use allowed, commercial requires permission
  - Forking and contributions: Personal forks OK, public contributions via PR

- Comprehensive FAQ (12 questions):
  - Can I use infrastructure tools? Personal yes, commercial no
  - Can I extract code examples? No for redistribution, yes for learning
  - Can I modify SDD tools? Yes for personal, no for distribution

**NOTICE File Updated** (237 lines):
- Emphasizes "NO dual licensing" — everything is CC BY-NC-ND 4.0
- Clear table showing all content types under same license
- "Open source repository" vs "Open source license" distinction explained
- Infrastructure usage clarifications:
  - Personal use: ✅ Allowed
  - Commercial use: ❌ Requires permission
- Contribution terms updated (all contributions become CC BY-NC-ND 4.0)

**plan.md Updated**:
- Changed licensing strategy section from dual to unified
- Removed "MIT (fully open source, commercial use allowed)" language
- Updated to "CC BY-NC-ND 4.0 (free to read and use personally, NOT free to resell or use commercially)"

**Key Implications**:

1. **Stronger Protection**: No one can take the infrastructure code and create commercial products without permission

2. **Consistent Licensing**: Eliminates confusion about which parts have which license

3. **Personal Use Preserved**: Learners can still use all materials (book + tools) for personal learning

4. **Commercial Barrier**: Companies cannot use telemetry infrastructure or SDD tools commercially without licensing

5. **Contribution Clarity**: All PR contributions become CC BY-NC-ND 4.0, maintaining licensing consistency

**What Changed for Users**:

| Use Case | Before (Dual License) | After (Unified CC BY-NC-ND) |
|----------|----------------------|----------------------------|
| Read book for free | ✅ CC BY-NC-ND | ✅ CC BY-NC-ND |
| Use tools personally | ✅ MIT (no restrictions) | ✅ CC BY-NC-ND (personal only) |
| Use tools commercially | ✅ MIT (allowed) | ❌ CC BY-NC-ND (prohibited) |
| Modify and distribute | ✅ MIT (allowed) | ❌ CC BY-NC-ND (prohibited) |
| Sell copies | ❌ Both | ❌ CC BY-NC-ND |

**Impact on Commercial Users**:
- Previously: Could use infrastructure code (MIT) but not book content (CC BY-NC-ND)
- Now: Must contact Panaversity for licensing to use ANY part commercially

## Outcome

- ✅ Impact: Unified licensing under CC BY-NC-ND 4.0 provides maximum protection against unauthorized commercial exploitation while preserving free access for learners. Eliminates dual-license complexity and prevents scenario where infrastructure is used commercially while book content cannot be.
- 🧪 Tests: Licensing validation — PASS (all components now under single CC BY-NC-ND 4.0 license)
- 📁 Files: 3 files modified (LICENSE: 236 lines, NOTICE: 237 lines, plan.md: licensing strategy section)
- 🔁 Next prompts: `/sp.tasks` to proceed with implementation, OR user review of unified licensing terms
- 🧠 Reflection: User's decision to remove MIT licensing is strategically sound for protecting intellectual property. Dual licensing created a potential loophole where commercial entities could use infrastructure (MIT) without engaging with book creators. Unified CC BY-NC-ND ensures that ANY commercial use requires partnership with Panaversity, supporting sustainable educational content development. Personal learners unaffected—they retain full access to all materials.

## Evaluation notes (flywheel)

- Failure modes observed: Dual licensing introduced unnecessary complexity and potential commercial exploitation pathway (MIT-licensed tools could be commercialized independently from book content)
- Graders run and results (PASS/FAIL): Unified licensing validation — PASS (single CC BY-NC-ND 4.0 license applied consistently across all content types)
- Prompt variant (if applicable): N/A (strategic licensing decision, not prompt optimization)
- Next experiment (smallest change to try): Monitor for community feedback on unified licensing; some open-source advocates may object to NC/ND restrictions on infrastructure code. If negative feedback is significant, can create separate "tools-only" repository under MIT while maintaining CC BY-NC-ND for this integrated educational repository.
