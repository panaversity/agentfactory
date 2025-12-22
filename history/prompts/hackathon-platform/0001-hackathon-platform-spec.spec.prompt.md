---
id: 0001
title: hackathon-platform-spec
stage: spec
date: 2025-12-22
surface: claude-code
model: claude-opus-4-5
feature: hackathon-platform
branch: hackathon-platform
user: mjs
command: /sp.specify hackathon-platform
labels: [hackathon, platform, next.js-16, sso-integration, specification]
links:
  spec: specs/046-hackathon-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/046-hackathon-platform/spec.md
  - specs/046-hackathon-platform/checklists/requirements.md
tests:
  - null
---

## Prompt

/sp.specify hackathon-platform

(Continuation from Phase 0 orchestration: Design and complete hackathon app for Panaversity. AI-first hackathon platform to standardize ongoing hackathons. Integrates with existing SSO as OAuth client. Supports organizers, managers, judges, mentors, participants, and teams.)

## Response snapshot

Created comprehensive specification for the Hackathon Platform at `specs/046-hackathon-platform/spec.md`.

Key specification elements:
- **7 User Stories** prioritized P1-P3: Organizer creates hackathon (P1), Participant registers/joins team (P1), Judge scores (P2), Team submits (P2), Role management (P2), Mentor guidance (P3), Analytics (P3)
- **25 Functional Requirements** covering auth, hackathon management, team/participant flows, submissions, judging, role assignment, and communication
- **7 Key Entities**: Hackathon, Team, TeamMember, Submission, Score, HackathonRole, JudgingCriterion
- **7 Success Criteria** with quantified metrics (30s creation, 3min registration, 5min judging, 500 concurrent users, 95% success rate, 2s page load)
- **4 Constraints**: No SSO modification, Nx monorepo, Next.js 16 patterns, Panaversity org model integration
- **9 Non-Goals**: AI features, video conf, payments, mobile apps, notifications, sponsors, max teams capacity, auto leader promotion

Spec-architect validation:
- Applied formal verification (5 invariants hold)
- Small scope tests passed
- Resolved 2 minor ambiguities with recommended defaults (manual leader promotion, unlimited teams in MVP)
- Quantified "reasonable refresh period" to 30 seconds
- Final verdict: READY (10/10 readiness score)

Manual SSO steps documented for user to execute:
1. Add `hackathon-public-client` to trusted-clients.ts
2. Add client description
3. Run `pnpm run seed:setup`

## Outcome

- ✅ Impact: Comprehensive specification ready for planning phase
- 🧪 Tests: All invariants verified via formal verification (org isolation, team size, single submission, role scoping, deadline enforcement)
- 📁 Files: spec.md (291 lines), checklists/requirements.md (196 lines)
- 🔁 Next prompts: /sp.plan hackathon-platform
- 🧠 Reflection: Spec-architect validation crucial for catching ambiguities before planning. Applied recommended defaults for 2 clarifications since user unavailable.

## Evaluation notes (flywheel)

- Failure modes observed: None - context from Phase 0 carried forward correctly
- Graders run and results (PASS/FAIL): spec-architect PASS (10/10)
- Prompt variant (if applicable): Standard /sp.specify flow
- Next experiment (smallest change to try): None needed - proceed to planning
