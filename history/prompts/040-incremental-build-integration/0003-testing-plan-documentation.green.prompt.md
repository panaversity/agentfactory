---
id: 0003
title: testing-plan-documentation
stage: green
date: 2025-12-09T05:30:00Z
surface: claude-code
model: claude-sonnet-4.5
feature: 040-incremental-build-integration
branch: incremental-build-integration
user: mjs
command: manual
labels: [testing, documentation, local-testing, api-keys]
links:
  spec: specs/040-incremental-build-integration/spec.md
  ticket: N/A
  adr: N/A
  pr: https://github.com/panaversity/ai-native-software-development/pull/333
files:
  - path: panaversity-fs/TESTING.md
    status: created
    lines: +511
    purpose: Comprehensive local testing guide with 3-phase validation strategy
tests:
  - name: test_path_mapper.py
    count: 39
    status: passing
    time: 0.23s
  - name: test_source_scanner.py
    count: 21
    status: passing
    time: 0.22s
---

## Prompt

User: "Document your testing plan and compact first. I think you can take thr phase 3 testing phases to achive 95% confidence also Q: will you need M2M service api key as well for localy testing?"

Context: After implementing all 6 security fixes, user requested:
1. Documented testing plan
2. Simplified/compacted version
3. Focus on 3 phases for 95% confidence
4. Clarify if M2M service API key is needed for local testing

## Response snapshot

Created comprehensive `TESTING.md` with:

### Structure
- **Prerequisites**: MCP server setup, Python environment, API key options
- **Phase 1: Initial Sync** (Full Upload) - ~2-5 minutes
- **Phase 2: Incremental Sync** (Delta Upload) - ~5-10 seconds
- **Phase 3: Hydration** (Download) - 3 sub-parts:
  - Part A: Initial hydration (full download) - ~30 seconds
  - Part B: Incremental hydration - no changes - <5 seconds
  - Part C: Incremental hydration - after changes - ~10 seconds
- **Unit Tests** (Optional) - 60 tests, <1 second
- **Quick Test Script** - Automated all-in-one test
- **Troubleshooting** - Common issues and fixes

### Key Clarifications

**Q: Do you need M2M service API key for local testing?**

**A: NO** - Three options explained:

1. **Development (No Auth)** - Localhost testing
   ```bash
   export PANAVERSITY_MCP_URL="http://localhost:8000"
   # No API key needed
   ```

2. **Simple API Key (Recommended for Testing)** ✅
   ```bash
   export PANAVERSITY_MCP_URL="https://test-server.run.app"
   export PANAVERSITY_API_KEY="test-key-12345"
   ```

3. **M2M Service Account (Production Only)**
   ```bash
   export PANAVERSITY_API_KEY="eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9..."
   # OAuth2 client credentials flow - production only
   ```

**For local testing: Development or Simple API Key is sufficient. M2M is overkill.**

### Asset Handling Confirmed
- Supports: .png, .jpg, .jpeg, .gif, .svg, .webp, .mp4, .mp3, .pdf
- Binary files base64 encoded before upload
- Uses `upload_asset` MCP tool
- Path mapping: `Part-01/Chapter-01/img/test.png` → `static/img/test.png`

### Validation Checklists
Each phase has specific validation checklist:
- Phase 1: All files uploaded, no errors, assets included, fast execution
- Phase 2: Only changed files uploaded (<10s), correct delta detection
- Phase 3A: All files downloaded, correct directory structure
- Phase 3B: No re-download when unchanged (<5s)
- Phase 3C: Only changed files downloaded after edit

### 95% Confidence Statement
**Passing all 3 local phases = 95% confidence for production deployment.**

**What's validated:**
- ✅ Scripts work correctly
- ✅ MCP connectivity
- ✅ Delta detection (sync + hydration)
- ✅ Asset handling
- ✅ Path mapping and validation
- ✅ Security fixes
- ✅ Upload verification
- ✅ Manifest management

**What's NOT tested locally:**
- GitHub Actions workflow execution (5% risk)
- Concurrency groups
- Secrets injection

## Outcome

- ✅ Impact: Complete local testing guide created. User can now achieve 95% confidence before GitHub deployment. API key confusion resolved - NO M2M needed for local testing. Clear phase-by-phase validation checklists prevent missed issues.
- 🧪 Tests: 60/60 passing (existing test suite unchanged). Testing guide provides validation commands and expected outputs for each phase.
- 📁 Files: 1 new file (TESTING.md, 511 lines). Comprehensive guide with prerequisites, 3 phases, validation checklists, troubleshooting, quick test script.
- 🔁 Next prompts: User will perform local testing following TESTING.md. If Phase 1-3 pass, deploy to private test repo. Monitor build times, cache hit rates, fallback activations in GitHub Actions.
- 🧠 Reflection: User's request for "compact" testing plan was critical - original 9-phase plan was overwhelming. 3-phase approach with sub-parts (Phase 3: A/B/C) strikes right balance between thoroughness and simplicity. API key clarification prevents user from wasting time on M2M setup (production-only complexity). Asset handling confirmation was important since not explicitly tested yet.

## Evaluation notes (flywheel)

- Failure modes observed: None. Documentation task completed without issues.
- Graders run and results (PASS/FAIL): Manual review PASS. All checklists have measurable validation criteria. Expected outputs provided for each phase.
- Prompt variant (if applicable): N/A - direct user request
- Next experiment (smallest change to try): User will execute local testing. Monitor for: (1) Which phase fails first (if any)? (2) Are validation checklists sufficient to diagnose issues? (3) Does quick test script work end-to-end? Consider adding progress indicators to scripts (currently minimal feedback during long operations).
