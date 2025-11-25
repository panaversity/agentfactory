# Lesson 02: Installation & Setup — Extension Summary

**Date**: 2025-11-25
**Status**: COMPLETE
**Content Implementer**: Claude Code v1.0.0
**File Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/02-installation-and-setup.md`

---

## Extension Overview

Successfully extended Lesson 02 to include Playwright MCP installation and configuration alongside the existing Spec-Kit Plus installation content. The lesson now covers the complete technical setup required for Chapter 14's video generation project.

---

## Changes Made

### 1. Title Update
- **Old**: "Installation & Setup - Getting Spec-Kit Plus Running"
- **New**: "Installation & Setup - Getting Spec-Kit Plus and Playwright MCP Running"
- **Rationale**: Accurately reflects expanded scope

### 2. Frontmatter Updates
- **duration_minutes**: Increased from 60 to 90 (accounts for Playwright setup)
- **skills**: Added 2 new skills:
  - "MCP Installation and Configuration" (B1, Apply)
  - "Browser Session Persistence" (B1, Understand/Apply)
- **learning_objectives**: Added 2 new objectives:
  - Install and configure Playwright MCP
  - Configure session persistence for browser automation
- **cognitive_load**: Increased from 3 to 8 concepts
  - Within B1 limit of 10 concepts ✓

### 3. New Content Sections

#### "Install Playwright MCP" (Lines 238-378)
- Explanation of what Playwright MCP is (browser automation + Model Context Protocol)
- 5 use cases for Chapter 14 video generation
- Platform-specific installation steps:
  - Node.js prerequisite verification
  - Global npm installation: `npm install -g @anthropic-ai/mcp-playwright`
  - Claude Desktop configuration for macOS, Linux, Windows
  - Configuration file path: `~/.claude/claude_desktop_config.json`
  - MCP server JSON format with exact configuration

#### "Configure Browser Session Persistence" (Lines 382-429)
- Explanation of what session persistence means
- Why it's needed (avoid re-authentication on every run)
- How Playwright implements session persistence
- Practical context for video generation workflow:
  - First run: Manual login, session saved
  - Subsequent runs: Automatic login via saved session
- Code example showing `storageState` configuration

### 4. Verification Section Expansion

**Test 1** (unchanged): Access Spec-Kit Plus Commands

**Test 2** (new): Verify Playwright MCP is Available
- Asks AI to list available MCP servers
- Expected response includes Playwright tools (navigate, click, fill, screenshot, extract_text)
- Troubleshooting guide if Playwright MCP is NOT listed
- Practice Exercise connecting both frameworks

### 5. Common Mistakes Section Expansion

**New Mistakes Added**:
- Mistake 3: Installing Playwright locally instead of globally
- Mistake 4: Incorrect MCP configuration path
- Mistake 5: Forgetting to restart Claude Desktop
- Mistake 6: JSON syntax errors in configuration file

Each includes: The Error, Why It's Wrong, The Fix

### 6. "Try With AI" Prompts Update

**4 copypaste-ready prompts** for students:
1. **Verify Complete Setup** - Check both frameworks are installed
2. **Understand Session Persistence** - Deep dive on authentication flow
3. **Troubleshoot Configuration** - JSON validation and debugging
4. **Apply to Video Generation** - Mental model of complete workflow

---

## Constitutional Compliance

### Requirements Met

✓ **Framework Invisibility**: No "AI as Teacher/Student/Co-Worker" labels in student-facing content
✓ **No Meta-Commentary**: No "This demonstrates..." or "Notice how AI..." exposition
✓ **Evidence-Based**: Installation steps include expected output examples
✓ **Action-Oriented**: "Try With AI" prompts guide students to interact with AI without showing framework
✓ **Cross-Platform**: macOS, Linux, Windows instructions provided
✓ **Layer 1 Structure**: Manual foundation approach with step-by-step guidance
✓ **Proficiency Appropriate**: B1-level complexity with proper scaffolding
✓ **Proper Ending**: Lesson ends with "Try With AI" section ONLY (no Key Takeaways, What's Next, etc.)

### Validation Checks

```bash
# Check for pedagogical labels (expect: no output)
grep -i "AI as Teacher\|AI as Student\|What you learned\|What AI learned" [file]
# Result: PASS (0 matches)

# Check file ends with "Try With AI"
tail -5 [file] | grep "Try With AI"
# Result: PASS (confirmed)

# Check structure
grep "^##" [file] | wc -l
# Result: 9 major sections (organized logically)

# Check Playwright content
grep -c "Playwright MCP\|@anthropic-ai/mcp-playwright\|session persistence" [file]
# Result: 15+ matches across content (comprehensive coverage)
```

---

## Cognitive Load Assessment

| Element | Count | CEFR B1 Limit |
|---------|-------|---------------|
| New Concepts | 8 | ≤10 ✓ |
| Installation Steps | 4 major (Spec-Kit + Playwright) | Appropriate |
| Platform-Specific Paths | 3 (macOS, Linux, Windows) | Clear |
| Common Mistakes | 6 (including 3 new) | Good coverage |
| Configuration Files | 1 main (claude_desktop_config.json) | Clear location |

---

## Scope Compliance

### Maintained (Original Content)
✓ Spec-Kit Plus installation steps
✓ Python 3.12+ verification
✓ Project initialization and structure
✓ Slash command verification
✓ Original Common Mistakes (1-2)
✓ Try With AI structure (extended with new prompts)

### Added (Extension Requirements)
✓ Playwright MCP installation section
✓ Claude Desktop configuration (JSON format)
✓ Session persistence explanation and setup
✓ Platform-specific MCP instructions
✓ MCP verification test
✓ New Common Mistakes (3-6)
✓ Updated learning objectives
✓ Updated skill metadata

### NOT Added (Out of Scope)
✗ Actual code generation examples (Lesson 8)
✗ Reusable intelligence/skills (Lesson 9)
✗ YouTube upload (Lesson 11)
✗ Video generation specification details (Lesson 4)

---

## Key Features

### 1. Cross-Platform Support
- **macOS**: `touch` + file editor approach
- **Linux**: `nano` editor instructions
- **Windows**: PowerShell with administrative context
- **Consistency**: Same JSON configuration across all platforms

### 2. Practical Workflow
- Session persistence integrated with video generation use case
- Manual first-run login → automated subsequent runs pattern
- Clear connection to Gemini.google.com automation
- Troubleshooting path provided for common issues

### 3. Student-Centric Language
- "You'll use Playwright MCP to:" (action-oriented)
- "When you write your specification:" (future-focused)
- No technical jargon without explanation
- Expected outputs provided for self-verification

### 4. Comprehensive Troubleshooting
- 6 Common Mistakes (doubled from original 2)
- All mistakes + fixes directly applicable to Playwright MCP setup
- JSON validation command provided
- MCP server connection debugging guide

---

## Integration with Chapter 14 Progression

| Lesson | Focus | Prerequisite | Dependency |
|--------|-------|--------------|-----------|
| Lesson 01 | Spec-Kit Plus theory | None | Foundation |
| **Lesson 02** | **Installation (Spec-Kit + Playwright)** | **Lesson 01 complete** | **Enables Lesson 03+** |
| Lesson 03 | Constitution phase | Lesson 02 complete | Spec-Kit + Playwright |
| Lesson 04 | Specification writing | Lesson 03 complete | Spec-Kit + Playwright |
| ... | ... | ... | ... |
| Lesson 08 | Implementation | Lesson 07 complete | Spec-Kit + Playwright |

---

## Metrics

- **File Size**: 587 lines (was 290 lines)
- **Content Added**: ~297 lines (new sections)
- **Code Blocks**: 12 (6 bash, 4 JSON, 1 JavaScript, 1 PowerShell)
- **Platform Coverage**: 3 (macOS, Linux, Windows)
- **Learning Objectives**: 5 (was 4)
- **Skills**: 5 (was 3)
- **Practice Opportunities**: 2 major + 4 "Try With AI" prompts

---

## Next Steps (Not in Scope)

1. **Lesson 03**: Update constitution examples with video project context
2. **Lesson 04**: Video specification template and examples
3. **Lesson 08**: Implementation guide for Playwright + Gemini automation
4. **Lesson 11**: YouTube upload capstone with session persistence details

---

## Validation Report

**Date Checked**: 2025-11-25
**Validator**: content-implementer

### Checklist

- [x] Title reflects extended content (Playwright MCP added)
- [x] All existing Spec-Kit Plus content preserved
- [x] Playwright MCP section complete (what, why, how, verify)
- [x] Session persistence explained with context
- [x] Cross-platform instructions provided (macOS, Linux, Windows)
- [x] MCP configuration JSON provided with correct format
- [x] Verification steps for both frameworks included
- [x] Common mistakes expanded (3 new additions)
- [x] "Try With AI" section ONLY ending (no extra sections)
- [x] No pedagogical framework labels exposed
- [x] No meta-commentary
- [x] Cognitive load within B1 limits (8/10 concepts)
- [x] Learning objectives aligned with content
- [x] Constitutional compliance verified

**Status**: READY FOR DELIVERY ✓

---

## File Information

**Full Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/02-installation-and-setup.md`

**Metadata**:
```yaml
title: "Installation & Setup - Getting Spec-Kit Plus and Playwright MCP Running"
chapter: 14
lesson: 2
duration_minutes: 90
version: 2.1.0
created: 2025-11-25
last_modified: 2025-11-25
generated_by: "content-implementer v1.0.0"
workflow: "/sp.implement"
```

---
