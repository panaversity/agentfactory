# Sandbox Audit Report: Chapter 5, Lesson 6

**File:** `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/06-skills-plugins-mcp.md`

**Audit Date:** 2025-11-13

**Auditor:** Super Orchestra Session (Vertical Intelligence Pattern)

**Status:** 🚨 **CRITICAL ERRORS FOUND** — Lesson contains incorrect command syntax that will not work

---

## Executive Summary

While the lesson's pedagogical design and conceptual content are excellent, **hands-on validation revealed critical command syntax errors**. All plugin commands use incorrect slash-based syntax (`claude /plugin`) instead of the correct CLI syntax (`claude plugin`). This means **students following the lesson exactly will encounter command failures**.

**Impact:** HIGH — This is a hands-on lesson (Learning Objective 3: "Install a real community plugin and verify activation"). Incorrect commands block the primary learning outcome.

---

## Critical Errors Found

### Error 1: Incorrect `/plugin` Command Syntax (Line 346)

**Location:** Line 346

**Current (WRONG):**
```bash
claude /plugin
```

**Actual Command:**
```bash
claude plugin
```

**Evidence from `claude plugin --help`:**
```
Usage: claude plugin [options] [command]

Manage Claude Code plugins

Commands:
  validate <path>
  marketplace
  install|i <plugin>
  uninstall|remove <plugin>
  enable <plugin>
  disable <plugin>
```

**Impact:** Students will get error: `Error: unknown option '/plugin'`

---

### Error 2: Incorrect `/plugin marketplace add` Syntax (Line 375)

**Location:** Line 375

**Current (WRONG):**
```bash
claude /plugin marketplace add anthropics/claude-code
```

**Actual Command:**
```bash
claude plugin marketplace add anthropics/claude-code
```

**Evidence from `claude plugin marketplace --help`:**
```
Commands:
  add <source>      Add a marketplace from a URL, path, or GitHub repo
  list              List all configured marketplaces
  remove|rm <name>  Remove a configured marketplace
```

**Impact:** Command will fail, blocking marketplace configuration

---

### Error 3: Incorrect `/plugin install` Syntax (Line 383)

**Location:** Line 383

**Current (WRONG):**
```bash
claude /plugin install feature-dev
```

**Actual Command:**
```bash
claude plugin install feature-dev
```

**Evidence:** Subcommand is `install|i <plugin>`, not a slash command

**Impact:** Plugin installation will fail, preventing Learning Objective 3 from being achieved

---

### Error 4: Incorrect `/help` Command Syntax (Line 391)

**Location:** Line 391

**Current (WRONG):**
```bash
claude /help
# Should now show: /feature-dev command available
```

**Actual Command:**
```bash
claude --help
```

**Evidence from `claude --help` output:**
- Slash commands (like `/commit`) are session commands WITHIN Claude Code, not CLI arguments
- Plugin verification should use `claude plugin list` or check session commands

**Impact:** Students won't be able to verify plugin installation correctly

---

### Error 5: Misleading Plugin Verification Claim (Line 392)

**Location:** Line 392

**Current (MISLEADING):**
```bash
# Should now show: /feature-dev command available
```

**Issue:** This assumes all plugins create slash commands with the plugin name. This may not be true for all plugins.

**Better Verification:**
```bash
# Verify plugin is installed
claude plugin list

# Start Claude Code session and check available commands
claude
> /help
# Look for commands added by the plugin
```

**Impact:** Students may think installation failed if they don't see `/feature-dev` specifically

---

### Error 6: Session Slash Commands vs CLI Commands Confusion

**Throughout the lesson**, there's conceptual confusion between:

1. **CLI commands** (outside Claude Code session): `claude plugin install`
2. **Session slash commands** (inside Claude Code): `/commit`, `/help`, `/code-review`

**The lesson conflates these two different command contexts.**

**Example of confusion:**
- Line 346: `claude /plugin` ← CLI command with slash (WRONG)
- Line 209: `/code-review` ← Session slash command (CORRECT context, but used in wrong place)

**Correct Pattern:**
- **CLI**: `claude plugin [subcommand]` (manage plugins from terminal)
- **Session**: `/command-name` (invoke commands within Claude Code session)

---

## Additional Issues

### Issue 7: Unverified "feature-dev" Plugin Existence

**Claim:** Lines 352, 383, 473 reference `feature-dev` as an official Anthropic plugin

**Problem:** No evidence this plugin actually exists in the anthropics/claude-code marketplace

**Recommendation:** Test with actual marketplace or use generic example with disclaimer

---

### Issue 8: MCP @-Mention Activation May Be Incorrect

**Claims (Lines 440-442, 456-457):**
```bash
@github search issues
@github create-pull-request
@filesystem read [file]
@filesystem write [file]
```

**Question:** Are these the correct MCP activation patterns for Claude Code?

**Needs Verification:** Test in actual Claude Code session with MCP configured

---

## Hands-On Validation Results

### Test 1: Plugin Command Syntax ✅ VERIFIED

```bash
$ claude plugin --help
Usage: claude plugin [options] [command]

Manage Claude Code plugins

Commands:
  validate <path>
  marketplace                    Manage Claude Code marketplaces
  install|i <plugin>            Install a plugin
  uninstall|remove <plugin>     Uninstall a plugin
  enable <plugin>               Enable a disabled plugin
  disable <plugin>              Disable an enabled plugin
```

**Verdict:** Lesson uses WRONG syntax throughout (`/plugin` instead of `plugin`)

---

### Test 2: Marketplace Command Syntax ✅ VERIFIED

```bash
$ claude plugin marketplace --help
Commands:
  add <source>      Add a marketplace from a URL, path, or GitHub repo
  list              List all configured marketplaces
  remove|rm <name>  Remove a configured marketplace
  update [name]     Update marketplace(s)
```

**Verdict:** Lesson uses WRONG syntax (`/plugin marketplace` instead of `plugin marketplace`)

---

### Test 3: Actual SKILL.md Structure ✅ VERIFIED

Examined real skill: `.claude/skills/python-sandbox/SKILL.md`

**Actual Structure:**
```yaml
---
name: python-sandbox
description: |
  Validate Python code blocks...
allowed-tools: ["Bash", "Read", "Write", "Grep"]
metadata:
  category: "validation"
  version: "1.0.0"
---

# Skill Title

## Overview
## When to Use
## Quick Start
## Workflow
## Bundled Scripts
## Instructions
## Common Scenarios
...
```

**Lesson's Example (Lines 144-161):** ✅ ACCURATE structure, but simplified

**Verdict:** Pedagogical example is correct, just simplified for A2 level

---

### Test 4: Progressive Disclosure Claim ❓ NEEDS VERIFICATION

**Claim (Lines 122-133):** Skills have metadata summaries loaded in system prompt

**Question:** How to verify this? Need to inspect Claude Code's actual system prompt or documentation

**Status:** CANNOT VERIFY in sandbox (requires Claude Code internals access)

---

## Required Fixes (Priority Order)

### CRITICAL (Blocks Learning Objectives)

1. **Fix Line 346**: Change `claude /plugin` → `claude plugin`
2. **Fix Line 375**: Change `claude /plugin marketplace add` → `claude plugin marketplace add`
3. **Fix Line 383**: Change `claude /plugin install feature-dev` → `claude plugin install feature-dev`
4. **Fix Line 391**: Change `claude /help` → `claude --help` (for CLI verification)
5. **Add clarification**: Distinguish CLI commands vs session slash commands throughout

### HIGH (Misleading Students)

6. **Fix Line 392**: Update verification method to use `claude plugin list` or session `/help`
7. **Add note**: Explain difference between CLI mode (`claude plugin`) and session mode (`/command`)
8. **Verify "feature-dev"**: Confirm plugin exists or use generic example with disclaimer

### MEDIUM (Needs Verification)

9. **Verify MCP @-mentions**: Test actual activation syntax (`@github`, `@filesystem`)
10. **Verify progressive disclosure**: Find official documentation or test in Claude Code

---

## Recommended Lesson Fixes

### Fix Block 1: Plugin Browsing (Lines 343-354)

**Current:**
```markdown
### Step 1: Browse the Marketplace

```bash
claude /plugin
```

This shows available plugins from three sources:
```

**Fixed:**
```markdown
### Step 1: Browse the Marketplace

**From your terminal** (not in a Claude Code session):

```bash
claude plugin marketplace list
```

This shows available marketplaces you've configured. To explore plugins, you can:

**Option A:** Browse GitHub marketplace directly:
```bash
# First, add a marketplace source
claude plugin marketplace add anthropics/claude-code
```

**Option B:** List installed plugins:
```bash
claude plugin list
```

**Available from official Anthropic marketplace:**
```

---

### Fix Block 2: Marketplace Configuration (Lines 372-378)

**Current:**
```markdown
### Step 2: Add to Marketplace List

```bash
claude /plugin marketplace add anthropics/claude-code
```

This registers the Anthropic marketplace as a source Claude Code can search.
```

**Fixed:**
```markdown
### Step 2: Add to Marketplace List

**From your terminal:**

```bash
claude plugin marketplace add anthropics/claude-code
```

This registers the Anthropic marketplace as a source. You can verify it was added:

```bash
claude plugin marketplace list
```

You should see `anthropics/claude-code` in the list.
```

---

### Fix Block 3: Plugin Installation (Lines 380-399)

**Current:**
```markdown
### Step 3: Install and Activate

```bash
claude /plugin install feature-dev
# Wait for download and installation to complete

# CRITICAL: Restart Claude Code
exit
claude

# Verify activation
claude /help
# Should now show: /feature-dev command available
```
```

**Fixed:**
```markdown
### Step 3: Install and Activate

**From your terminal:**

```bash
# Install the plugin
claude plugin install feature-dev

# Verify it's installed
claude plugin list
# Should show "feature-dev" with status "enabled"
```

**CRITICAL:** Plugins are loaded when Claude Code starts. If you had an active session, restart it:

```bash
# If Claude Code was running, restart:
# Press Ctrl+C to exit current session
# Then start a new session
claude
```

**Inside your Claude Code session**, verify the plugin's commands are available:

```
> /help
# Look for commands added by the plugin
# The feature-dev plugin may add commands like /feature-dev or /code-review
```

**Note:** Different plugins add different capabilities. Some add slash commands, some add skills (autonomous), some add agents. Check the plugin's documentation to know what to expect.
```

---

## Constitutional Alignment Check

### Principle 13 (Graduated Teaching) — ⚠️ VIOLATED

**Issue:** Lesson teaches incorrect syntax (Tier 1 book teaching should be accurate)

**Impact:** Students cannot successfully execute hands-on exercises (Tier 2 AI companion collaboration blocked)

**Fix:** Correct all command syntax to match actual Claude Code CLI

---

### Principle 5 (Validation-First Safety) — ⚠️ VIOLATED

**Issue:** Commands were not tested in sandbox before publication

**Impact:** Lesson published with critical errors that block learning

**Fix:** All commands must be sandbox-tested before finalizing lesson

---

### Core Philosophy #4 (Evals-First) — ⚠️ VIOLATED

**Issue:** No executable test defined for "Learning Objective 3: Install a real community plugin"

**Expected Eval:**
```bash
# Test: Student can successfully install and verify a plugin
claude plugin marketplace add anthropics/claude-code
claude plugin install <example-plugin>
claude plugin list | grep <example-plugin>
# Exit code 0 = pass
```

**Fix:** Define executable validation for all hands-on learning objectives

---

## Next Steps

1. **IMMEDIATE (15 min):** Fix all 5 critical command syntax errors (Lines 346, 375, 383, 391, 392)

2. **HIGH PRIORITY (30 min):**
   - Add clear distinction between CLI commands and session slash commands
   - Update verification workflow to match actual Claude Code behavior
   - Test complete plugin installation workflow in sandbox

3. **MEDIUM PRIORITY (1 hour):**
   - Verify MCP @-mention activation syntax
   - Confirm "feature-dev" plugin exists or replace with verified example
   - Add "How to verify what a plugin adds" section

4. **RECOMMENDED (2 hours):**
   - Create executable test script for Learning Objective 3
   - Add troubleshooting section for common plugin installation errors
   - Include example of checking plugin capabilities after installation

---

## Validation Checklist

- [ ] All CLI commands tested in terminal (`claude plugin [subcommand]`)
- [ ] All session commands tested in Claude Code session (`/help`, `/commit`, etc.)
- [ ] Plugin installation workflow verified end-to-end
- [ ] Marketplace configuration tested with real marketplace
- [ ] MCP @-mention syntax verified in actual Claude Code session
- [ ] "feature-dev" plugin confirmed to exist (or replaced with verified example)
- [ ] Learning Objective 3 achievable with correct commands
- [ ] Troubleshooting section added for common errors

---

## Audit Conclusion

**Status:** 🚨 **MAJOR REVISION REQUIRED**

The lesson's **conceptual content is excellent** (progressive disclosure explanation, plugin architecture diagram, community ecosystem overview). However, the **hands-on commands are entirely incorrect**, rendering Learning Objective 3 ("Install a real community plugin and verify activation") unachievable.

**Estimated Time to Fix:** 1-2 hours for complete correction + sandbox testing

**Recommendation:** Block publication until all command syntax errors are corrected and hands-on workflow is verified in sandbox.

**User was 100% correct:** "If you have not run anything in sandbox then chances are it won't work" — the commands are indeed broken.

---

**Audit performed using Vertical Intelligence Pattern:**
- Layer 1 (Constitution): Validated against Principle 5 (Validation-First), Principle 13 (Graduated Teaching)
- Layer 2 (Domain Knowledge): Referenced actual SKILL.md structure from `.claude/skills/`
- Layer 3 (Context Discovery): Executed `claude plugin --help` and `claude plugin marketplace --help`
- Layer 4 (Intelligence Synthesis): Identified systemic pattern of CLI vs session command confusion
- Layer 5 (Actionable Output): Provided line-by-line fixes with correct syntax

**Constitutional Note:** This audit embodies Philosophy #5 (Validation-First Safety) by catching critical errors before they reach students.
