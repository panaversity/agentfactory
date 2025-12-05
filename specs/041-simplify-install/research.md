# Research: Claude Code Installation Methods

**Date**: 2025-12-06
**Purpose**: Analyze existing installation methods to determine optimal primary method selection

## Current Installation Methods Analysis

### Method 1: macOS/Linux (curl script)
```bash
curl -fsSL https://claude.ai/install.sh | bash
```
- **Pros**: Universal for Unix-like systems, single command, no dependencies
- **Cons**: Requires internet, runs script with pipe to bash (security concern for some)
- **Suitability**: Best primary method for both macOS and Linux

### Method 2: Homebrew (macOS)
```bash
brew install --cask claude-code
```
- **Pros**: Package manager integration, easy updates, trusted source
- **Cons**: Requires Homebrew installation, macOS-only
- **Suitability**: Good alternative for macOS users with Homebrew

### Method 3: Windows (PowerShell)
```powershell
irm https://claude.ai/install.ps1 | iex
```
- **Pros**: Native Windows support, single command, no dependencies
- **Cons**: Requires internet, runs script with pipe (similar security concerns)
- **Suitability**: Best primary method for Windows

### Method 4: npm (Cross-platform)
```bash
npm install -g @anthropic-ai/claude-code
```
- **Pros**: Works everywhere Node.js is installed, familiar to developers
- **Cons**: Requires Node.js 18+, slower download from npm registry
- **Suitability**: Good alternative for developers with existing Node.js setup

## Primary Method Selection Rationale

### macOS/Linux: curl script (Method 1)
- Works on both operating systems with one command
- No prerequisite software needed
- Official Anthropic installer script
- Most universal approach

### Windows: PowerShell script (Method 3)
- Native Windows support without extra tools
- No prerequisite software needed
- Official Anthropic installer script
- Standard Windows installation pattern

## Verification Method

The `claude --version` command provides immediate feedback:
```bash
claude --version
# Expected: 2.0.37 (Claude Code) [or current version]
```

## Alternative Methods Strategy

Place Homebrew and npm methods in collapsible section:
- Advanced users who prefer package managers can still find them
- Doesn't overwhelm beginners with choice paralysis
- Preserves all existing functionality

## Cognitive Load Considerations

Current structure presents 4 choices simultaneously:
- Requires students to evaluate unfamiliar methods
- Increases decision time and anxiety
- May lead to analysis paralysis

Proposed structure:
1. OS detection (implicit based on student's system)
2. Single primary command
3. Immediate verification
4. Optional alternatives for advanced users

This reduces cognitive load from 4 parallel decisions to 1 sequential action.