---
sidebar_position: 4
title: "Roots: File System Permissions"
description: "Solve the path discovery problem by configuring roots—secure boundaries that enable Claude to find files while preventing unauthorized access"
keywords: [mcp, roots, file-permissions, path-discovery, security, list_roots, is_path_allowed]
chapter: 38
lesson: 4
duration_minutes: 22

skills:
  mcp-roots-security:
    proficiency_level: C2
    category: Technical
    bloom_level: Apply, Analyze
    digcomp_area: Security-and-Privacy
    measurable_at_this_level: "Students can implement path validation logic and configure multiple roots for different access boundaries"
  file-system-security:
    proficiency_level: C2
    category: Applied
    bloom_level: Analyze, Evaluate
    digcomp_area: Security-and-Privacy
    measurable_at_this_level: "Students can reason about permission boundaries and justify root configuration choices"

learning_objectives:
  - objective: "Explain the path discovery problem and how roots solve it"
    proficiency_level: C2
    bloom_level: Understand
    assessment_method: "Lesson walkthrough and Try With AI prompts"
  - objective: "Implement list_roots() to expose accessible directories"
    proficiency_level: C2
    bloom_level: Apply
    assessment_method: "Code implementation with tool integration"
  - objective: "Build is_path_allowed() validation pattern"
    proficiency_level: C2
    bloom_level: Apply
    assessment_method: "Security-conscious path validation implementation"
  - objective: "Integrate roots with file-processing tools safely"
    proficiency_level: C2
    bloom_level: Analyze
    assessment_method: "Multi-tool integration with permission boundaries"

cognitive_load:
  new_concepts: 4
  assessment: "Path discovery, roots configuration, path validation logic, security implications. Concepts build naturally from prior MCP knowledge."

differentiation:
  extension_for_advanced: "Implement cross-platform path normalization and symbolic link resolution in is_path_allowed()"
  remedial_for_struggling: "Provide pre-built is_path_allowed() template and focus on understanding the security implications through Try With AI"
---

# Roots: File System Permissions

## The Path Discovery Problem

Imagine you've built a video conversion server using MCP. The server implements a tool called `convert_video` that takes a file path and output format.

A user asks your MCP-connected Claude: "Convert biking.mp4 to mov format."

Here's the problem: Claude doesn't know where `biking.mp4` is. It could be on the user's Desktop, in Documents, in a project folder, or anywhere on the system. Without guidance, Claude has no way to search the filesystem intelligently.

Claude could try:

```
/home/user/biking.mp4  # Guess 1
/home/user/Desktop/biking.mp4  # Guess 2
/home/user/Documents/biking.mp4  # Guess 3
```

Each wrong guess is a failed tool call. The user experience becomes frustrating: "Why can't it just find my file?"

**Roots solve this.** They tell Claude: "Here are the directories you can access. When the user asks to convert a file, search within these boundaries."

## What Roots Provide

Roots solve TWO problems simultaneously:

### Problem 1: Path Discovery (UX)

**Without roots:**
- User: "Convert my biking.mp4"
- Claude: Has no way to locate the file
- Result: Tool fails, user frustrated

**With roots:**
- User: "Convert my biking.mp4"
- Claude: Calls `list_roots()` → Sees `["/Users/alex/Videos", "/Users/alex/Desktop"]`
- Claude: Calls `read_dir()` on each root → Finds `biking.mp4` in `/Users/alex/Videos`
- Claude: Calls `convert_video("/Users/alex/Videos/biking.mp4", "mov")`
- Result: Works smoothly

### Problem 2: Security (Safety)

**Without roots:**
- Server can access ANY directory on the system
- Malicious prompt: "Read /etc/passwd"
- Result: Security breach

**With roots:**
- Server configured with roots: `["/Users/alex/Videos", "/Users/alex/Desktop"]`
- Malicious prompt: "Read /etc/passwd"
- Claude tries: `read_file("/etc/passwd")`
- Server checks: Is `/etc/passwd` in allowed roots?
- Result: Blocked. Security maintained.

**Real-world example**: A document processing server should only access a specific `Documents` folder, not the entire system.

## Implementing Roots: The Three-Step Pattern

### Step 1: Define Your Roots

Start with a **specification** for what directories your server should access:

```markdown
## Roots Configuration Specification

**Purpose**: Enable video conversion server to find user files while preventing system access

**Accessible Directories**:
- ~/Videos (where video files typically live)
- ~/Downloads (where shared videos arrive)

**NOT Accessible**:
- /etc, /var, system directories
- ~/.ssh, ~/.aws (credentials)
- Other users' home directories

**Security Model**: Whitelist-based—only approved paths accessible
```

### Step 2: Implement list_roots()

The MCP spec defines a `ListRootsRequest` that servers can handle. Here's how:

```python
from mcp.server import Server
from mcp.types import Root
import os

mcp = Server("video-converter")

@mcp.list_roots()
async def list_roots() -> list[Root]:
    """
    Return the directories Claude can access.

    This enables path discovery: Claude calls this to learn
    where it can search for files.
    """
    home = os.path.expanduser("~")

    return [
        Root(uri=f"file://{home}/Videos", name="Videos Folder"),
        Root(uri=f"file://{home}/Downloads", name="Downloads Folder"),
    ]
```

**What happens:**

1. Client connects to your server
2. Client calls `list_roots()`
3. Server returns authorized directories
4. Claude now knows: "I can search in Videos and Downloads"

### Step 3: Implement Path Validation

Before allowing ANY file operation, validate that the path is within allowed roots:

```python
import os
from pathlib import Path

def get_roots() -> list[str]:
    """Get list of allowed root directories (absolute paths)"""
    home = os.path.expanduser("~")
    return [
        os.path.join(home, "Videos"),
        os.path.join(home, "Downloads"),
    ]

def is_path_allowed(requested_path: str) -> bool:
    """
    Validate that requested_path is within allowed roots.

    Returns True if path is permitted, False if outside boundaries.
    """
    # Normalize to absolute path (handles ~/file.txt, ./file.txt, etc)
    requested_path = os.path.abspath(requested_path)

    # Check each authorized root
    for root in get_roots():
        root_abs = os.path.abspath(root)

        # Use os.path.commonpath to check containment
        try:
            common = os.path.commonpath([root_abs, requested_path])
            # If common path equals root, then requested_path is within root
            if common == root_abs:
                return True
        except ValueError:
            # Different drives on Windows - not allowed
            continue

    return False

@mcp.tool()
async def read_file(path: str) -> str:
    """Read a file, with security validation."""

    # SECURITY CHECK FIRST
    if not is_path_allowed(path):
        raise ValueError(f"Path not in allowed roots: {path}")

    # Now safe to read
    with open(path, 'r') as f:
        return f.read()

@mcp.tool()
async def convert_video(path: str, output_format: str) -> dict:
    """Convert video file."""

    # SECURITY CHECK FIRST
    if not is_path_allowed(path):
        raise ValueError(f"Path not in allowed roots: {path}")

    # Rest of implementation...
    # (conversion logic here)

    return {"status": "converted", "output": output_path}
```

## Security Deep-Dive: Why is_path_allowed() Matters

The `is_path_allowed()` pattern prevents several attack vectors:

### Attack 1: Direct Path Traversal

**Attempt:**
```
Claude asks server to read: /etc/passwd
```

**What happens:**
```python
is_path_allowed("/etc/passwd")
# /etc/passwd is NOT under /home/user/Videos
# Returns False
# Tool call blocked
```

### Attack 2: Symbolic Link Escape

**Attempt:**
```
/home/user/Videos contains symlink: secret -> /etc/shadow
Claude asks to read: /home/user/Videos/secret
```

**Problem:** The symlink is technically within Videos folder, but points outside.

**Solution:** Use `os.path.realpath()` to resolve symlinks:

```python
def is_path_allowed(requested_path: str) -> bool:
    """Validate path, resolving symlinks."""

    requested_path = os.path.abspath(requested_path)
    # Resolve symlinks to get real path
    requested_path = os.path.realpath(requested_path)

    for root in get_roots():
        root_abs = os.path.abspath(root)
        root_real = os.path.realpath(root_abs)  # Also resolve root

        try:
            common = os.path.commonpath([root_real, requested_path])
            if common == root_real:
                return True
        except ValueError:
            continue

    return False
```

### Attack 3: Directory Traversal

**Attempt:**
```
Claude asks to read: /home/user/Videos/../../../etc/passwd
```

**What happens:**
```python
is_path_allowed("/home/user/Videos/../../../etc/passwd")
# After abspath: /etc/passwd
# Not in /home/user/Videos
# Returns False
```

The `os.path.abspath()` normalizes the path, so traversal attempts are blocked.

## Complete Integration Example

Here's how roots fit into a multi-tool server:

```python
from mcp.server import Server, Request
from mcp.types import Root, Tool
import os
import json

mcp = Server("document-processor")

# ===== ROOT MANAGEMENT =====

def get_roots() -> list[str]:
    home = os.path.expanduser("~")
    return [
        os.path.join(home, "Documents"),
        os.path.join(home, "Downloads"),
    ]

def is_path_allowed(requested_path: str) -> bool:
    requested_path = os.path.abspath(requested_path)
    requested_path = os.path.realpath(requested_path)

    for root in get_roots():
        root_abs = os.path.abspath(root)
        root_real = os.path.realpath(root_abs)

        try:
            if os.path.commonpath([root_real, requested_path]) == root_real:
                return True
        except ValueError:
            continue

    return False

@mcp.list_roots()
async def list_roots() -> list[Root]:
    """Expose accessible directories."""
    return [
        Root(
            uri=f"file://{os.path.expanduser('~')}/Documents",
            name="Documents"
        ),
        Root(
            uri=f"file://{os.path.expanduser('~')}/Downloads",
            name="Downloads"
        ),
    ]

# ===== TOOL IMPLEMENTATION =====

@mcp.tool()
async def read_document(path: str) -> str:
    """Read text document with security validation."""

    if not is_path_allowed(path):
        raise ValueError(f"Access denied: {path} not in allowed roots")

    with open(path, 'r') as f:
        return f.read()

@mcp.tool()
async def list_documents(directory: str) -> list[str]:
    """List files in directory with security validation."""

    if not is_path_allowed(directory):
        raise ValueError(f"Access denied: {directory} not in allowed roots")

    if not os.path.isdir(directory):
        raise ValueError(f"Not a directory: {directory}")

    return [
        f for f in os.listdir(directory)
        if os.path.isfile(os.path.join(directory, f))
    ]

@mcp.tool()
async def analyze_document(path: str) -> dict:
    """Analyze document structure with security validation."""

    if not is_path_allowed(path):
        raise ValueError(f"Access denied: {path} not in allowed roots")

    # Analysis logic here
    return {
        "file": path,
        "lines": 100,
        "words": 2500,
        "paragraphs": 15
    }
```

## Common Mistakes to Avoid

### Mistake 1: Forgetting to Validate in Every Tool

```python
# WRONG - only checks read_file, not list_documents
@mcp.tool()
async def read_file(path: str) -> str:
    if not is_path_allowed(path):  # ✓ Checked
        raise ValueError("Not allowed")
    return open(path).read()

@mcp.tool()
async def list_documents(directory: str) -> list[str]:
    # ✗ NO VALIDATION - Security hole!
    return os.listdir(directory)
```

**Fix:** Validate in EVERY tool that accesses the filesystem:

```python
@mcp.tool()
async def list_documents(directory: str) -> list[str]:
    if not is_path_allowed(directory):  # ✓ Now validated
        raise ValueError("Access denied")
    return os.listdir(directory)
```

### Mistake 2: Not Normalizing Paths

```python
# WRONG - doesn't handle relative paths
def is_path_allowed(requested_path: str) -> bool:
    for root in get_roots():
        if requested_path.startswith(root):  # ✗ Fragile
            return True
    return False
```

Attack: `"./../../etc"` might pass string matching but resolve outside roots.

**Fix:** Always normalize:

```python
def is_path_allowed(requested_path: str) -> bool:
    requested_path = os.path.abspath(requested_path)  # ✓ Normalize
    requested_path = os.path.realpath(requested_path)  # ✓ Resolve symlinks

    for root in get_roots():
        root_abs = os.path.abspath(root)
        root_real = os.path.realpath(root_abs)

        try:
            if os.path.commonpath([root_real, requested_path]) == root_real:
                return True
        except ValueError:
            continue

    return False
```

### Mistake 3: Too-Permissive Roots

```python
# WRONG - allows access to home directory and everything under it
def get_roots() -> list[str]:
    return [os.path.expanduser("~")]  # ✗ Too broad
```

This defeats the security boundary. Better:

```python
def get_roots() -> list[str]:
    home = os.path.expanduser("~")
    return [
        os.path.join(home, "Documents"),  # ✓ Specific folders
        os.path.join(home, "Downloads"),  # ✓ Not entire home
    ]
```

## Try With AI

### Prompt 1: Understanding Path Validation

Ask Claude:

```
I have an MCP server that processes videos. Users store videos in different folders, and I need Claude to find them and convert them. But I'm worried about security—I don't want Claude to access system directories or other users' files.

Explain the path validation pattern for roots. Why is os.path.realpath() important for security?
```

**What you're learning**: The security implications of path normalization and symlink resolution.

### Prompt 2: Building is_path_allowed()

Ask Claude:

```
Write a secure is_path_allowed() function for my video conversion server. The function should:

1. Accept a requested file path (may be relative, absolute, or contain ..)
2. Verify it's within /home/user/Videos or /home/user/Desktop
3. Handle symbolic links (symlinks)
4. Return True if allowed, False if outside boundaries
5. Use os.path.realpath() and os.path.commonpath()

Include comments explaining the security approach.
```

**What you're learning**: Implementing the pattern with attention to symlinks and path normalization.

Compare your result to the implementation in this lesson. Ask Claude: "What edge cases does this handle that naive string matching wouldn't?"

### Prompt 3: Integration Testing

Ask Claude:

```
I've implemented roots with list_roots() and is_path_allowed(). Now I need to test it.

Create test cases for the is_path_allowed() function that verify:
1. Normal paths within roots are allowed
2. Paths outside roots are denied
3. Relative paths are normalized correctly
4. Symlinks can't escape the root boundary
5. Directory traversal attempts (..) are blocked

Write these as pytest test cases.
```

**What you're learning**: How to validate your security implementation systematically.

Once Claude generates the tests, **run them against your actual implementation**. Fix any failures—this is real security validation, not theoretical.

