#!/usr/bin/env python3
"""Verify building-with-claude-agent-sdk skill is correctly configured."""
import os
import sys

def main():
    skill_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    skill_md = os.path.join(skill_path, "SKILL.md")

    if not os.path.isfile(skill_md):
        print(f"✗ SKILL.md not found")
        sys.exit(1)

    with open(skill_md, 'r') as f:
        content = f.read()

    # Check YAML frontmatter exists
    if not content.startswith('---'):
        print("✗ Missing YAML frontmatter")
        sys.exit(1)

    # Check required sections
    required = [
        '## Overview',
        '## Quick Reference',
        '## Built-in Tools',
        '## Core Patterns'
    ]
    for section in required:
        if section not in content:
            print(f"✗ Missing section: {section}")
            sys.exit(1)

    # Check references exist
    refs_path = os.path.join(skill_path, "references")
    expected_refs = ['python-patterns.md', 'typescript-patterns.md', 'hosting.md']
    for ref in expected_refs:
        if not os.path.isfile(os.path.join(refs_path, ref)):
            print(f"✗ Missing reference: {ref}")
            sys.exit(1)

    print("✓ building-with-claude-agent-sdk valid")
    sys.exit(0)

if __name__ == "__main__":
    main()
