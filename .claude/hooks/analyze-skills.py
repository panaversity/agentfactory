#!/usr/bin/env python3
"""Analyze skill usage from activity logs."""

import json
from collections import defaultdict
from datetime import datetime
from pathlib import Path

SKILLS_DIR = Path(".claude/skills")
LOGS_DIR = Path(".claude/activity-logs")
SKILL_USAGE_LOG = LOGS_DIR / "skill-usage.jsonl"
PROMPTS_LOG = LOGS_DIR / "prompts.jsonl"


def get_skill_type(skill_name: str) -> str:
    """Detect skill type by checking for verify.py."""
    verify_path = SKILLS_DIR / skill_name / "scripts" / "verify.py"
    return "procedural" if verify_path.exists() else "content"


def load_jsonl(path: Path) -> list:
    """Load JSONL file into list of dicts."""
    if not path.exists():
        return []
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return entries


def analyze():
    """Run skill usage analysis."""
    print("=" * 70)
    print("SKILL USAGE ANALYSIS")
    print("=" * 70)
    print()
    print(f"Analysis date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"Log directory: {LOGS_DIR.absolute()}")
    print()

    # Load logs
    prompts = load_jsonl(PROMPTS_LOG)
    events = load_jsonl(SKILL_USAGE_LOG)

    print(f"Total prompts logged: {len(prompts)}")

    # Aggregate by skill
    invocations = defaultdict(int)
    successes = defaultdict(int)
    failures = defaultdict(int)

    for event in events:
        skill = event.get("skill", "unknown")
        event_type = event.get("event")

        if event_type == "start":
            invocations[skill] += 1
        elif event_type == "verify":
            status = event.get("status")
            if status == "success":
                successes[skill] += 1
            elif status == "failure":
                failures[skill] += 1

    total_invocations = sum(invocations.values())
    print(f"Total skill invocations: {total_invocations}")
    print()

    # Get all skills from directory
    all_skills = set()
    if SKILLS_DIR.exists():
        for skill_dir in SKILLS_DIR.iterdir():
            if skill_dir.is_dir() and (skill_dir / "SKILL.md").exists():
                all_skills.add(skill_dir.name)

    # Separate by type
    procedural_skills = {s for s in all_skills if get_skill_type(s) == "procedural"}
    content_skills = all_skills - procedural_skills

    # Print procedural skills
    if procedural_skills or any(get_skill_type(s) == "procedural" for s in invocations):
        print("PROCEDURAL SKILLS (with verify.py):")
        print(f"{'Skill':<40} {'Invocations':>12} {'Success':>10} {'Failure':>10} {'Rate':>8}")
        print("-" * 80)

        for skill in sorted(procedural_skills | {s for s in invocations if get_skill_type(s) == "procedural"}):
            inv = invocations.get(skill, 0)
            succ = successes.get(skill, 0)
            fail = failures.get(skill, 0)
            total_verify = succ + fail
            rate = f"{(succ / total_verify * 100):.1f}%" if total_verify > 0 else "N/A"
            print(f"{skill:<40} {inv:>12} {succ:>10} {fail:>10} {rate:>8}")
        print()

    # Print content skills
    if content_skills or any(get_skill_type(s) == "content" for s in invocations):
        print("CONTENT SKILLS (no verify.py):")
        print(f"{'Skill':<40} {'Invocations':>12} {'Success Rate':>15}")
        print("-" * 70)

        for skill in sorted(content_skills | {s for s in invocations if get_skill_type(s) == "content"}):
            inv = invocations.get(skill, 0)
            print(f"{skill:<40} {inv:>12} {'N/A':>15}")
        print()

    # Unused skills
    used_skills = set(invocations.keys())
    unused = all_skills - used_skills
    if unused:
        print(f"UNUSED SKILLS ({len(unused)}):")
        for skill in sorted(unused):
            skill_type = get_skill_type(skill)
            print(f"   - {skill} ({skill_type})")
        print()

    # High failure rate (procedural only)
    high_failure = []
    for skill in procedural_skills:
        total_verify = successes.get(skill, 0) + failures.get(skill, 0)
        if total_verify > 0:
            failure_rate = failures.get(skill, 0) / total_verify
            if failure_rate > 0.3:
                high_failure.append((skill, failure_rate))

    if high_failure:
        print("HIGH FAILURE RATE SKILLS (>30%):")
        for skill, rate in sorted(high_failure, key=lambda x: -x[1]):
            print(f"   - {skill}: {rate*100:.0f}% failure rate")
        print()

    # Overall success rate for procedural skills
    total_succ = sum(successes.values())
    total_fail = sum(failures.values())
    total_verify = total_succ + total_fail
    if total_verify > 0:
        overall_rate = total_succ / total_verify * 100
        print("=" * 70)
        print(f"Procedural skills success rate: {overall_rate:.1f}%")
        print("=" * 70)


if __name__ == "__main__":
    analyze()
