#!/usr/bin/env python3
"""Analyze skill and subagent usage from activity logs.

Usage:
  python3 analyze-skills.py                    # Full analysis (all sessions)
  python3 analyze-skills.py --session <id>     # Filter by session ID
  python3 analyze-skills.py --last             # Only last session
  python3 analyze-skills.py --nested           # Show nested skill usage by subagents
  python3 analyze-skills.py --json             # Output as JSON (for automation)
"""

import argparse
import json
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

SKILLS_DIR = Path(".claude/skills")
AGENTS_DIR = Path(".claude/agents")
LOGS_DIR = Path(".claude/activity-logs")
SKILL_USAGE_LOG = LOGS_DIR / "skill-usage.jsonl"
SUBAGENT_USAGE_LOG = LOGS_DIR / "subagent-usage.jsonl"
PROMPTS_LOG = LOGS_DIR / "prompts.jsonl"


def get_skill_type(skill_name: str) -> str:
    """Detect skill type by checking for verify.py."""
    verify_path = SKILLS_DIR / skill_name / "scripts" / "verify.py"
    return "procedural" if verify_path.exists() else "content"


def load_jsonl(path: Path, session_filter: str = None) -> list:
    """Load JSONL file into list of dicts, optionally filtering by session."""
    if not path.exists():
        return []
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    entry = json.loads(line)
                    if session_filter is None or entry.get("session_id") == session_filter:
                        entries.append(entry)
                except json.JSONDecodeError:
                    continue
    return entries


def get_last_session_id() -> str:
    """Get the session ID of the most recent prompt."""
    prompts = load_jsonl(PROMPTS_LOG)
    if prompts:
        return prompts[-1].get("session_id", "unknown")
    return None


def get_all_session_ids() -> list:
    """Get all unique session IDs from prompts log."""
    prompts = load_jsonl(PROMPTS_LOG)
    return list(dict.fromkeys(p.get("session_id") for p in prompts if p.get("session_id")))


def analyze_skills(session_filter: str = None, show_unused: bool = True):
    """Analyze skill usage."""
    print("\n" + "=" * 70)
    print("SKILL USAGE ANALYSIS")
    if session_filter:
        print(f"(Filtered: session {session_filter[:8]}...)")
    print("=" * 70)

    events = load_jsonl(SKILL_USAGE_LOG, session_filter)

    if not events:
        print("\nNo skill usage recorded yet.")
        return {}

    # Aggregate by skill
    invocations = defaultdict(int)
    invoke_tool = defaultdict(int)  # Via Skill tool
    read_skill = defaultdict(int)   # Via Read SKILL.md
    successes = defaultdict(int)
    failures = defaultdict(int)

    for event in events:
        skill = event.get("skill", "unknown")
        event_type = event.get("event")
        source = event.get("source", "")

        if event_type == "invoke":
            invocations[skill] += 1
            invoke_tool[skill] += 1
        elif event_type == "start":
            invocations[skill] += 1
            read_skill[skill] += 1
        elif event_type == "verify":
            status = event.get("status")
            if status == "success":
                successes[skill] += 1
            elif status == "failure":
                failures[skill] += 1

    # Get all skills from directory (only for full analysis)
    all_skills = set()
    if show_unused and SKILLS_DIR.exists():
        for skill_dir in SKILLS_DIR.iterdir():
            if skill_dir.is_dir() and (skill_dir / "SKILL.md").exists():
                all_skills.add(skill_dir.name)

    total_invocations = sum(invocations.values())
    print(f"\nTotal skill activations: {total_invocations}")
    print(f"  - Via Skill tool: {sum(invoke_tool.values())}")
    print(f"  - Via Read SKILL.md: {sum(read_skill.values())}")

    # Print skills table (only used skills if session filter)
    skills_to_show = set(invocations.keys()) if session_filter else (set(invocations.keys()) | all_skills)

    if skills_to_show:
        print(f"\n{'Skill':<35} {'Total':>8} {'Invoke':>8} {'Read':>8} {'Success':>8} {'Fail':>6}")
        print("-" * 75)

        for skill in sorted(skills_to_show):
            inv = invocations.get(skill, 0)
            inv_t = invoke_tool.get(skill, 0)
            read_s = read_skill.get(skill, 0)
            succ = successes.get(skill, 0)
            fail = failures.get(skill, 0)

            if inv > 0 or (not session_filter and skill in all_skills):
                print(f"{skill:<35} {inv:>8} {inv_t:>8} {read_s:>8} {succ:>8} {fail:>6}")

    # Unused skills (only for full analysis)
    if show_unused:
        used_skills = set(invocations.keys())
        unused = all_skills - used_skills
        if unused:
            print(f"\nUNUSED SKILLS ({len(unused)}):")
            for skill in sorted(unused)[:10]:
                print(f"   - {skill}")
            if len(unused) > 10:
                print(f"   ... and {len(unused) - 10} more")

    # Return data for JSON output
    return {
        "total": total_invocations,
        "via_skill_tool": sum(invoke_tool.values()),
        "via_read": sum(read_skill.values()),
        "by_skill": dict(invocations)
    }


def analyze_subagents(session_filter: str = None):
    """Analyze subagent usage."""
    print("\n" + "=" * 70)
    print("SUBAGENT USAGE ANALYSIS")
    if session_filter:
        print(f"(Filtered: session {session_filter[:8]}...)")
    print("=" * 70)

    events = load_jsonl(SUBAGENT_USAGE_LOG, session_filter)

    if not events:
        print("\nNo subagent usage recorded yet.")
        return {}

    # Aggregate by subagent type
    spawns = defaultdict(int)
    completions = defaultdict(int)
    errors = defaultdict(int)
    background_spawns = defaultdict(int)
    models_used = defaultdict(lambda: defaultdict(int))
    # Track agent_ids for nested skill correlation
    agent_ids_by_type = defaultdict(list)

    for event in events:
        agent = event.get("subagent", "unknown")
        event_type = event.get("event")

        if event_type == "spawn":
            spawns[agent] += 1
            if event.get("background"):
                background_spawns[agent] += 1
            model = event.get("model", "inherit")
            models_used[agent][model] += 1
        elif event_type == "complete":
            status = event.get("status", "completed")
            agent_id = event.get("agent_id", "")
            if agent_id:
                agent_ids_by_type[agent].append(agent_id)
            if status == "error":
                errors[agent] += 1
            else:
                completions[agent] += 1

    total_spawns = sum(spawns.values())
    total_bg = sum(background_spawns.values())
    print(f"\nTotal subagent spawns: {total_spawns}")
    print(f"  - Background (async): {total_bg}")
    print(f"  - Foreground (sync): {total_spawns - total_bg}")

    # Print subagents table
    print(f"\n{'Subagent':<30} {'Spawns':>8} {'Async':>8} {'Complete':>10} {'Errors':>8}")
    print("-" * 70)

    for agent in sorted(spawns.keys()):
        sp = spawns.get(agent, 0)
        bg = background_spawns.get(agent, 0)
        comp = completions.get(agent, 0)
        err = errors.get(agent, 0)
        print(f"{agent:<30} {sp:>8} {bg:>8} {comp:>10} {err:>8}")

    # Model usage
    print("\nMODEL USAGE BY SUBAGENT:")
    for agent in sorted(models_used.keys()):
        models = models_used[agent]
        model_str = ", ".join(f"{m}: {c}" for m, c in sorted(models.items()))
        print(f"   {agent}: {model_str}")

    # Return data for JSON output
    return {
        "total_spawns": total_spawns,
        "background": total_bg,
        "foreground": total_spawns - total_bg,
        "by_type": dict(spawns),
        "completions": dict(completions),
        "errors": dict(errors),
        "models": {k: dict(v) for k, v in models_used.items()},
        "agent_ids": dict(agent_ids_by_type)
    }


def analyze_nested_usage(session_filter: str = None):
    """Analyze skill usage by subagents (nested usage).

    This correlates subagent agent_ids with skill usage sessions to show
    which skills were used by which subagent types.
    """
    print("\n" + "=" * 70)
    print("NESTED USAGE ANALYSIS (Skills Used by Subagents)")
    if session_filter:
        print(f"(Filtered: session {session_filter[:8]}...)")
    print("=" * 70)

    # Load all data
    subagent_events = load_jsonl(SUBAGENT_USAGE_LOG, session_filter)
    skill_events = load_jsonl(SKILL_USAGE_LOG)  # Need all skills to correlate

    if not subagent_events:
        print("\nNo subagent data available for nested analysis.")
        return {}

    # Build mapping: agent_id -> subagent_type
    agent_id_to_type = {}
    for event in subagent_events:
        if event.get("event") == "complete":
            agent_id = event.get("agent_id", "")
            agent_type = event.get("subagent", "unknown")
            if agent_id:
                agent_id_to_type[agent_id] = agent_type

    # Build mapping: session_id -> skills used
    # (Subagents get their own session_id when they run)
    session_to_skills = defaultdict(list)
    for event in skill_events:
        sid = event.get("session_id", "")
        skill = event.get("skill", "unknown")
        if sid:
            session_to_skills[sid].append(skill)

    # Try to correlate (this is approximate - subagents may reuse session patterns)
    # For now, show skill usage summary that could be from subagents
    nested_usage = defaultdict(lambda: defaultdict(int))

    # Note: True nested tracking requires subagents to pass their agent_id
    # to skill hooks, which isn't implemented yet. For now, show available data.

    if agent_id_to_type:
        print("\nSubagent completions tracked (with agent_ids):")
        for agent_id, agent_type in sorted(agent_id_to_type.items()):
            print(f"   {agent_type}: {agent_id}")

        print("\nNote: Full nested skill tracking requires agent_id propagation")
        print("to skill hooks. Consider updating hooks to include parent_agent_id.")
    else:
        print("\nNo agent_id data available yet.")
        print("Subagent completions will track agent_ids for correlation.")

    return {
        "agent_id_mapping": agent_id_to_type,
        "sessions_with_skills": len(session_to_skills),
        "note": "Full nested tracking requires hook enhancement"
    }


def analyze_prompts():
    """Analyze prompt submissions."""
    print("\n" + "=" * 70)
    print("PROMPT ANALYSIS")
    print("=" * 70)

    prompts = load_jsonl(PROMPTS_LOG)

    if not prompts:
        print("\nNo prompts recorded yet.")
        return

    print(f"\nTotal prompts: {len(prompts)}")

    # Analyze prompt patterns
    slash_commands = defaultdict(int)
    mentions = defaultdict(int)

    for entry in prompts:
        prompt = entry.get("prompt", "")

        # Count slash commands
        for word in prompt.split():
            if word.startswith("/"):
                cmd = word.split()[0].rstrip(".,!?")
                slash_commands[cmd] += 1

            # Count @ mentions
            if word.startswith("@"):
                mention = word.rstrip(".,!?")
                mentions[mention] += 1

    if slash_commands:
        print("\nSlash commands used:")
        for cmd, count in sorted(slash_commands.items(), key=lambda x: -x[1]):
            print(f"   {cmd}: {count}")

    if mentions:
        print("\n@ mentions:")
        for mention, count in sorted(mentions.items(), key=lambda x: -x[1]):
            print(f"   {mention}: {count}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze Claude Code skill and subagent usage from activity logs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 analyze-skills.py                    # Full analysis
  python3 analyze-skills.py --last             # Last session only
  python3 analyze-skills.py --session abc123   # Specific session
  python3 analyze-skills.py --nested           # Include nested usage
  python3 analyze-skills.py --json             # JSON output for automation
        """
    )
    parser.add_argument("--session", "-s", help="Filter by session ID")
    parser.add_argument("--last", "-l", action="store_true",
                        help="Analyze only the last session")
    parser.add_argument("--nested", "-n", action="store_true",
                        help="Include nested skill usage by subagents")
    parser.add_argument("--json", "-j", action="store_true",
                        help="Output as JSON for automation")
    parser.add_argument("--list-sessions", action="store_true",
                        help="List all session IDs")

    args = parser.parse_args()

    # Handle session filtering
    session_filter = None
    if args.last:
        session_filter = get_last_session_id()
        if not session_filter:
            print("No sessions found.")
            sys.exit(1)
    elif args.session:
        session_filter = args.session

    # List sessions mode
    if args.list_sessions:
        sessions = get_all_session_ids()
        print(f"Found {len(sessions)} sessions:")
        for sid in sessions[-10:]:  # Show last 10
            print(f"   {sid}")
        if len(sessions) > 10:
            print(f"   ... and {len(sessions) - 10} more")
        sys.exit(0)

    # JSON output mode
    if args.json:
        results = {
            "analysis_date": datetime.now().isoformat(),
            "session_filter": session_filter,
            "skills": analyze_skills(session_filter, show_unused=False) if session_filter else {},
            "subagents": analyze_subagents(session_filter),
        }
        if args.nested:
            results["nested"] = analyze_nested_usage(session_filter)
        # Suppress print output for JSON mode by redirecting
        print(json.dumps(results, indent=2, default=str))
        sys.exit(0)

    # Normal output mode
    print("=" * 70)
    print("CLAUDE CODE ACTIVITY ANALYSIS")
    print("=" * 70)
    print(f"\nAnalysis date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"Log directory: {LOGS_DIR.absolute()}")
    if session_filter:
        print(f"Session filter: {session_filter}")

    analyze_prompts()
    analyze_skills(session_filter, show_unused=not session_filter)
    analyze_subagents(session_filter)

    if args.nested:
        analyze_nested_usage(session_filter)

    print("\n" + "=" * 70)
    print("END OF ANALYSIS")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
