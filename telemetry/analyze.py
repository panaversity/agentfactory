#!/usr/bin/env python3
"""
Analyze parsed Claude Code telemetry data.

Provides insights following Andrew Ng's recommendations:
- Break down data silos (aggregate across sessions)
- Error analysis (identify failure patterns)
- Evals-first (measure improvement over time)
"""

import json
import re
from pathlib import Path
from datetime import datetime
from collections import defaultdict, Counter
from typing import Dict, List, Any
import sys


class TelemetryAnalyzer:
    """Analyze parsed telemetry data."""

    def __init__(self, data_dir: Path):
        self.data_dir = data_dir
        self.sessions: List[Dict[str, Any]] = []

    def load_data(self) -> None:
        """Load all parsed JSON files."""
        json_files = list(self.data_dir.glob("session-*.json"))

        if not json_files:
            print(f"⚠️  No parsed data files found in {self.data_dir}")
            print(f"   Run: python telemetry/parser.py first")
            sys.exit(0)

        for json_file in sorted(json_files):
            with open(json_file, 'r', encoding='utf-8') as f:
                self.sessions.append(json.load(f))

        print(f"📊 Loaded {len(self.sessions)} session(s)")

    def summary_report(self) -> Dict[str, Any]:
        """Generate summary statistics across all sessions."""
        total_events = sum(s['events_count'] for s in self.sessions)
        total_metrics = sum(s['metrics_count'] for s in self.sessions)

        # Count event types
        event_types = Counter()
        for session in self.sessions:
            for event in session.get('events', []):
                event_types[event['event_type']] += 1

        # Extract costs and tokens (from events)
        total_cost = 0.0
        total_tokens = 0
        api_requests = []

        for session in self.sessions:
            for event in session.get('events', []):
                if 'api_request' in event['event_type']:
                    data = event.get('data', {})
                    api_requests.append(event)

                    # Extract cost and tokens from event data
                    # Try direct field access first (most reliable)
                    if isinstance(data, dict):
                        # Multiple field name variations
                        cost = (data.get('cost') or
                               data.get('cost_usd') or
                               data.get('costUsd') or
                               data.get('price'))
                        if cost is not None:
                            total_cost += float(cost)

                        tokens = (data.get('tokens') or
                                 data.get('total_tokens') or
                                 data.get('totalTokens') or
                                 data.get('token_count'))
                        if tokens is not None:
                            total_tokens += int(tokens)

                    # Fallback: Try regex on string representation
                    if total_cost == 0.0 and 'cost' in str(data).lower():
                        cost_match = re.search(r'["\']?cost["\']?\s*[:=]\s*([0-9.]+)', str(data), re.I)
                        if cost_match:
                            total_cost += float(cost_match.group(1))

                    if total_tokens == 0 and 'token' in str(data).lower():
                        token_match = re.search(r'["\']?tokens?["\']?\s*[:=]\s*([0-9]+)', str(data), re.I)
                        if token_match:
                            total_tokens += int(token_match.group(1))

        # Session durations (approximate from timestamps)
        session_count = len(self.sessions)

        return {
            "summary": {
                "total_sessions": session_count,
                "total_events": total_events,
                "total_metrics": total_metrics,
                "total_api_requests": len(api_requests),
                "estimated_total_cost_usd": round(total_cost, 4),
                "estimated_total_tokens": total_tokens
            },
            "event_types": dict(event_types),
            "sessions": [
                {
                    "file": s['file'],
                    "session_id": s['session'].get('session_id'),
                    "events": s['events_count'],
                    "metrics": s['metrics_count']
                }
                for s in self.sessions
            ]
        }

    def error_analysis(self) -> Dict[str, Any]:
        """Identify error patterns for improvement (Andrew Ng's methodology)."""
        errors = []
        tool_failures = []
        high_retry_sessions = []

        for session in self.sessions:
            session_errors = 0
            session_tool_failures = 0

            for event in session.get('events', []):
                event_type = event['event_type']

                # Track API errors
                if 'error' in event_type:
                    errors.append({
                        "session_id": session['session'].get('session_id'),
                        "file": session['file'],
                        "event": event
                    })
                    session_errors += 1

                # Track tool failures (look for failure status)
                if 'tool_result' in event_type:
                    data_str = str(event.get('data', {}))
                    if 'fail' in data_str.lower() or 'error' in data_str.lower():
                        tool_failures.append({
                            "session_id": session['session'].get('session_id'),
                            "file": session['file'],
                            "event": event
                        })
                        session_tool_failures += 1

            # Flag sessions with high error rates
            if session_errors >= 3 or session_tool_failures >= 5:
                high_retry_sessions.append({
                    "session_id": session['session'].get('session_id'),
                    "file": session['file'],
                    "errors": session_errors,
                    "tool_failures": session_tool_failures
                })

        return {
            "total_errors": len(errors),
            "total_tool_failures": len(tool_failures),
            "high_retry_sessions": len(high_retry_sessions),
            "errors": errors[:10],  # First 10 for detailed review
            "tool_failures": tool_failures[:10],
            "sessions_needing_review": high_retry_sessions
        }

    def cost_analysis(self) -> Dict[str, Any]:
        """Analyze costs and token usage."""
        sessions_with_costs = []

        for session in self.sessions:
            session_cost = 0.0
            session_tokens = 0
            api_calls = 0

            for event in session.get('events', []):
                if 'api_request' in event['event_type']:
                    api_calls += 1
                    data = event.get('data', {})

                    # Extract cost - try direct access first
                    if isinstance(data, dict):
                        cost = (data.get('cost') or
                               data.get('cost_usd') or
                               data.get('costUsd') or
                               data.get('price'))
                        if cost is not None:
                            session_cost += float(cost)

                        tokens = (data.get('tokens') or
                                 data.get('total_tokens') or
                                 data.get('totalTokens') or
                                 data.get('token_count'))
                        if tokens is not None:
                            session_tokens += int(tokens)

                    # Fallback: regex on string representation
                    if session_cost == 0.0:
                        data_str = str(data)
                        cost_match = re.search(r'["\']?cost["\']?\s*[:=]\s*([0-9.]+)', data_str, re.I)
                        if cost_match:
                            session_cost += float(cost_match.group(1))

                    if session_tokens == 0:
                        data_str = str(data)
                        token_match = re.search(r'["\']?tokens?["\']?\s*[:=]\s*([0-9]+)', data_str, re.I)
                        if token_match:
                            session_tokens += int(token_match.group(1))

            if api_calls > 0:
                sessions_with_costs.append({
                    "file": Path(session['file']).name,
                    "session_id": session['session'].get('session_id'),
                    "api_calls": api_calls,
                    "cost_usd": round(session_cost, 4),
                    "tokens": session_tokens,
                    "cost_per_call": round(session_cost / api_calls, 6) if api_calls > 0 else 0
                })

        # Sort by cost descending
        sessions_with_costs.sort(key=lambda x: x['cost_usd'], reverse=True)

        total_cost = sum(s['cost_usd'] for s in sessions_with_costs)
        total_tokens = sum(s['tokens'] for s in sessions_with_costs)

        return {
            "total_cost_usd": round(total_cost, 4),
            "total_tokens": total_tokens,
            "sessions_analyzed": len(sessions_with_costs),
            "top_sessions_by_cost": sessions_with_costs[:10]
        }

    def print_report(self) -> None:
        """Print formatted analysis report."""
        print("\n" + "="*80)
        print("📊 CLAUDE CODE TELEMETRY ANALYSIS REPORT")
        print("="*80 + "\n")

        # Summary
        summary = self.summary_report()
        print("📈 SUMMARY")
        print("-" * 80)
        for key, value in summary['summary'].items():
            print(f"  {key.replace('_', ' ').title()}: {value}")
        print()

        if summary['event_types']:
            print("  Event Types:")
            for event_type, count in summary['event_types'].items():
                print(f"    - {event_type}: {count}")
        print()

        # Cost Analysis
        costs = self.cost_analysis()
        print("💰 COST ANALYSIS")
        print("-" * 80)
        print(f"  Total Cost: ${costs['total_cost_usd']:.4f} USD")
        print(f"  Total Tokens: {costs['total_tokens']:,}")
        print(f"  Sessions Analyzed: {costs['sessions_analyzed']}")
        print()

        if costs['top_sessions_by_cost']:
            print("  Top Sessions by Cost:")
            for i, session in enumerate(costs['top_sessions_by_cost'][:5], 1):
                print(f"    {i}. {session['file']}")
                print(f"       Cost: ${session['cost_usd']:.4f}, "
                      f"Tokens: {session['tokens']:,}, "
                      f"API Calls: {session['api_calls']}")
        print()

        # Error Analysis
        errors = self.error_analysis()
        print("🔍 ERROR ANALYSIS (Andrew Ng Methodology)")
        print("-" * 80)
        print(f"  Total Errors: {errors['total_errors']}")
        print(f"  Total Tool Failures: {errors['total_tool_failures']}")
        print(f"  High-Retry Sessions: {errors['high_retry_sessions']}")
        print()

        if errors['sessions_needing_review']:
            print("  ⚠️  Sessions Needing Review:")
            for session in errors['sessions_needing_review'][:5]:
                print(f"    - {Path(session['file']).name}")
                print(f"      Errors: {session['errors']}, "
                      f"Tool Failures: {session['tool_failures']}")
        else:
            print("  ✅ No problematic sessions detected!")
        print()

        # Recommendations
        print("💡 RECOMMENDATIONS")
        print("-" * 80)

        if errors['total_errors'] > 0:
            print("  1. Review error events to identify common failure patterns")
            print("     - Check API rate limits, network issues, or prompt complexity")
        else:
            print("  1. ✅ No errors detected - system operating smoothly")

        if costs['total_cost_usd'] > 1.0:
            print(f"  2. Monitor costs - ${costs['total_cost_usd']:.2f} spent across sessions")
            print("     - Consider prompt optimization for expensive sessions")
        else:
            print("  2. ✅ Costs within reasonable range")

        if len(self.sessions) < 5:
            print(f"  3. Collect more data - only {len(self.sessions)} session(s) analyzed")
            print("     - Continue using Claude Code with telemetry enabled")

        print()
        print("="*80)
        print("📁 Data Location: " + str(self.data_dir))
        print("="*80 + "\n")


def main():
    """Run telemetry analysis."""
    telemetry_dir = Path.home() / ".claude-code-telemetry"
    data_dir = telemetry_dir / "data"

    if not data_dir.exists():
        print(f"❌ Data directory not found: {data_dir}")
        print(f"   Run: python telemetry/parser.py first")
        sys.exit(1)

    analyzer = TelemetryAnalyzer(data_dir)
    analyzer.load_data()
    analyzer.print_report()

    # Save report to file
    report_file = telemetry_dir / "latest-report.json"
    with open(report_file, 'w', encoding='utf-8') as f:
        json.dump({
            "generated_at": datetime.now().isoformat(),
            "summary": analyzer.summary_report(),
            "costs": analyzer.cost_analysis(),
            "errors": analyzer.error_analysis()
        }, f, indent=2)

    print(f"📄 Full report saved to: {report_file}")


if __name__ == "__main__":
    main()
