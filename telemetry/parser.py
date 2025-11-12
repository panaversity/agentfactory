#!/usr/bin/env python3
"""
Parse Claude Code console telemetry logs into structured JSON.

This parser extracts OpenTelemetry metrics and events from console output
and saves them as structured JSON for analysis.
"""

import json
import re
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any
import sys


class TelemetryParser:
    """Parse console telemetry output into structured data."""

    def __init__(self, log_file: Path):
        self.log_file = log_file
        self.events: List[Dict[str, Any]] = []
        self.metrics: List[Dict[str, Any]] = []

    def parse(self) -> Dict[str, Any]:
        """Parse the log file and extract all telemetry data."""
        with open(self.log_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract session metadata from resource attributes
        session_info = self._extract_session_info(content)

        # Parse events (logs)
        self._parse_events(content)

        # Parse metrics
        self._parse_metrics(content)

        return {
            "session": session_info,
            "file": str(self.log_file),
            "parsed_at": datetime.now().isoformat(),
            "events_count": len(self.events),
            "metrics_count": len(self.metrics),
            "events": self.events,
            "metrics": self.metrics
        }

    def _extract_session_info(self, content: str) -> Dict[str, Any]:
        """Extract session-level information from resource attributes."""
        session_info = {
            "session_id": None,
            "account_uuid": None,
            "app_version": None,
            "os_type": None,
            "team": None,
            "project": None
        }

        # Look for resource attributes in console output
        # Format varies but typically includes service.name, session.id, etc.

        # Extract session ID (various formats)
        session_match = re.search(r'session[._]id["\s:=]+([a-f0-9\-]+)', content, re.I)
        if session_match:
            session_info["session_id"] = session_match.group(1)

        # Extract account UUID
        account_match = re.search(r'account[._]uuid["\s:=]+([a-f0-9\-]+)', content, re.I)
        if account_match:
            session_info["account_uuid"] = account_match.group(1)

        # Extract team/project from resource attributes
        team_match = re.search(r'team["\s:=]+([a-zA-Z0-9\-_]+)', content, re.I)
        if team_match:
            session_info["team"] = team_match.group(1)

        project_match = re.search(r'project["\s:=]+([a-zA-Z0-9\-_]+)', content, re.I)
        if project_match:
            session_info["project"] = project_match.group(1)

        return session_info

    def _parse_events(self, content: str) -> None:
        """Parse event logs from console output."""
        # Events can appear in multiple formats:
        # 1. JSON with event name embedded
        # 2. Event name followed by JSON
        # 3. Event name with inline data

        event_patterns = {
            'user_prompt': r'claude_code\.user_prompt',
            'tool_result': r'claude_code\.tool_result',
            'api_request': r'claude_code\.api_request',
            'api_error': r'claude_code\.api_error',
            'tool_decision': r'claude_code\.tool_decision'
        }

        for line in content.split('\n'):
            # Skip empty lines
            if not line.strip():
                continue

            # Try multiple parsing strategies
            parsed = False

            # Strategy 1: Event name followed by colon and JSON
            # Example: claude_code.user_prompt: {"timestamp": "...", "data": ...}
            for event_name, pattern in event_patterns.items():
                if re.search(pattern, line) and '{' in line:
                    try:
                        # Find JSON portion
                        json_start = line.index('{')
                        json_str = line[json_start:]
                        data = json.loads(json_str)

                        self.events.append({
                            "event_type": f"claude_code.{event_name}",
                            "timestamp": data.get("timestamp") or data.get("observedTimestamp") or datetime.now().isoformat(),
                            "data": data
                        })
                        parsed = True
                        break
                    except (ValueError, json.JSONDecodeError):
                        continue

            # Strategy 2: JSON object with event name inside
            # Example: {"eventName": "claude_code.user_prompt", "data": ...}
            if not parsed and '{' in line and '}' in line:
                try:
                    json_match = re.search(r'\{.*\}', line)
                    if json_match:
                        data = json.loads(json_match.group(0))

                        # Check if event type is in the JSON or on the line
                        event_type = None
                        for event_name, pattern in event_patterns.items():
                            if re.search(pattern, str(data), re.I) or re.search(pattern, line, re.I):
                                event_type = f"claude_code.{event_name}"
                                break

                        if event_type:
                            self.events.append({
                                "event_type": event_type,
                                "timestamp": data.get("timestamp") or data.get("observedTimestamp") or datetime.now().isoformat(),
                                "data": data
                            })
                            parsed = True
                except json.JSONDecodeError:
                    pass

    def _parse_metrics(self, content: str) -> None:
        """Parse metrics from console output."""
        # Metrics can appear in multiple formats:
        # 1. Simple format: "claude_code.session.count value 1"
        # 2. OTEL format: Multi-line with "-> Name: claude_code.session.count" then "Value: 1"

        metric_patterns = {
            "session_count": r'claude_code\.session\.count',
            "token_usage": r'claude_code\.token\.usage',
            "cost_usage": r'claude_code\.cost\.usage',
            "lines_of_code": r'claude_code\.lines_of_code\.count',
            "commits": r'claude_code\.commit\.count',
            "pull_requests": r'claude_code\.pull_request\.count',
            "active_time": r'claude_code\.active_time\.total'
        }

        lines = content.split('\n')
        i = 0
        while i < len(lines):
            line = lines[i]

            # Strategy 1: Simple format on single line
            for metric_name, pattern in metric_patterns.items():
                if re.search(pattern, line, re.I):
                    # Try to extract value from same line
                    value_match = re.search(r'value["\s:=]+([0-9.]+)', line, re.I)
                    if value_match:
                        self.metrics.append({
                            "metric": metric_name,
                            "pattern": pattern,
                            "value": float(value_match.group(1)),
                            "raw_line": line.strip()
                        })
                        break

                    # Strategy 2: OTEL multi-line format
                    # Name on current line, Value a few lines below
                    if '-> Name:' in line or 'Name:' in line:
                        # Look ahead for Value line (typically 5-10 lines)
                        for j in range(i+1, min(i+15, len(lines))):
                            value_line = lines[j]
                            if 'Value:' in value_line:
                                value_match = re.search(r'Value:\s*([0-9.]+)', value_line, re.I)
                                if value_match:
                                    self.metrics.append({
                                        "metric": metric_name,
                                        "pattern": pattern,
                                        "value": float(value_match.group(1)),
                                        "raw_line": line.strip()
                                    })
                                    break
                        break

            i += 1


def parse_log_file(log_file: Path, output_dir: Path) -> Path:
    """Parse a single log file and save as JSON."""
    parser = TelemetryParser(log_file)
    data = parser.parse()

    # Generate output filename
    output_file = output_dir / f"{log_file.stem}.json"

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)

    print(f"✅ Parsed {log_file.name}")
    print(f"   Events: {data['events_count']}, Metrics: {data['metrics_count']}")
    print(f"   Output: {output_file}")

    return output_file


def main():
    """Parse all log files in the telemetry logs directory."""
    telemetry_dir = Path.home() / ".claude-code-telemetry"
    logs_dir = telemetry_dir / "logs"
    data_dir = telemetry_dir / "data"

    if not logs_dir.exists():
        print(f"❌ Logs directory not found: {logs_dir}")
        print(f"   Run telemetry/enable-telemetry.sh first")
        sys.exit(1)

    # Create data directory
    data_dir.mkdir(parents=True, exist_ok=True)

    # Find all log files
    log_files = list(logs_dir.glob("session-*.log"))

    if not log_files:
        print(f"⚠️  No log files found in {logs_dir}")
        print(f"   Run Claude Code with telemetry enabled first")
        sys.exit(0)

    print(f"📊 Found {len(log_files)} session log(s)")
    print("")

    parsed_files = []
    for log_file in sorted(log_files):
        output_file = parse_log_file(log_file, data_dir)
        parsed_files.append(output_file)
        print("")

    print(f"✅ Parsed {len(parsed_files)} log file(s)")
    print(f"📁 Structured data saved to: {data_dir}")
    print("")
    print("Next steps:")
    print("  python3 telemetry/analyze.py    # Run analysis on parsed data")


if __name__ == "__main__":
    main()
