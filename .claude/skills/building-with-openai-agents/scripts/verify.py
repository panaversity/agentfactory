#!/usr/bin/env python3
"""Verify OpenAI Agents SDK skill is properly configured."""

import subprocess
import sys

def main():
    """Check that openai-agents package can be imported."""
    try:
        result = subprocess.run(
            [sys.executable, "-c", "import agents; print(agents.__name__)"],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            print("✓ openai-agents SDK available")
            sys.exit(0)
        else:
            print("✗ openai-agents not installed")
            print("  Run: pip install openai-agents")
            sys.exit(1)
    except FileNotFoundError:
        print("✗ Python not found")
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print("✗ Import timed out")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
