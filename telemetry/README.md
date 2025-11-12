# Claude Code Telemetry - Simple File-Based Collection

**No Docker. No databases. Just simple, effective data collection and analysis.**

This is a lightweight implementation of Claude Code telemetry collection following Andrew Ng's recommendations for data-driven AI improvement:

1. **Break down data silos** - Centralize team usage data
2. **Error analysis** - Identify failure patterns systematically
3. **Evals-first development** - Measure improvements over time

## Quick Start

### 1. Enable Telemetry

```bash
# Run the setup script
bash telemetry/enable-telemetry.sh
```

This creates:
- `~/.claude-code-telemetry/` - Main telemetry directory
- `~/.claude-code-telemetry/logs/` - Raw session logs
- `~/.claude-code-telemetry/data/` - Parsed JSON data
- `~/.claude-code-telemetry/env.sh` - Environment configuration

### 2. Start Collecting Data

**Option A: Use the wrapper script (recommended)**

```bash
~/.claude-code-telemetry/claude-with-telemetry.sh
```

**Option B: Manual setup**

```bash
# Load telemetry environment
source ~/.claude-code-telemetry/env.sh

# Run Claude Code with log capture
claude 2>&1 | tee ~/.claude-code-telemetry/logs/session-$(date +%Y%m%d-%H%M%S).log
```

**Option C: Add to shell profile (permanent)**

Add this to `~/.zshrc` or `~/.bashrc`:

```bash
alias claude-telemetry='~/.claude-code-telemetry/claude-with-telemetry.sh'
```

Then reload: `source ~/.zshrc`

### 3. Parse Logs into Structured Data

```bash
python telemetry/parser.py
```

This reads raw logs from `~/.claude-code-telemetry/logs/` and creates structured JSON files in `~/.claude-code-telemetry/data/`.

### 4. Analyze Your Data

```bash
python telemetry/analyze.py
```

This generates:
- Summary statistics (sessions, events, metrics)
- Cost analysis (total spend, per-session costs)
- Error analysis (failures, tool issues, high-retry sessions)
- Recommendations for improvement

## What Gets Collected

### Events (Logs)
- **User prompts** - What you asked Claude Code to do
- **Tool results** - File edits, commands executed, success/failure
- **API requests** - Token counts, costs, response times
- **API errors** - Rate limits, network issues, failures
- **Tool decisions** - Permission accepts/rejects

### Metrics (Time Series)
- **Session count** - Number of Claude Code sessions
- **Token usage** - Input/output tokens consumed
- **Cost usage** - Estimated USD spend
- **Lines of code** - Code modifications
- **Commits** - Git commits created
- **Pull requests** - PRs created
- **Active time** - Total active session time

All data includes:
- Session ID (unique per Claude Code session)
- User account UUID
- Timestamps
- Resource attributes (team, project)

## Data Flow

```
┌─────────────────┐
│  Claude Code    │
│  (telemetry on) │
└────────┬────────┘
         │
         │ Console output (stdout/stderr)
         ▼
┌─────────────────┐
│   Session Log   │
│  (.log file)    │
└────────┬────────┘
         │
         │ parser.py
         ▼
┌─────────────────┐
│ Structured JSON │
│  (.json file)   │
└────────┬────────┘
         │
         │ analyze.py
         ▼
┌─────────────────┐
│ Analysis Report │
│  (console + JSON)│
└─────────────────┘
```

## Privacy & Security

- ✅ All data stays **local** - nothing sent to external services
- ✅ User prompts logging is **opt-in** (`OTEL_LOG_USER_PROMPTS=1`)
- ✅ File contents are **never logged** (Claude Code sanitizes automatically)
- ✅ API keys and credentials are **never logged**
- ✅ You control what's collected via environment variables

## Andrew Ng's Data Strategy

This implementation follows Andrew Ng's recommendations from:
- [Tear Down Data Silos](https://www.deeplearning.ai/the-batch/tear-down-data-silos/)
- [Improve Agentic Performance with Evals and Error Analysis](https://www.deeplearning.ai/the-batch/improve-agentic-performance-with-evals-and-error-analysis-part-2/)

### 1. Break Down Data Silos

**Problem**: Each team member's usage data stays isolated on their machine.

**Solution**: Centralized collection in `~/.claude-code-telemetry/`
- Share parsed JSON files with team coordinator
- Aggregate across team for collective insights
- Identify patterns invisible to individuals

### 2. Error Analysis Workflow

**Problem**: Hard to identify systematic failure patterns.

**Solution**: Structured error tracking
1. `analyze.py` flags high-retry sessions
2. Review those session logs for patterns
3. Document common issues (vague specs, missing context)
4. Update templates/guidelines based on findings
5. Measure improvement in subsequent sessions

### 3. Evals-First Development

**Problem**: Can't measure if changes actually improve outcomes.

**Solution**: Telemetry as ground truth
- Baseline: Measure failure rates before changes
- Change: Update spec templates, prompt guidelines
- Eval: Compare failure rates after changes
- Iterate: Continuous improvement loop

## File Structure

```
~/.claude-code-telemetry/
├── logs/                           # Raw session logs
│   ├── session-20250112-143022.log
│   ├── session-20250112-151834.log
│   └── ...
├── data/                           # Parsed JSON data
│   ├── session-20250112-143022.json
│   ├── session-20250112-151834.json
│   └── ...
├── env.sh                          # Environment configuration
├── claude-with-telemetry.sh        # Wrapper script
├── shell-integration.txt           # Shell profile snippet
└── latest-report.json              # Latest analysis report
```

## Troubleshooting

### No events appearing in logs?

1. Check environment variables are set:
   ```bash
   env | grep CLAUDE_CODE_ENABLE_TELEMETRY
   env | grep OTEL
   ```

2. Verify console exporter is active (should see telemetry output in session logs)

3. Try running Claude Code directly to see raw output:
   ```bash
   source ~/.claude-code-telemetry/env.sh
   claude
   ```

### Parser not finding events?

- Console exporter output format varies - parser uses regex patterns
- Check raw log files manually to see what format telemetry is using
- Update `parser.py` regex patterns if needed for your Claude Code version

### Analysis showing zero costs/tokens?

- Cost/token extraction depends on event data structure
- Check parsed JSON files to see what data is available
- May need to adjust extraction logic in `analyze.py` based on actual format

## Future Enhancements

Once basic collection works, consider:

1. **Team aggregation** - Collect JSON files from all team members into shared storage
2. **Advanced queries** - Build custom analysis scripts for specific questions
3. **Visualization** - Generate charts/graphs from JSON data
4. **Automated evals** - Score sessions against quality criteria
5. **Integration** - Link telemetry to chapter specs for content-specific analysis

## Requirements

- **Python 3.8+** (no external dependencies for basic scripts)
- **Claude Code** with OpenTelemetry support (built-in)
- **Bash** (for setup scripts)

## License

Part of the AI Native Software Development book project.

## Support

For issues or questions:
1. Check existing session logs to verify data collection
2. Review parsed JSON to see what data is available
3. Consult Claude Code monitoring docs: https://code.claude.com/docs/en/monitoring-usage
