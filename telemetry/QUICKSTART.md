# Quick Start - 5 Minutes to Telemetry

Get Claude Code telemetry up and running in 5 minutes. No Docker, no databases.

## Step 1: Setup (30 seconds)

```bash
cd telemetry
bash enable-telemetry.sh
```

**What it does**: Creates `~/.claude-code-telemetry/` with scripts and directories.

## Step 2: Start Collecting (10 seconds)

```bash
~/.claude-code-telemetry/claude-with-telemetry.sh
```

**What it does**: Runs Claude Code with telemetry enabled, logs everything to a file.

## Step 3: Use Claude Code Normally (2-3 minutes)

Just work with Claude Code as usual:
- Ask questions
- Edit files
- Run commands
- Create commits

The telemetry wrapper captures everything automatically.

## Step 4: Parse Your Data (10 seconds)

Exit Claude Code (Ctrl+C), then:

```bash
python parser.py
```

**What it does**: Converts raw logs into structured JSON files.

## Step 5: Analyze Results (10 seconds)

```bash
python analyze.py
```

**What it does**: Generates a report showing:
- Total sessions, events, metrics
- Costs and token usage
- Errors and failures
- Recommendations

## Example Output

```
📊 CLAUDE CODE TELEMETRY ANALYSIS REPORT
================================================================================

📈 SUMMARY
--------------------------------------------------------------------------------
  Total Sessions: 3
  Total Events: 127
  Total Metrics: 45
  Total Api Requests: 38
  Estimated Total Cost Usd: 0.0234
  Estimated Total Tokens: 12,450

💰 COST ANALYSIS
--------------------------------------------------------------------------------
  Total Cost: $0.0234 USD
  Total Tokens: 12,450
  Sessions Analyzed: 3

🔍 ERROR ANALYSIS
--------------------------------------------------------------------------------
  Total Errors: 0
  Total Tool Failures: 1
  High-Retry Sessions: 0

  ✅ No problematic sessions detected!

💡 RECOMMENDATIONS
--------------------------------------------------------------------------------
  1. ✅ No errors detected - system operating smoothly
  2. ✅ Costs within reasonable range
  3. Collect more data - only 3 session(s) analyzed
     - Continue using Claude Code with telemetry enabled
```

## That's It!

You now have:
- ✅ Telemetry enabled
- ✅ Session data collected
- ✅ Structured data parsed
- ✅ Analysis report generated

## Next Steps

### Make It Permanent

Add to your `~/.zshrc` or `~/.bashrc`:

```bash
alias claude='~/.claude-code-telemetry/claude-with-telemetry.sh'
```

Now `claude` always runs with telemetry.

### Share With Your Team

Each team member:
1. Runs `bash telemetry/enable-telemetry.sh`
2. Uses `claude-with-telemetry.sh` wrapper
3. Shares their `~/.claude-code-telemetry/data/*.json` files with coordinator

Coordinator runs `analyze.py` on aggregated data.

### Continuous Improvement (Andrew Ng's Method)

1. **Baseline**: Run analysis on current data
2. **Identify**: Find patterns in high-cost or high-error sessions
3. **Improve**: Update specs, prompts, or templates
4. **Measure**: Compare new sessions to baseline
5. **Iterate**: Repeat

## Troubleshooting

**Not seeing telemetry output?**
```bash
env | grep CLAUDE_CODE_ENABLE_TELEMETRY
```
Should show: `CLAUDE_CODE_ENABLE_TELEMETRY=1`

**Parser not finding logs?**
```bash
ls ~/.claude-code-telemetry/logs/
```
Should show `session-*.log` files.

**Analysis showing no data?**
```bash
ls ~/.claude-code-telemetry/data/
```
Should show `session-*.json` files. Run `python parser.py` first.

## Privacy

- All data stays on your local machine
- No external services involved
- User prompts only logged if you enable `OTEL_LOG_USER_PROMPTS=1`
- File contents never logged

## Learn More

- [Full README](./README.md) - Complete documentation
- [Claude Code Monitoring Docs](https://code.claude.com/docs/en/monitoring-usage) - Official docs
- [Andrew Ng's Data Strategy](https://www.deeplearning.ai/the-batch/tear-down-data-silos/) - Why this matters
