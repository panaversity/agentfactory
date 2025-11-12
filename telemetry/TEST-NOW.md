# Test Telemetry System Right Now

Since you're already in a Claude Code session, here's how to test it immediately:

---

## Option 1: Quick Test in New Terminal (Recommended)

**Why**: Start a fresh Claude Code session with telemetry enabled from the beginning.

### Steps:

1. **Open a new terminal** (keep this session running)

2. **Run the setup**:
   ```bash
   cd /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ch5-6
   bash telemetry/enable-telemetry.sh
   ```

3. **Start Claude Code with telemetry**:
   ```bash
   ~/.claude-code-telemetry/claude-with-telemetry.sh
   ```

4. **Do some normal work** (2-3 minutes):
   - Ask a question: "What files are in this directory?"
   - Edit a file: "Add a comment to README.md"
   - Run a command: "List Python files"

5. **Exit Claude Code** (Ctrl+C or type "exit")

6. **Parse your session**:
   ```bash
   python3 telemetry/parser.py
   ```

7. **Analyze results**:
   ```bash
   python3 telemetry/analyze.py
   ```

8. **Review the data**:
   ```bash
   cat ~/.claude-code-telemetry/latest-report.json
   ```

**Expected Results**:
- ✅ Events count > 0 (should see user_prompt, api_request, tool_result)
- ✅ Cost > $0.00 (should show actual API costs)
- ✅ Tokens > 0 (should show token usage)
- ✅ Sessions: 1

---

## Option 2: Test with Existing Setup (Already Done)

**Why**: We already created the setup, just need to use it.

### Steps:

1. **The setup is already complete**:
   ```bash
   ls -la ~/.claude-code-telemetry/
   ```
   Should show: `env.sh`, `claude-with-telemetry.sh`, `logs/`, `data/`

2. **In a new terminal, start collecting**:
   ```bash
   ~/.claude-code-telemetry/claude-with-telemetry.sh
   ```

3. **Work normally, then analyze**:
   ```bash
   python3 telemetry/parser.py
   python3 telemetry/analyze.py
   ```

---

## Option 3: Test with Sample Data (Instant)

**Why**: See the system work immediately without waiting for real data.

### Steps:

1. **The test data is already there**:
   ```bash
   ls ~/.claude-code-telemetry/logs/
   ```
   You should see: `session-test.log` and `session-real-format.log`

2. **Parse it**:
   ```bash
   python3 telemetry/parser.py
   ```

3. **Analyze it**:
   ```bash
   python3 telemetry/analyze.py
   ```

4. **Check the results**:
   ```bash
   cat ~/.claude-code-telemetry/latest-report.json
   ```

**What You'll See**:
```
📊 SUMMARY
  Total Sessions: 2
  Total Events: 9
  Total Metrics: 8
  Total Api Requests: 4
  Estimated Total Cost Usd: 0.0321
  Estimated Total Tokens: 1,704

💰 COST ANALYSIS
  Total Cost: $0.0321 USD
  Total Tokens: 1,704
  Sessions Analyzed: 2

  Top Sessions by Cost:
    1. session-real-format.log
       Cost: $0.0248, Tokens: 1,234, API Calls: 2
    2. session-test.log
       Cost: $0.0073, Tokens: 470, API Calls: 2
```

---

## Option 4: Test Current Session (Advanced)

**Problem**: This session didn't start with telemetry enabled, so it won't be capturing to a file.

**Workaround**: We can still test the parser/analyzer on any log file.

### Create a Test Log Manually:

```bash
# Create a fake session log
cat > ~/.claude-code-telemetry/logs/session-manual-test.log << 'EOF'
Session started at 2025-01-12 22:00:00

LogRecord #0
Body: Str({"event":"claude_code.user_prompt","prompt_length":25,"session_id":"test-123"})

LogRecord #1
Body: Str({"event":"claude_code.api_request","tokens":350,"cost":0.0035,"model":"claude-sonnet-4.5"})

LogRecord #2
Body: Str({"event":"claude_code.tool_result","tool":"Read","file":"README.md","success":true})
EOF

# Parse it
python3 telemetry/parser.py

# Analyze it
python3 telemetry/analyze.py
```

---

## What to Verify

After running any of the above, check:

### 1. Parser Output ✅
```bash
cat ~/.claude-code-telemetry/data/session-*.json | python3 -m json.tool | head -30
```

**Should show**:
- session metadata (session_id, team, project)
- events array with parsed events
- metrics array with parsed metrics
- events_count > 0
- metrics_count >= 0

### 2. Analyzer Output ✅
```bash
python3 telemetry/analyze.py | grep -A 10 "SUMMARY"
```

**Should show**:
- Total sessions
- Total events (should be > 0)
- Total API requests
- Cost in USD (should be > 0 if API calls present)
- Tokens (should be > 0 if API calls present)

### 3. Report File ✅
```bash
cat ~/.claude-code-telemetry/latest-report.json | python3 -m json.tool | grep -A 5 "summary"
```

**Should contain**:
- JSON with summary, costs, errors sections
- All numbers matching console output

---

## Troubleshooting

### Parser finds 0 events?

**Check log format**:
```bash
head -20 ~/.claude-code-telemetry/logs/session-*.log
```

**Should see**:
- Either simple format: `claude_code.user_prompt: {"data": ...}`
- Or OTEL format: `Body: Str({"event":"claude_code.api_request",...})`

**If you see neither**:
- Log might be from Claude Code without telemetry enabled
- Try creating a new session with telemetry enabled

### Analyzer shows $0.00 cost?

**Check if events were parsed**:
```bash
cat ~/.claude-code-telemetry/data/session-*.json | grep -c "api_request"
```

**Should be > 0**. If 0:
- No API requests in that session
- Or parser didn't find them (check log format)

### No log files found?

**Check directory exists**:
```bash
ls -la ~/.claude-code-telemetry/logs/
```

**If empty**:
- Run setup: `bash telemetry/enable-telemetry.sh`
- Start session: `~/.claude-code-telemetry/claude-with-telemetry.sh`
- Or use test data (Option 3 above)

---

## Quick Verification Script

Run this to test everything at once:

```bash
#!/bin/bash
echo "🧪 Testing Telemetry System..."
echo ""

# Check setup
echo "1. Checking setup..."
if [ -f ~/.claude-code-telemetry/env.sh ]; then
    echo "   ✅ Setup complete"
else
    echo "   ❌ Setup missing - run: bash telemetry/enable-telemetry.sh"
    exit 1
fi

# Check logs
echo "2. Checking logs..."
LOG_COUNT=$(ls ~/.claude-code-telemetry/logs/*.log 2>/dev/null | wc -l)
echo "   Found $LOG_COUNT log file(s)"

if [ $LOG_COUNT -eq 0 ]; then
    echo "   ⚠️  No logs yet - start a session or use test data"
fi

# Run parser
echo "3. Running parser..."
python3 telemetry/parser.py > /tmp/parser-output.txt 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ Parser succeeded"
    grep "Parsed" /tmp/parser-output.txt | tail -5
else
    echo "   ❌ Parser failed"
    cat /tmp/parser-output.txt
    exit 1
fi

# Check parsed data
echo "4. Checking parsed data..."
JSON_COUNT=$(ls ~/.claude-code-telemetry/data/*.json 2>/dev/null | wc -l)
echo "   Found $JSON_COUNT parsed session(s)"

# Run analyzer
echo "5. Running analyzer..."
python3 telemetry/analyze.py > /tmp/analyzer-output.txt 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ Analyzer succeeded"
    grep -A 5 "SUMMARY" /tmp/analyzer-output.txt
else
    echo "   ❌ Analyzer failed"
    cat /tmp/analyzer-output.txt
    exit 1
fi

echo ""
echo "✅ All tests passed!"
echo ""
echo "📊 View full report:"
echo "   python3 telemetry/analyze.py"
echo ""
echo "📁 View parsed data:"
echo "   ls -lh ~/.claude-code-telemetry/data/"
```

Save as `test-telemetry.sh` and run:
```bash
chmod +x test-telemetry.sh
./test-telemetry.sh
```

---

## Recommended Test Flow

**For first-time validation**:

1. ✅ Use Option 3 (test with sample data) - **instant verification**
2. ✅ Use Option 1 (new session) - **real-world validation**
3. ✅ Review outputs to confirm accuracy
4. ✅ Share with team if all looks good

**Time required**: 5 minutes total

---

## Success Criteria

After testing, you should see:

- ✅ Parser completes without errors
- ✅ Events count > 0
- ✅ Cost/tokens extracted correctly
- ✅ Analyzer produces readable report
- ✅ JSON files created in data directory
- ✅ Report matches what you expect

If all ✅, the system is working correctly!
