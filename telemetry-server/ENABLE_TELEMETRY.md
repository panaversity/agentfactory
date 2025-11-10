# Enable Claude Code Telemetry

## Quick Setup

### Option 1: Environment Variables (Recommended for Testing)

Run these commands in your terminal before starting Claude Code:

```bash
# Enable telemetry
export CLAUDE_CODE_ENABLE_TELEMETRY=1

# Configure OTLP exporters
export OTEL_METRICS_EXPORTER=otlp
export OTEL_LOGS_EXPORTER=otlp
export OTEL_EXPORTER_OTLP_PROTOCOL=grpc
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317

# Shorter intervals for testing (10s metrics, 5s logs)
export OTEL_METRIC_EXPORT_INTERVAL=10000
export OTEL_LOGS_EXPORT_INTERVAL=5000

# Enable user prompt logging (optional)
export OTEL_LOG_USER_PROMPTS=1

# Start Claude Code
claude
```

### Option 2: Shell Profile (Permanent)

Add to your `~/.zshrc` or `~/.bashrc`:

```bash
# Claude Code Telemetry
export CLAUDE_CODE_ENABLE_TELEMETRY=1
export OTEL_METRICS_EXPORTER=otlp
export OTEL_LOGS_EXPORTER=otlp
export OTEL_EXPORTER_OTLP_PROTOCOL=grpc
export OTEL_EXPORTER_OTLP_ENDPOINT=http://localhost:4317
export OTEL_METRIC_EXPORT_INTERVAL=10000
export OTEL_LOGS_EXPORT_INTERVAL=5000
export OTEL_LOG_USER_PROMPTS=1
```

Then reload: `source ~/.zshrc` (or `source ~/.bashrc`)

### Option 3: Managed Settings (Organization-wide)

For administrators, create `/Library/Application Support/ClaudeCode/managed-settings.json`:

```json
{
  "env": {
    "CLAUDE_CODE_ENABLE_TELEMETRY": "1",
    "OTEL_METRICS_EXPORTER": "otlp",
    "OTEL_LOGS_EXPORTER": "otlp",
    "OTEL_EXPORTER_OTLP_PROTOCOL": "grpc",
    "OTEL_EXPORTER_OTLP_ENDPOINT": "http://localhost:4317",
    "OTEL_METRIC_EXPORT_INTERVAL": "10000",
    "OTEL_LOGS_EXPORT_INTERVAL": "5000",
    "OTEL_LOG_USER_PROMPTS": "1"
  }
}
```

## Verify It's Working

### 1. Check Telemetry Services Are Running

```bash
cd telemetry-server
docker-compose ps
```

Both ClickHouse and OTLP Collector should be "Up".

### 2. Start Claude Code with Telemetry

```bash
# Set environment variables (Option 1 above)
claude
```

### 3. Interact with Claude Code

Type a simple prompt like "hi" or "what's the weather?"

### 4. Check for Events (wait 10-15 seconds)

```bash
cd telemetry-server
docker exec telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password 'V/E8RdwmsZz1RCXPGu2D7bruCw5nx7YH' \
  --query "SELECT count() FROM telemetry.telemetry_events"
```

You should see a count > 0!

### 5. View Recent Events

```bash
docker exec telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password 'V/E8RdwmsZz1RCXPGu2D7bruCw5nx7YH' \
  --query "SELECT event_type, timestamp FROM telemetry.telemetry_events ORDER BY timestamp DESC LIMIT 10"
```

## What Gets Collected

According to the [Claude Code documentation](https://docs.claude.com/en/docs/claude-code/monitoring):

**Metrics**:
- `claude_code.session.count` - Session starts
- `claude_code.token.usage` - Token consumption
- `claude_code.cost.usage` - Estimated costs
- `claude_code.lines_of_code.count` - Code modifications
- `claude_code.commit.count` - Git commits
- `claude_code.pull_request.count` - PRs created
- `claude_code.code_edit_tool.decision` - Edit decisions
- `claude_code.active_time.total` - Active time tracking

**Events** (via logs):
- `claude_code.user_prompt` - User prompts
- `claude_code.tool_result` - Tool execution results
- `claude_code.api_request` - API calls to Claude
- `claude_code.api_error` - API errors
- `claude_code.tool_decision` - Tool permission decisions

## Troubleshooting

### No events appearing?

1. **Check environment variables are set**:
   ```bash
   env | grep OTEL
   env | grep CLAUDE_CODE_ENABLE_TELEMETRY
   ```

2. **Check OTLP Collector logs**:
   ```bash
   docker logs telemetry-otel-collector --tail 50
   ```

3. **Verify ClickHouse is accessible**:
   ```bash
   curl -s "http://localhost:8123/?user=telemetry_user&password=V/E8RdwmsZz1RCXPGu2D7bruCw5nx7YH&query=SELECT%201"
   ```

### Still not working?

- Restart Claude Code after setting environment variables
- Check Docker containers are healthy: `docker-compose ps`
- Try console exporter for debugging: `export OTEL_METRICS_EXPORTER=console,otlp`

## Privacy Note

- User prompts are logged only when `OTEL_LOG_USER_PROMPTS=1`
- Sensitive data (API keys, file contents) are never included
- All data stays local (not sent to external services)
- Telemetry is completely opt-in

## Next Steps

Once data is flowing, explore the analytics queries:

```bash
cd telemetry-server/queries
cat README.md  # See available queries

# Run error analysis
docker exec telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password 'V/E8RdwmsZz1RCXPGu2D7bruCw5nx7YH' \
  --database telemetry --multiquery < error-analysis.sql
```
