#!/bin/bash

# Claude Code Telemetry Setup (Simple Console-Based)
# No Docker, no databases - just file-based logging

set -e

TELEMETRY_DIR="${HOME}/.claude-code-telemetry"
LOGS_DIR="${TELEMETRY_DIR}/logs"
DATA_DIR="${TELEMETRY_DIR}/data"

echo "🔧 Setting up Claude Code telemetry..."
echo ""

# Check prerequisites
echo "Checking prerequisites..."

# Check for Claude Code
if ! command -v claude &> /dev/null; then
    echo "❌ ERROR: Claude Code (claude command) not found"
    echo "   Install Claude Code first: https://code.claude.com"
    exit 1
fi
echo "  ✓ Claude Code installed"

# Check for Python 3
if ! command -v python3 &> /dev/null; then
    echo "❌ ERROR: Python 3 not found"
    echo "   Install Python 3.8+: https://python.org"
    exit 1
fi

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | grep -oP '\d+\.\d+' || echo "0.0")
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

if [ "$PYTHON_MAJOR" -lt 3 ] || { [ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 8 ]; }; then
    echo "❌ ERROR: Python 3.8+ required (found: $PYTHON_VERSION)"
    echo "   Update Python: https://python.org"
    exit 1
fi
echo "  ✓ Python $PYTHON_VERSION installed"

# Check if already setup
if [ -f "${TELEMETRY_DIR}/env.sh" ]; then
    echo ""
    echo "⚠️  Telemetry already setup at: ${TELEMETRY_DIR}"
    echo "   Overwrite existing configuration? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "   Setup cancelled."
        exit 0
    fi
fi

echo ""
echo "Creating directories..."

# Create directories
mkdir -p "${LOGS_DIR}"
mkdir -p "${DATA_DIR}"

# Create environment setup file
cat > "${TELEMETRY_DIR}/env.sh" <<'EOF'
#!/bin/bash
# Claude Code Telemetry Environment Variables
# Source this file before starting Claude Code: source ~/.claude-code-telemetry/env.sh

# Enable telemetry
export CLAUDE_CODE_ENABLE_TELEMETRY=1

# Use console exporter (logs to stdout/stderr)
export OTEL_METRICS_EXPORTER=console
export OTEL_LOGS_EXPORTER=console

# Export intervals (in milliseconds)
export OTEL_METRIC_EXPORT_INTERVAL=60000  # 1 minute
export OTEL_LOGS_EXPORT_INTERVAL=5000     # 5 seconds

# Include session and account in metrics
export OTEL_METRICS_INCLUDE_SESSION_ID=true
export OTEL_METRICS_INCLUDE_ACCOUNT_UUID=true

# Enable user prompt logging (optional - set to 0 for privacy)
export OTEL_LOG_USER_PROMPTS=1

# Resource attributes for filtering
export OTEL_RESOURCE_ATTRIBUTES="team=panaversity,project=ai-native-book"

echo "✅ Claude Code telemetry environment loaded"
echo "📁 Logs will be saved to: ${HOME}/.claude-code-telemetry/logs/"
echo ""
echo "To start collecting data, run Claude Code with output redirection:"
echo "  claude 2>&1 | tee ~/.claude-code-telemetry/logs/session-\$(date +%Y%m%d-%H%M%S).log"
EOF

chmod +x "${TELEMETRY_DIR}/env.sh"

# Create wrapper script for Claude Code with telemetry
cat > "${TELEMETRY_DIR}/claude-with-telemetry.sh" <<'EOF'
#!/bin/bash

# Wrapper script to run Claude Code with telemetry enabled

TELEMETRY_DIR="${HOME}/.claude-code-telemetry"
LOGS_DIR="${TELEMETRY_DIR}/logs"
SESSION_LOG="${LOGS_DIR}/session-$(date +%Y%m%d-%H%M%S).log"

# Load telemetry environment
source "${TELEMETRY_DIR}/env.sh"

echo "🚀 Starting Claude Code with telemetry enabled..."
echo "📝 Session log: ${SESSION_LOG}"
echo ""

# Run Claude Code and capture all output
claude 2>&1 | tee "${SESSION_LOG}"
EOF

chmod +x "${TELEMETRY_DIR}/claude-with-telemetry.sh"

# Create shell profile integration snippet
cat > "${TELEMETRY_DIR}/shell-integration.txt" <<'EOF'
# Add to your ~/.zshrc or ~/.bashrc for automatic telemetry

# Alias to run Claude Code with telemetry
alias claude-telemetry='~/.claude-code-telemetry/claude-with-telemetry.sh'

# Or auto-enable telemetry environment (manual redirection needed)
# source ~/.claude-code-telemetry/env.sh
EOF

echo ""
echo "✅ Telemetry setup complete!"
echo ""
echo "📋 Next steps:"
echo ""
echo "1. Run Claude Code with telemetry:"
echo "   ${TELEMETRY_DIR}/claude-with-telemetry.sh"
echo ""
echo "2. OR add to your shell profile (${HOME}/.zshrc or ${HOME}/.bashrc):"
echo "   cat ${TELEMETRY_DIR}/shell-integration.txt"
echo ""
echo "3. View collected logs:"
echo "   ls -lh ${LOGS_DIR}/"
echo ""
echo "4. Analyze data (after collecting some sessions):"
echo "   python3 telemetry/analyze.py"
echo ""
echo "📁 All telemetry data stored in: ${TELEMETRY_DIR}/"
echo ""
