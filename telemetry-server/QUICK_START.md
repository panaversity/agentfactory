# 🚀 Quick Start - 1 Command Setup

## Automated Setup (Recommended)

Just run this slash command in Claude Code:

```
/sp.telemetry-setup
```

That's it! The command will:
- ✅ Check Docker is installed
- ✅ Verify system resources
- ✅ Generate secure passwords
- ✅ Create configuration
- ✅ Start services
- ✅ Verify everything works
- ✅ Enable telemetry in Claude Code

**Time**: 10-15 minutes (mostly automated)

---

## Manual Setup (If Needed)

If you prefer manual setup, follow these 5 steps:

### 1. Check Docker
```bash
docker --version
docker ps
```

### 2. Navigate & Verify
```bash
cd telemetry-server
./scripts/verify-setup.sh
```

### 3. Configure
```bash
cp .env.template .env
nano .env  # Change passwords on lines 7 and 11
```

### 4. Start Services
```bash
docker-compose up -d
sleep 30  # Wait for initialization
./scripts/test-connection.sh
```

### 5. Enable Telemetry
```bash
# Add to ~/.zshrc or ~/.bashrc
export CLAUDE_ENV_CONFIG="$HOME/path/to/.claude/env-config/telemetry-enabled.env"
source ~/.zshrc
```

---

## Verify It's Working

```bash
# Check services are running
docker-compose ps

# Count events (use after using Claude Code for a bit)
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user \
  --password YOUR_PASSWORD \
  --query "SELECT count() FROM telemetry.telemetry_events"
```

---

## Daily Usage

```bash
# Start (if stopped)
cd telemetry-server && docker-compose start

# Stop (to save resources)
cd telemetry-server && docker-compose stop

# View logs
cd telemetry-server && docker-compose logs -f

# Run analysis
cd telemetry-server
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password YOUR_PASSWORD \
  --database telemetry --multiquery < queries/session-analysis.sql
```

---

## Documentation

- **SETUP.md** - Detailed manual setup guide
- **README.md** - Full documentation
- **docs/troubleshooting.md** - Common issues
- **queries/README.md** - Query examples

---

**Recommended: Use `/sp.telemetry-setup` for automated guided setup! 🎯**
