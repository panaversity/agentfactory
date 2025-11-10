# 🎯 Start Here - Telemetry Setup

## ⚡ One Command Setup (100% Automatic)

In Claude Code, just run:

```
/sp.telemetry-setup
```

**That's it!** The command runs everything automatically:
- ✅ Checks Docker is installed and running
- ✅ Generates secure passwords
- ✅ Configures .env file
- ✅ Starts ClickHouse and OTLP Collector
- ✅ Verifies all connections
- ✅ Enables Claude Code telemetry
- ✅ Creates quick reference with credentials

**You don't need to run ANY commands manually!**

**Time**: 10-15 minutes (fully automated)

---

## What You Get

After setup, you can:

### 📊 Analyze Errors (Andrew Ng's Methodology)
```bash
cd telemetry-server
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password YOUR_PASSWORD \
  --database telemetry --multiquery < queries/error-analysis.sql
```

### 📈 Track Productivity
```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password YOUR_PASSWORD \
  --database telemetry --multiquery < queries/session-analysis.sql
```

### 🔧 Monitor Tool Usage
```bash
docker exec -it telemetry-clickhouse clickhouse-client \
  --user telemetry_user --password YOUR_PASSWORD \
  --database telemetry --multiquery < queries/tool-usage.sql
```

---

## Daily Commands

```bash
# Start telemetry
cd telemetry-server && docker-compose start

# Stop telemetry (saves resources)
cd telemetry-server && docker-compose stop

# View live data
docker-compose logs -f
```

---

## Need Help?

- **Automated setup**: Run `/sp.telemetry-setup` in Claude Code
- **Manual setup**: See `QUICK_START.md`
- **Troubleshooting**: See `docs/troubleshooting.md`
- **Query examples**: See `queries/README.md`

---

## Privacy

✅ Your prompts are **SHA256 hashed** (not stored in plaintext)  
✅ PII automatically filtered  
✅ Data stays local (in `clickhouse_data/` - git-ignored)  
✅ 90-day automatic deletion

**No data leaves your machine.**

---

**Recommended: Use `/sp.telemetry-setup` for the easiest experience! 🚀**
