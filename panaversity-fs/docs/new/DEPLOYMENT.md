# Deployment Checklist: Incremental Builds

## What We Built

✅ **Incremental sync** - Only upload changed files to PanaversityFS
✅ **Incremental hydration** - Only download delta before build
✅ **Fallback safety** - Builds never break if MCP fails
✅ **Clean separation** - `docs/` for authors, `build-source/` for builds

## Files Changed

| File | Change |
|------|--------|
| `.github/workflows/deploy.yml` | Added hydration step with fallback |
| `.github/workflows/sync-content.yml` | Already uses ingest-book.py ✓ |
| `book-source/docusaurus.config.ts` | Updated to use ../build-source |
| `.gitignore` | Added build-source/ and .panaversity/ |
| `panaversity-fs/scripts/*` | Created in previous session ✓ |
| `ARCHITECTURE.md` | Documentation ✓ |

## Pre-Deployment Checklist

### 1. Verify Branch

```bash
git branch
# Should show: incremental-build-integration

git status
# Should show modified files
```

### 2. Run Tests

```bash
cd panaversity-fs
pytest tests/scripts/ -v

# Expected:
# test_path_mapper.py: 33 passed ✓
# test_source_scanner.py: 21 passed ✓
```

### 3. Commit Changes

```bash
git add .
git commit -m "feat: implement incremental builds with PanaversityFS

- Hydrate to build-source/ instead of docsfs/
- Add fallback to local docs/ if hydration fails
- Separate author workspace (docs/) from build artifacts (build-source/)
- Update workflows for incremental sync and hydration
- Add comprehensive architecture documentation

Solves: 502 timeout errors with 1000+ lessons
Build time: 10 min → ~2 min
"

git push origin incremental-build-integration
```

## Deployment Steps

### Step 1: Deploy PanaversityFS MCP Server

```bash
# In panaversity-fs/ directory
gcloud run deploy panaversity-fs \
  --source . \
  --region us-central1 \
  --allow-unauthenticated \
  --set-env-vars="ENVIRONMENT=production,R2_BUCKET=panaversity-prod"

# Note the deployed URL: https://panaversity-fs-XXXXX.run.app
```

### Step 2: Configure GitHub Secrets

```bash
# Set production secrets
gh secret set PANAVERSITY_SERVER_URL --body "https://panaversity-fs-XXXXX.run.app"
gh secret set PANAVERSITY_API_KEY --body "your-api-key-here"

# Enable the plugin
gh variable set PANAVERSITY_PLUGIN_ENABLED --body "true"
```

### Step 3: Initial Content Sync

```bash
# Trigger full sync manually to populate PanaversityFS
gh workflow run sync-content.yml -f full_sync=true

# Monitor:
gh run list --workflow=sync-content.yml
gh run view --log  # Check logs
```

### Step 4: Test Build

```bash
# Trigger deploy workflow
gh workflow run deploy.yml

# Monitor:
gh run list --workflow=deploy.yml
gh run view --log

# Check logs for:
# ✓ "Hydrating content from PanaversityFS"
# ✓ "Downloaded X files (delta) in Y seconds"
# ✓ "Build completed"
```

### Step 5: Verify Deployment

```bash
# Open deployed site
open https://your-github-username.github.io/tutorsgpt

# Verify:
# 1. Site loads
# 2. Content is present
# 3. No 502 errors
# 4. Build time in GitHub Actions logs
```

## Post-Deployment Validation

### Test Incremental Sync

```bash
# 1. Edit a lesson in book-source/docs/
echo "\nTest update" >> book-source/docs/Part-01/Chapter-01/01-introduction.md

# 2. Commit and push
git add .
git commit -m "test: verify incremental sync"
git push

# 3. Watch workflows
gh run list --workflow=sync-content.yml --limit 1
# Should show: "Uploaded 1 file" (not all files)

gh run list --workflow=deploy.yml --limit 1
# Should show: "Downloaded 1 file (delta)"
```

### Test Fallback

```bash
# 1. Temporarily disable MCP server (or set invalid URL)
gh secret set PANAVERSITY_SERVER_URL --body "https://invalid-url.com"

# 2. Trigger build
gh workflow run deploy.yml

# 3. Check logs
gh run view --log
# Should show: "⚠️ Hydration failed, using local docs/ as fallback"
# Build should still succeed

# 4. Restore correct URL
gh secret set PANAVERSITY_SERVER_URL --body "https://panaversity-fs-XXXXX.run.app"
```

## Rollback Plan (If Needed)

### Option 1: Disable PanaversityFS

```bash
# Revert to local docs/ immediately
gh variable set PANAVERSITY_PLUGIN_ENABLED --body "false"

# Next build will use book-source/docs/ directly
# No code changes needed
```

### Option 2: Revert Branch

```bash
# If deploy workflow has issues
git revert HEAD
git push origin incremental-build-integration

# Workflow will use old configuration
```

## Success Metrics

After first successful build, verify:

```
Metric                  Target    Actual
─────────────────────── ───────── ──────
Build Time              < 3 min   _____
Sync Time (changed)     < 30 sec  _____
Hydration Time (delta)  < 30 sec  _____
Cache Hit Rate          > 70%     _____
Fallback Activations    0         _____
```

## Monitoring Setup (Optional)

### GitHub Actions Insights

```bash
# View workflow metrics
open "https://github.com/YOUR-ORG/tutorsgpt/actions/workflows/deploy.yml"

# Check:
# - Build duration trend
# - Success rate
# - Cache hit rate (in logs)
```

### Slack Alerts (Optional)

Add to `.github/workflows/deploy.yml`:

```yaml
- name: Notify on failure
  if: failure()
  uses: slackapi/slack-github-action@v1
  with:
    payload: |
      {
        "text": "🚨 Deploy failed: ${{ github.event.head_commit.message }}"
      }
  env:
    SLACK_WEBHOOK_URL: ${{ secrets.SLACK_WEBHOOK_URL }}
```

## Next Steps (Future)

After 1-2 weeks of stable operation:

- [ ] Phase 2: Enable daily consistency audits
- [ ] Phase 3: Create separate content repo (if needed)
- [ ] Phase 4: Decouple book-interface (if multi-book)

## Troubleshooting

### Issue: "Manifest cache not found"

**Cause:** First build or cache expired
**Impact:** Full download (slower, but works)
**Fix:** None needed, subsequent builds will be incremental

### Issue: "Hydration timeout"

**Cause:** Network issues or server slow
**Impact:** Fallback to local docs/ (builds still work)
**Fix:** Check MCP server health, may need to scale up

### Issue: "Content mismatch between git and MCP"

**Cause:** Sync failed or manual upload
**Fix:**
```bash
# Force full re-sync
gh workflow run sync-content.yml -f full_sync=true
```

## Questions?

- Deployment blocked? Check GitHub Actions logs
- MCP server issues? Check Cloud Run logs: `gcloud run logs read panaversity-fs`
- Sync not working? Verify `PANAVERSITY_PLUGIN_ENABLED=true`
- Need help? Ping #panaversity-ops

---

**Ready to deploy?** ✓ Complete checklist above and push to main.
