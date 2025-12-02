# Translation Cache in CI/CD Deployment

## Problem

**Without cache persistence:**
- Each deployment starts with a fresh runner
- `.translation-cache/` directory doesn't exist
- **Every deployment re-translates ALL files**
- Cost: ~$0.12 per deployment (65 files)
- Build time: 5-10 minutes per deployment

## Solution

**GitHub Actions Cache:**
- Persist `.translation-cache/` between builds
- Cache key based on content hash of all markdown files
- Only changed files trigger cache invalidation
- **Result**: Only changed files re-translated

## Implementation

### GitHub Actions Workflow Update

Added cache restore/save steps in `.github/workflows/deploy.yml`:

```yaml
- name: Restore translation cache
  uses: actions/cache@v4
  with:
    path: robolearn-interface/.translation-cache
    key: translation-cache-${{ runner.os }}-${{ hashFiles('robolearn-interface/docs/**/*.md') }}
    restore-keys: |
      translation-cache-${{ runner.os }}-

- name: Build site
  # ... build step ...

- name: Save translation cache
  uses: actions/cache@v4
  with:
    path: robolearn-interface/.translation-cache
    key: translation-cache-${{ runner.os }}-${{ hashFiles('robolearn-interface/docs/**/*.md') }}
```

### How It Works

1. **Cache Key**: Based on hash of all markdown files
   - If no files changed → cache hit → no translations needed
   - If files changed → cache miss → only changed files translated

2. **Restore Keys**: Falls back to previous cache if exact match not found
   - Ensures partial cache hits (some files already translated)

3. **Cache Persistence**: GitHub Actions caches up to 10GB
   - Cache persists for 7 days if not accessed
   - Automatically cleaned up if size limit exceeded

## Cost Impact

### Before (No Cache Persistence)

| Scenario | Files Translated | Cost |
|----------|------------------|------|
| **Every Deployment** | 65 (all) | **$0.12** |
| **10 deployments/month** | 650 | **$1.20** |
| **Annual** | 7,800 | **$14.40** |

### After (With Cache Persistence)

| Scenario | Files Translated | Cost |
|----------|------------------|------|
| **First Deployment** | 65 (all) | **$0.12** |
| **No Changes** | 0 | **$0.00** |
| **10% Changed** | 6-7 | **$0.019** |
| **10 deployments/month (avg 10% changes)** | ~65 | **$0.19** |
| **Annual** | ~780 | **$2.28** |

**Savings**: ~84% reduction in translation costs

## Setup Requirements

### 1. Add GEMINI_API_KEY Secret

In GitHub repository settings:
1. Go to **Settings → Secrets and variables → Actions**
2. Click **New repository secret**
3. Name: `GEMINI_API_KEY`
4. Value: Your Gemini API key
5. Click **Add secret**

### 2. Verify Workflow

The workflow already includes:
- ✅ Cache restore step (before build)
- ✅ Cache save step (after build)
- ✅ GEMINI_API_KEY environment variable

### 3. Test Deployment

```bash
# Push to main branch
git push origin main

# Check GitHub Actions logs:
# 1. Should see "Cache restored from key: translation-cache-..."
# 2. Build should use cached translations
# 3. Only changed files should be translated
```

## Cache Behavior

### Cache Hit (No Changes)

```
Restore cache → Cache found → Build uses cached translations → Save cache
```

**Result**: 
- 0 API calls
- Build time: 2-3 minutes (vs 5-10 minutes)
- Cost: $0.00

### Cache Miss (Files Changed)

```
Restore cache → Cache not found → Translate changed files → Save cache
```

**Result**:
- Only changed files translated
- Build time: 3-5 minutes
- Cost: ~$0.019 per 10% change

### First Deployment

```
No cache exists → Translate all files → Save cache
```

**Result**:
- All 65 files translated
- Build time: 5-10 minutes
- Cost: $0.12

## Monitoring Cache Effectiveness

### Check GitHub Actions Logs

Look for these log messages:

```
Cache restored from key: translation-cache-ubuntu-latest-<hash>
```

Or:

```
Cache not found for input keys: translation-cache-ubuntu-latest-<hash>
```

### Check Translation Plugin Logs

During build, look for:

```
[Auto-Translate] Cache hit: docs/module-1/chapter-4/lesson.md
[Auto-Translate] Translating: docs/module-1/chapter-5/new-lesson.md
[Auto-Translate] Complete: 1 translated, 64 cached, 0 errors
```

## Troubleshooting

### Cache Not Restoring

**Symptoms**: All files translated every deployment

**Check**:
1. Cache key matches file hash: `hashFiles('robolearn-interface/docs/**/*.md')`
2. Cache path correct: `robolearn-interface/.translation-cache`
3. Cache size within limits (<10GB)

**Fix**: Verify cache key in workflow matches actual file paths

### Cache Too Large

**Symptoms**: Cache save fails

**Check**: 
- Cache size: `du -sh robolearn-interface/.translation-cache`
- Should be <100MB for 65 files

**Fix**: 
- Clean old cache entries
- Reduce cache retention (if needed)

### API Key Missing

**Symptoms**: Build fails with "GEMINI_API_KEY is required"

**Fix**: Add `GEMINI_API_KEY` secret in repository settings

## Best Practices

1. **Monitor Cache Hit Rate**: Should be >80% after first deployment
2. **Review Costs**: Check API usage in Google AI Studio dashboard
3. **Cache Cleanup**: GitHub Actions automatically manages cache
4. **Fallback Strategy**: Plugin handles cache misses gracefully

## References

- GitHub Actions Cache: https://docs.github.com/en/actions/using-workflows/caching-dependencies-to-speed-up-workflows
- Translation Plugin: `robolearn-interface/plugins/docusaurus-plugin-auto-translate/`
- Cost Estimation: `specs/006-i18n-auto-translate-gemini/COST_ESTIMATION.md`

---

**Status**: ✅ Cache persistence implemented in GitHub Actions workflow

**Next Steps**:
1. Add `GEMINI_API_KEY` secret to repository
2. Test deployment to verify cache works
3. Monitor cache hit rate and costs

