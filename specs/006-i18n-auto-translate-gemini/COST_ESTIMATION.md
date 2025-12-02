# Translation Cost Estimation

## Content Analysis

**Current Content:**
- **Total Files**: 65 markdown files
- **Total Words**: ~99,092 words
- **Total Characters**: ~742,184 characters (~742K chars)

## Cost Calculation

### Gemini API Pricing (as of 2025)

**Model**: `gemini-flash-lite-latest` (points to `gemini-2.5-flash-lite-preview-09-2025`)

**Pricing Structure:**
- **Input**: $0.10 per 1M tokens
- **Output**: $0.40 per 1M tokens
- **Free Tier**: 15 requests/minute (rate limit)
- **Knowledge Cutoff**: January 2025

**Token Estimation:**
- English: ~4 characters = 1 token
- Urdu: ~3 characters = 1 token (more compact script)
- Average: ~3.5 characters = 1 token

### First Build (All Files Translated)

**Input Tokens:**
- Source content: 742,184 characters ÷ 3.5 = ~212,052 tokens
- Prompt overhead: ~500 tokens per file × 65 = ~32,500 tokens
- **Total Input**: ~244,552 tokens

**Output Tokens:**
- Translated content: ~742,184 characters ÷ 3 = ~247,395 tokens
- **Total Output**: ~247,395 tokens

**Cost Calculation:**
```
Input Cost:  (244,552 / 1,000,000) × $0.10 = $0.024
Output Cost: (247,395 / 1,000,000) × $0.40 = $0.099
─────────────────────────────────────────────────────
Total First Build:                              $0.123
```

**Rounded Estimate: $0.12 - $0.15** (accounting for variations)

### Subsequent Builds (Cache Hits)

**Cache Hit Rate**: >80% (expected)

**Scenario 1: No Changes**
- Files changed: 0
- New translations: 0
- **Cost**: $0.00

**Scenario 2: 10% Files Changed**
- Files changed: 6-7 files
- New input tokens: ~37,000 tokens
- New output tokens: ~38,000 tokens
- **Cost**: ~$0.019

**Scenario 3: 20% Files Changed**
- Files changed: 13 files
- New input tokens: ~75,000 tokens
- New output tokens: ~77,000 tokens
- **Cost**: ~$0.038

## Cost Breakdown by Scenario

| Scenario | Files Translated | Input Tokens | Output Tokens | Cost |
|----------|------------------|--------------|---------------|------|
| **First Build** | 65 (all) | ~245K | ~247K | **$0.12** |
| **No Changes** | 0 | 0 | 0 | **$0.00** |
| **10% Changed** | 6-7 | ~37K | ~38K | **$0.019** |
| **20% Changed** | 13 | ~75K | ~77K | **$0.038** |
| **50% Changed** | 32-33 | ~122K | ~124K | **$0.075** |

## Monthly Cost Projection

**Assumptions:**
- Weekly builds: 4 builds/month
- Average 10% content changes per build
- Cache hit rate: 80%

**Monthly Cost:**
```
First build:     $0.12
Subsequent (3):  $0.019 × 3 = $0.057
─────────────────────────────
Monthly Total:   $0.177
```

**Annual Cost**: ~$2.12

## Free Tier Considerations

**Gemini API Free Tier:**
- **Rate Limit**: 15 requests/minute
- **Daily Limit**: Varies by model (check current limits)
- **Monthly Limit**: Varies (check current limits)

**For 65 files:**
- Sequential processing: ~65 requests
- At 15 req/min: ~4.3 minutes minimum
- **No cost if within free tier limits**

**Recommendation**: Check current free tier limits at https://ai.google.dev/pricing

## Cost Optimization Strategies

### 1. Caching (Already Implemented)
- ✅ SHA256 file hashing
- ✅ Cache hit rate >80%
- ✅ Only changed files re-translated

### 2. Batch Processing
- Process files in batches to respect rate limits
- Avoid API throttling

### 3. Model Selection
- `gemini-2.5-flash-lite`: Lower cost, good for translation
- `gemini-2.0-flash-exp`: Same cost, better quality

### 4. Chunking (For Large Files)
- Plugin already implements chunking for files >50K chars
- Prevents token limit errors

## Real-World Cost Examples

### Example 1: Small Site (10 files, ~50K chars)
- First build: **$0.01**
- Monthly: **$0.02**

### Example 2: Medium Site (65 files, ~742K chars) ← **Your Site**
- First build: **$0.12**
- Monthly: **$0.18**

### Example 3: Large Site (200 files, ~2M chars)
- First build: **$0.30**
- Monthly: **$0.50**

## Cost Comparison: Alternatives

| Method | First Build | Monthly | Notes |
|--------|-------------|---------|-------|
| **Gemini API** | $0.12 | $0.18 | ✅ Current choice |
| OpenAI GPT-4 | $2-5 | $5-10 | ❌ More expensive |
| Azure Translator | $10/1M chars | $0.74 | ⚠️ Less context-aware |
| Manual Translation | $500-2000 | $0 | ❌ One-time, no updates |

## Summary

**Your Site (65 files, ~742K chars):**

✅ **First Build**: ~$0.12  
✅ **Monthly**: ~$0.18  
✅ **Annual**: ~$2.12  

**Key Points:**
- Extremely cost-effective (<$0.20/month)
- Caching reduces costs by 80%+
- Free tier may cover all costs
- Much cheaper than manual translation
- Automatic updates as content changes

## Verification

To verify actual costs:
1. Run first build: `npm run build`
2. Check API usage in Google AI Studio dashboard
3. Monitor costs over first month
4. Adjust model if needed

## References

- Gemini API Pricing: https://ai.google.dev/pricing
- Current Free Tier: https://ai.google.dev/pricing
- Token Calculator: Use Gemini API docs for exact token counts

---

**Bottom Line**: Translation costs are **negligible** (<$0.20/month) and likely covered by free tier. The caching system ensures only changed content incurs costs.

