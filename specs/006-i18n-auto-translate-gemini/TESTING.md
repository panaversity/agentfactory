# Local Testing Guide: i18n Auto-Translate Plugin

## Prerequisites

1. **Gemini API Key**: Get your free API key from [Google AI Studio](https://makersuite.google.com/app/apikey)
2. **Node.js 20+**: Required for Docusaurus 3.9.2
3. **Git**: For version control

## Step-by-Step Testing

### 1. Install Dependencies

```bash
cd robolearn-interface
npm install
```

This will install:
- `@google/generative-ai` (Gemini SDK)
- `gray-matter` (Frontmatter parsing)
- `glob` (if not already installed)

### 2. Set Environment Variable

Create or update `.env` file in `robolearn-interface/`:

```bash
cd robolearn-interface
echo "GEMINI_API_KEY=your-actual-api-key-here" > .env
```

**Important**: Replace `your-actual-api-key-here` with your actual Gemini API key.

### 3. Verify Configuration

Check that plugin is configured in `docusaurus.config.ts`:

```typescript
plugins: [
  // ... other plugins
  [
    "./plugins/docusaurus-plugin-auto-translate",
    {
      enabled: true,
      sourceLocale: "en",
      targetLocales: ["ur"],
      apiProvider: "gemini",
      model: "gemini-2.0-flash-exp",
      apiKey: process.env.GEMINI_API_KEY,
      cacheDir: ".translation-cache",
      docsPath: "docs",
    },
  ],
],
```

Check i18n configuration:

```typescript
i18n: {
  defaultLocale: "en",
  locales: ["en", "ur"],
  localeConfigs: {
    ur: {
      label: "اردو",
      direction: "rtl",
    },
  },
},
```

### 4. Test Translation (First Build)

Run the build process:

```bash
cd robolearn-interface
npm run build
```

**What to expect:**
- Plugin logs: `[Auto-Translate] Starting translation process...`
- File scanning: `[Auto-Translate] Found X markdown files`
- Translation progress: `[Auto-Translate] Translating: [file path]`
- Cache hits: `[Auto-Translate] Cache hit: [file path]`
- Summary: `[Auto-Translate] Complete: X translated, Y cached, Z errors`

**First build will:**
- Translate ALL English files to Urdu
- Create `i18n/ur/docusaurus-plugin-content-docs/current/` structure
- Generate cache entries in `.translation-cache/`
- Take 5-10 minutes (depending on number of files)

### 5. Verify Translations Generated

Check that translations were created:

```bash
# List translated files
ls -la robolearn-interface/i18n/ur/docusaurus-plugin-content-docs/current/

# Check a specific translation
cat robolearn-interface/i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-4/lesson.md

# Verify cache was created
ls -la robolearn-interface/.translation-cache/
```

**Expected results:**
- ✅ `i18n/ur/` directory exists
- ✅ Translated `.md` files in `i18n/ur/docusaurus-plugin-content-docs/current/`
- ✅ Cache files in `.translation-cache/`
- ✅ Code blocks preserved (not translated)
- ✅ Technical terms preserved (ROS, URDF, API, etc.)

### 6. Test Cache Behavior (Second Build)

Run build again without changing any files:

```bash
npm run build
```

**Expected results:**
- ✅ Most files show: `[Auto-Translate] Cache hit: [file path]`
- ✅ No API calls for cached files
- ✅ Build completes faster (2-3 minutes vs 5-10 minutes)
- ✅ Cache hit rate >80%

### 7. Test Language Toggle

Start the development server:

```bash
npm start
```

**Test steps:**
1. Open browser: `http://localhost:3000`
2. Navigate to any docs page: `http://localhost:3000/docs/module-1/chapter-4/lesson`
3. Look for locale dropdown in navbar (top right)
4. Click dropdown, select "اردو" (Urdu)
5. Verify:
   - ✅ URL changes to `/ur/docs/...`
   - ✅ Content displays in Urdu
   - ✅ Text direction is RTL (right-to-left)
   - ✅ Code blocks remain LTR (left-to-right)
   - ✅ Technical terms preserved in English

### 8. Test RTL Layout

When viewing Urdu content:

**Visual checks:**
- ✅ Text flows right-to-left
- ✅ Navigation menus align right
- ✅ Code blocks remain left-to-right
- ✅ Lists and tables adapt to RTL
- ✅ Font renders correctly (Urdu characters)

**Technical checks:**
```bash
# In browser console (F12)
document.documentElement.dir  // Should return "rtl" for Urdu pages
```

### 9. Test Error Handling

**Test API unavailable:**
1. Set invalid API key: `GEMINI_API_KEY=invalid-key`
2. Run build: `npm run build`
3. Expected: Error logged, build continues, existing translations used

**Test with no API key:**
1. Remove `GEMINI_API_KEY` from `.env`
2. Run build: `npm run build`
3. Expected: Warning logged, plugin skips translation

### 10. Test Cache Invalidation

**Test file change detection:**
1. Modify a source file: `docs/module-1/chapter-4/lesson.md`
2. Run build: `npm run build`
3. Expected: Only that file is re-translated, others use cache

**Test cache clear:**
```bash
# Clear cache
rm -rf robolearn-interface/.translation-cache/

# Run build
npm run build
```
Expected: All files re-translated (no cache)

## Troubleshooting

### Issue: Translations not generating

**Check:**
- [ ] `GEMINI_API_KEY` is set in `.env`
- [ ] Plugin is enabled in `docusaurus.config.ts`
- [ ] API key is valid (test with curl or API tester)
- [ ] Check build logs for errors

**Debug:**
```bash
# Check if API key is loaded
node -e "require('dotenv').config(); console.log(process.env.GEMINI_API_KEY ? 'Key found' : 'Key missing')"
```

### Issue: Build fails with module not found

**Fix:**
```bash
cd robolearn-interface
npm install @google/generative-ai gray-matter
```

### Issue: Translations missing code blocks

**Check:**
- Review translation prompt in `lib/translator.js`
- Verify code blocks are preserved in translated files
- May need to refine prompt for better code preservation

### Issue: Language toggle not appearing

**Check:**
- [ ] `locales: ["en", "ur"]` in `docusaurus.config.ts`
- [ ] `type: "localeDropdown"` in navbar items
- [ ] Translations exist in `i18n/ur/`
- [ ] Browser cache cleared

### Issue: RTL not working

**Check:**
- [ ] `direction: "rtl"` in `localeConfigs.ur`
- [ ] `rtl.css` imported in `custom.css`
- [ ] HTML `dir` attribute set (check browser inspector)

## Quick Test Checklist

- [ ] Dependencies installed (`npm install`)
- [ ] API key set in `.env`
- [ ] First build completes successfully
- [ ] Translations generated in `i18n/ur/`
- [ ] Cache created in `.translation-cache/`
- [ ] Second build uses cache (faster)
- [ ] Language toggle appears in navbar
- [ ] Locale switching works (`/en/` ↔ `/ur/`)
- [ ] Urdu content displays RTL
- [ ] Code blocks preserved (not translated)
- [ ] Technical terms preserved (ROS, URDF, etc.)

## Performance Benchmarks

**Expected build times:**
- First build (no cache): 5-10 minutes (100 files)
- Second build (with cache): 2-3 minutes
- Cache hit rate: >80%

**API costs (Gemini 2.5 Flash Lite):**
- Free tier: 15 requests/minute
- Cost: ~$0.10 per 1M input tokens, $0.40 per 1M output tokens
- Estimated: ~$0.50-2.00 for initial translation of 100 files

## Next Steps After Testing

1. **Verify translation quality**: Review a few translated files manually
2. **Refine prompts**: Adjust translation prompt if code blocks/terms not preserved
3. **Optimize caching**: Fine-tune cache invalidation logic if needed
4. **Add more languages**: Extend to other locales (extend `targetLocales` array)
5. **Deploy**: Commit changes and deploy to production

## Manual Translation Quality Check

Review translated files for:
- ✅ Code blocks untranslated
- ✅ Technical terms (ROS, URDF, API) preserved
- ✅ Markdown formatting intact
- ✅ Frontmatter preserved
- ✅ Urdu text readable and natural
- ✅ RTL layout correct

## Support

If issues persist:
1. Check build logs for specific errors
2. Verify API key is valid and has quota
3. Review plugin README: `robolearn-interface/plugins/docusaurus-plugin-auto-translate/README.md`
4. Check Docusaurus i18n docs: https://docusaurus.io/docs/i18n/introduction

