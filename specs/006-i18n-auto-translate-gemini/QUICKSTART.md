# Quick Start: Test i18n Translation Locally

## 🚀 5-Minute Setup

### Step 1: Get Gemini API Key

1. Visit: https://makersuite.google.com/app/apikey
2. Sign in with Google account
3. Click "Create API Key"
4. Copy the key

### Step 2: Set Environment Variable

```bash
cd robolearn-interface

# Create .env file (if it doesn't exist)
echo "GEMINI_API_KEY=your-api-key-here" >> .env

# Or edit .env manually and add:
# GEMINI_API_KEY=your-actual-api-key
```

**Replace `your-api-key-here` with your actual Gemini API key.**

### Step 3: Install Dependencies

```bash
cd robolearn-interface
npm install
```

This installs:
- `@google/generative-ai` (Gemini SDK)
- `gray-matter` (Frontmatter parsing)

### Step 4: Test Translation

```bash
# Run build (this will translate all files)
npm run build
```

**Watch for:**
- `[Auto-Translate] Starting translation process...`
- `[Auto-Translate] Found X markdown files`
- `[Auto-Translate] Translating: [file]`
- `[Auto-Translate] Complete: X translated, Y cached`

**First build takes 5-10 minutes** (translating all files)

### Step 5: Verify Translations

```bash
# Check translations were created
ls -la i18n/ur/docusaurus-plugin-content-docs/current/

# View a translated file
cat i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-4/lesson.md | head -30
```

### Step 6: Test Language Toggle

```bash
# Start dev server
npm start
```

1. Open: `http://localhost:3000`
2. Go to any docs page
3. Click locale dropdown (top right navbar)
4. Select "اردو" (Urdu)
5. Verify content switches to Urdu with RTL layout

## ✅ Success Indicators

- ✅ Build completes without errors
- ✅ `i18n/ur/` folder contains translated files
- ✅ `.translation-cache/` folder created
- ✅ Language toggle appears in navbar
- ✅ Clicking toggle switches to `/ur/` route
- ✅ Urdu content displays RTL

## 🔧 Troubleshooting

**No translations generated?**
- Check `GEMINI_API_KEY` is set: `echo $GEMINI_API_KEY`
- Verify API key is valid
- Check build logs for errors

**Language toggle not showing?**
- Verify `locales: ["en", "ur"]` in `docusaurus.config.ts`
- Check translations exist: `ls i18n/ur/`
- Clear browser cache

**Build fails?**
- Run `npm install` to ensure dependencies installed
- Check Node.js version: `node --version` (needs 20+)

## 📊 Expected Results

**First Build:**
- Time: 5-10 minutes
- Translations: All files translated
- Cache: Created for all files

**Second Build (no changes):**
- Time: 2-3 minutes
- Translations: 0 new (all cached)
- Cache hit rate: >80%

## 🎯 Quick Test Checklist

```bash
# 1. Install dependencies
cd robolearn-interface && npm install

# 2. Set API key
echo "GEMINI_API_KEY=your-key" > .env

# 3. Build (translates files)
npm run build

# 4. Verify translations
ls i18n/ur/docusaurus-plugin-content-docs/current/

# 5. Start dev server
npm start

# 6. Test in browser
# - Open http://localhost:3000
# - Click locale dropdown
# - Switch to Urdu
```

## 💡 Tips

- **First build is slow**: Normal, translating all files
- **Subsequent builds are fast**: Uses cache
- **Test with small subset**: Comment out most docs in config to test faster
- **Check API quota**: Free tier has rate limits

For detailed testing guide, see [TESTING.md](./TESTING.md)

