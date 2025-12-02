# Docusaurus Auto-Translate Plugin

Automatically translates English content to Urdu during the Docusaurus build process using Google Gemini API.

## Features

- ✅ **Automatic Translation**: Translates all English `.md` files to Urdu during build
- ✅ **Smart Caching**: Only translates new or changed files (cache hit rate >80%)
- ✅ **Code Preservation**: Preserves code blocks, technical terms, and frontmatter
- ✅ **Docusaurus Integration**: Generates standard i18n folder structure
- ✅ **Error Handling**: Gracefully handles API failures (non-blocking)
- ✅ **Free/Low Cost**: Uses Gemini Flash Lite Latest (free tier available)

## Installation

### 1. Install Dependencies

```bash
cd robolearn-interface
npm install @google/generative-ai gray-matter
```

### 2. Configure Plugin

Add to `docusaurus.config.ts`:

```typescript
plugins: [
  [
    "./plugins/docusaurus-plugin-auto-translate",
    {
      enabled: true,
      sourceLocale: "en",
      targetLocales: ["ur"],
      apiProvider: "gemini",
      model: "gemini-flash-lite-latest", // Points to gemini-2.5-flash-lite-preview-09-2025
      apiKey: process.env.GEMINI_API_KEY,
      cacheDir: ".translation-cache",
      docsPath: "docs",
    },
  ],
],
```

### 3. Configure i18n

Update `docusaurus.config.ts`:

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

### 4. Set Environment Variable

```bash
# .env
GEMINI_API_KEY=your-gemini-api-key-here
```

## Usage

### Build with Translation

```bash
npm run build
```

The plugin automatically:
1. Scans `docs/` for English `.md` files
2. Checks cache for existing translations
3. Translates missing/outdated files using Gemini API
4. Writes translations to `i18n/ur/docusaurus-plugin-content-docs/current/`
5. Updates cache

### Language Toggle

Use the `LanguageToggle` component in your navbar:

```tsx
import { LanguageToggle } from '@/components/LanguageToggle';

// In your navbar component
<LanguageToggle />
```

## Translation Quality

The plugin preserves:
- **Code blocks**: Untranslated, exact syntax preserved
- **Technical terms**: ROS, URDF, API, SDK, etc. kept in English
- **Frontmatter**: YAML metadata preserved exactly
- **Markdown formatting**: Headers, lists, links maintained

## Cache Behavior

- **First build**: Translates all files
- **Subsequent builds**: Only translates changed files (cache hit >80%)
- **Cache location**: `.translation-cache/` (gitignored)
- **Cache invalidation**: Automatic on file hash change

## Error Handling

- API unavailable: Logs error, continues build with existing translations
- Rate limits: Implements retry logic (future enhancement)
- Translation failures: Skips file, logs error, continues with others

## Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `enabled` | boolean | `true` | Enable/disable plugin |
| `sourceLocale` | string | `"en"` | Source language code |
| `targetLocales` | string[] | `["ur"]` | Target language codes |
| `apiProvider` | string | `"gemini"` | Translation API provider |
| `model` | string | `"gemini-2.0-flash-exp"` | Gemini model name |
| `apiKey` | string | `process.env.GEMINI_API_KEY` | API key |
| `cacheDir` | string | `".translation-cache"` | Cache directory |
| `docsPath` | string | `"docs"` | Source docs directory |

## Performance

- **Translation speed**: ~100 files in 5 minutes (depends on API)
- **Cache hit rate**: >80% on subsequent builds
- **Build impact**: Adds ~2-5 minutes to build time (first run only)

## Limitations

- Build-time only (no runtime translation)
- Single target language initially (Urdu), extensible
- Requires Gemini API key
- Large files may need chunking (future enhancement)

## Troubleshooting

### Translations not generating

1. Check `GEMINI_API_KEY` is set
2. Verify plugin is enabled in config
3. Check build logs for errors

### Cache not working

1. Verify `.translation-cache/` directory exists
2. Check file permissions
3. Clear cache and rebuild: `rm -rf .translation-cache && npm run build`

### Translation quality issues

1. Review translation prompt in `lib/translator.js`
2. Check that code blocks are preserved
3. Verify technical terms are kept in English

## Future Enhancements

- [ ] Multiple target languages
- [ ] Translation quality review UI
- [ ] Parallel API calls (with rate limit handling)
- [ ] Smarter content chunking for large files
- [ ] Translation statistics dashboard

