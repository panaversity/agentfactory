# Implementation Plan: Automatic i18n Translation with Gemini

**Branch**: `006-i18n-auto-translate-gemini` | **Date**: 2025-01-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-i18n-auto-translate-gemini/spec.md`

## Summary

Build a Docusaurus plugin that automatically translates English content to Urdu during the build process using Google Gemini 2.5 Flash Lite API. The plugin integrates with Docusaurus i18n infrastructure, caches translations to avoid re-translation, and provides a LanguageToggle component for users to switch between locales. This enables zero-manual-work multilingual content delivery.

## Technical Context

**Language/Version**: Node.js 20+, JavaScript (CommonJS for plugin compatibility)  
**Primary Dependencies**: 
- `@google/generative-ai` (Gemini SDK)
- `crypto` (Node.js built-in for file hashing)
- `fs`, `path`, `glob` (Node.js built-in for file operations)
- `gray-matter` (parse frontmatter, preserve during translation)

**Storage**: File system (translation cache in `.translation-cache/`, translations in `i18n/ur/`)  
**Testing**: Manual testing (build process, translation quality, UI component)  
**Target Platform**: Docusaurus 3.9.2 build process (Node.js environment)  
**Project Type**: Docusaurus plugin (single module)  
**Performance Goals**: 
- Translate 100 files (~200KB total) in under 5 minutes
- Cache hit rate >80% on subsequent builds
- Language toggle <500ms response time

**Constraints**: 
- Must not break existing build (failures non-blocking)
- Must respect Docusaurus i18n folder structure
- Must handle API rate limits gracefully
- Must preserve code blocks and technical terms

**Scale/Scope**: 
- ~100-200 markdown files in `docs/` directory
- Single target locale (Urdu) initially, extensible to more
- Build-time only (no runtime translation)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Platform Principles Alignment

✅ **Specification Primacy**: Spec clearly defines intent, success criteria, and constraints  
✅ **Progressive Complexity**: Feature is self-contained, doesn't add complexity to content  
✅ **Factual Accuracy**: Uses official Gemini API, follows Docusaurus conventions  
✅ **Intelligence Accumulation**: Creates reusable translation workflow pattern  
✅ **Anti-Convergence**: Custom solution (not using default Docusaurus i18n plugin)  
✅ **Minimal Content**: Only translates existing content, doesn't create new content  
✅ **Hardware-Awareness**: N/A (build-time feature, no hardware requirements)  
✅ **Simulation-First**: N/A (not robotics content)  
✅ **Safety-Critical**: N/A (translation feature, no safety concerns)

**No violations detected.** Feature aligns with platform principles.

## Project Structure

### Documentation (this feature)

```text
specs/006-i18n-auto-translate-gemini/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
robolearn-interface/
├── plugins/
│   └── docusaurus-plugin-auto-translate/
│       ├── index.js                    # Main plugin entry point
│       ├── lib/
│       │   ├── translator.js           # Gemini API integration
│       │   ├── cache.js                # Translation cache management
│       │   ├── file-processor.js      # File reading, parsing, writing
│       │   └── i18n-structure.js      # Docusaurus i18n folder structure helpers
│       ├── README.md                   # Plugin documentation
│       └── package.json                # Plugin dependencies (if needed)
│
├── src/
│   ├── components/
│   │   └── LanguageToggle/
│   │       ├── index.tsx               # LanguageToggle component
│   │       └── styles.module.css       # Component styles
│   └── css/
│       └── rtl.css                     # RTL support styles
│
├── i18n/
│   └── ur/                              # Generated translations
│       ├── code.json                    # UI labels translation
│       ├── docusaurus-plugin-content-docs/
│       │   └── current/                 # Translated docs
│       └── docusaurus-theme-classic/   # Theme translations
│           ├── navbar.json
│           └── footer.json
│
└── .translation-cache/                  # Cache directory (gitignored)
    └── [file-hash].json                 # Cache entries
```

**Structure Decision**: Single Docusaurus plugin module with helper libraries, plus React component for LanguageToggle. Translations generated into standard Docusaurus i18n structure.

## Architecture

### Component Breakdown

1. **Auto-Translate Plugin** (`plugins/docusaurus-plugin-auto-translate/`)
   - **Purpose**: Main plugin that hooks into Docusaurus build lifecycle
   - **Responsibilities**:
     - Scan `docs/` for English `.md` files
     - Check cache for existing translations
     - Call Gemini API for missing/outdated translations
     - Write translations to `i18n/ur/` structure
     - Handle errors gracefully
   - **Lifecycle Hook**: `loadContent()` (runs before build)

2. **Translator Module** (`lib/translator.js`)
   - **Purpose**: Gemini API integration
   - **Responsibilities**:
     - Initialize Gemini client
     - Format translation prompts (preserve code blocks, technical terms)
     - Handle API rate limits and errors
     - Return translated content

3. **Cache Module** (`lib/cache.js`)
   - **Purpose**: Translation caching
   - **Responsibilities**:
     - Generate file hash (MD5/SHA256)
     - Check if translation exists and is up-to-date
     - Store cache entries
     - Invalidate cache on source file change

4. **File Processor** (`lib/file-processor.js`)
   - **Purpose**: File I/O and content parsing
   - **Responsibilities**:
     - Read source markdown files
     - Parse frontmatter (preserve during translation)
     - Extract translatable content
     - Write translated files to i18n structure
     - Handle special characters and encoding

5. **i18n Structure Helper** (`lib/i18n-structure.js`)
   - **Purpose**: Docusaurus i18n folder structure management
   - **Responsibilities**:
     - Map source file path to i18n target path
     - Create directory structure
     - Handle Docusaurus doc ID normalization (strip numeric prefixes)

6. **LanguageToggle Component** (`src/components/LanguageToggle/`)
   - **Purpose**: User-facing language switcher
   - **Responsibilities**:
     - Display current locale
     - Switch between `/en/` and `/ur/` routes
     - Persist preference (localStorage/cookie)
   - **Integration**: Can swizzle Docusaurus theme or create custom component

7. **RTL CSS** (`src/css/rtl.css`)
   - **Purpose**: Right-to-left layout support
   - **Responsibilities**:
     - RTL direction styles
     - Code block LTR override
     - Typography adjustments

### Data Flow

```
Build Process:
1. Docusaurus starts build
2. Auto-Translate Plugin.loadContent() executes
3. Scan docs/ → Find all .md files
4. For each file:
   a. Generate hash → Check cache
   b. If cache miss → Call Gemini API
   c. Translate content (preserve code blocks, technical terms)
   d. Write to i18n/ur/docusaurus-plugin-content-docs/current/...
   e. Update cache
5. Docusaurus continues build with translations ready
6. Build outputs /en/ and /ur/ routes

Runtime:
1. User visits page → LanguageToggle shows current locale
2. User clicks toggle → Navigate to /ur/ route
3. Docusaurus serves Urdu content from i18n/ur/
4. RTL CSS applies → Content displays right-to-left
```

### Dependencies

**Plugin Dependencies** (add to `robolearn-interface/package.json`):
- `@google/generative-ai`: ^0.21.0 (Gemini SDK)
- `gray-matter`: ^4.0.3 (Frontmatter parsing)
- `glob`: ^10.3.10 (File pattern matching - may already exist)

**No additional runtime dependencies** (plugin runs at build time only)

## Implementation Phases

### Phase 0: Research & Setup

**Deliverables**:
- [ ] Research Gemini API SDK usage
- [ ] Test Gemini API with sample content
- [ ] Verify Docusaurus i18n folder structure conventions
- [ ] Test file hashing for cache invalidation
- [ ] Document translation prompt format

**Research Areas**:
- Gemini API authentication and initialization
- API rate limits and error handling
- Translation prompt engineering (preserve code blocks, technical terms)
- Docusaurus i18n doc ID normalization rules
- File hash algorithm selection (MD5 vs SHA256)

### Phase 1: Core Plugin Implementation

**Deliverables**:
- [ ] Create plugin directory structure
- [ ] Implement `lib/cache.js` (file hashing, cache management)
- [ ] Implement `lib/file-processor.js` (file I/O, frontmatter parsing)
- [ ] Implement `lib/i18n-structure.js` (path mapping, directory creation)
- [ ] Implement `lib/translator.js` (Gemini API integration)
- [ ] Implement main plugin (`index.js`) with `loadContent()` hook
- [ ] Add plugin to `docusaurus.config.ts`

**Success Criteria**:
- Plugin runs during build
- Scans docs/ directory correctly
- Generates file hashes
- Creates i18n folder structure

### Phase 2: Translation Logic

**Deliverables**:
- [ ] Implement translation prompt (preserve code blocks, technical terms)
- [ ] Implement Gemini API call with error handling
- [ ] Implement content chunking for large files
- [ ] Implement cache check and update logic
- [ ] Test translation quality (technical terms preserved)

**Success Criteria**:
- Translations generated successfully
- Code blocks remain untranslated
- Technical terms (ROS, URDF, API) preserved in English
- Cache prevents re-translation of unchanged files

### Phase 3: UI Components

**Deliverables**:
- [ ] Create LanguageToggle component
- [ ] Implement locale switching logic
- [ ] Add language preference persistence (localStorage)
- [ ] Create RTL CSS styles
- [ ] Integrate LanguageToggle into navbar (swizzle or custom)

**Success Criteria**:
- LanguageToggle displays and functions correctly
- Locale switches between /en/ and /ur/
- Preference persists across navigation
- RTL layout applies to Urdu content

### Phase 4: Configuration & Polish

**Deliverables**:
- [ ] Add plugin configuration options (API key, enabled/disabled, locales)
- [ ] Add environment variable support (GEMINI_API_KEY)
- [ ] Implement error handling (API failures, rate limits)
- [ ] Add logging and progress indicators
- [ ] Update Docusaurus i18n config (add "ur" locale)

**Success Criteria**:
- Plugin configurable via docusaurus.config.ts
- API key from environment variable
- Errors handled gracefully (build continues)
- Clear logging during translation process

### Phase 5: Testing & Validation

**Deliverables**:
- [ ] Test with sample docs (10-20 files)
- [ ] Verify translation quality
- [ ] Test cache behavior (second build skips translation)
- [ ] Test error scenarios (API unavailable, rate limits)
- [ ] Test LanguageToggle functionality
- [ ] Verify RTL layout

**Success Criteria**:
- All acceptance scenarios pass
- Translation quality acceptable (technical terms preserved)
- Cache hit rate >80% on second build
- LanguageToggle works correctly
- RTL layout displays properly

## Integration Points

### Docusaurus Configuration

```typescript
// docusaurus.config.ts
i18n: {
  defaultLocale: "en",
  locales: ["en", "ur"], // Add Urdu locale
  localeConfigs: {
    ur: {
      label: "اردو",
      direction: "rtl", // RTL support
    },
  },
},

plugins: [
  [
    "./plugins/docusaurus-plugin-auto-translate",
    {
      enabled: true,
      sourceLocale: "en",
      targetLocales: ["ur"],
      apiProvider: "gemini",
      model: "gemini-2.0-flash-exp", // or gemini-2.5-flash-lite
      apiKey: process.env.GEMINI_API_KEY,
      cacheDir: ".translation-cache",
      docsPath: "docs",
    },
  ],
],
```

### Environment Variables

```bash
# .env
GEMINI_API_KEY=your-gemini-api-key-here
```

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Gemini API unavailable | Fail gracefully, log error, continue build with existing translations |
| API rate limits | Implement retry logic with exponential backoff, batch requests |
| Translation quality issues | Test with sample content, refine prompts, preserve technical terms explicitly |
| Build time increase | Cache aggressively, only translate changed files, parallelize API calls if possible |
| Large file handling | Chunk content, handle in batches, respect API token limits |
| Cache corruption | Validate cache entries, regenerate on corruption detection |

## Complexity Tracking

> **No violations detected** - Feature is self-contained plugin, doesn't violate constitution principles.

## Next Steps

1. Proceed to Phase 0: Research Gemini API and test translation quality
2. Implement core plugin structure (Phase 1)
3. Add translation logic (Phase 2)
4. Create UI components (Phase 3)
5. Configure and polish (Phase 4)
6. Test and validate (Phase 5)
