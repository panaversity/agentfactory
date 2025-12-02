# Feature Specification: Automatic i18n Translation with Gemini

**Feature Branch**: `006-i18n-auto-translate-gemini`  
**Created**: 2025-01-27  
**Status**: Draft  
**Input**: User description: "Automatic i18n translation system using Gemini 2.5 Flash Lite - Build-time Docusaurus plugin that automatically translates English content to Urdu during build process, uses Gemini API for free/low-cost translation, caches translations to avoid re-translation, integrates with Docusaurus i18n infrastructure"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Automatic Translation During Build (Priority: P1)

As a content author, when I run `npm run build`, the system automatically translates all English content to Urdu and generates the i18n folder structure, so I don't need to manually translate or manage translation files.

**Why this priority**: This is the core value proposition - zero manual work for authors. Without this, the feature doesn't deliver on its promise of "automatic" translation.

**Independent Test**: Can be fully tested by running `npm run build` with English content in `docs/` and verifying that `i18n/ur/docusaurus-plugin-content-docs/current/` contains translated Urdu files. Delivers automatic translation workflow.

**Acceptance Scenarios**:

1. **Given** English content exists in `docs/module-1/chapter-4/lesson.md`, **When** I run `npm run build`, **Then** Urdu translation appears in `i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-4/lesson.md`
2. **Given** English content was modified, **When** I run `npm run build`, **Then** only modified files are re-translated (caching works)
3. **Given** English content exists but Urdu translation already exists and is up-to-date, **When** I run `npm run build`, **Then** existing translation is reused (no API call)

---

### User Story 2 - Language Toggle UI Component (Priority: P1)

As a student, when I visit a lesson page, I can click a language toggle button to switch between English and Urdu, so I can read content in my preferred language.

**Why this priority**: This is the user-facing feature that enables the translation value. Without this, translations exist but users can't access them.

**Independent Test**: Can be fully tested by visiting any docs page and clicking the language toggle, verifying URL changes to `/ur/` route and content displays in Urdu. Delivers multilingual access.

**Acceptance Scenarios**:

1. **Given** I'm viewing `/docs/module-1/chapter-4/lesson` in English, **When** I click the language toggle, **Then** URL changes to `/ur/docs/module-1/chapter-4/lesson` and content displays in Urdu
2. **Given** I'm viewing content in Urdu, **When** I click the language toggle, **Then** URL changes to `/en/` route and content displays in English
3. **Given** I've selected Urdu language, **When** I navigate to a new page, **Then** my language preference persists (cookie/localStorage)

---

### User Story 3 - RTL Support for Urdu (Priority: P2)

As a student reading Urdu content, the text displays right-to-left with proper typography and layout, so the reading experience feels natural.

**Why this priority**: RTL is essential for Urdu readability. Without it, Urdu content is difficult to read even if translated correctly.

**Independent Test**: Can be fully tested by viewing any Urdu-translated page and verifying text direction is RTL, fonts render correctly, and layout adapts. Delivers proper Urdu reading experience.

**Acceptance Scenarios**:

1. **Given** I'm viewing Urdu content, **When** the page loads, **Then** text direction is RTL (`dir="rtl"`) and layout adapts accordingly
2. **Given** Urdu content contains code blocks, **When** the page renders, **Then** code blocks remain LTR while surrounding text is RTL
3. **Given** Urdu content contains technical terms in English, **When** the page renders, **Then** English terms are preserved with proper RTL context

---

### User Story 4 - Translation Caching & Performance (Priority: P2)

As a developer running builds, the translation system only calls the Gemini API for new or changed content, so build times remain fast and API costs stay low.

**Why this priority**: Without caching, every build would re-translate everything, making builds slow and expensive. Caching is essential for practical use.

**Independent Test**: Can be fully tested by running two builds in a row with no content changes, verifying second build skips translation (no API calls). Delivers performance and cost efficiency.

**Acceptance Scenarios**:

1. **Given** content was translated in a previous build, **When** I run `npm run build` again with no changes, **Then** no Gemini API calls are made (cached translations used)
2. **Given** one file was modified, **When** I run `npm run build`, **Then** only that file is re-translated, other files use cache
3. **Given** translation cache exists, **When** I delete `i18n/ur/` folder, **Then** next build re-translates all content

---

### Edge Cases

- What happens when Gemini API is unavailable or rate-limited?
  - System should fail gracefully with clear error message, allow build to continue with existing translations
- How does system handle very large files (>100KB)?
  - Should chunk content or handle in batches to avoid API limits
- What happens when translation fails mid-process?
  - Should log error, skip that file, continue with others, report failures at end
- How does system handle special characters, code blocks, frontmatter?
  - Should preserve code blocks exactly, handle frontmatter metadata, escape special characters properly
- What happens when source file is deleted?
  - Should detect and remove corresponding translation file
- How does system handle concurrent builds?
  - Cache should be thread-safe or use file locks to prevent race conditions

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST automatically translate English `.md` files in `docs/` to Urdu during Docusaurus build process
- **FR-002**: System MUST use Gemini 2.5 Flash Lite API (or configurable model) for translation
- **FR-003**: System MUST preserve code blocks, frontmatter, and technical terms (ROS, URDF, API, etc.) in original English
- **FR-004**: System MUST generate translations in Docusaurus i18n folder structure: `i18n/ur/docusaurus-plugin-content-docs/current/[path]/[file].md`
- **FR-005**: System MUST cache translations based on file hash/modified date to avoid re-translating unchanged content
- **FR-006**: System MUST support RTL (right-to-left) layout for Urdu content
- **FR-007**: System MUST provide LanguageToggle component that switches between `/en/` and `/ur/` routes
- **FR-008**: System MUST persist language preference (cookie or localStorage) across page navigation
- **FR-009**: System MUST handle translation errors gracefully (log, skip file, continue build)
- **FR-010**: System MUST be configurable via `docusaurus.config.ts` plugin options (API key, enabled/disabled, locales, cache dir)
- **FR-011**: System MUST translate UI labels in `code.json` files for navbar, footer, etc.
- **FR-012**: System MUST respect Docusaurus i18n conventions (locale codes, folder structure, JSON format)

### Key Entities *(include if feature involves data)*

- **Translation Cache**: Stores file hash → translation mapping to avoid re-translation. Key attributes: source file path, hash, target locale, translation file path, timestamp
- **Translation Job**: Represents a single translation task. Key attributes: source file path, target locale, status (pending/completed/failed), API response, error message
- **Language Preference**: User's selected language. Key attributes: locale code, persistence method (cookie/localStorage), expiration

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Build process completes translation for 100 docs files in under 5 minutes (assuming average 2KB per file)
- **SC-002**: Translation cache hit rate is >80% on subsequent builds (only 20% of files need re-translation)
- **SC-003**: Language toggle switches locale in under 500ms (instant user experience)
- **SC-004**: Urdu content displays with proper RTL layout (verified via visual inspection and `dir="rtl"` attribute)
- **SC-005**: Technical terms (ROS, URDF, API, etc.) are preserved in English with Urdu explanations (verified via content inspection)
- **SC-006**: Code blocks remain untranslated and functional (verified by checking code syntax and execution)
- **SC-007**: Build fails gracefully when API is unavailable (error logged, build continues with existing translations)
- **SC-008**: Zero manual translation work required - all translations generated automatically during build

## Constraints

- **C-001**: Must work with Docusaurus 3.9.2 (current version)
- **C-002**: Must use Gemini API (free tier or low-cost model)
- **C-003**: Must not break existing build process (failures should be non-blocking)
- **C-004**: Must respect Docusaurus i18n folder structure conventions
- **C-005**: Must handle large content files (chunking if needed for API limits)
- **C-006**: Must be configurable via environment variables (API key, etc.)

## Non-Goals

- **NG-001**: Real-time translation (translations happen at build time only)
- **NG-002**: Translation of user-generated content (only static docs content)
- **NG-003**: Multiple target languages in initial version (Urdu only, extensible later)
- **NG-004**: Translation quality review/editing UI (translations are generated automatically)
- **NG-005**: Manual translation workflow (this is fully automatic)
- **NG-006**: Translation of images, videos, or other media (text content only)

## Technical Notes

- Plugin should hook into Docusaurus `loadContent` lifecycle (runs before build)
- Use file hashing (MD5/SHA256) for cache invalidation
- Gemini API: `gemini-2.0-flash-exp` or `gemini-2.5-flash-lite` model
- Translation prompt should include instructions for technical term preservation
- RTL support via CSS (`direction: rtl`) and HTML `dir` attribute
- LanguageToggle can swizzle Docusaurus theme component or create custom component
