# Remark Content Enhancements Plugin

**Version**: 1.0.0
**Purpose**: Composable, metadata-driven content enhancement system for Docusaurus

## Overview

This plugin implements a **3-layer separation architecture** that decouples content from presentation logic:

```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 1: CONTENT (Platform-Agnostic)                       │
│  - Frontmatter metadata only                                 │
│  - Zero JSX imports                                          │
│  - Pure markdown                                             │
├─────────────────────────────────────────────────────────────┤
│  LAYER 2: BUILD-TIME TRANSFORMATION (Remark Plugins)        │
│  - Parse metadata                                            │
│  - Inject components                                         │
│  - Resolve paths/URLs                                        │
│  - Composable plugin architecture                           │
├─────────────────────────────────────────────────────────────┤
│  LAYER 3: RUNTIME COMPONENTS (React)                         │
│  - PDFViewer, InteractivePython, etc.                       │
│  - Pure presentation logic                                   │
│  - Platform-specific rendering                               │
└─────────────────────────────────────────────────────────────┘
```

## Key Benefits

- **Composability**: Add new enhancements by creating transformers
- **Decoupling**: Content (YAML) ↔ Platform (Docusaurus) ↔ Runtime (React)
- **Cloud-Ready**: Switch local → CDN URLs without touching markdown
- **Maintainability**: Update all chapters from single location
- **Portability**: Remark plugins work across SSGs (Docusaurus, Astro, Next.js)

---

## Installation

Already included in this project. Dependencies:

```bash
npm install unist-util-visit
```

---

## Usage

### Configuration (docusaurus.config.ts)

```typescript
docs: {
  remarkPlugins: [
    [require('./plugins/remark-content-enhancements'), {
      enableSlides: true,
      slidesConfig: {
        defaultHeight: 700,
      },
    }],
  ],
}
```

### Content (Frontmatter)

#### Simple Format (String)

```yaml
---
title: "Module 1"
slides: "slides/module-1-slides.pdf"
---
```

#### Advanced Format (Object)

```yaml
---
title: "Module 1"
slides:
  source: "slides/module-1-slides.pdf"  # Local path
  # OR: source: "https://cdn.example.com/slides/module-1.pdf"  # Cloud URL
  placement: "before-what-you-learn"
  height: 700
  title: "Module 1 Slides"
---
```

---

## Features

### 1. Slides Transformation

**Frontmatter** → **Build-time** → **Rendered Component**

#### Supported Sources

- **Local paths**: `"slides/module-1.pdf"` → Served from `/static/slides/`
- **Cloud URLs**: `"https://cdn.example.com/slide.pdf"` → Direct URL
- **Automatic detection**: Plugin detects URL vs path transparently

#### Placement Options

- `"before-what-you-learn"` (default): Inject before "What You'll Learn" heading
- `"after-intro"`: Inject after first H2 heading
- Future: Custom heading selectors

#### Example

**Input** (markdown):
```yaml
---
slides: "slides/module-1-slides.pdf"
---

# Module 1

## What You'll Learn
...
```

**Output** (after build):
```jsx
# Module 1

<PDFViewer
  src="/slides/module-1-slides.pdf"
  title="Chapter Slides"
  height={700}
/>

## What You'll Learn
...
```

### 2. Future: Interactive Code

Placeholder for unified interactive code transformation (currently handled by `remark-interactive-python`).

---

## Architecture

### File Structure

```
plugins/remark-content-enhancements/
├── index.js                 # Main orchestrator plugin
├── transformers/
│   ├── slides.js            # Slides transformer
│   └── (future)
│       ├── interactive-code.js
│       ├── quiz.js
│       └── assessment.js
├── utils/
│   └── (future) ast-helpers.js
├── package.json
└── README.md
```

### Plugin Flow

```
1. Remark parses markdown → AST
2. Docusaurus extracts frontmatter → file.data.frontmatter
3. remark-content-enhancements reads frontmatter
4. IF slides metadata exists:
   a. Normalize metadata (string → object)
   b. Detect URL vs local path
   c. Find injection point in AST
   d. Create PDFViewer JSX node
   e. Inject into AST
5. Docusaurus compiles MDX → React components
6. Browser renders PDFViewer component
```

### Transformer API

Each transformer exports:

```javascript
async function transform(tree, file, metadata, config) {
  // tree: Markdown AST (unist)
  // file: VFile with frontmatter in file.data.frontmatter
  // metadata: Specific metadata for this transformer
  // config: Global configuration options

  // Modify tree in-place (AST transformation)
}

module.exports = { transform };
```

---

## Migration Guide

### Before (Manual JSX)

```markdown
---
title: "Module 1"
---

import PDFViewer from '@site/src/components/PDFViewer';

# Module 1

<PDFViewer
  src="slides/module-1-slides.pdf"
  title="Module 1 Slides"
  height={700}
/>

## What You'll Learn
...
```

**Problems**:
- Manual imports in every file
- Tight coupling to Docusaurus
- Can't switch to cloud URLs without editing all files

### After (Metadata-Driven)

```markdown
---
title: "Module 1"
slides: "slides/module-1-slides.pdf"
---

# Module 1

## What You'll Learn
...
```

**Benefits**:
- Zero imports needed
- Decoupled from platform
- Switch to cloud: Update frontmatter only

### Cloud Migration (Zero Content Changes)

```yaml
---
slides: "https://cdn.example.com/slides/module-1.pdf"
---
```

---

## Configuration Options

### Plugin Options

```typescript
{
  enableSlides: boolean;          // Enable slides transformation (default: true)
  enableInteractiveCode: boolean; // Enable code transformation (default: false, future)
  slidesConfig: {
    defaultHeight: number;        // Default PDF viewer height (default: 700)
  };
  codeConfig: {
    excludeMeta: string[];        // Meta strings to exclude (future)
  };
}
```

### Frontmatter Schema

#### Slides (Simple)

```yaml
slides: string  # "local/path.pdf" or "https://url.com/slide.pdf"
```

#### Slides (Advanced)

```yaml
slides:
  source: string       # Required: Path or URL
  placement: string    # Optional: Injection point (default: "before-what-you-learn")
  height: number       # Optional: Viewer height (default: 700)
  title: string        # Optional: Accessibility title (default: "Chapter Slides")
```

---

## Troubleshooting

### Slides Not Appearing

1. **Check frontmatter syntax**:
   ```yaml
   slides: "slides/module-1.pdf"  # ✅ Correct
   slides: slides/module-1.pdf    # ❌ Wrong (missing quotes)
   ```

2. **Verify file path**:
   - Local: File must exist in `static/slides/`
   - URL: Must start with `http://` or `https://`

3. **Check injection point**:
   - Default: Looks for "What You'll Learn" heading
   - If not found: Check console for warning

4. **Verify plugin is loaded**:
   ```bash
   # Build and check logs
   npm run build
   # Look for: [Slides Transformer] ✅ Injected ...
   ```

### Build Warnings

```
[Slides Transformer] Could not find injection point "before-what-you-learn"
```

**Solution**: Add "What You'll Learn" heading or use different placement:

```yaml
slides:
  source: "slides/chapter.pdf"
  placement: "after-intro"
```

### Invalid URLs

Plugin validates URLs but doesn't check if they're accessible at build time (graceful degradation). Check browser console if PDFs don't load.

---

## Development

### Adding New Transformers

1. **Create transformer** in `transformers/`:

```javascript
// transformers/quiz.js
async function transform(tree, file, quizMetadata, config) {
  // Implement transformation logic
}

module.exports = { transform };
```

2. **Register in orchestrator** (`index.js`):

```javascript
const quizTransformer = require('./transformers/quiz');

// In plugin function:
if (enableQuiz && frontmatter.quiz) {
  transformers.push(
    quizTransformer.transform(tree, file, frontmatter.quiz, quizConfig)
  );
}
```

3. **Update frontmatter schema**:

```yaml
---
quiz:
  source: "quizzes/module-1.json"
  placement: "after-lesson"
---
```

### Testing

```bash
# Build and verify
npm run build

# Check specific file
npm run build 2>&1 | grep "Slides Transformer"

# Development mode
npm run start
```

---

## Comparison with Other Approaches

### vs. Manual JSX Imports

| Aspect | Manual JSX | Metadata-Driven |
|--------|-----------|-----------------|
| **Imports** | Required in every file | Zero imports |
| **Coupling** | Tight (Docusaurus-specific) | Loose (portable) |
| **Maintenance** | Update all files manually | Update plugin once |
| **Cloud Migration** | Edit all files | Update frontmatter |
| **Complexity** | High (JSX in markdown) | Low (YAML only) |

### vs. Runtime Injection

| Aspect | Runtime Injection | Build-Time (This Plugin) |
|--------|-------------------|-------------------------|
| **Performance** | Client-side overhead | Zero runtime cost |
| **SEO** | Potential issues | Fully rendered HTML |
| **Reliability** | JS required | Works without JS |
| **Complexity** | Higher (client logic) | Lower (build-time) |

---

## Roadmap

- [ ] Interactive code transformer (migrate from `remark-interactive-python`)
- [ ] Quiz transformer
- [ ] Assessment transformer
- [ ] Exercise transformer
- [ ] Validation utilities (check file existence at build time)
- [ ] AST helper utilities
- [ ] TypeScript type definitions

---

## License

MIT
