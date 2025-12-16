# Data Model: Metadata-Driven Slides Architecture

**Feature**: `035-metadata-driven-slides`
**Date**: 2025-11-23
**Phase**: Phase 1 (Design & Contracts)

## Overview

This document defines the data structures for metadata-driven slides architecture, including frontmatter schema, AST node structures, and TypeScript interfaces.

---

## 1. Frontmatter Schema

### Chapter README Metadata

**Location**: `apps/learn-app/docs/[part]/[chapter]/README.md`

**Schema**:

```yaml
---
sidebar_position: 2
title: "Chapter 2: The AI Turning Point"
slides: "slides/chapter-02-slides.pdf" # NEW: Optional string field
---
```

**Field Definition**:
| Field | Type | Required | Format | Description |
|-------|------|----------|--------|-------------|
| `slides` | `string \| undefined` | No | Local path OR full URL | Path to PDF slide deck. Supports both local static files and cloud URLs. |

**Examples**:

1. **Local path** (relative to static directory):

```yaml
slides: "slides/chapter-02-slides.pdf"
# Resolves to: /slides/chapter-02-slides.pdf
```

2. **Cloud URL** (absolute):

```yaml
slides: "https://r2.cloudflare.com/slides/chapter-02-slides.pdf"
# Used as-is in PDFViewer src prop
```

3. **No slides** (omit field):

```yaml
# slides field not present
# Result: No injection, no errors
```

**Validation Rules**:

- If `slides` is defined:
  - ✅ MUST be non-empty string
  - ✅ Local paths MAY start with `/` (normalized automatically)
  - ✅ URLs MUST start with `http://` or `https://`
  - ⚠️ Invalid format → Log warning, skip injection
- If `slides` is undefined or empty:
  - ✅ No action (valid state)

---

## 2. AST Node Structures

### 2.1 YAML Frontmatter Node

**Type**: `yaml` (from remark-frontmatter)

**Structure**:

```typescript
interface YAMLNode {
  type: "yaml";
  value: string; // Raw YAML string
  position: Position;
}
```

**Parsed Data** (accessed via `vfile.data.frontMatter`):

```typescript
interface FrontMatter {
  sidebar_position?: number;
  title?: string;
  slides?: string; // NEW: Optional slides field
  [key: string]: any;
}
```

---

### 2.2 Heading Node (Injection Landmark)

**Type**: `heading` (mdast)

**Structure**:

```typescript
interface HeadingNode {
  type: "heading";
  depth: 1 | 2 | 3 | 4 | 5 | 6; // H2 = depth 2
  children: PhrasingContent[]; // Text content
  position: Position;
}
```

**Target Heading**:

- Text content: `"What You'll Learn"`
- Depth: `2` (H2 heading)
- Extraction: `mdast-util-to-string(node)` yields heading text

---

### 2.3 Import Node (Injected)

**Type**: `import` (mdx)

**Structure**:

```typescript
interface ImportNode {
  type: "import";
  value: string; // Full import statement
}
```

**Generated Value**:

```typescript
"import PDFViewer from '@site/src/components/PDFViewer';";
```

---

### 2.4 JSX Node (Injected Component)

**Type**: `jsx` (mdx)

**Structure**:

```typescript
interface JSXNode {
  type: "jsx";
  value: string; // JSX code as string
}
```

**Generated Value** (local path):

```jsx
<PDFViewer
  src="/slides/chapter-02-slides.pdf"
  title="Chapter 2: The AI Turning Point"
  height={700}
/>
```

**Generated Value** (cloud URL):

```jsx
<PDFViewer
  src="https://cdn.example.com/chapter-02.pdf"
  title="Chapter 2: The AI Turning Point"
  height={700}
/>
```

---

### 2.5 Heading Node (Injected Section Title)

**Type**: `heading` (mdast)

**Structure**:

```typescript
interface HeadingNode {
  type: "heading";
  depth: 2; // H2
  children: [{ type: "text"; value: "📊 Chapter Slides" }];
}
```

**Generated Output**:

```markdown
## 📊 Chapter Slides
```

---

## 3. TypeScript Interfaces

### 3.1 Plugin Configuration

```typescript
/**
 * Remark plugin configuration (currently no options)
 */
interface PluginOptions {
  // Future: Add options if needed (e.g., custom heading text)
}
```

---

### 3.2 Transformer Context

```typescript
/**
 * Context passed to AST transformer
 */
interface TransformerContext {
  /** Markdown AST (mutable) */
  tree: Root;

  /** File metadata */
  file: VFile;

  /** Parsed frontmatter */
  frontMatter: FrontMatter;

  /** File path (for logging) */
  filePath: string;
}
```

---

### 3.3 Injection Result

```typescript
/**
 * Result of slides injection attempt
 */
interface InjectionResult {
  /** Whether injection occurred */
  injected: boolean;

  /** Reason if injection skipped */
  reason?: "no-slides-field" | "no-target-heading" | "error";

  /** Error details (if applicable) */
  error?: Error;
}
```

---

### 3.4 Path Detection

```typescript
/**
 * Detected path type
 */
type PathType = "local" | "url";

/**
 * Normalized path information
 */
interface NormalizedPath {
  /** Original path from frontmatter */
  original: string;

  /** Detected type */
  type: PathType;

  /** Normalized path (with leading slash for local) */
  normalized: string;
}
```

**Detection Logic**:

```typescript
function detectPathType(path: string): PathType {
  return path.startsWith("http://") || path.startsWith("https://")
    ? "url"
    : "local";
}

function normalizePath(path: string, type: PathType): string {
  if (type === "url") {
    return path; // Use as-is
  }
  // Ensure leading slash for local paths
  return path.startsWith("/") ? path : `/${path}`;
}
```

---

## 4. State Transitions

### Document Processing Flow

```
1. [Docusaurus Build Start]
   ↓
2. [Load Markdown File]
   ↓
3. [Parse Frontmatter] → FrontMatter object
   ↓
4. [Parse Markdown to AST] → Root node with children
   ↓
5. [Apply Remark Plugins]
   ↓
6. [remark-slides-metadata] → Check frontMatter.slides
   |
   ├─ slides: undefined → Skip (no injection)
   |
   ├─ slides: string →
   |    ├─ Detect path type (local vs URL)
   |    ├─ Find "What You'll Learn" heading
   |    |   ├─ Found → Inject nodes after heading
   |    |   └─ Not found → Warn, skip injection
   |    └─ Return modified AST
   |
   └─ Invalid slides → Warn, skip injection
   ↓
7. [Convert AST to MDX]
   ↓
8. [Compile MDX to React]
   ↓
9. [Render Page]
```

---

## 5. Entity Relationships

### Conceptual Model

```
┌─────────────────────────────────────────────────────────┐
│ Chapter README (Markdown File)                           │
│ ┌─────────────────────────────────────────────────────┐ │
│ │ Frontmatter (YAML)                                   │ │
│ │ ┌─────────────────┐                                 │ │
│ │ │ slides: string? │ ─────┐                          │ │
│ │ └─────────────────┘      │                          │ │
│ └──────────────────────────┼──────────────────────────┘ │
│                            │                            │
│ ┌──────────────────────────┼──────────────────────────┐ │
│ │ Markdown Content         │                          │ │
│ │                          ▼                          │ │
│ │  ## What You'll Learn  ◄────── INJECTION POINT     │ │
│ │                                                      │ │
│ │  [Plugin injects here]                              │ │
│ │  ┌────────────────────────────────────────────┐    │ │
│ │  │ ## 📊 Chapter Slides                       │    │ │
│ │  │ <PDFViewer src="..." title="..." />        │    │ │
│ │  └────────────────────────────────────────────┘    │ │
│ │                                                      │ │
│ │  ## Lesson Content...                               │ │
│ └──────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
        ┌──────────────────────────────────────┐
        │ PDFViewer Component (React)          │
        │ ┌──────────────────────────────────┐ │
        │ │ Props:                           │ │
        │ │  - src: string (path or URL)     │ │
        │ │  - title: string (chapter title) │ │
        │ │  - height: number (default 700)  │ │
        │ └──────────────────────────────────┘ │
        └──────────────────────────────────────┘
                           │
                           ▼
        ┌──────────────────────────────────────┐
        │ PDF File (Local or Cloud)            │
        │  - Local: /static/slides/*.pdf       │
        │  - Cloud: https://cdn.com/*.pdf      │
        └──────────────────────────────────────┘
```

---

## 6. Validation Rules

### Frontmatter Validation

| Rule                   | Check                        | Action                    |
| ---------------------- | ---------------------------- | ------------------------- |
| `slides` is string     | `typeof slides === 'string'` | ✅ Proceed                |
| `slides` is non-empty  | `slides.trim().length > 0`   | ✅ Proceed                |
| `slides` is undefined  | `!slides`                    | ✅ Skip injection (valid) |
| `slides` is other type | `typeof slides !== 'string'` | ⚠️ Warn, skip injection   |

### Path Validation

| Rule          | Check                                                             | Action                                              |
| ------------- | ----------------------------------------------------------------- | --------------------------------------------------- |
| Local path    | `!slides.startsWith('http')`                                      | Normalize with leading `/`                          |
| URL           | `slides.startsWith('http://') \|\| slides.startsWith('https://')` | Use as-is                                           |
| Malformed URL | URL parse fails                                                   | ⚠️ Warn (but still inject - browser handles errors) |

### AST Validation

| Rule                       | Check                            | Action                            |
| -------------------------- | -------------------------------- | --------------------------------- |
| "What You'll Learn" exists | Heading found with matching text | ✅ Inject after heading           |
| Heading not found          | No matching heading              | ⚠️ Warn, skip injection           |
| Import already exists      | Check for existing import        | ✅ Skip import injection (dedupe) |

---

## Summary

**Key Entities**:

1. **Frontmatter** (`slides: string?`) - Metadata in chapter README
2. **AST Nodes** (yaml, heading, import, jsx) - Build-time structures
3. **PDFViewer Component** (React) - Existing renderer
4. **PDF Files** (local/cloud) - Actual slide content

**Relationships**:

- Frontmatter → Drives injection decision
- Heading ("What You'll Learn") → Injection landmark
- Generated JSX → References PDFViewer component
- PDFViewer → Renders PDF from local/cloud source

**State Flow**:

- Parse → Detect → Find → Inject → Render

**Next Phase**: Proceed to quickstart.md (content creator usage guide)
