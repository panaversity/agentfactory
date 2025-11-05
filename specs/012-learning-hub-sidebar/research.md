# Research: Learning Hub Sidebar

**Feature**: 012-learning-hub-sidebar  
**Date**: 2025-11-06  
**Purpose**: Resolve technical unknowns and establish implementation patterns

## Research Tasks Completed

### 1. Google Gemini API Integration Strategy

**Decision**: Use `@google/generative-ai` SDK with gemini-2.0-flash model

**Rationale**:
- Official Google SDK provides TypeScript support with full type safety
- gemini-2.0-flash offers best balance: fast (<3s responses), cost-effective, high quality
- Native streaming support via `generateContentStream()` method
- Free tier: 15 RPM, 1M TPM sufficient for educational use case
- Excellent at educational content understanding and generation

**Alternatives Considered**:
- **Direct REST API**: More control but requires manual request building, error handling, streaming implementation
- **OpenAI GPT-4**: Higher quality but slower (4-8s typical), more expensive, tighter rate limits
- **Anthropic Claude**: Excellent reasoning but no official streaming in browser, higher latency
- **Local models (Ollama)**: Privacy benefits but requires infrastructure, slower responses, inconsistent quality

**Implementation Pattern**:
```typescript
import { GoogleGenerativeAI } from '@google/generative-ai';

const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);
const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash' });

// Streaming for chat
const result = await model.generateContentStream({
  contents: [{ role: 'user', parts: [{ text: prompt }] }]
});

for await (const chunk of result.stream) {
  const chunkText = chunk.text();
  // Update UI incrementally
}
```

**Key References**:
- [Gemini API Docs](https://ai.google.dev/gemini-api/docs)
- [SDK Reference](https://ai.google.dev/gemini-api/docs/get-started/tutorial?lang=node)

---

### 2. Docusaurus Theme Swizzling Strategy

**Decision**: Swizzle `DocRoot` component to inject LearningHub sidebar

**Rationale**:
- `DocRoot` wraps all documentation pages, perfect injection point
- Maintains Docusaurus upgrade compatibility (safe swizzle)
- Allows conditional rendering based on route/page type
- Provides access to Docusaurus context (router, metadata, theme)
- Non-invasive: doesn't break existing layout or navigation

**Alternatives Considered**:
- **Plugin approach**: More complex, requires plugin lifecycle management, overkill for UI feature
- **Swizzle Layout**: Too broad, affects all pages including non-docs (homepage, blog)
- **Wrapper component**: Requires manual integration in every MDX file, not scalable
- **Script injection**: Difficult to integrate with React state, loses type safety

**Implementation Pattern**:
```bash
# Swizzle command
npm run swizzle @docusaurus/theme-classic DocRoot -- --wrap

# This creates: src/theme/DocRoot.tsx
```

```typescript
// src/theme/DocRoot.tsx
import React, { lazy, Suspense } from 'react';
import DocRoot from '@theme-original/DocRoot';
import { useLocation } from '@docusaurus/router';

const LearningHub = lazy(() => import('../LearningHub'));

export default function DocRootWrapper(props) {
  const location = useLocation();
  const showLearningHub = location.pathname.startsWith('/docs/');
  
  return (
    <>
      <DocRoot {...props} />
      {showLearningHub && (
        <Suspense fallback={null}>
          <LearningHub />
        </Suspense>
      )}
    </>
  );
}
```

**Key References**:
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [Theme Architecture](https://docusaurus.io/docs/advanced/client#theme-components)

---

### 3. MDX Content Extraction from Docusaurus Pages

**Decision**: Use DOM traversal with `.markdown` class selector + text sanitization

**Rationale**:
- Docusaurus renders MDX to HTML with predictable structure
- Main content wrapped in `.markdown` or `.theme-doc-markdown` classes
- DOM API (`querySelector`, `textContent`) provides clean text extraction
- Handles React components rendered within MDX (code blocks, callouts, tabs)
- Works client-side without build-time processing

**Alternatives Considered**:
- **Raw MDX parsing**: Requires access to source files, breaks with dynamic imports, not available at runtime
- **Remark/Rehype plugins**: Build-time only, can't handle user navigation
- **Content API**: Not exposed by Docusaurus for client-side access
- **Full page innerHTML**: Includes navigation, headers, footers - too noisy

**Implementation Pattern**:
```typescript
function extractPageContent(): string {
  const article = document.querySelector('article.markdown') ||
                  document.querySelector('.theme-doc-markdown');
  
  if (!article) return '';
  
  // Clone to avoid modifying DOM
  const clone = article.cloneNode(true) as HTMLElement;
  
  // Remove code blocks, navigation elements
  clone.querySelectorAll('pre, nav, .table-of-contents').forEach(el => el.remove());
  
  // Extract text
  const text = clone.textContent || '';
  
  // Clean whitespace
  return text.replace(/\s+/g, ' ').trim();
}

function extractMetadata() {
  const title = document.querySelector('h1')?.textContent || '';
  const headings = Array.from(document.querySelectorAll('h2, h3'))
    .map(h => h.textContent?.trim() || '');
  
  return { title, headings, url: window.location.pathname };
}
```

**Key References**:
- [Docusaurus HTML Structure](https://github.com/facebook/docusaurus/tree/main/packages/docusaurus-theme-classic/src/theme)
- [MDX Runtime](https://mdxjs.com/docs/using-mdx/)

---

### 4. localStorage Schema Design with Versioning

**Decision**: Namespaced keys with schema version and migration support

**Rationale**:
- Prevents key collisions with other features
- Schema versioning enables safe data migrations
- JSON structure allows complex nested data
- Separate keys for different data types enable targeted reads/writes
- Quota management strategy handles storage limits gracefully

**Schema Design**:
```typescript
// Schema version for migrations
const SCHEMA_VERSION = 1;

// Namespaced keys
const STORAGE_KEYS = {
  CHAT_HISTORY: 'learningHub_chat_v1',      // Session-only
  HIGHLIGHTS: 'learningHub_highlights_v1',   // Persistent
  PROGRESS: 'learningHub_progress_v1',       // Persistent
  KEY_CONCEPTS: 'learningHub_concepts_v1',   // Cache with TTL
  RELATED_TOPICS: 'learningHub_topics_v1',   // Cache with TTL
  ERROR_LOG: 'learningHub_errors_v1',        // Last 50 errors
  SCHEMA_VERSION: 'learningHub_schema_version'
};

// Data structures
interface StorageData {
  version: number;
  timestamp: number;
  data: any;
}

interface CachedContent {
  contentHash: string;  // MD5 of page content
  cachedAt: number;     // Unix timestamp
  expiresAt: number;    // timestamp + 7 days
  data: any;
}
```

**Migration Strategy**:
```typescript
function migrateIfNeeded() {
  const currentVersion = parseInt(
    localStorage.getItem(STORAGE_KEYS.SCHEMA_VERSION) || '0'
  );
  
  if (currentVersion < SCHEMA_VERSION) {
    // Run migrations
    if (currentVersion < 1) {
      // Migration from unversioned to v1
      // ... transform data ...
    }
    localStorage.setItem(STORAGE_KEYS.SCHEMA_VERSION, SCHEMA_VERSION.toString());
  }
}
```

**Quota Management**:
```typescript
function handleQuotaExceeded() {
  // Priority: Remove oldest cached content first
  const cacheKeys = [STORAGE_KEYS.KEY_CONCEPTS, STORAGE_KEYS.RELATED_TOPICS];
  
  for (const key of cacheKeys) {
    const data = JSON.parse(localStorage.getItem(key) || '{}');
    // Remove entries older than 3 days (instead of 7)
    const threeDaysAgo = Date.now() - (3 * 24 * 60 * 60 * 1000);
    Object.keys(data).forEach(pageUrl => {
      if (data[pageUrl].cachedAt < threeDaysAgo) {
        delete data[pageUrl];
      }
    });
    localStorage.setItem(key, JSON.stringify(data));
  }
}
```

**Key References**:
- [Web Storage API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Storage_API)
- [Storage Quotas](https://web.dev/storage-for-the-web/)

---

### 5. Content Hashing Strategy for Cache Invalidation

**Decision**: MD5 hash of extracted page content using crypto-js library

**Rationale**:
- MD5 provides fast hashing suitable for cache keys (not cryptographic security)
- crypto-js is lightweight (35KB gzipped) with no native dependencies
- Deterministic: same content always produces same hash
- Collision risk negligible for this use case (<10K pages)
- Widely supported across browsers

**Alternatives Considered**:
- **SHA-256 via Web Crypto API**: Slower, overkill for cache invalidation, async complexity
- **Simple string length check**: Misses subtle content changes (typo fixes)
- **Last-modified timestamp**: Not accessible in static site, unreliable
- **Content snapshot storage**: Wastes localStorage quota, complex comparison

**Implementation Pattern**:
```typescript
import CryptoJS from 'crypto-js';

function generateContentHash(content: string): string {
  return CryptoJS.MD5(content).toString();
}

function shouldRegenerateCache(
  pageUrl: string,
  currentContent: string
): boolean {
  const cached = getCachedData(pageUrl);
  
  if (!cached) return true;  // No cache
  
  const currentHash = generateContentHash(currentContent);
  const SEVEN_DAYS = 7 * 24 * 60 * 60 * 1000;
  const age = Date.now() - cached.cachedAt;
  
  // Regenerate if content changed OR cache older than 7 days
  return cached.contentHash !== currentHash || age > SEVEN_DAYS;
}
```

**Cache Structure**:
```typescript
interface CachedKeyConcepts {
  [pageUrl: string]: {
    contentHash: string;
    cachedAt: number;
    concepts: Array<{
      title: string;
      description: string;
      sectionId: string;
    }>;
  };
}
```

**Key References**:
- [crypto-js Documentation](https://github.com/brix/crypto-js)
- [Cache Invalidation Strategies](https://web.dev/http-cache/)

---

### 6. Text Selection Detection and Highlighting

**Decision**: Use `window.getSelection()` API with `mouseup` event + range preservation

**Rationale**:
- Native browser API, no dependencies
- Cross-browser compatible (Chrome, Firefox, Safari, Edge)
- Provides precise text range information (start/end nodes, offsets)
- Can serialize/deserialize ranges for persistence
- Supports multi-element selections (spans paragraphs)

**Implementation Pattern**:
```typescript
// Detection
function handleTextSelection() {
  document.addEventListener('mouseup', () => {
    const selection = window.getSelection();
    if (!selection || selection.isCollapsed) return;
    
    const selectedText = selection.toString().trim();
    if (selectedText.length < 10) return;  // Minimum length
    
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();
    
    // Show popup near selection
    showHighlightPopup({
      text: selectedText,
      position: { x: rect.left, y: rect.bottom },
      range: serializeRange(range)
    });
  });
}

// Serialization for persistence
function serializeRange(range: Range): SerializedRange {
  return {
    startContainerPath: getXPath(range.startContainer),
    startOffset: range.startOffset,
    endContainerPath: getXPath(range.endContainer),
    endOffset: range.endOffset
  };
}

// Restoration for re-highlighting
function restoreHighlight(saved: SavedHighlight) {
  const range = deserializeRange(saved.range);
  if (!range) return null;
  
  const span = document.createElement('span');
  span.className = 'learning-hub-highlight';
  span.style.backgroundColor = saved.backgroundColor || '#fff3cd';
  span.onclick = () => showExplanation(saved.id);
  
  range.surroundContents(span);
  return span;
}
```

**XPath Helper** (for stable node references):
```typescript
function getXPath(node: Node): string {
  const parts: string[] = [];
  let current: Node | null = node;
  
  while (current && current !== document.body) {
    if (current.nodeType === Node.ELEMENT_NODE) {
      const element = current as Element;
      let index = 1;
      let sibling = element.previousElementSibling;
      
      while (sibling) {
        if (sibling.tagName === element.tagName) index++;
        sibling = sibling.previousElementSibling;
      }
      
      parts.unshift(`${element.tagName.toLowerCase()}[${index}]`);
    }
    current = current.parentNode;
  }
  
  return '/' + parts.join('/');
}
```

**Key References**:
- [Selection API](https://developer.mozilla.org/en-US/docs/Web/API/Selection)
- [Range API](https://developer.mozilla.org/en-US/docs/Web/API/Range)

---

### 7. Error Boundary and Logging Strategy

**Decision**: React Error Boundaries + console logging + localStorage error ring buffer

**Rationale**:
- React Error Boundaries catch component tree errors without crashing app
- Console logging provides developer visibility during development
- localStorage ring buffer (last 50 errors) gives users troubleshooting data
- Structured error format enables pattern analysis
- No external dependencies or privacy concerns

**Implementation Pattern**:
```typescript
// Error Boundary Component
class LearningHubErrorBoundary extends React.Component<Props, State> {
  state = { hasError: false, error: null };
  
  static getDerivedStateFromError(error: Error) {
    return { hasError: true, error };
  }
  
  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    logError({
      type: 'REACT_ERROR',
      component: 'LearningHub',
      message: error.message,
      stack: error.stack,
      componentStack: errorInfo.componentStack
    });
  }
  
  render() {
    if (this.state.hasError) {
      return (
        <div className="learning-hub-error">
          <p>Learning Hub encountered an error.</p>
          <button onClick={() => this.setState({ hasError: false })}>
            Retry
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}

// Error Logger Service
interface ErrorLogEntry {
  timestamp: number;
  type: string;
  component: string;
  message: string;
  stack?: string;
  metadata?: any;
}

function logError(entry: Omit<ErrorLogEntry, 'timestamp'>) {
  const error: ErrorLogEntry = {
    timestamp: Date.now(),
    ...entry
  };
  
  // Console logging
  console.error('[LearningHub]', error);
  
  // localStorage ring buffer
  const key = 'learningHub_errors_v1';
  const stored = JSON.parse(localStorage.getItem(key) || '[]');
  stored.push(error);
  
  // Keep only last 50
  if (stored.length > 50) {
    stored.shift();
  }
  
  try {
    localStorage.setItem(key, JSON.stringify(stored));
  } catch (e) {
    // Quota exceeded - clear old errors
    stored.splice(0, 25);
    localStorage.setItem(key, JSON.stringify(stored));
  }
}

// User-accessible error viewer (dev tools or UI)
function getErrorLog(): ErrorLogEntry[] {
  const key = 'learningHub_errors_v1';
  return JSON.parse(localStorage.getItem(key) || '[]');
}

function clearErrorLog() {
  localStorage.removeItem('learningHub_errors_v1');
}
```

**Error Categories**:
- `REACT_ERROR`: Component errors caught by Error Boundary
- `API_ERROR`: Gemini API failures (timeout, rate limit, network)
- `STORAGE_ERROR`: localStorage quota exceeded or unavailable
- `PARSE_ERROR`: Content extraction or JSON parsing failures
- `VALIDATION_ERROR`: Invalid data or state inconsistencies

**Key References**:
- [Error Boundaries](https://react.dev/reference/react/Component#catching-rendering-errors-with-an-error-boundary)
- [Console API](https://developer.mozilla.org/en-US/docs/Web/API/Console)

---

## Summary of Decisions

| Area | Decision | Key Benefit |
|------|----------|-------------|
| AI Provider | Google Gemini 2.0 Flash | Fast, cost-effective, streaming support |
| Integration | Docusaurus DocRoot swizzling | Non-invasive, upgrade-safe, lazy loadable |
| Content Extraction | DOM traversal with `.markdown` selector | Client-side, handles MDX/React components |
| Storage Schema | Namespaced keys with versioning | Migration support, quota management |
| Cache Invalidation | MD5 content hash + 7-day TTL | Detects changes, ensures freshness |
| Text Selection | Native Selection API + XPath | No dependencies, cross-browser, persistent |
| Error Handling | Error Boundaries + console + localStorage | Developer visibility, user troubleshooting |

All technical unknowns resolved. Ready for Phase 1: Design & Contracts.
