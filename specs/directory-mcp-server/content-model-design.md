# Directory MCP Server: Content Model Design

**Version**: 1.0 (Refined with Headless CMS Patterns)
**Date**: 2025-11-21
**Status**: Design Refinement

---

## Executive Summary

This document refines the **Directory MCP Server** concept by incorporating proven **headless CMS patterns** from industry leaders (Contentful, Strapi, DatoCMS).

**Core Refinement**: Move from "files with metadata" to **"Content as Data"** — structured, queryable, schema-validated content that can be consumed by AI agents, websites, mobile apps, etc.

---

## Headless CMS Patterns Applied to Directory MCP

### Pattern 1: Content Modeling (Schema-First)

**From Headless CMS Research**:

> "In a headless CMS, the content model creates a schema that structures and validates the content. A content model consists of content types and fields."

**Applied to Directory MCP**:

Instead of:

```
❌ tutorsgpt/chapters/05/lesson-01.md  (just a markdown file)
```

We have:

```
✅ Content Type: Lesson
   Fields:
   - id: string
   - title: string
   - content: rich_text
   - duration: number
   - proficiency: enum(A1, A2, B1, B2, C1, C2)
   - skills_applied: reference(Skill)[]
   - learning_objectives: string[]
   - metadata: object
```

---

### Pattern 2: Separation of Content and Presentation

**From Headless CMS Research**:

> "Fundamental principles of headless content management emphasize reusability and scalability through clean separation between content and visual representation."

**Applied to Directory MCP**:

**Content Layer** (Storage):

```json
{
  "id": "chapter-05-lesson-01",
  "type": "lesson",
  "attributes": {
    "title": "Introduction to Claude Code",
    "content": "...",
    "duration": 10,
    "proficiency": "A2"
  },
  "relationships": {
    "chapter": { "id": "chapter-05" },
    "skills": [{ "id": "learning-objectives" }, { "id": "concept-scaffolding" }]
  }
}
```

**Presentation Layer** (Consumers):

- **AI Agents**: Query via MCP, use for reference/generation
- **Website**: Render as HTML with styling
- **Mobile App**: Display in native UI
- **PDF Export**: Format for printing
- **Audio**: Text-to-speech for accessibility

**Same content, multiple presentations.**

---

### Pattern 3: API-First Architecture

**From Headless CMS Research**:

> "Contentful is an enterprise-grade headless CMS built for performance at scale, with structured content modeling and API-first architecture."

**Applied to Directory MCP**:

**Content API** (MCP Resources):

```typescript
// Query content by type
GET /api/content?type=lesson&chapter=05

// Response (JSON:API format)
{
  "data": [
    {
      "id": "lesson-05-01",
      "type": "lesson",
      "attributes": { ... },
      "relationships": { ... }
    }
  ],
  "included": [
    { "type": "chapter", "id": "chapter-05", ... },
    { "type": "skill", "id": "learning-objectives", ... }
  ]
}

// Get single content item
GET /api/content/lesson-05-01?include=chapter,skills

// Search content
GET /api/content?search=python&type=lesson

// Filter by proficiency
GET /api/content?type=lesson&proficiency=A2
```

---

### Pattern 4: Content Types & Fields

**From Headless CMS Research**:

> "A content model consists of chunks we call content types, and is a collection of content types. A content type consists of fields."

**Applied to Directory MCP**:

#### Content Types for "Book" Directory

**1. Book (Root)**

```typescript
{
  type: "book",
  fields: {
    id: { type: "string", required: true, unique: true },
    title: { type: "string", required: true },
    description: { type: "text" },
    author: { type: "string" },
    version: { type: "string" },
    language: { type: "string", default: "en" },
    tags: { type: "string[]" },
    parts: { type: "reference[]", to: "part" }
  }
}
```

**2. Part**

```typescript
{
  type: "part",
  fields: {
    id: { type: "string", required: true },
    number: { type: "number", required: true },
    title: { type: "string", required: true },
    description: { type: "text" },
    book: { type: "reference", to: "book" },
    chapters: { type: "reference[]", to: "chapter" }
  }
}
```

**3. Chapter**

```typescript
{
  type: "chapter",
  fields: {
    id: { type: "string", required: true },
    number: { type: "number", required: true },
    title: { type: "string", required: true },
    description: { type: "text" },
    proficiency: { type: "enum", values: ["A1", "A2", "B1", "B2", "C1", "C2"] },
    duration: { type: "number" }, // minutes
    part: { type: "reference", to: "part" },
    lessons: { type: "reference[]", to: "lesson" },
    prerequisites: { type: "reference[]", to: "chapter" },
    learning_objectives: { type: "string[]" },
    skills_applied: { type: "reference[]", to: "skill" }
  }
}
```

**4. Lesson**

```typescript
{
  type: "lesson",
  fields: {
    id: { type: "string", required: true },
    number: { type: "number", required: true },
    title: { type: "string", required: true },
    content: { type: "rich_text", required: true },
    duration: { type: "number" },
    proficiency: { type: "enum", values: ["A1", "A2", "B1", "B2", "C1", "C2"] },
    chapter: { type: "reference", to: "chapter" },
    learning_objectives: { type: "string[]" },
    skills_applied: { type: "reference[]", to: "skill" },
    code_examples: { type: "object[]" },
    exercises: { type: "reference[]", to: "exercise" },
    metadata: { type: "object" }
  }
}
```

**5. Skill**

```typescript
{
  type: "skill",
  fields: {
    id: { type: "string", required: true },
    name: { type: "string", required: true },
    description: { type: "text" },
    category: { type: "enum", values: ["pedagogical", "technical", "meta"] },
    persona: { type: "text" },
    questions: { type: "string[]" },
    principles: { type: "string[]" }
  }
}
```

**6. Exercise**

```typescript
{
  type: "exercise",
  fields: {
    id: { type: "string", required: true },
    title: { type: "string", required: true },
    description: { type: "text" },
    type: { type: "enum", values: ["practice", "challenge", "project"] },
    difficulty: { type: "enum", values: ["beginner", "intermediate", "advanced"] },
    instructions: { type: "rich_text" },
    starter_code: { type: "code" },
    solution: { type: "code" },
    validation_criteria: { type: "string[]" }
  }
}
```

---

### Pattern 5: Rich Content Fields

**Field Types** (inspired by Contentful/Strapi):

```typescript
type FieldType =
  | "string"           // Short text
  | "text"             // Long text
  | "rich_text"        // Markdown/HTML
  | "number"
  | "boolean"
  | "date"
  | "enum"             // Fixed values
  | "reference"        // Link to another content type
  | "reference[]"      // Array of references
  | "object"           // JSON object
  | "object[]"         // Array of objects
  | "asset"            // File (image, video, pdf)
  | "code"             // Code block with syntax highlighting
  | "location"         // Geographic coordinates
  | "json"             // Raw JSON

// Rich text example
{
  type: "rich_text",
  format: "markdown",
  value: "# Introduction\n\nThis lesson covers...",
  metadata: {
    wordCount: 1250,
    readingTime: 8,
    headings: ["Introduction", "Core Concepts", "Try With AI"]
  }
}

// Code example
{
  type: "code",
  language: "python",
  value: "def hello():\n    print('Hello, World!')",
  metadata: {
    tested: true,
    framework: "pytest",
    validatedAt: "2025-11-21T10:00:00Z"
  }
}

// Reference example
{
  type: "reference",
  to: "skill",
  id: "learning-objectives",
  resolved: {  // Optional: Include full object
    id: "learning-objectives",
    name: "Learning Objectives",
    category: "pedagogical"
  }
}
```

---

## Content Storage Format

### Option 1: JSON Files (Structured)

**Directory structure**:

```
tutorsgpt/
├── content/
│   ├── book.json                      # Book metadata
│   ├── parts/
│   │   ├── part-01.json
│   │   ├── part-02.json
│   ├── chapters/
│   │   ├── chapter-01.json
│   │   ├── chapter-02.json
│   ├── lessons/
│   │   ├── lesson-01-01.json
│   │   ├── lesson-01-02.json
│   ├── skills/
│   │   ├── learning-objectives.json
│   │   ├── concept-scaffolding.json
│   └── exercises/
│       ├── exercise-001.json
├── assets/
│   ├── images/
│   ├── videos/
│   └── downloads/
└── index.json                         # Content index
```

**Example lesson file** (`lessons/lesson-05-01.json`):

```json
{
  "id": "lesson-05-01",
  "type": "lesson",
  "number": 1,
  "title": "Introduction to Claude Code",
  "content": {
    "format": "markdown",
    "value": "# Introduction to Claude Code\n\nClaude Code is...",
    "metadata": {
      "wordCount": 1250,
      "readingTime": 8
    }
  },
  "duration": 10,
  "proficiency": "A2",
  "chapter": {
    "type": "reference",
    "to": "chapter",
    "id": "chapter-05"
  },
  "learning_objectives": [
    "Understand what Claude Code is",
    "Install Claude Code CLI",
    "Run first AI-assisted coding session"
  ],
  "skills_applied": [
    { "type": "reference", "to": "skill", "id": "learning-objectives" },
    { "type": "reference", "to": "skill", "id": "concept-scaffolding" }
  ],
  "code_examples": [
    {
      "language": "bash",
      "title": "Install Claude Code",
      "code": "npm install -g @anthropic-ai/claude-code",
      "tested": true
    }
  ],
  "metadata": {
    "created_at": "2025-11-15T10:00:00Z",
    "updated_at": "2025-11-20T14:30:00Z",
    "author": "Panaversity",
    "constitutional_compliance": true,
    "validation_status": "approved"
  }
}
```

**Benefits**:

- ✅ **Queryable**: Can search/filter by any field
- ✅ **Structured**: Schema validation ensures consistency
- ✅ **Flexible**: Add new fields without breaking existing content
- ✅ **Cacheable**: Easy to cache in memory/Redis
- ✅ **Versionable**: Git-friendly (JSON diffs are readable)

---

### Option 2: Hybrid (Markdown + Frontmatter + JSON Index)

**Keep markdown for content, but add structured frontmatter**:

**File**: `lessons/lesson-05-01.md`

```yaml
---
id: lesson-05-01
type: lesson
number: 1
title: Introduction to Claude Code
duration: 10
proficiency: A2
chapter: chapter-05
learning_objectives:
  - Understand what Claude Code is
  - Install Claude Code CLI
  - Run first AI-assisted coding session
skills_applied:
  - learning-objectives
  - concept-scaffolding
metadata:
  created_at: 2025-11-15T10:00:00Z
  updated_at: 2025-11-20T14:30:00Z
  constitutional_compliance: true
---

# Introduction to Claude Code

Claude Code is an AI-native development environment...

## What You'll Learn

In this lesson, you'll discover...

## Try With AI

[Try with AI activity...]
```

**Plus JSON index** (`index.json`):

```json
{
  "version": "1.0",
  "updated_at": "2025-11-21T10:00:00Z",
  "content_types": {
    "lessons": [
      {
        "id": "lesson-05-01",
        "file": "lessons/lesson-05-01.md",
        "title": "Introduction to Claude Code",
        "proficiency": "A2",
        "chapter": "chapter-05"
      }
    ],
    "chapters": [...],
    "skills": [...]
  }
}
```

**Benefits**:

- ✅ **Human-readable**: Markdown content is easy to edit
- ✅ **Queryable**: Index provides structured access
- ✅ **Flexible**: Frontmatter for metadata, markdown for content
- ⚠️ **Hybrid complexity**: Two sources of truth (frontmatter + index)

---

## Query API Design

### REST-like API (via MCP Tools)

**1. List Content**

```typescript
// List all lessons
GET /content?type=lesson

// List lessons for chapter 5
GET /content?type=lesson&chapter=chapter-05

// List A2 proficiency content
GET /content?proficiency=A2

// Response (JSON:API format)
{
  "data": [
    {
      "id": "lesson-05-01",
      "type": "lesson",
      "attributes": {
        "title": "Introduction to Claude Code",
        "duration": 10,
        "proficiency": "A2"
      },
      "relationships": {
        "chapter": { "data": { "type": "chapter", "id": "chapter-05" } },
        "skills": { "data": [
          { "type": "skill", "id": "learning-objectives" }
        ]}
      }
    }
  ],
  "meta": {
    "total": 1,
    "page": 1,
    "perPage": 20
  }
}
```

**2. Get Single Content**

```typescript
// Get lesson with relationships
GET /content/lesson-05-01?include=chapter,skills

// Response
{
  "data": {
    "id": "lesson-05-01",
    "type": "lesson",
    "attributes": { ... },
    "relationships": { ... }
  },
  "included": [
    {
      "id": "chapter-05",
      "type": "chapter",
      "attributes": { ... }
    },
    {
      "id": "learning-objectives",
      "type": "skill",
      "attributes": { ... }
    }
  ]
}
```

**3. Search Content**

```typescript
// Full-text search
GET /content/search?q=python+fundamentals

// Search with filters
GET /content/search?q=loops&type=lesson&proficiency=A2

// Response
{
  "data": [
    {
      "id": "lesson-12-03",
      "type": "lesson",
      "attributes": { ... },
      "score": 0.95,
      "highlights": {
        "content": "...Python <em>loops</em> allow you to..."
      }
    }
  ]
}
```

**4. GraphQL-Style Queries** (Advanced)

```graphql
query {
  chapter(id: "chapter-05") {
    title
    lessons {
      title
      duration
      skills {
        name
        category
      }
    }
  }
}
```

---

## MCP Server Implementation

### Resources (Read Operations)

```typescript
// List all content of a type
resources/list?type=lesson&directory=tutorsgpt

// Read single content item
resources/read?uri=dir://tutorsgpt/content/lesson-05-01

// Subscribe to content changes
resources/subscribe?uri=dir://tutorsgpt/content
```

### Tools (Query Operations)

```typescript
// Search across content
tools/call: content_search {
  directory: "tutorsgpt",
  query: "python fundamentals",
  filters: {
    type: "lesson",
    proficiency: "A2"
  }
}

// Get content with relationships
tools/call: content_get {
  directory: "tutorsgpt",
  id: "lesson-05-01",
  include: ["chapter", "skills", "exercises"]
}

// List content by type
tools/call: content_list {
  directory: "tutorsgpt",
  type: "lesson",
  filters: {
    chapter: "chapter-05"
  },
  limit: 20,
  offset: 0
}

// Query content (advanced)
tools/call: content_query {
  directory: "tutorsgpt",
  query: {
    type: "lesson",
    where: {
      proficiency: { in: ["A2", "B1"] },
      duration: { lte: 15 }
    },
    include: ["chapter", "skills"],
    orderBy: { field: "number", direction: "asc" }
  }
}
```

---

## Content Indexing Strategy

### In-Memory Index (Fast Queries)

**On server startup**:

1. Load all content files (JSON or Markdown + frontmatter)
2. Build in-memory index:
   ```typescript
   {
     types: {
       lesson: Map<id, LessonContent>,
       chapter: Map<id, ChapterContent>,
       skill: Map<id, SkillContent>
     },
     indexes: {
       lessonsByChapter: Map<chapterId, lessonId[]>,
       lessonsByProficiency: Map<proficiency, lessonId[]>,
       skillsByCategory: Map<category, skillId[]>
     },
     search: {
       fullText: SearchIndex  // Lunr.js, FlexSearch, etc.
     }
   }
   ```

**Query performance**:

- **Type listing**: O(1) lookup in Map
- **Filtering**: O(n) scan with index optimization
- **Full-text search**: O(log n) with search index
- **Reference resolution**: O(1) lookup in Maps

---

### File Watching (Keep Index Fresh)

**Watch for changes**:

```typescript
// Watch content directory
chokidar.watch("content/**/*.{json,md}").on("change", (path) => {
  // Re-index changed file
  const content = loadContent(path);
  updateIndex(content);

  // Notify subscribers (MCP resources/subscribe)
  notifySubscribers({ uri: `dir://tutorsgpt/content/${content.id}` });
});
```

**Benefits**:

- ✅ Hot reload during development
- ✅ Real-time updates to AI agents
- ✅ No server restart needed

---

## Migration Strategy

### Phase 1: Extract Structured Data from Existing Content

**Current**: Markdown files in `apps/learn-app/docs/`

**Step 1**: Parse existing markdown + frontmatter

```bash
# Script to extract structured data
node scripts/extract-content-model.js \
  --source book-source/docs \
  --output content/ \
  --format json
```

**Step 2**: Generate content JSON files

```
apps/learn-app/docs/05-Claude-Code/01-introduction.md
  ↓
content/lessons/lesson-05-01.json
```

**Step 3**: Build content index

```bash
node scripts/build-index.js --content content/ --output index.json
```

---

### Phase 2: Serve via Directory MCP Server

**Start server**:

```bash
ri-directory serve \
  --directory tutorsgpt \
  --content-path ./content \
  --storage local
```

**Agents can now query**:

```typescript
// Get all Chapter 5 lessons
const lessons = await mcp.callTool("content_list", {
  directory: "tutorsgpt",
  type: "lesson",
  filters: { chapter: "chapter-05" },
});
```

---

### Phase 3: Deploy to R2 (Cloud Storage)

**Upload structured content to R2**:

```bash
ri-directory deploy tutorsgpt \
  --source ./content \
  --storage r2 \
  --bucket panaversity-books \
  --prefix tutorsgpt/
```

**Server configuration**:

```json
{
  "id": "tutorsgpt",
  "storage": {
    "backend": "r2",
    "bucket": "panaversity-books",
    "prefix": "tutorsgpt/content/"
  }
}
```

**Same API, cloud storage backend.**

---

## Comparison: Before vs After

### Before (Filesystem-bound)

```typescript
// Agent needs to know filesystem structure
const lessonPath = "apps/learn-app/docs/05-Claude-Code/01-introduction.md";
const content = await fs.readFile(lessonPath, "utf-8");

// Manual parsing
const frontmatter = extractFrontmatter(content);
const markdown = extractMarkdown(content);

// No structured queries
// Can't filter by proficiency, chapter, skills, etc.
```

### After (Content-as-Data)

```typescript
// Agent queries via MCP
const lessons = await mcp.callTool("content_list", {
  directory: "tutorsgpt",
  type: "lesson",
  filters: {
    chapter: "chapter-05",
    proficiency: "A2",
  },
  include: ["chapter", "skills"],
});

// Structured response
lessons.forEach((lesson) => {
  console.log(lesson.title);
  console.log(lesson.skills.map((s) => s.name));
});

// Storage backend is transparent (local or R2)
```

---

## Success Criteria for Refined Design

**Content Modeling**:

- ✅ All content types defined (Book, Part, Chapter, Lesson, Skill, Exercise)
- ✅ Field schemas validated (required, types, references)
- ✅ Relationships modeled (chapter → lessons, lesson → skills)

**API Quality**:

- ✅ Queryable by any field (type, proficiency, chapter, etc.)
- ✅ Full-text search across content
- ✅ Reference resolution (include related content)
- ✅ Pagination for large result sets

**Performance**:

- ✅ <50ms for indexed queries (type listing, filters)
- ✅ <200ms for full-text search
- ✅ <100ms for single content retrieval with references

**Flexibility**:

- ✅ Supports JSON or Markdown + frontmatter
- ✅ Schema extensible (add new content types, fields)
- ✅ Storage-agnostic (local, R2, S3)

---

## Next Steps

**1. Approve Refined Design**

- Content model (types, fields, relationships)
- Query API (REST-like via MCP tools)
- Storage format (JSON vs Markdown + frontmatter)

**2. Create Implementation Plan**

- Content model schema definitions
- Index builder
- Query engine
- MCP server with content tools

**3. Migration Script**

- Extract structured data from existing markdown
- Generate JSON content files
- Build initial index

**4. Prototype**

- Implement content model
- Build in-memory index
- Expose via MCP tools
- Test with tutorsgpt

---

**Document Status**: Refined Design
**Approval Required**: Content model, API design, storage format
**Next Action**: Get approval, then create implementation plan

---

**END OF DOCUMENT**
