# Data Model: Learning Hub Sidebar

**Feature**: 012-learning-hub-sidebar  
**Date**: 2025-11-06  
**Purpose**: Define data structures, relationships, and validation rules

## Entity Definitions

### 1. ChatMessage

**Purpose**: Represents a single message in the AI chat conversation

**Attributes**:
- `id` (string, required): Unique identifier (UUID v4)
- `role` (enum, required): `'user'` | `'assistant'`
- `content` (string, required): Message text (1-10000 characters)
- `timestamp` (number, required): Unix timestamp in milliseconds
- `pageUrl` (string, required): Context page URL (relative path)
- `tokens` (number, optional): Token count for API usage tracking

**Validation Rules**:
- `id`: Must be valid UUID v4 format
- `role`: Must be exactly 'user' or 'assistant'
- `content`: Not empty, max 10,000 characters
- `timestamp`: Must be positive integer
- `pageUrl`: Must start with '/' (relative path)

**Relationships**:
- Multiple ChatMessages per session
- Session-scoped: cleared on tab close
- Ordered by timestamp ascending

**Storage**:
- Location: Session storage (memory)
- Key: `learningHub_chat_v1`
- Structure: Array of ChatMessage objects
- Max size: 50 messages per session (FIFO cleanup)

**TypeScript Definition**:
```typescript
interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
  pageUrl: string;
  tokens?: number;
}

type ChatHistory = ChatMessage[];
```

---

### 2. Highlight

**Purpose**: Represents a saved text selection with AI explanation

**Attributes**:
- `id` (string, required): Unique identifier (UUID v4)
- `pageUrl` (string, required): Page where highlight was created
- `selectedText` (string, required): The highlighted text content (10-1000 chars)
- `textPosition` (object, required): Serialized Range for re-highlighting
  - `startContainerPath` (string): XPath to start node
  - `startOffset` (number): Character offset in start node
  - `endContainerPath` (string): XPath to end node
  - `endOffset` (number): Character offset in end node
- `explanation` (string, required): AI-generated explanation
- `createdAt` (number, required): Unix timestamp
- `backgroundColor` (string, optional): Highlight color (default: '#fff3cd')

**Validation Rules**:
- `selectedText`: 10-1000 characters, not empty
- `textPosition`: All XPath fields must be non-empty strings
- `textPosition.startOffset` and `endOffset`: Must be non-negative integers
- `explanation`: 1-5000 characters
- `backgroundColor`: Valid CSS color (hex, rgb, rgba, or named)

**Relationships**:
- Multiple Highlights per page
- Persistent across sessions
- No relationship to ChatMessage (independent feature)

**Storage**:
- Location: localStorage (persistent)
- Key: `learningHub_highlights_v1`
- Structure: Map of pageUrl to array of Highlights
- Max size: 100 highlights per page, 1000 total (FIFO per page)

**TypeScript Definition**:
```typescript
interface TextPosition {
  startContainerPath: string;
  startOffset: number;
  endContainerPath: string;
  endOffset: number;
}

interface Highlight {
  id: string;
  pageUrl: string;
  selectedText: string;
  textPosition: TextPosition;
  explanation: string;
  createdAt: number;
  backgroundColor?: string;
}

type HighlightsStorage = {
  [pageUrl: string]: Highlight[];
};
```

---

### 3. QuizQuestion

**Purpose**: Represents a generated quiz question for a page

**Attributes**:
- `id` (string, required): Unique identifier (UUID v4)
- `pageUrl` (string, required): Source page for question
- `question` (string, required): Question text (10-500 chars)
- `choices` (string[], required): Array of 4 answer options
- `correctAnswer` (number, required): Index of correct choice (0-3)
- `explanation` (string, required): Why the answer is correct
- `difficulty` (enum, required): `'easy'` | `'medium'` | `'hard'`
- `generatedAt` (number, required): Unix timestamp

**Validation Rules**:
- `question`: 10-500 characters
- `choices`: Exactly 4 strings, each 1-200 characters, all unique
- `correctAnswer`: Integer 0-3 (valid index into choices array)
- `explanation`: 10-1000 characters
- `difficulty`: Must be 'easy', 'medium', or 'hard'

**Relationships**:
- 3-5 QuizQuestions generated per page per quiz session
- Ephemeral: regenerated on each quiz attempt (not cached)
- Generated from page content (uses current content, not cached)

**Storage**:
- Location: Session storage (memory only)
- Not persisted to localStorage
- Cleared when navigating away from page or clicking "Retake Quiz"

**TypeScript Definition**:
```typescript
type QuizDifficulty = 'easy' | 'medium' | 'hard';

interface QuizQuestion {
  id: string;
  pageUrl: string;
  question: string;
  choices: [string, string, string, string];  // Exactly 4
  correctAnswer: 0 | 1 | 2 | 3;
  explanation: string;
  difficulty: QuizDifficulty;
  generatedAt: number;
}

interface QuizSession {
  pageUrl: string;
  questions: QuizQuestion[];
  userAnswers: (number | null)[];  // Parallel array to questions
  score?: { correct: number; total: number };
}
```

---

### 4. KeyConcept

**Purpose**: Represents an extracted key concept from page content

**Attributes**:
- `id` (string, required): Unique identifier (UUID v4)
- `pageUrl` (string, required): Source page URL
- `title` (string, required): Concept name (5-100 chars)
- `description` (string, required): Brief explanation (10-500 chars)
- `sectionId` (string, optional): HTML element ID to scroll to
- `importance` (number, required): Ranking 1-10 (for display order)
- `contentHash` (string, required): MD5 hash of source page content
- `cachedAt` (number, required): Unix timestamp

**Validation Rules**:
- `title`: 5-100 characters, not empty
- `description`: 10-500 characters
- `sectionId`: If provided, must be valid HTML ID format
- `importance`: Integer 1-10
- `contentHash`: 32-character hex string (MD5 format)

**Relationships**:
- 5-7 KeyConcepts per page (target count)
- Cached with 7-day TTL
- Regenerated if content hash changes OR cache expires

**Storage**:
- Location: localStorage (persistent with TTL)
- Key: `learningHub_concepts_v1`
- Structure: Map of pageUrl to CachedKeyConcepts object

**Cache Invalidation**:
- Regenerate if: `contentHash !== currentHash` OR `age > 7 days`

**TypeScript Definition**:
```typescript
interface KeyConcept {
  id: string;
  pageUrl: string;
  title: string;
  description: string;
  sectionId?: string;
  importance: number;  // 1-10
  contentHash: string;
  cachedAt: number;
}

interface CachedKeyConcepts {
  contentHash: string;
  cachedAt: number;
  expiresAt: number;  // cachedAt + 7 days
  concepts: KeyConcept[];
}

type KeyConceptsStorage = {
  [pageUrl: string]: CachedKeyConcepts;
};
```

---

### 5. RelatedTopic

**Purpose**: Represents a related chapter or section recommendation

**Attributes**:
- `id` (string, required): Unique identifier (UUID v4)
- `pageUrl` (string, required): Source page URL
- `title` (string, required): Topic/chapter title (5-200 chars)
- `url` (string, required): Link to related page (relative path)
- `description` (string, required): Brief summary (10-300 chars)
- `relevanceScore` (number, required): 0.0-1.0 (how related)
- `contentHash` (string, required): MD5 hash of source page
- `cachedAt` (number, required): Unix timestamp

**Validation Rules**:
- `title`: 5-200 characters
- `url`: Must start with '/' (relative path), valid URL format
- `description`: 10-300 characters
- `relevanceScore`: Float 0.0-1.0
- `contentHash`: 32-character hex string

**Relationships**:
- 3-5 RelatedTopics per page
- Cached with 7-day TTL (same strategy as KeyConcepts)
- No circular references (A relates to B doesn't require B relates to A)

**Storage**:
- Location: localStorage (persistent with TTL)
- Key: `learningHub_topics_v1`
- Structure: Map of pageUrl to CachedRelatedTopics

**TypeScript Definition**:
```typescript
interface RelatedTopic {
  id: string;
  pageUrl: string;
  title: string;
  url: string;
  description: string;
  relevanceScore: number;  // 0.0-1.0
  contentHash: string;
  cachedAt: number;
}

interface CachedRelatedTopics {
  contentHash: string;
  cachedAt: number;
  expiresAt: number;
  topics: RelatedTopic[];
}

type RelatedTopicsStorage = {
  [pageUrl: string]: CachedRelatedTopics;
};
```

---

### 6. ProgressRecord

**Purpose**: Represents user's engagement with a page

**Attributes**:
- `pageUrl` (string, required): Unique identifier (page URL)
- `visitCount` (number, required): Number of times viewed
- `totalDuration` (number, required): Cumulative read time in seconds
- `firstVisitedAt` (number, required): Unix timestamp of first visit
- `lastVisitedAt` (number, required): Unix timestamp of most recent visit
- `completed` (boolean, required): Whether marked as "done"

**Validation Rules**:
- `pageUrl`: Non-empty string, starts with '/'
- `visitCount`: Positive integer ≥ 1
- `totalDuration`: Non-negative integer (seconds)
- `firstVisitedAt`: Positive integer, <= lastVisitedAt
- `lastVisitedAt`: Positive integer, >= firstVisitedAt

**Relationships**:
- One ProgressRecord per page (keyed by pageUrl)
- Aggregated for overall progress statistics
- Independent of other entities

**Computed Fields** (not stored):
- `averageDuration`: `totalDuration / visitCount`
- `ageInDays`: `(now - firstVisitedAt) / (24*60*60*1000)`

**Storage**:
- Location: localStorage (persistent)
- Key: `learningHub_progress_v1`
- Structure: Map of pageUrl to ProgressRecord

**TypeScript Definition**:
```typescript
interface ProgressRecord {
  pageUrl: string;
  visitCount: number;
  totalDuration: number;  // seconds
  firstVisitedAt: number;
  lastVisitedAt: number;
  completed: boolean;
}

type ProgressStorage = {
  [pageUrl: string]: ProgressRecord;
};

// Aggregated statistics (computed)
interface ProgressSummary {
  totalPagesVisited: number;
  totalTimeSpent: number;  // seconds
  totalHighlights: number;
  completionPercentage: number;  // 0-100
  streak: number;  // consecutive days with visits
}
```

---

### 7. LearningHubState

**Purpose**: Represents the overall sidebar application state

**Attributes**:
- `isOpen` (boolean, required): Whether sidebar is visible
- `activeTab` (enum, required): Current tab name
- `chatHistory` (ChatMessage[], required): Current session messages
- `savedHighlights` (HighlightsStorage, required): All persisted highlights
- `progressRecords` (ProgressStorage, required): All page progress
- `errorLog` (ErrorLogEntry[], required): Recent errors

**Validation Rules**:
- `activeTab`: Must be one of: 'chat' | 'highlights' | 'quiz' | 'concepts' | 'topics' | 'progress'
- `chatHistory`: Max 50 messages
- `errorLog`: Max 50 entries

**State Management**:
- Managed via React Context + useReducer
- Synced to localStorage on state changes (debounced 500ms)
- Hydrated from localStorage on mount

**TypeScript Definition**:
```typescript
type TabName = 'chat' | 'highlights' | 'quiz' | 'concepts' | 'topics' | 'progress';

interface LearningHubState {
  isOpen: boolean;
  activeTab: TabName;
  chatHistory: ChatMessage[];
  savedHighlights: HighlightsStorage;
  progressRecords: ProgressStorage;
  errorLog: ErrorLogEntry[];
}

// Actions for state updates
type LearningHubAction =
  | { type: 'TOGGLE_SIDEBAR' }
  | { type: 'SET_ACTIVE_TAB'; payload: TabName }
  | { type: 'ADD_CHAT_MESSAGE'; payload: ChatMessage }
  | { type: 'CLEAR_CHAT_HISTORY' }
  | { type: 'ADD_HIGHLIGHT'; payload: Highlight }
  | { type: 'DELETE_HIGHLIGHT'; payload: string }  // highlight ID
  | { type: 'UPDATE_PROGRESS'; payload: ProgressRecord }
  | { type: 'LOG_ERROR'; payload: ErrorLogEntry };
```

---

## Entity Relationships Diagram

```
┌─────────────────┐
│ LearningHubState│ (Root State)
└────────┬────────┘
         │
         ├─── chatHistory: ChatMessage[]      (session-only)
         │
         ├─── savedHighlights ──┬─> Highlight (pageUrl1)
         │                      ├─> Highlight (pageUrl2)
         │                      └─> ...
         │
         ├─── progressRecords ──┬─> ProgressRecord (pageUrl1)
         │                      ├─> ProgressRecord (pageUrl2)
         │                      └─> ...
         │
         └─── errorLog: ErrorLogEntry[]

┌──────────────┐
│ Cache Layer  │
└──────┬───────┘
       │
       ├─── KeyConcepts ──┬─> CachedKeyConcepts (pageUrl1)
       │                  ├─> CachedKeyConcepts (pageUrl2)
       │                  └─> ... (7-day TTL)
       │
       └─── RelatedTopics ──┬─> CachedRelatedTopics (pageUrl1)
                            ├─> CachedRelatedTopics (pageUrl2)
                            └─> ... (7-day TTL)

┌──────────────┐
│ Session Only │
└──────┬───────┘
       │
       └─── QuizSession: { questions, userAnswers, score }
```

---

## Storage Strategy Summary

| Entity | Storage | Persistence | Max Size | Cleanup Strategy |
|--------|---------|-------------|----------|------------------|
| ChatMessage | Session memory | Tab lifetime | 50 messages | FIFO |
| Highlight | localStorage | Persistent | 1000 total | FIFO per page (100/page) |
| QuizQuestion | Session memory | Quiz session | 5 questions | Clear on nav/retake |
| KeyConcept | localStorage | 7-day cache | Unlimited | TTL + quota management |
| RelatedTopic | localStorage | 7-day cache | Unlimited | TTL + quota management |
| ProgressRecord | localStorage | Persistent | Unlimited | Manual clear only |
| LearningHubState | localStorage | Persistent | N/A | Quota management |
| ErrorLogEntry | localStorage | Persistent | 50 entries | FIFO |

**Total localStorage Budget Estimate**:
- Highlights: ~500KB (1000 * 500 bytes avg)
- Progress: ~50KB (500 pages * 100 bytes)
- Cache: ~1-2MB (depends on content, cleaned up by TTL)
- Errors: ~50KB (50 * 1KB avg)
- **Total**: ~2-3MB (well under typical 5-10MB quota)

---

## Data Validation Utilities

```typescript
// Validation helper functions
const validators = {
  isUUID: (str: string): boolean => 
    /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(str),
  
  isMD5: (str: string): boolean => 
    /^[a-f0-9]{32}$/i.test(str),
  
  isValidUrl: (str: string): boolean => 
    str.startsWith('/') && str.length > 1,
  
  isInRange: (num: number, min: number, max: number): boolean =>
    num >= min && num <= max,
  
  isValidColor: (str: string): boolean =>
    /^#[0-9a-f]{6}$/i.test(str) || /^rgb/.test(str) || CSS.supports('color', str)
};

// Type guards
function isChatMessage(obj: any): obj is ChatMessage {
  return (
    typeof obj === 'object' &&
    validators.isUUID(obj.id) &&
    ['user', 'assistant'].includes(obj.role) &&
    typeof obj.content === 'string' &&
    obj.content.length > 0 &&
    typeof obj.timestamp === 'number' &&
    validators.isValidUrl(obj.pageUrl)
  );
}

function isHighlight(obj: any): obj is Highlight {
  return (
    typeof obj === 'object' &&
    validators.isUUID(obj.id) &&
    validators.isValidUrl(obj.pageUrl) &&
    typeof obj.selectedText === 'string' &&
    obj.selectedText.length >= 10 &&
    obj.selectedText.length <= 1000 &&
    typeof obj.textPosition === 'object' &&
    typeof obj.explanation === 'string' &&
    typeof obj.createdAt === 'number'
  );
}

// Add similar type guards for other entities...
```

---

## Migration Strategy

When schema changes are needed:

1. **Version Bumping**: Increment `SCHEMA_VERSION` constant
2. **Migration Function**: Create migration logic for each version increment
3. **Graceful Degradation**: Handle missing/invalid data by resetting affected entities
4. **User Notification**: Inform users when data migration occurs (if significant)

```typescript
function migrateSchema(fromVersion: number, toVersion: number) {
  if (fromVersion < 1 && toVersion >= 1) {
    // Example: v0 -> v1 migration
    // Convert old highlight format to new format with textPosition
    const oldHighlights = JSON.parse(localStorage.getItem('highlights') || '{}');
    const newHighlights: HighlightsStorage = {};
    
    for (const [url, highlights] of Object.entries(oldHighlights)) {
      newHighlights[url] = (highlights as any[]).map(h => ({
        ...h,
        textPosition: h.range || generateDefaultPosition(h.selectedText)
      }));
    }
    
    localStorage.setItem('learningHub_highlights_v1', JSON.stringify(newHighlights));
    localStorage.removeItem('highlights');  // Clean up old key
  }
  
  // Add more migrations as versions increase
  
  localStorage.setItem('learningHub_schema_version', toVersion.toString());
}
```

---

Data model complete and ready for implementation in Phase 2.
