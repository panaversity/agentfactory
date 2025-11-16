# Search Implementation Documentation

## Overview

This document describes the real-time search functionality integrated into the AI Native Software Development book site.

The search indexes two types of content:
1. **Pages** - All markdown files in the sidebar navigation (248 pages)
2. **Headings** - All section headings within pages (4,409 headings)

Total searchable items: **4,657**

## Architecture

The search system consists of three main components:

### 1. Search Index Generation (`scripts/generate-search-index.mjs`)

A Node.js script that:
- Scans all markdown/mdx files in the `docs/` directory
- Extracts metadata (title, description, breadcrumbs, keywords)
- Parses frontmatter using `gray-matter`
- Generates clean searchable text (removes code blocks, HTML, markdown syntax)
- Outputs a JSON search index to `static/search-index.json`

**Usage:**
```bash
npm run search:index
```

**Output:**
- File: `static/search-index.json`
- Format: Array of search result objects
- Size: 4,657 searchable items (248 pages + 4,409 headings)
- File size: ~2.3MB

### 2. Search Context (`src/contexts/SearchContext.tsx`)

A React context provider that:
- Loads the search index on application startup
- Provides fuzzy search functionality
- Implements intelligent ranking algorithm
- Exposes `search()` function and `isLoading` state

**Search Algorithm:**
The search uses weighted fuzzy matching with the following scoring:

| Field | Weight | Priority |
|-------|--------|----------|
| Title | 10x | Highest |
| Keywords | 5x | High |
| Breadcrumb | 3x | Medium |
| Description | 2x | Medium-Low |
| Content | 1x | Low |

**Matching Types:**
1. **Exact match** (1000 points) - Query exactly matches field
2. **Starts with** (500 points) - Field starts with query
3. **Contains** (100 points) - Field contains whole query
4. **Fuzzy match** (10 points per character) - All query characters appear in order

**Features:**
- Multi-word query support
- Case-insensitive matching
- Bonus scoring for matching all words
- Top 20 results returned
- Real-time search (no debouncing needed - instant)

### 3. CustomNavbar Integration

The search UI in `src/components/CustomNavbar/index.tsx`:

**Features:**
- Keyboard shortcut: `Ctrl+K` / `Cmd+K` to open
- Modal overlay with blur effect
- Real-time results as you type
- Keyboard navigation (↑/↓ arrows)
- Enter to navigate to result
- ESC to close modal

**User Experience:**
1. User clicks search bar or presses `Ctrl+K`
2. Modal appears with auto-focused input
3. Search results appear instantly as user types
4. Results show:
   - **Page icon** (📄) for page results
   - **Hash icon** (#) for heading results
   - Title, breadcrumb, and parent page
5. Hover or keyboard navigation highlights results
6. Click or Enter navigates to selected page/heading (with anchor link)

## Build Integration

The search index is automatically generated during the build process:

```json
{
  "scripts": {
    "build": "npm run search:index && docusaurus build",
    "search:index": "node scripts/generate-search-index.mjs"
  }
}
```

**Note:** The search index file (`static/search-index.json`) is gitignored because it's auto-generated during builds.

## File Structure

```
book-source/
├── scripts/
│   └── generate-search-index.mjs      # Index generator
├── src/
│   ├── contexts/
│   │   └── SearchContext.tsx          # Search provider & logic
│   ├── components/
│   │   └── CustomNavbar/
│   │       ├── index.tsx              # Search UI
│   │       └── customNavbar.css       # Search modal styles
│   └── theme/
│       └── Root.tsx                   # SearchProvider wrapper
└── static/
    └── search-index.json              # Generated index (gitignored)
```

## Dependencies

- `gray-matter` (v4.0.3) - Parses markdown frontmatter
- React Context API - State management
- Docusaurus Link - Client-side navigation

## Performance

- **Index Size:** 4,657 searchable items = ~2.3MB JSON
- **Load Time:** ~200ms on initial page load
- **Search Time:** <10ms for typical queries
- **Memory Usage:** ~3-4MB client-side
- **Network:** Gzipped size is ~400KB (85% compression)

## Future Enhancements

Potential improvements:

1. **Search Analytics**
   - Track popular search queries
   - Identify content gaps

2. **Enhanced Ranking**
   - View count-based boosting
   - Recency scoring
   - Click-through rate optimization

3. **Advanced Features**
   - Search history
   - Recent searches
   - Popular searches
   - Search suggestions
   - Synonyms and aliases

4. **Highlighting**
   - Highlight matching text in results
   - Show context snippets with matches

5. **Filters**
   - Filter by chapter/section
   - Filter by tags/keywords
   - Filter by content type

## Troubleshooting

### Search returns no results

1. Verify search index was generated:
   ```bash
   ls -lh book-source/static/search-index.json
   ```

2. Check browser console for errors loading index

3. Regenerate index:
   ```bash
   npm run search:index
   ```

### Search index is outdated

The index is generated during build. If you add/update content:

1. Regenerate index:
   ```bash
   npm run search:index
   ```

2. Or run full build:
   ```bash
   npm run build
   ```

### Build fails on search index generation

1. Verify `gray-matter` is installed:
   ```bash
   npm install gray-matter
   ```

2. Check for syntax errors in markdown files

3. Review script output for specific file errors

## Maintenance

### Adding new searchable fields

To add new metadata fields to search:

1. Update `parseMarkdownFile()` in `generate-search-index.mjs`
2. Update `SearchResult` interface in `SearchContext.tsx`
3. Update scoring logic in `performSearch()` function
4. Regenerate index

### Changing search ranking

Modify weight multipliers in `performSearch()`:

```typescript
// In SearchContext.tsx
totalScore += titleScore * 10;      // Change weight here
totalScore += breadcrumbScore * 3;  // Change weight here
// etc.
```

## Testing

Manual testing checklist:

- [ ] Search returns relevant results for common queries
- [ ] Keyboard shortcuts work (Ctrl+K, ESC, arrows, Enter)
- [ ] Results appear instantly without lag
- [ ] Modal closes properly
- [ ] Navigation works from search results
- [ ] Works in both light and dark mode
- [ ] Mobile responsive (search shows on larger screens, hidden on mobile)

## Credits

Search implementation created for AI Native Software Development book (November 2025).
