# Collapsible Summary Feature

## Overview

This feature adds an AI-generated summary section at the bottom of each chapter page and its nested pages in the book. The summary is generated using Google Gemini AI and is cached for future use.

## Features

- **Collapsible UI**: Summary section is collapsed by default
- **On-Demand Generation**: Summary is only generated when user opens the section for the first time
- **Smart Caching**: Generated summaries are saved and reused
- **Loading State**: Shows "Generating..." placeholder while API is working
- **Structured Output**: Summaries are formatted in readable markdown (up to 500 words)
- **Chapter-Only**: Only appears on chapter pages (docs/XX-Chapter-Name/...)

## Architecture

### Components

1. **CollapsibleSummary** (`src/components/CollapsibleSummary/`)
   - React component that renders the collapsible summary section
   - Handles state management for open/closed, loading, error states
   - Makes API calls to generate or fetch summaries
   - Renders markdown content with proper formatting

2. **usePageContent Hook** (`src/hooks/usePageContent.ts`)
   - Custom hook that determines if current page is a chapter page
   - Extracts page path and content for summary generation
   - Uses Docusaurus metadata to identify chapter pages

3. **DocItem/Content** (`src/theme/DocItem/Content/index.tsx`)
   - Swizzled Docusaurus component
   - Integrates CollapsibleSummary at the end of doc pages
   - Only renders summary for chapter pages

### Backend

1. **Summary Service** (`server/services/summaryService.js`)
   - Handles Gemini AI integration
   - Manages summary storage and retrieval
   - Generates summaries with structured prompts (max 500 words)

2. **Summary Routes** (`server/routes/summary.js`)
   - `POST /api/summary/generate`: Generate new summary
   - `GET /api/summary/check`: Check if summary exists

3. **Storage** (`summary/`)
   - JSON files storing generated summaries
   - Naming: `Chapter__Subchapter__filename.json`
   - Ignored in git (except README)

## File Structure

```
book-source/
├── src/
│   ├── components/
│   │   └── CollapsibleSummary/
│   │       ├── index.tsx          # Main component
│   │       └── styles.module.css  # Component styles
│   ├── hooks/
│   │   └── usePageContent.ts      # Page content hook
│   └── theme/
│       └── DocItem/
│           └── Content/
│               └── index.tsx      # Swizzled component
├── server/
│   ├── routes/
│   │   └── summary.js             # API routes
│   ├── services/
│   │   └── summaryService.js      # Gemini integration
│   └── index.js                   # Updated with summary routes
├── summary/
│   └── README.md                  # Documentation
└── .gitignore                     # Updated to ignore *.json in summary/
```

## How It Works

### User Flow

1. User visits a chapter page (e.g., `docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/readme.md`)
2. Collapsible summary section appears at the bottom (collapsed)
3. User clicks to expand the section
4. Component checks if summary exists via API
5. If not exists:
   - Shows "Generating..." loading state
   - Sends page content to Gemini API
   - Receives and displays structured summary
   - Saves summary to `summary/` folder
6. If exists:
   - Displays cached summary immediately
7. On subsequent opens/closes, no API calls are made

### API Flow

```
Client (CollapsibleSummary)
  ↓
  1. Check cache: GET /api/summary/check?pagePath=...
  ↓
  If not exists:
    2. Generate: POST /api/summary/generate
       Body: { pagePath, pageContent }
  ↓
  3. Gemini AI generates summary (max 500 words)
  ↓
  4. Save to summary/XX__YY__ZZ.json
  ↓
  5. Return summary to client
```

## Configuration

### Environment Variables

Required in `.env`:
```
GEMINI_API_KEY=your_api_key_here
```

### API Endpoint

Default: `http://localhost:3001/api/summary`

Can be configured in `src/components/CollapsibleSummary/index.tsx`:
```typescript
const API_BASE = 'http://localhost:3001/api/summary';
```

## Usage

### Running the Application

1. **Start the API server**:
   ```bash
   npm run start:api
   ```

2. **Start Docusaurus** (in another terminal):
   ```bash
   npm start
   ```

3. **Or run both together**:
   ```bash
   npm run dev
   ```

### Testing

1. Navigate to any chapter page (e.g., `/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution`)
2. Scroll to the bottom
3. Click on "AI-Generated Page Summary"
4. Wait for summary generation (first time only)
5. Verify summary displays correctly
6. Close and reopen - should show "Cached" badge
7. Refresh page and reopen - should use cached version

## Styling

The component uses CSS modules with Docusaurus theming variables:

- Supports light/dark mode automatically
- Uses Docusaurus color tokens
- Responsive design
- Accessible keyboard navigation
- Focus states for accessibility

### Key CSS Classes

- `.summaryContainer`: Main wrapper
- `.summaryToggle`: Clickable header button
- `.summaryContent`: Expanded content area
- `.loadingState`: Loading spinner and text
- `.errorState`: Error message display
- `.summaryText`: Formatted markdown content

## Customization

### Changing Summary Length

Edit `server/services/summaryService.js`:
```javascript
const prompt = `...
REQUIREMENTS:
1. Maximum length: 500 words  // Change this
...`;
```

### Changing AI Model

Edit `server/services/summaryService.js`:
```javascript
this.model = this.genAI.getGenerativeModel({
  model: 'gemini-2.0-flash'  // Change model here
});
```

### Customizing Summary Prompt

Edit the prompt in `server/services/summaryService.js` in the `generateSummary` method.

### Styling Customization

Edit `src/components/CollapsibleSummary/styles.module.css` to change colors, spacing, animations, etc.

### Filtering Pages

To change which pages show summaries, edit the regex pattern in `src/hooks/usePageContent.ts`:
```typescript
// Current: Only numbered chapters (01-, 02-, etc.)
const isChapter = /^@site\/docs\/\d{2}-[^/]+\//.test(sourcePath);

// Example: All docs pages
const isChapter = /^@site\/docs\//.test(sourcePath);
```

## Troubleshooting

### Summary doesn't appear

- Check if page is under a numbered chapter directory
- Verify the page path matches the pattern in `usePageContent.ts`
- Check browser console for errors

### "Failed to generate summary" error

- Verify `GEMINI_API_KEY` is set in `.env`
- Check API server is running (`npm run start:api`)
- Check network tab for failed requests
- Verify API endpoint URL is correct

### Summary generates every time

- Check `summary/` folder has write permissions
- Verify summary JSON file was created
- Check browser console for fetch errors
- Clear browser cache and try again

### CORS errors

- Verify server has CORS enabled (`server/index.js`)
- Check API endpoint URL matches server port (default: 3001)

## Performance Considerations

- **First Load**: 2-5 seconds to generate summary (one-time cost)
- **Subsequent Loads**: Instant (cached)
- **Storage**: ~1-5KB per summary JSON file
- **API Calls**: Only on first open, then zero

## Future Enhancements

Potential improvements:
- [ ] Regenerate summary button
- [ ] Summary version control
- [ ] Multi-language summaries
- [ ] Summary quality rating
- [ ] Admin panel to manage summaries
- [ ] Bulk summary generation script
- [ ] Summary search/filtering

## Security Notes

- API endpoint is accessible without authentication
- Rate limiting should be added for production
- Summary files are gitignored (not committed)
- Page content is sent to Gemini API (ensure compliance)

## License

Same as the main project.
