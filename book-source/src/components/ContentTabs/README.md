# ContentTabs Component

Interactive content tabs for Docusaurus with AI-powered summarization, caching, and authentication.

## Features

- 📄 **Original Tab**: Display unmodified page content
- 🤖 **Summary Tab**: AI-generated streaming summaries with caching
- ✨ **Personalized Tab**: Placeholder for future personalization features
- 🔐 Authentication flow with dummy login
- 💾 Session-scoped caching
- 📱 Mobile-responsive design
- 🛡️ Error boundary for graceful error handling

## Installation

This component is integrated into the Docusaurus theme via swizzling.

### Prerequisites

```bash
cd book-source
npm install
```

### Theme Integration

The component is already integrated via swizzled `DocItem/Content`:

```bash
npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap --typescript
```

## Usage

### Basic Usage

The component automatically wraps all Docusaurus content pages:

```tsx
import ContentTabs from '@site/src/components/ContentTabs';

<ContentTabs pageId="my-page-id">
  {/* Your page content */}
</ContentTabs>
```

### Props

| Prop | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `children` | `ReactNode` | Yes | - | Page content to display in Original tab |
| `pageId` | `string` | No | `''` | Unique identifier for caching and API calls |

## Component Structure

```
ContentTabs/
├── index.tsx              # Main component with tab state
├── TabBar.tsx             # Tab navigation UI
├── OriginalTab.tsx        # Original content display
├── SummaryTab.tsx         # AI summary with streaming
├── PersonalizedTab.tsx    # Placeholder for future features
├── DummyLogin.tsx         # Temporary authentication UI
├── ErrorBoundary.tsx      # Error handling wrapper
└── styles.module.css      # Component styles
```

## Services

### Authentication Service

```typescript
import * as authService from '@site/src/services/authService';

// Check authentication status
const isAuthenticated = authService.isAuthenticated();

// Get current token
const token = authService.getToken();

// Set authentication token
authService.setToken('token_value');

// Clear authentication
authService.clearToken();
```

### Cache Service

```typescript
import * as cacheService from '@site/src/services/cacheService';

// Store data
cacheService.set('key', { data: 'value' });

// Retrieve data
const data = cacheService.get<DataType>('key');

// Check if key exists
const exists = cacheService.has('key');

// Remove specific key
cacheService.remove('key');

// Clear all cache
cacheService.clear();
```

### Summary Service

```typescript
import * as summaryService from '@site/src/services/summaryService';

await summaryService.fetchSummary(
  'page-id',
  'content text...',
  'auth-token',
  (chunk) => console.log('Chunk:', chunk),
  () => console.log('Complete'),
  (error) => console.error('Error:', error)
);
```

## Styling

### CSS Modules

Styles are scoped using CSS modules. Override styles by targeting classes:

```css
/* Override tab button styles */
.tabButton {
  padding: 1rem 2rem;
  font-size: 1.1rem;
}

/* Override active tab style */
.tabButton.active {
  background: var(--ifm-color-primary-lightest);
}
```

### CSS Variables

Component respects Docusaurus theme variables:

- `--ifm-color-primary`: Primary brand color
- `--ifm-color-emphasis-*`: Emphasis colors (text, borders)
- `--ifm-background-color`: Background color
- `--ifm-color-danger`: Error states

### Mobile Responsiveness

Automatically adapts to mobile screens (<768px):
- Vertical tab layout
- Reduced padding
- Touch-friendly buttons

## State Management

### Tab State

```typescript
const [activeTab, setActiveTab] = useState<TabType>('original');
```

Tab types: `'original' | 'summary' | 'personalized'`

### Summary State

```typescript
const [summary, setSummary] = useState<string>('');
const [isLoading, setIsLoading] = useState(false);
const [error, setError] = useState<string | null>(null);
```

## Caching Strategy

Summaries are cached in `sessionStorage` with key format: `summary_{pageId}`

**Cache Entry Structure:**
```typescript
interface SummaryCacheEntry {
  pageId: string;
  summary: string;
  timestamp: number;
}
```

**Cache Lifecycle:**
- Stored on summary completion
- Retrieved on tab switch
- Cleared on session end
- Scoped to current tab

## Error Handling

### Error Boundary

Catches React errors in child components:

```tsx
<ErrorBoundary>
  <ContentTabs pageId="page-id">
    {children}
  </ContentTabs>
</ErrorBoundary>
```

### Error States

- **Network errors**: Retry button displayed
- **Authentication errors**: Login prompt shown
- **React errors**: Error boundary fallback UI
- **API errors**: Error message with details

## Backend Integration

### API Configuration

Default backend URL: `http://localhost:8000/api/v1`

Override in production via environment variable:

```typescript
const apiUrl = process.env.NODE_ENV === 'production' 
  ? '/api/v1/summarize' 
  : 'http://localhost:8000/api/v1/summarize';
```

### SSE Streaming

Summary uses Server-Sent Events for real-time streaming:

```typescript
const es = new EventSource(`${url}?pageId=${pageId}&token=${token}&content=${content}`);

es.onmessage = (event) => {
  const data = JSON.parse(event.data);
  if (data.chunk) onChunk(data.chunk);
  if (data.done) es.close();
};
```

## TypeScript Types

```typescript
// Tab types
type TabType = 'original' | 'summary' | 'personalized';

// Cache entry
interface SummaryCacheEntry {
  pageId: string;
  summary: string;
  timestamp: number;
}

// Authentication state
interface AuthState {
  token: string | null;
  isAuthenticated: boolean;
}
```

## Development

### Local Development

1. Start Docusaurus dev server:
```bash
cd book-source
npm start
```

2. Start backend API:
```bash
cd api
python -m uvicorn src.main:app --reload --port 8000
```

### Testing

Manual testing checklist:
- [ ] Click each tab and verify content switches
- [ ] Click Summary tab without auth → see login
- [ ] Login and verify summary streams
- [ ] Switch tabs and return → verify cache works
- [ ] Refresh page → verify cache cleared
- [ ] Resize to mobile → verify responsive layout

## Troubleshooting

### Summary not loading

1. Check backend is running on port 8000
2. Verify authentication token exists
3. Check browser console for errors
4. Verify content extraction from `<article>` element

### CORS errors

Add frontend URL to backend `CORS_ORIGINS`:
```env
CORS_ORIGINS=http://localhost:3000
```

### Cache not persisting

Cache is session-scoped and clears on:
- Tab/window close
- Manual cache clear
- Session end

## Future Enhancements

- [ ] Replace dummy authentication with SSO
- [ ] Add summary export functionality
- [ ] Implement personalized content recommendations
- [ ] Add summary customization (length, style)
- [ ] Support multiple AI models
- [ ] Add A/B testing for summary quality

## License

See project root LICENSE file.
