# Developer Quickstart: Learning Hub Sidebar

**Feature**: 012-learning-hub-sidebar  
**Estimated Setup Time**: 15-20 minutes  
**Prerequisites**: Node.js 18+, Git, VS Code (recommended)

---

## 🚀 Quick Setup

### 1. Clone and Install

```bash
# Navigate to book project
cd book-source

# Install dependencies (if not already done)
pnpm install

# Install additional dependencies for Learning Hub
pnpm add @google/generative-ai crypto-js
pnpm add -D @types/crypto-js
```

### 2. Get Gemini API Key

1. Go to [Google AI Studio](https://aistudio.google.com/app/apikey)
2. Click "Get API Key" → "Create API key in new project"
3. Copy the generated key (starts with `AIzaSy...`)

### 3. Configure Environment

Create `.env.local` in `book-source/`:

```bash
# .env.local
GEMINI_API_KEY=your_api_key_here
```

**Important**: Add `.env.local` to `.gitignore`:

```bash
echo ".env.local" >> .gitignore
```

### 4. Swizzle DocRoot Component

```bash
cd book-source
pnpm swizzle @docusaurus/theme-classic DocRoot --wrap
```

This creates `src/theme/DocRoot/index.tsx`.

### 5. Verify Setup

```bash
# Start development server
pnpm start

# Open http://localhost:3000
# Look for Learning Hub toggle button (bottom-right corner)
```

---

## 📁 Project Structure

After setup, you'll have:

```
book-source/
├── .env.local                          # API keys (git-ignored)
├── src/
│   └── theme/
│       ├── DocRoot/
│       │   └── index.tsx               # Swizzled wrapper (adds LearningHub)
│       └── LearningHub/                # Learning Hub implementation
│           ├── index.tsx               # Main component
│           ├── components/
│           │   ├── ChatTab.tsx
│           │   ├── HighlightsTab.tsx
│           │   ├── QuizTab.tsx
│           │   ├── ConceptsTab.tsx
│           │   ├── TopicsTab.tsx
│           │   └── ProgressTab.tsx
│           ├── hooks/
│           │   ├── useGeminiChat.ts
│           │   ├── useHighlights.ts
│           │   ├── useQuiz.ts
│           │   └── useProgress.ts
│           ├── services/
│           │   ├── geminiService.ts
│           │   ├── storageService.ts
│           │   └── rateLimiter.ts
│           ├── types/
│           │   └── index.ts
│           ├── utils/
│           │   ├── contentExtractor.ts
│           │   ├── textSelection.ts
│           │   └── errorLogger.ts
│           └── styles/
│               └── LearningHub.module.css
```

---

## 🛠️ Development Workflow

### Running Locally

```bash
# Development mode (hot reload)
pnpm start

# Production build (test before deployment)
pnpm build
pnpm serve
```

### Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `GEMINI_API_KEY` | Yes | Google Gemini 2.0 Flash API key |
| `NODE_ENV` | Auto | `development` or `production` |

### Testing

```bash
# Run all tests
pnpm test

# Unit tests only
pnpm test:unit

# Integration tests (requires API key)
pnpm test:integration

# E2E tests (Playwright)
pnpm test:e2e

# Watch mode
pnpm test --watch
```

### Linting and Formatting

```bash
# Check TypeScript types
pnpm typecheck

# Lint code
pnpm lint

# Fix linting issues
pnpm lint:fix

# Format code
pnpm format
```

---

## 🧪 Testing Guide

### Unit Tests (Jest + React Testing Library)

**Location**: `src/theme/LearningHub/**/__tests__/`

**Example**:
```typescript
// src/theme/LearningHub/components/__tests__/ChatTab.test.tsx
import { render, screen, fireEvent } from '@testing-library/react';
import ChatTab from '../ChatTab';

describe('ChatTab', () => {
  it('should send chat message', async () => {
    render(<ChatTab />);
    
    const input = screen.getByPlaceholderText('Ask a question...');
    const button = screen.getByText('Send');
    
    fireEvent.change(input, { target: { value: 'What is AI?' } });
    fireEvent.click(button);
    
    expect(await screen.findByText(/AI stands for/i)).toBeInTheDocument();
  });
});
```

### Integration Tests (MSW for API Mocking)

**Location**: `tests/integration/learning-hub/`

**Setup**:
```typescript
// tests/integration/setup.ts
import { setupServer } from 'msw/node';
import { geminiHandlers } from './mocks/geminiHandlers';

export const server = setupServer(...geminiHandlers);

beforeAll(() => server.listen());
afterEach(() => server.resetHandlers());
afterAll(() => server.close());
```

**Example**:
```typescript
// tests/integration/learning-hub/chat.test.ts
import { sendChatMessage } from '@site/src/theme/LearningHub/services/geminiService';

describe('Gemini Chat Integration', () => {
  it('should get real response from API', async () => {
    const response = await sendChatMessage({
      userMessage: 'What is Python?',
      pageContext: { /* ... */ },
      conversationHistory: []
    });
    
    expect(response).toContain('Python');
  });
});
```

### E2E Tests (Playwright)

**Location**: `tests/e2e/learning-hub/`

**Example**:
```typescript
// tests/e2e/learning-hub/sidebar.spec.ts
import { test, expect } from '@playwright/test';

test('should open and close Learning Hub', async ({ page }) => {
  await page.goto('http://localhost:3000/docs/intro');
  
  // Open sidebar
  await page.click('[data-testid="learning-hub-toggle"]');
  await expect(page.locator('[data-testid="learning-hub-sidebar"]')).toBeVisible();
  
  // Close sidebar
  await page.click('[data-testid="learning-hub-toggle"]');
  await expect(page.locator('[data-testid="learning-hub-sidebar"]')).not.toBeVisible();
});
```

---

## 🔍 Debugging Tips

### Enable Debug Logging

In browser console:

```javascript
// Enable all Learning Hub logs
localStorage.setItem('learningHub_debug', 'true');

// View error log
JSON.parse(localStorage.getItem('learningHub_errors_v1') || '[]');

// Clear all Learning Hub data
Object.keys(localStorage)
  .filter(key => key.startsWith('learningHub_'))
  .forEach(key => localStorage.removeItem(key));
```

### Common Issues

#### Issue: "API key not found"

**Solution**:
1. Check `.env.local` exists in `book-source/`
2. Restart dev server (`pnpm start`)
3. Verify key format: `AIzaSy...` (39 characters)

#### Issue: "Rate limit exceeded"

**Solution**:
- Wait 1 minute for rate limit reset
- Check console for remaining requests
- Use cached data when available

#### Issue: "Highlights not appearing"

**Solution**:
1. Open browser DevTools → Elements
2. Check if `.learning-hub-highlight` class is applied
3. Verify `textPosition` in localStorage
4. Clear highlight cache: `localStorage.removeItem('learningHub_highlights_v1')`

#### Issue: "Quiz not generating"

**Solution**:
1. Check page content length (need at least 500 words)
2. View API error in console
3. Verify API key has sufficient quota
4. Try regenerating after 1 minute

---

## 📊 Performance Monitoring

### Key Metrics to Track

```typescript
// Log performance data
const perfData = {
  chatResponseTime: performance.measure('chat-start', 'chat-end'),
  highlightLoadTime: performance.measure('highlight-start', 'highlight-end'),
  cacheHitRate: getCacheHitRate(),
  apiCallCount: getAPICallCount(),
  localStorageSize: getLocalStorageSize()
};

console.log('[Performance]', perfData);
```

### Performance Targets

- **Chat first token**: < 1s
- **Highlight explanation**: < 2s
- **Quiz generation**: < 3s
- **Sidebar open/close**: < 100ms
- **Tab switching**: < 50ms

---

## 🚀 Deployment Checklist

Before deploying to production:

- [ ] Run full test suite: `pnpm test`
- [ ] Build succeeds: `pnpm build`
- [ ] No TypeScript errors: `pnpm typecheck`
- [ ] No linting issues: `pnpm lint`
- [ ] Test in production mode: `pnpm serve`
- [ ] Verify API key is NOT committed to git
- [ ] Test on multiple browsers (Chrome, Firefox, Safari)
- [ ] Test on mobile devices
- [ ] Verify localStorage quota handling
- [ ] Check error logging works
- [ ] Test rate limiting behavior
- [ ] Verify cache invalidation logic

---

## 🔐 Security Best Practices

### API Key Management

1. **Never commit API keys** to git
2. Use environment variables (`.env.local`)
3. Rotate keys periodically (every 90 days)
4. Set up API key restrictions in Google Cloud Console:
   - Restrict to HTTP referrers (your domain)
   - Restrict to Gemini API only

### Data Privacy

1. **No server-side storage**: All data in user's localStorage
2. **No tracking**: No user data sent to analytics
3. **No cross-user data**: Each user's data is isolated
4. **Clear data on logout**: Provide "Clear All Data" button

### Input Sanitization

All user inputs are sanitized before API calls:

```typescript
function sanitizeInput(input: string): string {
  return input
    .trim()
    .replace(/<script[\s\S]*?>[\s\S]*?<\/script>/gi, '')
    .replace(/<[^>]+>/g, '')
    .slice(0, 10000);  // Max length
}
```

---

## 📚 Additional Resources

### Documentation

- [Google Gemini API Docs](https://ai.google.dev/docs)
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [React Testing Library](https://testing-library.com/react)
- [Playwright Docs](https://playwright.dev/)

### Related Files

- **Spec**: `specs/012-learning-hub-sidebar/spec.md`
- **Plan**: `specs/012-learning-hub-sidebar/plan.md`
- **Data Model**: `specs/012-learning-hub-sidebar/data-model.md`
- **Contracts**: `specs/012-learning-hub-sidebar/contracts/`

### Support

- Issues: Create GitHub issue with `[Learning Hub]` prefix
- Slack: #learning-hub-dev channel
- Email: dev-team@example.com

---

## 🎯 Next Steps

1. **Familiarize with codebase**: Read through `spec.md` and `data-model.md`
2. **Set up dev environment**: Follow setup steps above
3. **Run existing tests**: `pnpm test` to understand coverage
4. **Pick a task**: See `specs/012-learning-hub-sidebar/tasks.md` (to be created)
5. **Submit PR**: Follow git workflow in `CONTRIBUTING.md`

Happy coding! 🚀
