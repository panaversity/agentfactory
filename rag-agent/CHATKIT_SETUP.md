# ChatKit Integration Setup

This document describes how the ChatKit server is integrated with the RoboLearn interface.

## Architecture

- **Backend**: `rag-agent` - FastAPI server with ChatKit endpoint at `/chatkit`
- **Frontend**: `robolearn-interface` - Docusaurus site with ChatKit widget component

## Backend Configuration

### Environment Variables

The ChatKit server requires a PostgreSQL database connection. Configure one of these:

**Option 1 (Recommended):**
```bash
CHATKIT_STORE_DATABASE_URL=postgresql+asyncpg://user:password@host:port/database
```

**Option 2 (Fallback):**
```bash
DATABASE_URL=postgresql+asyncpg://user:password@host:port/database
```

### Database Schema

The ChatKit store automatically creates the required schema on startup:
- Schema name: `chatkit` (configurable via `CHATKIT_STORE_SCHEMA_NAME`)
- Tables: `threads`, `items`, `attachments`

### Endpoints

- `POST /chatkit` - Main ChatKit API endpoint
  - Requires `X-User-ID` header for user identification
  - Optional `X-Request-ID` header for tracing

## Frontend Configuration

### Environment Variables

In `robolearn-interface/.env` or via `docusaurus.config.ts`:

```bash
BACKEND_URL=http://localhost:8000  # URL of rag-agent server
CHATKIT_DOMAIN_KEY=domain_pk_local_dev  # Optional, dummy value for custom backend
```

### Widget Component

The ChatKit widget (`src/components/ChatKitWidget/index.tsx`) is automatically loaded via `Root.tsx`.

**Features:**
- Floating chat button (bottom-right)
- Text selection with "Ask" button
- Page context awareness
- User authentication via `X-User-ID` header

### User Identification

The widget uses the authenticated user's ID from `AuthContext`:
- If logged in: Uses `session.user.id`
- If anonymous: Uses `'anonymous'`

## Testing

1. **Start the backend:**
   ```bash
   cd rag-agent
   uv sync
   # Set DATABASE_URL or CHATKIT_STORE_DATABASE_URL in .env
   uvicorn app:app --reload --port 8000
   ```

2. **Start the frontend:**
   ```bash
   cd robolearn-interface
   npm install
   # Set BACKEND_URL in .env or docusaurus.config.ts
   npm start
   ```

3. **Test the widget:**
   - Click the chat button (bottom-right)
   - Send a message
   - Check backend logs for `/chatkit` requests

## Troubleshooting

### ChatKit widget not appearing
- Check browser console for errors
- Verify ChatKit script is loaded (check Network tab for `chatkit.js`)
- Check `BACKEND_URL` is correctly configured

### Backend errors
- Verify database connection string is correct
- Check database is accessible
- Review backend logs for initialization errors

### Authentication issues
- Verify `X-User-ID` header is being sent (check Network tab)
- Check `AuthContext` is providing user session

## Differences from car-fixer-platform

The robolearn implementation:
- Uses simpler authentication (no client secret endpoints)
- Direct `/chatkit` endpoint communication
- No domain key validation (uses dummy value for custom backend)
- Integrated with Docusaurus instead of Next.js


