# Content Summarization API

Backend API for generating AI-powered summaries of content pages with streaming support using OpenAI Agents SDK.

## Features

- 🤖 AI-powered content summarization using Gemini 2.0 Flash
- 📡 Server-Sent Events (SSE) streaming for real-time summary generation
- 🔐 Dummy authentication (ready for SSO integration)
- 💾 Proportional summary generation (150-500 words, 20-25% compression)
- 🚀 FastAPI framework with CORS support

## Prerequisites

- Python 3.11 or higher
- Google API key (for Gemini 2.0 Flash access)

## Setup Instructions

### 1. Install Dependencies

```bash
cd api
pip install -r requirements.txt
```

### 2. Environment Configuration

Create a `.env` file in the `api/` directory:

```env
# Google Gemini API Configuration
GOOGLE_API_KEY=your_google_api_key_here

# Logging Configuration
LOG_LEVEL=INFO

# CORS Configuration
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
```

### Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `GOOGLE_API_KEY` | Yes | - | Google API key for Gemini 2.0 Flash model access |
| `LOG_LEVEL` | No | `INFO` | Logging level (`DEBUG`, `INFO`, `WARNING`, `ERROR`) |
| `CORS_ORIGINS` | No | `http://localhost:3000` | Comma-separated list of allowed origins |

### 3. Run the Server

```bash
cd api
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000
```

Or with auto-reload for development:

```bash
python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

## API Endpoints

### Health Check

```http
GET /health
```

Returns API health status.

**Response:**
```json
{
  "status": "healthy",
  "service": "content-summarization-api"
}
```

### Summarize Content

```http
GET /api/v1/summarize?pageId={pageId}&token={token}&content={content}
```

Streams AI-generated summary using Server-Sent Events.

**Parameters:**
- `pageId` (required): Unique identifier for the content page
- `token` (required): Authentication token
- `content` (required): Page content text to summarize

**Response:** SSE stream with JSON events:
```json
data: {"chunk": "Summary text chunk...", "done": false}
data: {"chunk": "", "done": true}
```

### Dummy Login

```http
POST /api/v1/auth/dummy-login
```

Returns a dummy authentication token.

**Response:**
```json
{
  "token": "dummy_token_12345",
  "expires": "session",
  "user": {
    "id": "dummy_user",
    "name": "Anonymous User"
  }
}
```

### Verify Token

```http
GET /api/v1/auth/verify?token={token}
```

Validates authentication token (dummy implementation).

**Response:**
```json
{
  "valid": true,
  "user": {
    "id": "dummy_user",
    "name": "Anonymous User"
  }
}
```

## Project Structure

```
api/
├── src/
│   ├── main.py                 # FastAPI app entry point
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── summarize.py        # Summary streaming endpoint
│   │   └── auth.py             # Authentication endpoints
│   ├── services/
│   │   └── openai_agent.py     # OpenAI Agents SDK integration
│   └── models/
│       └── schemas.py          # Pydantic response models
├── requirements.txt            # Python dependencies
├── .env                        # Environment variables (not in git)
└── README.md                   # This file
```

## Development

### Testing the API

1. Start the server
2. Visit `http://localhost:8000/docs` for interactive API documentation
3. Use the `/health` endpoint to verify the API is running

### Logging

Logs are written to stdout with timestamps and log levels. Configure verbosity with `LOG_LEVEL` environment variable.

## Deployment

### Production Considerations

1. **Authentication**: Replace dummy authentication with proper SSO/JWT validation
2. **API Keys**: Store `GOOGLE_API_KEY` in secure secrets management (AWS Secrets Manager, Azure Key Vault)
3. **Rate Limiting**: Add rate limiting middleware for production traffic
4. **HTTPS**: Serve API over HTTPS with valid SSL certificates
5. **Monitoring**: Integrate with monitoring tools (Prometheus, DataDog)
6. **Error Tracking**: Add Sentry or similar for error reporting

### Environment-Specific Configuration

For production deployment, override environment variables:

```bash
export GOOGLE_API_KEY=prod_key_here
export LOG_LEVEL=WARNING
export CORS_ORIGINS=https://yourdomain.com
```

## Troubleshooting

### Common Issues

**"GOOGLE_API_KEY is not set"**
- Ensure `.env` file exists in `api/` directory
- Verify `GOOGLE_API_KEY` is defined in `.env`

**"Unable to open database file"**
- SQLiteSession uses in-memory storage by default
- No database file creation required

**CORS errors**
- Add frontend origin to `CORS_ORIGINS` in `.env`
- Restart the server after changes

## License

See project root LICENSE file.
