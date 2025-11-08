# API Contracts: TutorGPT MVP

**Date**: 2025-01-08
**Purpose**: Define all API contracts and interfaces for TutorGPT
**Status**: Design Phase

---

## Overview

This directory contains the API contracts for all TutorGPT endpoints. These contracts define:
- Request/response schemas
- Endpoints and methods
- Authentication requirements
- Error responses
- Example payloads

---

## API Base URL

```
Development: http://localhost:8000
Production: https://api.tutorgpt.example.com
```

---

## Authentication

### OpenAI API Key (Backend)
- **Header**: `Authorization: Bearer <OPENAI_API_KEY>`
- **Used for**: Creating ChatKit sessions, agent interactions
- **Stored in**: `.env` file (never committed)

### Google AI API Key (Backend)
- **Environment Variable**: `GOOGLE_API_KEY`
- **Used for**: Gemini embeddings
- **Stored in**: `.env` file (never committed)

### ChatKit Client Secret (Frontend)
- **Obtained from**: `POST /api/chatkit/session`
- **Used for**: ChatKit widget authentication
- **Lifetime**: Session-based (expires after inactivity)

---

## API Endpoints

### 1. ChatKit Session Management
- **File**: [chatkit-api.md](./chatkit-api.md)
- **Endpoints**:
  - `POST /api/chatkit/session` - Create ChatKit session

### 2. RAG Search
- **File**: [rag-api.md](./rag-api.md)
- **Endpoints**:
  - `POST /api/rag/search` - Search book content

### 3. Student Profile
- **File**: [profile-api.md](./profile-api.md)
- **Endpoints**:
  - `GET /api/profile/{session_id}` - Get student profile
  - `PUT /api/profile/{session_id}` - Update student profile

### 4. Health & Status
- **Endpoint**: `GET /health`
- **Response**:
  ```json
  {
    "status": "healthy",
    "version": "1.0.0",
    "services": {
      "database": "connected",
      "vectorstore": "connected",
      "embeddings": "connected"
    }
  }
  ```

---

## Common Headers

### Request Headers
```
Content-Type: application/json
Accept: application/json
X-Request-ID: <optional-uuid>  # For tracing
```

### Response Headers
```
Content-Type: application/json
X-Request-ID: <uuid>  # Echoed from request or generated
X-Response-Time: <milliseconds>
```

---

## Error Responses

All errors follow this standard format:

```json
{
  "error": "Error message summary",
  "details": [
    {
      "code": "ERROR_CODE",
      "message": "Detailed error message",
      "field": "field_name"  // Optional
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_xyz789"
}
```

### HTTP Status Codes

| Code | Meaning | Example |
|------|---------|---------|
| 200 | Success | Request completed successfully |
| 201 | Created | Session created successfully |
| 400 | Bad Request | Invalid request parameters |
| 401 | Unauthorized | Missing or invalid API key |
| 404 | Not Found | Session or resource not found |
| 422 | Validation Error | Request validation failed |
| 429 | Rate Limited | Too many requests |
| 500 | Server Error | Internal server error |
| 503 | Service Unavailable | Service temporarily unavailable |

---

## Rate Limiting

### Limits
- **ChatKit session creation**: 10 requests/minute per IP
- **RAG search**: 60 requests/minute per session
- **Profile updates**: 30 requests/minute per session

### Headers
```
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1704724800  # Unix timestamp
```

### Rate Limit Error
```json
{
  "error": "Rate limit exceeded",
  "details": [
    {
      "code": "RATE_LIMIT_EXCEEDED",
      "message": "You have exceeded the rate limit. Try again in 30 seconds."
    }
  ],
  "retry_after": 30,
  "timestamp": "2025-01-08T14:30:00Z"
}
```

---

## CORS Configuration

```python
# Allowed origins
origins = [
    "http://localhost:3000",  # Local Docusaurus dev
    "https://book.example.com"  # Production book site
]

# Allowed methods
methods = ["GET", "POST", "PUT", "DELETE", "OPTIONS"]

# Allowed headers
headers = ["Content-Type", "Authorization", "X-Request-ID"]
```

---

## Versioning

### Strategy
- URL-based versioning: `/api/v1/...`
- Current version: `v1`
- Backwards compatibility maintained within major versions

### Version Header (Optional)
```
API-Version: 1.0.0
```

---

## Testing

### Postman Collection
- **File**: `tutorgpt-api.postman_collection.json` (to be generated)

### Example cURL Commands

**Create Session**:
```bash
curl -X POST http://localhost:8000/api/chatkit/session \
  -H "Content-Type: application/json" \
  -d '{
    "page_path": "/docs/chapter-04/lesson-01-intro",
    "current_chapter": "04-python",
    "current_lesson": "01-intro"
  }'
```

**RAG Search**:
```bash
curl -X POST http://localhost:8000/api/rag/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is async programming?",
    "scope": "current_lesson",
    "current_chapter": "04-python",
    "current_lesson": "03-async"
  }'
```

---

## WebSocket Support (Future)

For real-time streaming responses (post-MVP):

```
ws://localhost:8000/ws/chat/{session_id}
```

---

## Files in this Directory

1. ✅ `README.md` - This file (overview)
2. ✅ `chatkit-api.md` - ChatKit session endpoints
3. ✅ `rag-api.md` - RAG search endpoints
4. ✅ `profile-api.md` - Student profile endpoints

---

## Next Steps

After defining contracts:
1. Implement FastAPI endpoints matching these contracts
2. Generate OpenAPI spec automatically via FastAPI
3. Create Postman collection for testing
4. Document all agent tools that use these endpoints
