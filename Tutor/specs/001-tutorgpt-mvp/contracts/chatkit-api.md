# ChatKit API Contract

**Endpoint Group**: ChatKit Session Management
**Base Path**: `/api/chatkit`
**Purpose**: Create and manage ChatKit sessions for student interactions

---

## Endpoints

### POST /api/chatkit/session

Create a new ChatKit session with context and configuration.

**Purpose**: Initialize a ChatKit session that connects to our TutorGPT agent with page context.

---

#### Request

**Method**: `POST`
**Path**: `/api/chatkit/session`
**Content-Type**: `application/json`

**Headers**:
```
Content-Type: application/json
X-Request-ID: <optional-uuid>
```

**Body**:
```json
{
  "page_path": "/docs/chapter-04/lesson-01-intro",
  "page_title": "Introduction to Python",
  "current_chapter": "04-python",
  "current_lesson": "01-intro",
  "highlighted_text": "async def main():",
  "student_email": "jane@example.com",
  "student_name": "Jane Doe",
  "learning_level": "intermediate"
}
```

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `page_path` | string | No | Current page URL path |
| `page_title` | string | No | Current page title |
| `current_chapter` | string | No | Chapter ID (e.g., "04-python") |
| `current_lesson` | string | No | Lesson ID (e.g., "01-intro") |
| `highlighted_text` | string | No | Text highlighted by student |
| `student_email` | string (email) | No | Student's email for session tracking |
| `student_name` | string | No | Student's name |
| `learning_level` | enum | No | "beginner", "intermediate", or "advanced" |

**Validation**:
- `student_email`: Must be valid email format
- `learning_level`: Must be one of: "beginner", "intermediate", "advanced"
- `page_path`: Should start with "/"
- `highlighted_text`: Max 1000 characters

---

#### Response (Success)

**Status**: `201 Created`
**Content-Type**: `application/json`

**Body**:
```json
{
  "client_secret": "sk_chatkit_abc123xyz789...",
  "session_id": "sess_def456uvw012",
  "expires_at": "2025-01-08T18:00:00Z"
}
```

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `client_secret` | string | OpenAI ChatKit client secret for authentication |
| `session_id` | string | Our internal session ID for tracking |
| `expires_at` | string (ISO 8601) | Session expiration timestamp |

---

#### Response (Errors)

**400 Bad Request** - Invalid request data
```json
{
  "error": "Invalid request",
  "details": [
    {
      "code": "INVALID_EMAIL",
      "message": "Invalid email format",
      "field": "student_email"
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_abc123"
}
```

**429 Rate Limited** - Too many session creation requests
```json
{
  "error": "Rate limit exceeded",
  "details": [
    {
      "code": "RATE_LIMIT_EXCEEDED",
      "message": "Too many session creation attempts. Try again in 60 seconds."
    }
  ],
  "retry_after": 60,
  "timestamp": "2025-01-08T14:30:00Z"
}
```

**500 Server Error** - Failed to create session
```json
{
  "error": "Failed to create session",
  "details": [
    {
      "code": "SESSION_CREATION_FAILED",
      "message": "Could not create ChatKit session. Please try again."
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_xyz789"
}
```

---

## Implementation Details

### Backend Flow

1. **Validate request** - Check all fields against schema
2. **Check rate limits** - Verify IP hasn't exceeded limits
3. **Get or create student session** - Look up by email or create new
4. **Update session context** - Store current chapter/lesson/page
5. **Create ChatKit session** - Call OpenAI ChatKit API
6. **Configure agent** - Set up TutorGPT agent with:
   - System prompt with student context
   - Tools (search_book_content, get_student_profile)
   - Session state
7. **Return credentials** - Send client_secret to frontend

### OpenAI ChatKit Session API Call

```python
from openai import OpenAI

client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

# Create ChatKit session
session = client.chatkit.sessions.create({
    "model": "gpt-4-turbo",
    "instructions": f"""
    You are TutorGPT, an AI tutor for the AI-Native Software Development book.

    Student Context:
    - Name: {student_name}
    - Level: {learning_level}
    - Current Location: Chapter {chapter} / Lesson {lesson}
    - Page: {page_path}
    {f"- Highlighted Text: {highlighted_text}" if highlighted_text else ""}

    Your role is to help students understand the book content through:
    1. Answering questions clearly at their level
    2. Searching book content when needed
    3. Providing examples and analogies
    4. Adapting to their learning pace

    Always be encouraging, patient, and precise.
    """,
    "tools": [
        {
            "type": "function",
            "function": {
                "name": "search_book_content",
                "description": "Search the book content for relevant information",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {"type": "string"},
                        "scope": {
                            "type": "string",
                            "enum": ["current_lesson", "current_chapter", "entire_book"]
                        }
                    },
                    "required": ["query"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "get_student_profile",
                "description": "Get student's learning profile and progress",
                "parameters": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            }
        }
    ],
    "temperature": 0.7,
    "max_tokens": 1000
})

client_secret = session.client_secret
```

---

## Usage Example

### Frontend (React/TypeScript)

```typescript
import { ChatKit, useChatKit } from '@openai/chatkit-react';

export function TutorChatWidget() {
  const { control } = useChatKit({
    api: {
      async getClientSecret(existing) {
        // Create session from our backend
        const response = await fetch('/api/chatkit/session', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            page_path: window.location.pathname,
            page_title: document.title,
            current_chapter: getCurrentChapter(),
            current_lesson: getCurrentLesson(),
            highlighted_text: getHighlightedText(),
            student_email: localStorage.getItem('student_email'),
            learning_level: localStorage.getItem('learning_level') || 'beginner'
          })
        });

        const data = await response.json();
        return data.client_secret;
      }
    }
  });

  return <ChatKit control={control} className="h-[600px] w-[320px]" />;
}
```

---

## Testing

### Test Cases

1. **✅ Success Case** - Valid request with all fields
2. **✅ Minimal Request** - Only required fields (none)
3. **✅ Invalid Email** - Should return 400
4. **✅ Invalid Learning Level** - Should return 400
5. **✅ Rate Limit** - Exceed 10 requests/minute
6. **✅ Long Highlighted Text** - Exceed 1000 chars
7. **✅ Existing Student** - Email already exists
8. **✅ New Student** - First time email

### cURL Examples

**Create session with full context**:
```bash
curl -X POST http://localhost:8000/api/chatkit/session \
  -H "Content-Type: application/json" \
  -d '{
    "page_path": "/docs/chapter-04/lesson-01-intro",
    "current_chapter": "04-python",
    "current_lesson": "01-intro",
    "student_email": "jane@example.com",
    "learning_level": "intermediate"
  }'
```

**Expected Response**:
```json
{
  "client_secret": "sk_chatkit_...",
  "session_id": "sess_abc123",
  "expires_at": "2025-01-08T18:00:00Z"
}
```

---

## Security Considerations

1. **Domain Whitelisting** - Configure OpenAI ChatKit to only work on approved domains
2. **Rate Limiting** - Prevent abuse with IP-based rate limits
3. **Client Secret Lifetime** - Sessions expire after inactivity
4. **No Sensitive Data** - Don't log or store client secrets
5. **HTTPS Only** - Always use HTTPS in production
6. **Email Validation** - Validate email format before storage

---

## Performance

**Target Metrics**:
- Response time: < 500ms (p95)
- Session creation: < 200ms (p95)
- Rate limit: 10 requests/minute per IP

---

## Future Enhancements (Post-MVP)

1. **Session Refresh** - Endpoint to refresh expired sessions
2. **Session Deletion** - Allow students to explicitly end sessions
3. **Session History** - Retrieve past session metadata
4. **OAuth Integration** - Support OAuth for student authentication
