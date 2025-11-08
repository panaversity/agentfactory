# Student Profile API Contract

**Endpoint Group**: Student Profile Management
**Base Path**: `/api/profile`
**Purpose**: Manage student learning profiles, progress, and preferences

---

## Endpoints

### GET /api/profile/{session_id}

Get student learning profile including progress, preferences, and recent activity.

**Purpose**: Retrieve complete student profile for personalization and context.

---

#### Request

**Method**: `GET`
**Path**: `/api/profile/{session_id}`

**Path Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `session_id` | string | Yes | Student session identifier |

**Query Parameters**: None

**Headers**:
```
X-Request-ID: <optional-uuid>
```

---

#### Response (Success)

**Status**: `200 OK`
**Content-Type**: `application/json`

**Body**:
```json
{
  "session_id": "sess_abc123xyz",
  "student_name": "Jane Doe",
  "student_email": "jane@example.com",
  "learning_level": "intermediate",
  "current_chapter": "04-python",
  "current_lesson": "03-async",
  "total_messages": 47,
  "total_lessons_visited": 12,
  "completed_lessons": [
    "04-python/01-intro",
    "04-python/02-variables",
    "05-ai/01-intro"
  ],
  "struggling_topics": [
    "async programming",
    "decorators"
  ],
  "last_active_at": "2025-01-08T14:30:00Z",
  "recent_questions": [
    "What is async/await?",
    "How do decorators work?",
    "What's the difference between async and threading?"
  ]
}
```

**Response Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `session_id` | string | Session identifier |
| `student_name` | string | Student's name |
| `student_email` | string | Student's email |
| `learning_level` | enum | "beginner", "intermediate", or "advanced" |
| `current_chapter` | string | Current chapter ID |
| `current_lesson` | string | Current lesson ID |
| `total_messages` | integer | Total messages sent |
| `total_lessons_visited` | integer | Number of unique lessons visited |
| `completed_lessons` | array | List of completed lesson IDs |
| `struggling_topics` | array | Topics student struggles with |
| `last_active_at` | string (ISO 8601) | Last activity timestamp |
| `recent_questions` | array | Last 5 questions asked |

---

#### Response (Errors)

**404 Not Found** - Session doesn't exist
```json
{
  "error": "Session not found",
  "details": [
    {
      "code": "SESSION_NOT_FOUND",
      "message": "Session sess_abc123 does not exist"
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_xyz789"
}
```

**500 Server Error**
```json
{
  "error": "Failed to retrieve profile",
  "details": [
    {
      "code": "PROFILE_RETRIEVAL_ERROR",
      "message": "An error occurred retrieving the profile"
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_def456"
}
```

---

### PUT /api/profile/{session_id}

Update student profile information and preferences.

**Purpose**: Allow students to update their profile, learning level, and preferences.

---

#### Request

**Method**: `PUT`
**Path**: `/api/profile/{session_id}`
**Content-Type**: `application/json`

**Path Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `session_id` | string | Yes | Student session identifier |

**Headers**:
```
Content-Type: application/json
X-Request-ID: <optional-uuid>
```

**Body**:
```json
{
  "student_name": "Jane Doe",
  "student_email": "jane.doe@example.com",
  "learning_level": "advanced",
  "current_chapter": "05-ai",
  "current_lesson": "02-ml-basics"
}
```

**Fields** (all optional):
| Field | Type | Description |
|-------|------|-------------|
| `student_name` | string | Student's name |
| `student_email` | string (email) | Student's email |
| `learning_level` | enum | "beginner", "intermediate", or "advanced" |
| `current_chapter` | string | Current chapter ID |
| `current_lesson` | string | Current lesson ID |

**Validation**:
- `student_email`: Must be valid email format
- `learning_level`: Must be one of: "beginner", "intermediate", "advanced"

---

#### Response (Success)

**Status**: `200 OK`
**Content-Type**: `application/json`

**Body**:
```json
{
  "session_id": "sess_abc123xyz",
  "student_name": "Jane Doe",
  "student_email": "jane.doe@example.com",
  "learning_level": "advanced",
  "current_chapter": "05-ai",
  "current_lesson": "02-ml-basics",
  "updated_at": "2025-01-08T14:35:00Z"
}
```

---

#### Response (Errors)

**400 Bad Request** - Invalid data
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
  "timestamp": "2025-01-08T14:30:00Z"
}
```

**404 Not Found** - Session doesn't exist
```json
{
  "error": "Session not found",
  "details": [
    {
      "code": "SESSION_NOT_FOUND",
      "message": "Session sess_abc123 does not exist"
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z"
}
```

---

## Implementation Details

### GET /api/profile/{session_id} Flow

1. **Validate session_id** - Check format and existence
2. **Query database**:
   - Get `student_sessions` record
   - Count lessons from `student_progress`
   - Get completed lessons from `student_progress`
   - Get recent questions from `interaction_history`
3. **Identify struggling topics**:
   - Analyze lessons with `needs_review = true`
   - Extract topics from recent questions
4. **Format response** - Build profile response
5. **Return profile**

```python
async def get_student_profile(session_id: str):
    # Get session
    session = await db.fetch_one(
        "SELECT * FROM student_sessions WHERE session_id = ?",
        session_id
    )
    if not session:
        raise HTTPException(404, "Session not found")

    # Get progress stats
    total_lessons = await db.fetch_val(
        "SELECT COUNT(DISTINCT lesson) FROM student_progress WHERE session_id = ?",
        session_id
    )

    completed = await db.fetch_all(
        "SELECT chapter || '/' || lesson FROM student_progress WHERE session_id = ? AND completed = 1",
        session_id
    )

    # Get recent questions
    recent_questions = await db.fetch_all(
        "SELECT user_message FROM interaction_history WHERE session_id = ? ORDER BY timestamp DESC LIMIT 5",
        session_id
    )

    # Get struggling topics
    struggling = await db.fetch_all(
        "SELECT chapter, lesson FROM student_progress WHERE session_id = ? AND needs_review = 1",
        session_id
    )

    return {
        "session_id": session_id,
        "student_name": session.student_name,
        "student_email": session.student_email,
        "learning_level": session.learning_level,
        "current_chapter": session.current_chapter,
        "current_lesson": session.current_lesson,
        "total_messages": session.total_messages,
        "total_lessons_visited": total_lessons,
        "completed_lessons": [row[0] for row in completed],
        "struggling_topics": extract_topics(struggling),
        "last_active_at": session.last_active_at,
        "recent_questions": [row[0] for row in recent_questions if row[0]]
    }
```

### PUT /api/profile/{session_id} Flow

1. **Validate session_id** - Check format and existence
2. **Validate request body** - Check all fields
3. **Update database**:
   - Update `student_sessions` table
   - Set `last_active_at` to current time
4. **Return updated profile**

```python
async def update_student_profile(
    session_id: str,
    updates: ProfileUpdateRequest
):
    # Check session exists
    session = await db.fetch_one(
        "SELECT 1 FROM student_sessions WHERE session_id = ?",
        session_id
    )
    if not session:
        raise HTTPException(404, "Session not found")

    # Build update query
    update_fields = []
    values = []
    if updates.student_name:
        update_fields.append("student_name = ?")
        values.append(updates.student_name)
    if updates.student_email:
        update_fields.append("student_email = ?")
        values.append(updates.student_email)
    # ... etc

    update_fields.append("last_active_at = CURRENT_TIMESTAMP")
    values.append(session_id)

    # Execute update
    await db.execute(
        f"UPDATE student_sessions SET {', '.join(update_fields)} WHERE session_id = ?",
        *values
    )

    # Return updated profile
    return await get_student_profile(session_id)
```

---

## Agent Tool Usage

```python
from agents import function_tool

@function_tool
async def get_student_profile(session_id: str) -> str:
    """
    Get student learning profile and progress.

    Args:
        session_id: Student session ID

    Returns:
        Formatted student profile
    """
    # Call profile API
    response = await fetch_profile(session_id)

    # Format for agent
    return f"""
Student Profile:
- Level: {response['learning_level'].capitalize()}
- Current Location: Chapter {response['current_chapter']} / Lesson {response['current_lesson']}
- Total Messages: {response['total_messages']}
- Lessons Visited: {response['total_lessons_visited']}
- Completed: {len(response['completed_lessons'])} lessons
- Struggling With: {', '.join(response['struggling_topics'])}
- Recent Questions:
{chr(10).join(f"  • {q}" for q in response['recent_questions'][:3])}
"""
```

---

## Usage Examples

### Frontend (TypeScript)

```typescript
// Get profile
async function getStudentProfile(sessionId: string) {
  const response = await fetch(`/api/profile/${sessionId}`);
  if (!response.ok) throw new Error('Failed to get profile');
  return await response.json();
}

// Update profile
async function updateStudentProfile(
  sessionId: string,
  updates: Partial<StudentProfile>
) {
  const response = await fetch(`/api/profile/${sessionId}`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(updates)
  });
  if (!response.ok) throw new Error('Failed to update profile');
  return await response.json();
}

// Usage
const profile = await getStudentProfile('sess_abc123');
console.log(`Learning level: ${profile.learning_level}`);
console.log(`Completed: ${profile.completed_lessons.length} lessons`);

// Update learning level
await updateStudentProfile('sess_abc123', {
  learning_level: 'advanced'
});
```

---

## Testing

### Test Cases

1. **✅ Get Existing Profile** - Valid session_id
2. **✅ Get Non-Existent Profile** - Should return 404
3. **✅ Update Profile** - Valid updates
4. **✅ Update with Invalid Email** - Should return 400
5. **✅ Update with Invalid Level** - Should return 400
6. **✅ Update Non-Existent Session** - Should return 404
7. **✅ Empty Recent Questions** - Should return empty array
8. **✅ No Struggling Topics** - Should return empty array

### cURL Examples

**Get profile**:
```bash
curl http://localhost:8000/api/profile/sess_abc123
```

**Update profile**:
```bash
curl -X PUT http://localhost:8000/api/profile/sess_abc123 \
  -H "Content-Type: application/json" \
  -d '{
    "learning_level": "advanced",
    "current_chapter": "05-ai",
    "current_lesson": "02-ml-basics"
  }'
```

---

## Performance

**Target Metrics**:
- GET response time: < 100ms (p95)
- PUT response time: < 150ms (p95)
- Database queries: < 50ms (p95)

---

## Privacy & Security

1. **No PII Logging** - Don't log emails or names
2. **Session Validation** - Verify session ownership
3. **Rate Limiting** - 30 updates/minute per session
4. **Data Retention** - Follow privacy policy
5. **GDPR Compliance** - Support data export/deletion

---

## Future Enhancements (Post-MVP)

1. **DELETE /api/profile/{session_id}** - Delete profile
2. **GET /api/profile/{session_id}/export** - Export all data (GDPR)
3. **POST /api/profile/{session_id}/feedback** - Submit feedback
4. **GET /api/profile/{session_id}/analytics** - Learning analytics
5. **Achievements & Badges** - Gamification
