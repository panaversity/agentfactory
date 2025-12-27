---
sidebar_position: 8
title: "JWT Authentication"
description: "Secure your API endpoints with JSON Web Tokens—implementing login and protected routes"
keywords: [jwt, authentication, oauth2, bearer-token, fastapi-security]
chapter: 40
lesson: 8
duration_minutes: 50

skills:
  - name: "JWT Token Generation"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "4.2 Protecting Personal Data"
    measurable_at_this_level: "Student creates and signs JWT tokens"

  - name: "OAuth2PasswordBearer Setup"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student configures OAuth2 password flow"

  - name: "Protected Route Implementation"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student creates endpoints requiring authentication"

learning_objectives:
  - objective: "Generate JWT tokens with expiration"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Token endpoint returns valid JWT"

  - objective: "Validate tokens and extract user identity"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Protected routes decode token correctly"

  - objective: "Implement login endpoint with OAuth2 password flow"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "POST /token accepts username/password, returns token"

cognitive_load:
  new_concepts: 5
  assessment: "JWT structure, python-jose, OAuth2PasswordBearer, token endpoint, get_current_user"

differentiation:
  extension_for_advanced: "Add refresh tokens and token revocation"
  remedial_for_struggling: "Focus on token generation before adding validation"
---

# JWT Authentication

Your API is open to anyone. That's a problem. You need to know WHO is making requests so you can:

- Return only THEIR tasks
- Apply rate limits per user
- Audit who did what

JWT (JSON Web Tokens) solves this. The user logs in once, gets a token, and includes it in every request. This lesson implements the token side. The next lesson adds secure password handling.

:::warning[Security Note]
This lesson uses a TEMPORARY insecure password check for demonstration. The next lesson (L09) replaces it with proper password hashing. Never use the placeholder password check in production.
:::

## How JWT Works

1. User sends username/password to `/token`
2. Server verifies credentials, creates signed token
3. Token contains user ID and expiration time
4. User includes token in future requests
5. Server validates token signature, extracts user

The key insight: **tokens are signed, not encrypted**. Anyone can read the payload, but only your server can create valid signatures.

## Installing Dependencies

```bash
uv add python-jose[cryptography]
```

- `python-jose` - JWT encoding/decoding library
- `[cryptography]` - Cryptographic backend for signing

## Settings for JWT

Add to your `config.py`:

```python
class Settings(BaseSettings):
    # ... existing settings ...

    secret_key: str  # For signing tokens
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30
```

Add to `.env`:

```bash
SECRET_KEY=your-secret-key-generate-with-openssl-rand-hex-32
```

Generate a secure key:

```bash
openssl rand -hex 32
```

## Creating Tokens

```python
# auth.py
from datetime import datetime, timedelta
from typing import Optional
from jose import jwt, JWTError
from config import get_settings

settings = get_settings()


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create a signed JWT token."""
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)

    to_encode.update({"exp": expire})

    encoded_jwt = jwt.encode(
        to_encode,
        settings.secret_key,
        algorithm=settings.algorithm
    )
    return encoded_jwt
```

**Output:**
```python
>>> from auth import create_access_token
>>> token = create_access_token({"sub": "user@example.com"})
>>> token
'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyQGV4YW1wbGUuY29tIiwiZXhwIjoxNzA1MzE4MjAwfQ.xxxxx'
```

The token has three parts (separated by dots):
1. **Header** - Algorithm info
2. **Payload** - Your data (`sub`, `exp`)
3. **Signature** - Proves the token is authentic

## Token Endpoint

FastAPI's OAuth2 expects a specific request format:

```python
# main.py
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from datetime import timedelta
from auth import create_access_token
from config import get_settings

app = FastAPI()
settings = get_settings()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


@app.post("/token")
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    """Issue JWT token for valid credentials."""

    # TEMPORARY: Insecure password check for demonstration
    # The next lesson replaces this with proper password hashing
    if form_data.password != "secret":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
    access_token = create_access_token(
        data={"sub": form_data.username},
        expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}
```

**Testing the endpoint:**

```bash
curl -X POST http://localhost:8000/token \
  -d "username=user@example.com&password=secret"
```

**Output:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer"
}
```

Note: OAuth2PasswordRequestForm expects form data (not JSON).

## Validating Tokens

```python
# auth.py (add to existing file)
def decode_token(token: str) -> Optional[dict]:
    """Decode and validate a JWT token."""
    try:
        payload = jwt.decode(
            token,
            settings.secret_key,
            algorithms=[settings.algorithm]
        )
        return payload
    except JWTError:
        return None
```

## Protecting Routes

Create a dependency that extracts the current user:

```python
# main.py (add to existing)
from auth import decode_token


async def get_current_user(token: str = Depends(oauth2_scheme)) -> str:
    """Extract and validate user from JWT token."""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    payload = decode_token(token)
    if payload is None:
        raise credentials_exception

    username: str = payload.get("sub")
    if username is None:
        raise credentials_exception

    return username


@app.get("/users/me")
async def read_users_me(current_user: str = Depends(get_current_user)):
    """Return current user info."""
    return {"username": current_user}
```

**Testing protected route:**

```bash
# Without token - fails
curl http://localhost:8000/users/me
# {"detail":"Not authenticated"}

# With token - succeeds
curl http://localhost:8000/users/me \
  -H "Authorization: Bearer eyJhbGci..."
# {"username": "user@example.com"}
```

## Protecting Task Routes

Apply authentication to your task endpoints:

```python
@app.post("/tasks", status_code=201)
def create_task(
    task: Task,
    session: Session = Depends(get_session),
    current_user: str = Depends(get_current_user)  # Add this
):
    task.owner = current_user  # Associate task with user
    session.add(task)
    session.commit()
    session.refresh(task)
    return task


@app.get("/tasks")
def list_tasks(
    session: Session = Depends(get_session),
    current_user: str = Depends(get_current_user)  # Add this
):
    # Only return tasks belonging to current user
    statement = select(Task).where(Task.owner == current_user)
    return session.exec(statement).all()
```

Now users can only see their own tasks.

## Complete Authentication Flow

```python
# auth.py
from datetime import datetime, timedelta
from typing import Optional
from jose import jwt, JWTError
from config import get_settings

settings = get_settings()


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=15))
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, settings.secret_key, algorithm=settings.algorithm)


def decode_token(token: str) -> Optional[dict]:
    try:
        return jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
    except JWTError:
        return None
```

```python
# main.py
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from datetime import timedelta
from auth import create_access_token, decode_token
from config import get_settings

app = FastAPI()
settings = get_settings()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


async def get_current_user(token: str = Depends(oauth2_scheme)) -> str:
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    payload = decode_token(token)
    if payload is None:
        raise credentials_exception
    username = payload.get("sub")
    if username is None:
        raise credentials_exception
    return username


@app.post("/token")
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    # TEMPORARY: Replace with proper password verification in L09
    if form_data.password != "secret":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token = create_access_token(
        data={"sub": form_data.username},
        expires_delta=timedelta(minutes=settings.access_token_expire_minutes)
    )
    return {"access_token": access_token, "token_type": "bearer"}


@app.get("/users/me")
async def read_users_me(current_user: str = Depends(get_current_user)):
    return {"username": current_user}
```

## Hands-On Exercise

**Step 1:** Add python-jose to your project:

```bash
uv add python-jose[cryptography]
```

**Step 2:** Update your Settings with JWT configuration

**Step 3:** Create auth.py with token creation/validation

**Step 4:** Add /token endpoint to main.py

**Step 5:** Create get_current_user dependency

**Step 6:** Test the flow:

```bash
# Get a token
curl -X POST http://localhost:8000/token \
  -d "username=test@example.com&password=secret"

# Use the token
curl http://localhost:8000/users/me \
  -H "Authorization: Bearer <your-token>"
```

**Step 7:** Add authentication to one of your task endpoints

## Swagger UI Integration

FastAPI's Swagger UI has built-in OAuth2 support. Open `/docs`:

1. Click the "Authorize" button (lock icon)
2. Enter username and password
3. Click "Authorize"
4. Now all your requests include the token automatically

This makes testing protected endpoints easy.

## Common Mistakes

**Mistake 1:** Using JSON for /token

```bash
# Wrong - OAuth2 expects form data
curl -X POST http://localhost:8000/token \
  -H "Content-Type: application/json" \
  -d '{"username": "test", "password": "secret"}'

# Correct - Use form data
curl -X POST http://localhost:8000/token \
  -d "username=test&password=secret"
```

**Mistake 2:** Forgetting WWW-Authenticate header

```python
# Wrong - missing header
raise HTTPException(status_code=401, detail="Not authenticated")

# Correct - includes header
raise HTTPException(
    status_code=status.HTTP_401_UNAUTHORIZED,
    detail="Not authenticated",
    headers={"WWW-Authenticate": "Bearer"},
)
```

**Mistake 3:** Exposing sensitive info in token

```python
# Wrong - password in token (anyone can decode!)
create_access_token({"sub": username, "password": password})

# Correct - only non-sensitive identifiers
create_access_token({"sub": username})
```

**Mistake 4:** Hardcoding the secret key

```python
# Wrong - exposed in code
SECRET_KEY = "my-secret-key"

# Correct - from environment
settings.secret_key  # Read from .env
```

## Security Reminder

The password check in this lesson is intentionally insecure:

```python
if form_data.password != "secret":  # NEVER do this in production
```

Real systems must:
1. Store hashed passwords (not plaintext)
2. Use timing-safe comparison
3. Apply rate limiting to prevent brute force

The next lesson (L09) implements proper password hashing and rate limiting.

## Try With AI

After completing the exercise, explore these scenarios.

**Prompt 1: Token Expiration Handling**

```text
My JWT tokens expire after 30 minutes. What happens when a user's
token expires mid-session? How should my frontend handle this?
Should I implement refresh tokens?
```

**What you're learning:** Token expiration is a UX and security tradeoff. Refresh tokens add complexity but improve user experience.

**Prompt 2: Custom Token Claims**

```text
I want to include user roles in my JWT token so I can check
permissions without a database query. Is this a good idea?
What claims should and shouldn't go in a JWT?
```

**What you're learning:** JWTs can carry any data, but there are tradeoffs. Understanding what belongs in tokens vs what needs database lookups is important.

**Prompt 3: Testing Authentication**

```text
How do I write pytest tests for my protected endpoints?
I need to:
1. Get a token in the test
2. Include it in requests
3. Test both authenticated and unauthenticated cases
```

**What you're learning:** Testing authentication requires setting up test users and tokens. There are patterns to make this clean.
