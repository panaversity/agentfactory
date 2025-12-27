---
sidebar_position: 9
title: "Password Hashing & Rate Limiting"
description: "Secure your authentication with Argon2 password hashing and protect against brute force with rate limiting"
keywords: [password-hashing, argon2, rate-limiting, security, slowapi, pwdlib]
chapter: 40
lesson: 9
duration_minutes: 45

skills:
  - name: "Password Hashing with Argon2"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "4.2 Protecting Personal Data"
    measurable_at_this_level: "Student hashes passwords with pwdlib/Argon2"

  - name: "Rate Limiting Implementation"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "4.1 Protecting Devices"
    measurable_at_this_level: "Student applies rate limits to sensitive endpoints"

  - name: "Security Headers"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "4.2 Protecting Personal Data"
    measurable_at_this_level: "Student explains purpose of rate limit headers"

learning_objectives:
  - objective: "Hash passwords securely with Argon2"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Passwords stored as hashes, verification works"

  - objective: "Apply rate limits to authentication endpoints"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Token endpoint returns 429 after limit exceeded"

  - objective: "Implement user signup with hashed passwords"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "POST /users/signup creates user with hashed password"

cognitive_load:
  new_concepts: 4
  assessment: "pwdlib, Argon2Hasher, slowapi, rate limit decorator"

differentiation:
  extension_for_advanced: "Add Redis-backed rate limiting for distributed systems"
  remedial_for_struggling: "Focus on password hashing before adding rate limiting"
---

# Password Hashing & Rate Limiting

The previous lesson used a placeholder password check. That's a critical security vulnerability. This lesson fixes it with:

1. **Argon2 password hashing** - Store passwords securely
2. **Rate limiting** - Prevent brute force attacks

After this lesson, your authentication system is production-ready.

## Why Password Hashing Matters

**Never store plaintext passwords.** If your database leaks:

- Plaintext: Attacker has all passwords immediately
- Hashed: Attacker has to crack each hash individually

Argon2 is the current recommendation for password hashing. It's:
- Memory-hard (expensive to parallelize)
- Configurable difficulty (can increase over time)
- Winner of the Password Hashing Competition

## Installing Dependencies

```bash
uv add pwdlib[argon2] slowapi
```

- `pwdlib` - Modern password hashing library
- `[argon2]` - Argon2 algorithm support
- `slowapi` - Rate limiting for FastAPI

## Password Hashing with pwdlib

```python
# security.py
from pwdlib import PasswordHash
from pwdlib.hashers.argon2 import Argon2Hasher

password_hash = PasswordHash((Argon2Hasher(),))


def hash_password(password: str) -> str:
    """Hash a password with Argon2."""
    return password_hash.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash."""
    return password_hash.verify(plain_password, hashed_password)
```

**Output:**
```python
>>> from security import hash_password, verify_password
>>> hashed = hash_password("mysecret")
>>> hashed
'$argon2id$v=19$m=65536,t=3,p=4$...'
>>> verify_password("mysecret", hashed)
True
>>> verify_password("wrongpassword", hashed)
False
```

Notice the hash includes algorithm parameters. This means you can upgrade security over time without breaking existing hashes.

## User Model with Password

Update your models to include users:

```python
# models.py
from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime


class User(SQLModel, table=True):
    """User account with hashed password."""
    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(unique=True, index=True)
    hashed_password: str
    created_at: datetime = Field(default_factory=datetime.utcnow)


class UserCreate(SQLModel):
    """Request model for user signup."""
    email: str
    password: str
```

## Signup Endpoint

```python
# main.py
from fastapi import FastAPI, Depends, HTTPException, status
from sqlmodel import Session, select
from models import User, UserCreate
from security import hash_password
from database import get_session

app = FastAPI()


@app.post("/users/signup", status_code=201)
def signup(
    user_data: UserCreate,
    session: Session = Depends(get_session)
):
    """Create a new user account."""
    # Check if email already exists
    existing = session.exec(
        select(User).where(User.email == user_data.email)
    ).first()

    if existing:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Create user with hashed password
    user = User(
        email=user_data.email,
        hashed_password=hash_password(user_data.password)
    )

    session.add(user)
    session.commit()
    session.refresh(user)

    # Don't return the password hash!
    return {"id": user.id, "email": user.email}
```

**Output:**
```bash
curl -X POST http://localhost:8000/users/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "user@example.com", "password": "SecurePass123"}'

{"id": 1, "email": "user@example.com"}
```

## Fixed Login Endpoint

Replace the insecure password check from L08:

```python
# main.py
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from security import verify_password
from auth import create_access_token

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


@app.post("/token")
async def login(
    form_data: OAuth2PasswordRequestForm = Depends(),
    session: Session = Depends(get_session)
):
    """Authenticate user and return JWT token."""
    # Find user by email
    user = session.exec(
        select(User).where(User.email == form_data.username)
    ).first()

    # Verify password (constant-time comparison built into pwdlib)
    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",  # Generic message
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token = create_access_token(data={"sub": user.email})
    return {"access_token": access_token, "token_type": "bearer"}
```

**Key security points:**

1. **Generic error message** - Don't reveal whether email exists
2. **Constant-time comparison** - Prevents timing attacks (built into pwdlib)
3. **Check user exists AND password matches** - Single error for both

## Rate Limiting with slowapi

Rate limiting prevents brute force attacks. An attacker can't try millions of passwords if they're limited to 5 attempts per minute.

```python
# main.py
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)


@app.post("/token")
@limiter.limit("5/minute")
async def login(
    request: Request,  # Required for rate limiter
    form_data: OAuth2PasswordRequestForm = Depends(),
    session: Session = Depends(get_session)
):
    # ... same implementation as above
```

**Testing rate limits:**

```bash
# First 5 requests succeed
for i in {1..5}; do
  curl -X POST http://localhost:8000/token \
    -d "username=test@example.com&password=wrong"
done

# 6th request is blocked
curl -X POST http://localhost:8000/token \
  -d "username=test@example.com&password=wrong"

# {"error": "Rate limit exceeded: 5 per 1 minute"}
```

## Rate Limit Headers

Successful responses include rate limit info:

```
X-RateLimit-Limit: 5
X-RateLimit-Remaining: 4
X-RateLimit-Reset: 1705318200
```

Clients can use these to implement backoff strategies.

## Complete Security Module

```python
# security.py
from pwdlib import PasswordHash
from pwdlib.hashers.argon2 import Argon2Hasher

password_hash = PasswordHash((Argon2Hasher(),))


def hash_password(password: str) -> str:
    """Hash a password with Argon2."""
    return password_hash.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash."""
    return password_hash.verify(plain_password, hashed_password)
```

```python
# main.py (relevant parts)
from fastapi import FastAPI, Depends, HTTPException, status, Request
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from sqlmodel import Session, select
from models import User, UserCreate
from security import hash_password, verify_password
from auth import create_access_token
from database import get_session

app = FastAPI()

# Rate limiting setup
limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


@app.post("/users/signup", status_code=201)
@limiter.limit("5/minute")
def signup(
    request: Request,
    user_data: UserCreate,
    session: Session = Depends(get_session)
):
    existing = session.exec(
        select(User).where(User.email == user_data.email)
    ).first()
    if existing:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    user = User(
        email=user_data.email,
        hashed_password=hash_password(user_data.password)
    )
    session.add(user)
    session.commit()
    session.refresh(user)
    return {"id": user.id, "email": user.email}


@app.post("/token")
@limiter.limit("5/minute")
async def login(
    request: Request,
    form_data: OAuth2PasswordRequestForm = Depends(),
    session: Session = Depends(get_session)
):
    user = session.exec(
        select(User).where(User.email == form_data.username)
    ).first()

    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token = create_access_token(data={"sub": user.email})
    return {"access_token": access_token, "token_type": "bearer"}
```

## Hands-On Exercise

**Step 1:** Install dependencies:

```bash
uv add pwdlib[argon2] slowapi
```

**Step 2:** Create security.py with hash/verify functions

**Step 3:** Add User model to models.py

**Step 4:** Implement /users/signup endpoint

**Step 5:** Update /token to use database and password verification

**Step 6:** Add rate limiting to both endpoints

**Step 7:** Test the flow:

```bash
# Signup
curl -X POST http://localhost:8000/users/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "MySecure123"}'

# Login
curl -X POST http://localhost:8000/token \
  -d "username=test@example.com&password=MySecure123"

# Test rate limit (run many times)
curl -X POST http://localhost:8000/token \
  -d "username=test@example.com&password=wrong"
```

## Common Mistakes

**Mistake 1:** Storing plaintext passwords

```python
# NEVER do this
user = User(email=email, password=password)

# ALWAYS hash
user = User(email=email, hashed_password=hash_password(password))
```

**Mistake 2:** Revealing whether email exists

```python
# Wrong - reveals user existence
if not user:
    raise HTTPException(detail="User not found")
if not verify_password(...):
    raise HTTPException(detail="Wrong password")

# Correct - generic message
if not user or not verify_password(...):
    raise HTTPException(detail="Incorrect email or password")
```

**Mistake 3:** Returning password hash in response

```python
# Wrong - exposes hash
return user

# Correct - only return safe fields
return {"id": user.id, "email": user.email}
```

**Mistake 4:** Forgetting Request parameter for rate limiter

```python
# Wrong - rate limiter won't work
@limiter.limit("5/minute")
def login(form_data: OAuth2PasswordRequestForm = Depends()):
    ...

# Correct - include Request
@limiter.limit("5/minute")
def login(request: Request, form_data: OAuth2PasswordRequestForm = Depends()):
    ...
```

## Security Checklist

Before deploying authentication:

- [ ] Passwords hashed with Argon2 (not MD5, SHA1, or plaintext)
- [ ] Rate limiting on /token and /signup
- [ ] Generic error messages (no email enumeration)
- [ ] No password hashes in responses
- [ ] HTTPS only in production
- [ ] Secret key from environment, not code

## Try With AI

After completing the exercise, explore these scenarios.

**Prompt 1: Password Requirements**

```text
I want to enforce password requirements:
- Minimum 8 characters
- At least one uppercase, one lowercase, one number

Should I validate in the Pydantic model or security.py?
Show me both approaches with tradeoffs.
```

**What you're learning:** Input validation location matters. Pydantic validates at API boundary; security module validates at hashing time. Both have uses.

**Prompt 2: Account Lockout**

```text
Rate limiting helps, but a determined attacker might wait between
attempts. How do I implement account lockout after 5 failed
attempts? What about unlocking - time-based or manual?
```

**What you're learning:** Rate limiting and account lockout are complementary. Understanding both gives you defense in depth.

**Prompt 3: Distributed Rate Limiting**

```text
slowapi uses in-memory storage. If I run multiple API instances
behind a load balancer, rate limits aren't shared. How do I
implement Redis-backed rate limiting for distributed systems?
```

**What you're learning:** In-memory rate limiting fails with horizontal scaling. Redis provides shared state across instances.
