"""
Authentication utilities.

This module provides:
- Password hashing and verification using bcrypt directly
- JWT token generation and validation
- User authentication helpers
"""

from datetime import datetime, timedelta
from typing import Optional
import os
import bcrypt
from jose import JWTError, jwt

# JWT configuration
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24 * 7  # 7 days


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt directly.

    Args:
        password: Plain text password

    Returns:
        str: Hashed password

    Example:
        >>> hashed = hash_password("mysecretpassword")
        >>> print(hashed)
        $2b$12$...
    """
    # Generate salt and hash password
    salt = bcrypt.gensalt()
    hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
    return hashed.decode('utf-8')


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a password against a hash using bcrypt directly.

    Args:
        plain_password: Plain text password to verify
        hashed_password: Hashed password to compare against

    Returns:
        bool: True if password matches, False otherwise

    Example:
        >>> hashed = hash_password("mypassword")
        >>> verify_password("mypassword", hashed)
        True
        >>> verify_password("wrongpassword", hashed)
        False
    """
    return bcrypt.checkpw(
        plain_password.encode('utf-8'),
        hashed_password.encode('utf-8')
    )


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        data: Dictionary with user data to encode in token
        expires_delta: Optional custom expiration time

    Returns:
        str: Encoded JWT token

    Example:
        >>> token = create_access_token({"sub": "user@example.com"})
        >>> print(token)
        eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

    return encoded_jwt


def decode_access_token(token: str) -> Optional[dict]:
    """
    Decode and validate a JWT access token.

    Args:
        token: JWT token string

    Returns:
        dict: Decoded token payload, or None if invalid

    Example:
        >>> token = create_access_token({"sub": "user@example.com"})
        >>> payload = decode_access_token(token)
        >>> print(payload["sub"])
        user@example.com
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None


def get_user_id_from_token(token: str) -> Optional[str]:
    """
    Extract user ID from JWT token.

    Args:
        token: JWT token string

    Returns:
        str: User ID, or None if token is invalid

    Example:
        >>> token = create_access_token({"sub": "user123"})
        >>> user_id = get_user_id_from_token(token)
        >>> print(user_id)
        user123
    """
    payload = decode_access_token(token)

    if payload is None:
        return None

    return payload.get("sub")


def create_user_token(user) -> str:
    """
    Create a JWT token for a user.

    Args:
        user: User object with id and email attributes

    Returns:
        str: JWT token

    Example:
        >>> from app.models.user import User
        >>> user = User(id="user123", email="ahmed@example.com")
        >>> token = create_user_token(user)
        >>> payload = decode_access_token(token)
        >>> print(payload["sub"], payload["email"])
        user123 ahmed@example.com
    """
    return create_access_token(
        data={
            "sub": user.id,
            "email": user.email,
            "type": "access"
        }
    )
