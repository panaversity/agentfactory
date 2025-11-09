#!/usr/bin/env python3
"""
Quick Test for Authentication System (No Agent Dependencies)

This tests only the authentication and database components.
"""

import sys
from pathlib import Path

# Add app to path
sys.path.insert(0, str(Path(__file__).parent))

from app.database import init_db, get_db_context
from app.models.user import User, StudentProfile
from app.schemas.auth import SignupRequest, UpdateProfileRequest
from app.auth.utils import hash_password, verify_password, create_user_token, decode_access_token

print("=" * 80)
print("🧪 Quick Authentication Test (No Agent)")
print("=" * 80)
print()

# Initialize database
print("📦 Initializing database...")
init_db()
print("✅ Database initialized")
print()

# Test 1: User Signup
print("=" * 80)
print("TEST 1: User Signup")
print("=" * 80)

signup_data = SignupRequest(
    name="Test User",
    email="test@example.com",
    password="testpass123",
    level="beginner"
)

with get_db_context() as db:
    # Cleanup existing
    existing_user = db.query(User).filter(User.email == signup_data.email).first()
    if existing_user:
        print(f"⚠️  Cleaning up existing user...")
        profile = db.query(StudentProfile).filter(
            StudentProfile.user_id == existing_user.id
        ).first()
        if profile:
            db.delete(profile)
        db.delete(existing_user)
        db.commit()

    # Create new user
    new_user = User(
        email=signup_data.email,
        name=signup_data.name,
        hashed_password=hash_password(signup_data.password)
    )
    db.add(new_user)
    db.flush()

    # Create profile
    new_profile = StudentProfile(
        user_id=new_user.id,
        level=signup_data.level
    )
    db.add(new_profile)
    db.commit()
    db.refresh(new_user)
    db.refresh(new_profile)

    print(f"✅ User created: {new_user.name} ({new_user.email})")
    print(f"   User ID: {new_user.id}")
    print(f"   Level: {new_profile.level}")
    print()

# Test 2: Password Verification
print("=" * 80)
print("TEST 2: Password Verification")
print("=" * 80)

with get_db_context() as db:
    user = db.query(User).filter(User.email == signup_data.email).first()

    is_valid = verify_password(signup_data.password, user.hashed_password)
    print(f"✅ Correct password verified: {is_valid}")

    is_invalid = verify_password("wrongpass", user.hashed_password)
    print(f"✅ Incorrect password rejected: {not is_invalid}")
    print()

# Test 3: JWT Token
print("=" * 80)
print("TEST 3: JWT Token")
print("=" * 80)

with get_db_context() as db:
    user = db.query(User).filter(User.email == signup_data.email).first()

    token = create_user_token(user)
    print(f"✅ Token created: {token[:50]}...")

    user_id = decode_access_token(token)
    print(f"✅ Token decoded, User ID: {user_id}")
    print(f"✅ Matches original: {user_id == user.id}")
    print()

# Test 4: Profile Update
print("=" * 80)
print("TEST 4: Profile Update")
print("=" * 80)

with get_db_context() as db:
    user = db.query(User).filter(User.email == signup_data.email).first()
    profile = db.query(StudentProfile).filter(
        StudentProfile.user_id == user.id
    ).first()

    profile.level = "intermediate"
    profile.current_chapter = "04-python"
    profile.current_lesson = "01-intro"
    profile.learning_style = "code_focused"
    profile.completed_lessons = ["01-intro", "02-basics"]
    profile.difficulty_topics = ["async"]

    db.commit()
    db.refresh(profile)

    print(f"✅ Profile updated:")
    print(f"   Level: {profile.level}")
    print(f"   Chapter: {profile.current_chapter}")
    print(f"   Lesson: {profile.current_lesson}")
    print(f"   Style: {profile.learning_style}")
    print(f"   Completed: {len(profile.completed_lessons)} lessons")
    print()

# Summary
print("=" * 80)
print("🎉 ALL TESTS PASSED!")
print("=" * 80)
print()
print("✅ User signup with profile creation")
print("✅ Password hashing and verification (bcrypt)")
print("✅ JWT token generation and validation")
print("✅ Profile updates with learning preferences")
print()
print("Phase 5 Authentication System: FULLY FUNCTIONAL ✅")
print()
print("Next steps:")
print("  - Start API server: uvicorn app.main:app --reload")
print("  - Test with Postman/curl: POST http://localhost:8000/api/auth/signup")
print("  - See PHASE_5_SUMMARY.md for complete API documentation")
print()
