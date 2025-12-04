#!/usr/bin/env python3
"""Integration test for PanaversityFS authentication.

Usage:
    # Test JWT authentication (Authorization: Bearer header)
    python scripts/test_auth_integration.py --bearer <jwt-token>

    # Test API key authentication (x-api-key header)
    python scripts/test_auth_integration.py --api-key <sk_test_xxx>

    # Test both
    python scripts/test_auth_integration.py --bearer <jwt> --api-key <key>

Environment:
    PANAVERSITY_AUTH_SERVER_URL: SSO server URL (required)

Example:
    export PANAVERSITY_AUTH_SERVER_URL=http://localhost:3001
    python scripts/test_auth_integration.py --api-key sk_test_xxxxx
"""

import argparse
import asyncio
import os
import sys

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


async def test_jwt_auth(token: str, auth_server_url: str) -> bool:
    """Test JWT Bearer token authentication."""
    from panaversity_fs.auth import JWKSTokenVerifier

    print(f"\n{'='*60}")
    print("Testing JWT Bearer Token Authentication")
    print(f"{'='*60}")
    print(f"SSO Server: {auth_server_url}")
    print(f"JWKS URL: {auth_server_url}/api/auth/jwks")
    print(f"Token (first 50 chars): {token[:50]}...")

    verifier = JWKSTokenVerifier(
        jwks_url=f"{auth_server_url}/api/auth/jwks",
        issuer=auth_server_url
    )

    try:
        result = await verifier.verify_token(token)

        if result:
            print(f"\n✅ JWT verification SUCCESSFUL")
            print(f"   User ID (sub): {result.client_id}")
            print(f"   Scopes: {result.scopes}")
            print(f"   Expires at: {result.expires_at}")
            return True
        else:
            print(f"\n❌ JWT verification FAILED")
            print("   Token is invalid, expired, or signature doesn't match")
            return False

    except Exception as e:
        print(f"\n❌ JWT verification ERROR: {e}")
        return False


async def test_api_key_auth(api_key: str, auth_server_url: str) -> bool:
    """Test API key authentication."""
    from panaversity_fs.auth import APIKeyVerifier

    print(f"\n{'='*60}")
    print("Testing API Key Authentication")
    print(f"{'='*60}")
    print(f"SSO Server: {auth_server_url}")
    print(f"Verify URL: {auth_server_url}/api/api-key/verify")
    print(f"API Key (first 20 chars): {api_key[:20]}...")

    verifier = APIKeyVerifier(
        verify_url=f"{auth_server_url}/api/api-key/verify"
    )

    try:
        result = await verifier.verify_api_key(api_key)

        if result:
            print(f"\n✅ API Key verification SUCCESSFUL")
            print(f"   User ID: {result.user_id}")
            print(f"   Auth Type: {result.auth_type}")
            print(f"   Permissions: {result.permissions}")
            print(f"   Metadata: {result.metadata}")
            return True
        else:
            print(f"\n❌ API Key verification FAILED")
            print("   Key is invalid or not found")
            return False

    except Exception as e:
        print(f"\n❌ API Key verification ERROR: {e}")
        return False
    finally:
        await verifier.close()


async def test_dual_auth_bearer(token: str, auth_server_url: str) -> bool:
    """Test DualAuthValidator with Bearer token."""
    from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier

    print(f"\n{'='*60}")
    print("Testing DualAuthValidator (Bearer)")
    print(f"{'='*60}")
    print(f"Authorization: Bearer {token[:30]}...")

    jwks_verifier = JWKSTokenVerifier(
        jwks_url=f"{auth_server_url}/api/auth/jwks",
        issuer=auth_server_url
    )
    api_key_verifier = APIKeyVerifier(
        verify_url=f"{auth_server_url}/api/auth/api-key/verify"
    )
    validator = DualAuthValidator(jwks_verifier, api_key_verifier)

    try:
        result = await validator.validate(authorization=f"Bearer {token}")

        if result:
            print(f"\n✅ DualAuth (Bearer) validation SUCCESSFUL")
            print(f"   User ID: {result.user_id}")
            print(f"   Auth Type: {result.auth_type}")
            print(f"   Role: {result.role}")
            print(f"   Tenant ID: {result.tenant_id}")
            print(f"   Permissions: {result.permissions}")
            return True
        else:
            print(f"\n❌ DualAuth (Bearer) validation FAILED")
            return False

    except Exception as e:
        print(f"\n❌ DualAuth (Bearer) validation ERROR: {e}")
        return False
    finally:
        await api_key_verifier.close()


async def test_dual_auth_api_key(api_key: str, auth_server_url: str) -> bool:
    """Test DualAuthValidator with x-api-key header."""
    from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier

    print(f"\n{'='*60}")
    print("Testing DualAuthValidator (x-api-key)")
    print(f"{'='*60}")
    print(f"x-api-key: {api_key[:20]}...")

    jwks_verifier = JWKSTokenVerifier(
        jwks_url=f"{auth_server_url}/api/auth/jwks",
        issuer=auth_server_url
    )
    api_key_verifier = APIKeyVerifier(
        verify_url=f"{auth_server_url}/api/api-key/verify"
    )
    validator = DualAuthValidator(jwks_verifier, api_key_verifier)

    try:
        result = await validator.validate(x_api_key=api_key)

        if result:
            print(f"\n✅ DualAuth (x-api-key) validation SUCCESSFUL")
            print(f"   User ID: {result.user_id}")
            print(f"   Auth Type: {result.auth_type}")
            print(f"   Role: {result.role}")
            print(f"   Tenant ID: {result.tenant_id}")
            print(f"   Permissions: {result.permissions}")
            return True
        else:
            print(f"\n❌ DualAuth (x-api-key) validation FAILED")
            return False

    except Exception as e:
        print(f"\n❌ DualAuth (x-api-key) validation ERROR: {e}")
        return False
    finally:
        await api_key_verifier.close()


async def main():
    parser = argparse.ArgumentParser(description="Test PanaversityFS authentication")
    parser.add_argument("--bearer", help="JWT Bearer token to test")
    parser.add_argument("--api-key", help="API key to test (sk_test_xxx or sk_live_xxx)")
    parser.add_argument("--server", help="SSO server URL (overrides env var)")
    args = parser.parse_args()

    # Get SSO server URL
    auth_server_url = args.server or os.environ.get("PANAVERSITY_AUTH_SERVER_URL")

    if not auth_server_url:
        print("❌ ERROR: PANAVERSITY_AUTH_SERVER_URL not set")
        print("   Set it via environment variable or --server flag")
        print("   Example: export PANAVERSITY_AUTH_SERVER_URL=https://sso.panaversity.org")
        sys.exit(1)

    if not args.bearer and not args.api_key:
        print("❌ ERROR: Provide --bearer <token> or --api-key <key>")
        parser.print_help()
        sys.exit(1)

    auth_server_url = auth_server_url.rstrip('/')
    results = []

    # Test JWT if provided
    if args.bearer:
        success = await test_jwt_auth(args.bearer, auth_server_url)
        results.append(("JWT Bearer", success))

        # Also test via DualAuthValidator
        success = await test_dual_auth_bearer(args.bearer, auth_server_url)
        results.append(("DualAuth (Bearer)", success))

    # Test API key if provided
    if args.api_key:
        success = await test_api_key_auth(args.api_key, auth_server_url)
        results.append(("API Key", success))

        # Also test via DualAuthValidator with x-api-key header
        success = await test_dual_auth_api_key(args.api_key, auth_server_url)
        results.append(("DualAuth (x-api-key)", success))

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    all_passed = True
    for name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"  {name}: {status}")
        if not success:
            all_passed = False

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    asyncio.run(main())
