# JWT & JWKS Documentation

This document explains the JWT (JSON Web Token) and JWKS (JSON Web Key Set) implementation in the RoboLearn Auth Server.

## Overview

The auth server uses **RS256** (RSA with SHA-256) for signing tokens instead of symmetric HS256. This provides:

- **Public key verification**: Clients can verify tokens without shared secrets
- **Standard OIDC compliance**: Compatible with all OIDC-compliant libraries
- **Key rotation support**: Keys can be rotated without invalidating existing tokens

## Endpoints

### JWKS Endpoint

```
GET /api/auth/jwks
```

Returns the public keys used to verify JWT signatures:

```json
{
  "keys": [
    {
      "kty": "RSA",
      "kid": "key-id-12345",
      "alg": "RS256",
      "use": "sig",
      "n": "base64url-encoded-modulus",
      "e": "AQAB"
    }
  ]
}
```

### OIDC Discovery

```
GET /api/auth/.well-known/openid-configuration
```

Includes the JWKS URI:

```json
{
  "issuer": "http://localhost:3001",
  "jwks_uri": "http://localhost:3001/api/auth/jwks",
  "id_token_signing_alg_values_supported": ["RS256", "EdDSA", "none"],
  ...
}
```

## Token Structure

### ID Token

The ID token is a JWT with three parts:

```
header.payload.signature
```

#### Header

```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "key-id-12345"
}
```

#### Payload

```json
{
  "sub": "user-unique-id",
  "iss": "http://localhost:3001",
  "aud": "client-id",
  "exp": 1234567890,
  "iat": 1234560000,
  "email": "user@example.com",
  "name": "User Name",
  "email_verified": true,
  "role": "user",
  "tenant_id": "org-id",
  "organization_ids": ["org-id"],
  "org_role": "owner",
  "software_background": "advanced",
  "hardware_tier": "tier1"
}
```

### Access Token

The access token is an opaque string used to authenticate API requests. Use it with the `Authorization: Bearer` header.

## Token Verification

### Node.js / TypeScript

```typescript
import { createRemoteJWKSet, jwtVerify } from 'jose';

const JWKS = createRemoteJWKSet(
  new URL('http://localhost:3001/api/auth/jwks')
);

async function verifyToken(token: string) {
  const { payload, protectedHeader } = await jwtVerify(token, JWKS, {
    issuer: 'http://localhost:3001',
  });
  return payload;
}

// Usage
const user = await verifyToken(idToken);
console.log(user.email, user.role);
```

### Python

```python
from jose import jwt
import httpx

JWKS_URL = "http://localhost:3001/api/auth/jwks"
ISSUER = "http://localhost:3001"

async def get_jwks():
    async with httpx.AsyncClient() as client:
        response = await client.get(JWKS_URL)
        return response.json()

async def verify_token(token: str):
    jwks = await get_jwks()
    header = jwt.get_unverified_header(token)

    for key in jwks["keys"]:
        if key["kid"] == header["kid"]:
            payload = jwt.decode(
                token,
                key,
                algorithms=["RS256"],
                issuer=ISSUER
            )
            return payload

    raise Exception("Key not found")
```

### Go

```go
import (
    "github.com/golang-jwt/jwt/v5"
    "github.com/lestrrat-go/jwx/v2/jwk"
)

func verifyToken(tokenString string) (*jwt.Token, error) {
    keySet, err := jwk.Fetch(context.Background(), "http://localhost:3001/api/auth/jwks")
    if err != nil {
        return nil, err
    }

    token, err := jwt.Parse(tokenString, func(token *jwt.Token) (interface{}, error) {
        kid, ok := token.Header["kid"].(string)
        if !ok {
            return nil, fmt.Errorf("kid header not found")
        }

        key, found := keySet.LookupKeyID(kid)
        if !found {
            return nil, fmt.Errorf("key not found")
        }

        var pubKey interface{}
        if err := key.Raw(&pubKey); err != nil {
            return nil, err
        }
        return pubKey, nil
    })

    return token, err
}
```

## Key Storage

### Database Schema

Keys are stored in the `jwks` table:

```sql
CREATE TABLE jwks (
  id TEXT PRIMARY KEY,
  public_key TEXT NOT NULL,
  private_key TEXT NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  expires_at TIMESTAMP
);
```

### Key Configuration

In `auth.ts`:

```typescript
import { jwt } from "better-auth/plugins";

plugins: [
  jwt({
    jwks: {
      keyPairConfig: {
        alg: "RS256",
      },
      disablePrivateKeyEncryption: true,
    },
  }),
]
```

## Key Rotation

### Automatic Rotation

Better Auth handles key rotation automatically. Old keys remain valid until their expiration.

### Manual Rotation

To force key rotation:

1. Generate new key pair
2. Add to JWKS endpoint
3. Wait for cache expiration
4. Remove old key

## Caching Recommendations

### Client-Side Caching

```typescript
// Cache JWKS for 1 hour
const jwksCache = new Map();
const CACHE_TTL = 60 * 60 * 1000; // 1 hour

async function getCachedJWKS() {
  const cached = jwksCache.get('jwks');
  if (cached && Date.now() - cached.timestamp < CACHE_TTL) {
    return cached.data;
  }

  const jwks = await fetch('http://localhost:3001/api/auth/jwks')
    .then(r => r.json());

  jwksCache.set('jwks', { data: jwks, timestamp: Date.now() });
  return jwks;
}
```

### Recommended TTL

| Environment | JWKS Cache TTL |
|-------------|----------------|
| Development | 5 minutes |
| Production | 1 hour |

## Security Considerations

1. **Always verify signature**: Never use tokens without verification
2. **Check expiration**: Verify `exp` claim is in the future
3. **Validate issuer**: Ensure `iss` matches your auth server
4. **Use HTTPS**: Always use HTTPS in production
5. **Rotate keys**: Enable key rotation in production

## Debugging

### Decode Token Without Verification

**Warning**: Only use for debugging. Never trust unverified tokens.

```bash
# Using jwt.io
# Paste token at https://jwt.io

# Using Node.js
node -e "console.log(JSON.parse(Buffer.from('TOKEN'.split('.')[1], 'base64url')))"

# Using Python
python -c "import base64, json; print(json.loads(base64.urlsafe_b64decode('TOKEN'.split('.')[1] + '==')))"
```

### Verify JWKS Endpoint

```bash
curl http://localhost:3001/api/auth/jwks | jq .
```

### Check Token Claims

```bash
# Get a token
node tests/test-pkce-oauth.js

# Decode ID token (don't verify)
echo "ID_TOKEN" | cut -d'.' -f2 | base64 -d | jq .
```
