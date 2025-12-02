# FastAPI OAuth Integration Examples

Simple examples showing how to integrate FastAPI with the RoboLearn auth server.

## Security Model

**Dynamic client registration is DISABLED** for security. You must:

1. Register OAuth clients via Admin UI (http://localhost:3001/admin/clients)
2. Use the pre-registered client credentials in your app

This ensures only authorized applications can use your auth server.

## Setup

```bash
cd examples/fastapi-integration
uv venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
uv pip install fastapi uvicorn httpx
```

## Examples

### 1. Public Client (PKCE) - Port 8000

For browser-based apps, SPAs, mobile apps.

```bash
# 1. Register in Admin UI:
#    - Type: Public
#    - Redirect URI: http://localhost:8000/callback
#    - Copy the client_id

# 2. Update CLIENT_ID in public_client.py (or use default robolearn-public-client)

# 3. Run:
uvicorn public_client:app --reload --port 8000

# 4. Test: http://localhost:8000
```

**Flow:**

- Uses PKCE (code_challenge + code_verifier)
- No client secret needed
- Tokens exchanged with code_verifier

### 2. Confidential Client (Secret) - Port 8001

For server-side apps, APIs, microservices.

```bash
# 1. Register in Admin UI:
#    - Type: Confidential
#    - Redirect URI: http://localhost:8001/callback
#    - Copy the client_id AND client_secret (shown once!)

# 2. Update CLIENT_ID and CLIENT_SECRET in confidential_client.py

# 3. Run:
uvicorn confidential_client:app --reload --port 8001

# 4. Test: http://localhost:8001
```

**Flow:**

- Uses client secret (sent via Basic Auth)
- No PKCE needed (secret provides security)
- Tokens exchanged with client_secret

## Comparison

| Feature        | Public (PKCE)                | Confidential (Secret) |
| -------------- | ---------------------------- | --------------------- |
| Port           | 8000                         | 8001                  |
| Security       | code_verifier                | client_secret         |
| Use Case       | Browser/Mobile               | Server/API            |
| Secret Storage | None (can't hide in browser) | Server-side only      |

## Quick Test

```bash
# Terminal 1: Auth server
cd auth-server && pnpm dev

# Terminal 2: Public client
cd examples/fastapi-integration
source .venv/bin/activate
uvicorn public_client:app --reload --port 8000

# Terminal 3: Confidential client (after configuring)
uvicorn confidential_client:app --reload --port 8001
```

## Tenant Claims

Both examples display tenant/organization claims from the userinfo endpoint:

- `tenant_id` - Primary organization ID
- `organization_ids` - All organization memberships
- `org_role` - Role in primary organization
