"""
Confidential Client (Secret) - FastAPI Integration
Uses pre-registered confidential client (no dynamic registration)

Prerequisites:
    1. Auth server running: cd auth-server && pnpm dev
    2. Register client via Admin UI: http://localhost:3001/admin/clients
       - Name: FastAPI Confidential Client
       - Type: Confidential
       - Redirect URI: http://localhost:8001/callback
    3. Copy the client_id and client_secret, set them below

Setup:
    cd examples/fastapi-integration
    uv venv && source .venv/bin/activate
    uv pip install fastapi uvicorn httpx

Run:
    uvicorn confidential_client:app --reload --port 8001

Test:
    http://localhost:8001 → Click "Login"
"""

from fastapi import FastAPI, HTTPException
from fastapi.responses import RedirectResponse, HTMLResponse
import httpx
import secrets

AUTH_SERVER = "http://localhost:3001"
REDIRECT_URI = "http://localhost:8001/callback"

# ⚠️ CONFIGURE THIS: Get credentials from Admin UI (http://localhost:3001/admin/clients)
# Register a CONFIDENTIAL client with redirect URI: http://localhost:8001/callback
CLIENT_ID = "your-confidential-client-id"  # From admin UI
CLIENT_SECRET = "your-confidential-client-secret"  # From admin UI (shown once on creation!)

# Session storage (use Redis in production)
sessions = {}

app = FastAPI(title="Confidential Client")

@app.get("/", response_class=HTMLResponse)
async def home():
    configured = CLIENT_ID != "your-confidential-client-id"
    
    if not configured:
        return """
        <html>
        <head><title>Confidential Client - Not Configured</title></head>
        <body style="font-family: system-ui; max-width: 600px; margin: 100px auto; padding: 20px;">
            <h1>⚠️ Client Not Configured</h1>
            <p>You need to register a confidential client first:</p>
            <ol>
                <li>Go to <a href="http://localhost:3001/admin/clients">Admin UI</a></li>
                <li>Click "Register New Client"</li>
                <li>Set Type: <strong>Confidential</strong></li>
                <li>Set Redirect URI: <code>http://localhost:8001/callback</code></li>
                <li>Copy the client_id and client_secret</li>
                <li>Update this file with the credentials</li>
            </ol>
            <p style="background: #fef3c7; padding: 12px; border-radius: 6px;">
                <strong>Important:</strong> The client_secret is only shown once when created!
            </p>
        </body>
        </html>
        """

    return f"""
    <html>
    <head><title>Confidential Client</title></head>
    <body style="font-family: system-ui; max-width: 600px; margin: 100px auto; text-align: center;">
        <h1>🔐 Confidential Client</h1>
        <p>Server-to-server OAuth 2.1 with Client Secret</p>
        <p><strong>Client ID:</strong> <code>{CLIENT_ID}</code></p>
        <p><strong>Client Secret:</strong> <code>{CLIENT_SECRET[:10]}...</code></p>
        <p><strong>Flow:</strong> Authorization Code + Basic Auth</p>
        <hr style="margin: 30px 0;">
        <a href="/login" style="display: inline-block; background: #059669; color: white;
           padding: 12px 24px; border-radius: 8px; text-decoration: none;">
            Login with Auth Server
        </a>
    </body>
    </html>
    """

@app.get("/login")
async def login():
    """Start OAuth flow"""
    if CLIENT_ID == "your-confidential-client-id":
        raise HTTPException(500, "Client not configured - see instructions at /")

    state = secrets.token_urlsafe(16)
    sessions[state] = {}

    # Authorization URL (no PKCE needed for confidential client)
    auth_url = (
        f"{AUTH_SERVER}/api/auth/oauth2/authorize"
        f"?client_id={CLIENT_ID}"
        f"&redirect_uri={REDIRECT_URI}"
        f"&response_type=code"
        f"&scope=openid profile email"
        f"&state={state}"
    )

    return RedirectResponse(auth_url)

@app.get("/callback")
async def callback(code: str, state: str):
    """Handle OAuth callback"""
    if state not in sessions:
        raise HTTPException(400, "Invalid state")

    sessions.pop(state)  # Remove used session

    async with httpx.AsyncClient() as client:
        # Exchange code with client secret (Basic Auth)
        token_response = await client.post(
            f"{AUTH_SERVER}/api/auth/oauth2/token",
            auth=(CLIENT_ID, CLIENT_SECRET),  # Basic Auth with secret
            data={
                "grant_type": "authorization_code",
                "code": code,
                "redirect_uri": REDIRECT_URI
            }
        )

        if token_response.status_code != 200:
            return HTMLResponse(f"""
            <h1>❌ Token Error</h1>
            <p>Status: {token_response.status_code}</p>
            <pre>{token_response.text}</pre>
            <p><a href="/">Try again</a></p>
            """)

        tokens = token_response.json()

        # Get user info
        userinfo = await client.get(
            f"{AUTH_SERVER}/api/auth/oauth2/userinfo",
            headers={"Authorization": f"Bearer {tokens['access_token']}"}
        )

        user = userinfo.json()

    return HTMLResponse(f"""
    <html>
    <head><title>Success</title></head>
    <body style="font-family: system-ui; max-width: 800px; margin: 50px auto; padding: 20px;">
        <h1>✅ Confidential Client Success!</h1>

        <div style="background: #d1fae5; padding: 16px; border-radius: 8px; margin: 20px 0;">
            <strong>Flow Verified:</strong>
            <ul style="margin: 10px 0 0 0;">
                <li>✓ Client secret sent via Basic Auth</li>
                <li>✓ Server-to-server authentication</li>
                <li>✓ No PKCE needed (secret provides security)</li>
                <li>✓ Access token received</li>
            </ul>
        </div>

        <div style="background: #f3f4f6; padding: 20px; border-radius: 8px; margin: 20px 0;">
            <h2>User Info</h2>
            <p><strong>Name:</strong> {user.get('name')}</p>
            <p><strong>Email:</strong> {user.get('email')}</p>
            <p><strong>User ID:</strong> <code>{user.get('sub')}</code></p>
        </div>

        <div style="background: #f3f4f6; padding: 20px; border-radius: 8px; margin: 20px 0;">
            <h2>Tenant Claims</h2>
            <p><strong>Tenant ID:</strong> <code>{user.get('tenant_id')}</code></p>
            <p><strong>Organizations:</strong> <code>{user.get('organization_ids')}</code></p>
            <p><strong>Role:</strong> <code>{user.get('org_role')}</code></p>
        </div>

        <div style="background: #f3f4f6; padding: 20px; border-radius: 8px; margin: 20px 0;">
            <h2>Tokens</h2>
            <p><strong>Access Token:</strong> <code>{tokens['access_token'][:40]}...</code></p>
            <p><strong>Expires In:</strong> {tokens['expires_in']} seconds</p>
            <p><strong>Has Refresh Token:</strong> {'✓' if 'refresh_token' in tokens else '✗'}</p>
        </div>

        <a href="/" style="display: inline-block; background: #059669; color: white;
           padding: 12px 24px; border-radius: 8px; text-decoration: none;">
            ← Back to Home
        </a>
    </body>
    </html>
    """)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
