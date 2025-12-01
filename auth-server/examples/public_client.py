"""
Public Client (PKCE) - FastAPI Integration
Uses pre-registered public client (no dynamic registration)

Prerequisites:
    1. Auth server running: cd auth-server && pnpm dev
    2. Register client via Admin UI: http://localhost:3001/admin/clients
       - Name: FastAPI Public Client
       - Type: Public
       - Redirect URI: http://localhost:8000/callback
    3. Copy the client_id and set CLIENT_ID below

Setup:
    cd examples/fastapi-integration
    uv venv && source .venv/bin/activate
    uv pip install fastapi uvicorn httpx

Run:
    uvicorn public_client:app --reload --port 8000

Test:
    http://localhost:8000 → Click "Login"
"""

from fastapi import FastAPI, HTTPException
from fastapi.responses import RedirectResponse, HTMLResponse
import httpx
import secrets
import base64
import hashlib

AUTH_SERVER = "http://localhost:3001"
REDIRECT_URI = "http://localhost:8000/callback"

# ⚠️ CONFIGURE THIS: Get client_id from Admin UI (http://localhost:3001/admin/clients)
# Register a PUBLIC client with redirect URI: http://localhost:8000/callback
CLIENT_ID = "robolearn-public-client"  # Or your custom client_id from admin

# Session storage (use Redis in production)
sessions = {}

app = FastAPI(title="Public Client (PKCE)")

@app.get("/", response_class=HTMLResponse)
async def home():
    return f"""
    <html>
    <head><title>Public Client (PKCE)</title></head>
    <body style="font-family: system-ui; max-width: 600px; margin: 100px auto; text-align: center;">
        <h1>🔓 Public Client (PKCE)</h1>
        <p>Browser-based OAuth 2.1 with PKCE</p>
        <p><strong>Client ID:</strong> <code>{CLIENT_ID}</code></p>
        <p><strong>Flow:</strong> Authorization Code + PKCE (no secret)</p>
        <hr style="margin: 30px 0;">
        <a href="/login" style="display: inline-block; background: #4F46E5; color: white;
           padding: 12px 24px; border-radius: 8px; text-decoration: none;">
            Login with Auth Server
        </a>
        <p style="margin-top: 30px; color: #666; font-size: 14px;">
            <strong>Note:</strong> Ensure client is registered in 
            <a href="http://localhost:3001/admin/clients">Admin UI</a>
        </p>
    </body>
    </html>
    """

@app.get("/login")
async def login():
    """Start OAuth flow with PKCE"""
    # Generate PKCE challenge
    code_verifier = secrets.token_urlsafe(64)
    code_challenge = base64.urlsafe_b64encode(
        hashlib.sha256(code_verifier.encode()).digest()
    ).decode().rstrip('=')

    state = secrets.token_urlsafe(16)
    sessions[state] = {"code_verifier": code_verifier}

    # Authorization URL with PKCE
    auth_url = (
        f"{AUTH_SERVER}/api/auth/oauth2/authorize"
        f"?client_id={CLIENT_ID}"
        f"&redirect_uri={REDIRECT_URI}"
        f"&response_type=code"
        f"&scope=openid profile email"
        f"&code_challenge={code_challenge}"
        f"&code_challenge_method=S256"
        f"&state={state}"
    )

    return RedirectResponse(auth_url)

@app.get("/callback")
async def callback(code: str, state: str):
    """Handle OAuth callback"""
    if state not in sessions:
        raise HTTPException(400, "Invalid state")

    session = sessions.pop(state)  # Remove used session

    async with httpx.AsyncClient() as client:
        # Exchange code with PKCE verifier (NO client secret)
        token_response = await client.post(
            f"{AUTH_SERVER}/api/auth/oauth2/token",
            data={
                "grant_type": "authorization_code",
                "code": code,
                "redirect_uri": REDIRECT_URI,
                "client_id": CLIENT_ID,
                "code_verifier": session["code_verifier"]
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
        <h1>✅ Public Client (PKCE) Success!</h1>

        <div style="background: #d1fae5; padding: 16px; border-radius: 8px; margin: 20px 0;">
            <strong>Flow Verified:</strong>
            <ul style="margin: 10px 0 0 0;">
                <li>✓ PKCE code challenge sent</li>
                <li>✓ Code verifier validated</li>
                <li>✓ No client secret used</li>
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
        </div>

        <a href="/" style="display: inline-block; background: #4F46E5; color: white;
           padding: 12px 24px; border-radius: 8px; text-decoration: none;">
            ← Back to Home
        </a>
    </body>
    </html>
    """)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
