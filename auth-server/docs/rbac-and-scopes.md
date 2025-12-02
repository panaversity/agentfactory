# RBAC & Scopes Documentation

This document describes the Role-Based Access Control (RBAC) system and OAuth scopes implemented in the RoboLearn Auth Server.

## OAuth Scopes

### Supported Scopes

| Scope | Description | Claims Included |
|-------|-------------|-----------------|
| `openid` | Required for OIDC. Returns user ID. | `sub`, `iss`, `aud`, `exp`, `iat` |
| `profile` | User profile information | `name`, `picture`, `given_name`, `family_name` |
| `email` | User email | `email`, `email_verified` |
| `offline_access` | Enables refresh tokens | (grants refresh_token) |

### Custom Claims (Always Included)

These claims are added to the userinfo response regardless of scope:

| Claim | Type | Description |
|-------|------|-------------|
| `role` | string | User role: "user" or "admin" |
| `software_background` | string | "beginner", "intermediate", "advanced" |
| `hardware_tier` | string | "tier1", "tier2", "tier3", "tier4" |
| `tenant_id` | string | Primary organization ID |
| `organization_ids` | string[] | All organizations |
| `org_role` | string | Role in primary organization |

### Requesting Scopes

When initiating the OAuth flow, request scopes in the `scope` parameter:

```
GET /api/auth/oauth2/authorize?
  client_id=your-client-id&
  redirect_uri=http://localhost:3000/callback&
  response_type=code&
  scope=openid%20profile%20email%20offline_access&
  code_challenge=...&
  code_challenge_method=S256
```

## User Roles

### System Roles

| Role | Description | Permissions |
|------|-------------|-------------|
| `user` | Default role for all users | Basic access, own profile |
| `admin` | Administrative access | All user permissions + admin features |

### Setting User Roles

```sql
-- Promote user to admin
UPDATE "user" SET role = 'admin' WHERE email = 'admin@example.com';

-- Demote to regular user
UPDATE "user" SET role = 'user' WHERE email = 'user@example.com';
```

### Admin Plugin Features

The admin plugin provides:

- User management (ban, unban, impersonate)
- User listing and search
- Role management

Admin endpoints require `role = 'admin'`:

```bash
# List all users (admin only)
GET /api/auth/admin/list-users

# Ban user (admin only)
POST /api/auth/admin/ban-user
{
  "userId": "user-id",
  "reason": "Violation of terms"
}

# Unban user (admin only)
POST /api/auth/admin/unban-user
{
  "userId": "user-id"
}
```

## Organization Roles

### Organization-Level Roles

Within an organization, users can have these roles:

| Role | Description | Permissions |
|------|-------------|-------------|
| `owner` | Organization creator | Full control, delete org, transfer ownership |
| `admin` | Organization administrator | Manage members, settings, invite |
| `member` | Regular member | Access org resources |

### Organization Membership

Users can belong to multiple organizations. The `tenant_id` claim represents the primary/active organization.

```json
{
  "sub": "user-id",
  "tenant_id": "org-123",
  "organization_ids": ["org-123", "org-456"],
  "org_role": "owner"
}
```

### Managing Organization Members

```bash
# Create organization
POST /api/auth/organization/create
{
  "name": "My Organization",
  "slug": "my-org"
}

# Invite member
POST /api/auth/organization/invite-member
{
  "email": "new-member@example.com",
  "role": "member"
}

# Update member role
POST /api/auth/organization/update-member-role
{
  "memberId": "member-id",
  "role": "admin"
}

# Remove member
POST /api/auth/organization/remove-member
{
  "memberId": "member-id"
}
```

## Access Control Patterns

### Frontend (React/Next.js)

```typescript
import { useSession } from '@/lib/auth-client';

function AdminPanel() {
  const { data: session } = useSession();

  // Check system role
  if (session?.user?.role !== 'admin') {
    return <div>Access denied</div>;
  }

  return <div>Admin content</div>;
}

function TenantDashboard() {
  const { data: session } = useSession();

  // Check organization membership
  if (!session?.user?.tenant_id) {
    return <div>Join an organization to access this feature</div>;
  }

  return <div>Tenant dashboard for {session.user.tenant_id}</div>;
}
```

### Backend (FastAPI)

```python
from app.auth import get_current_user, TokenPayload

# Role-based access
@app.get("/api/admin/dashboard")
async def admin_dashboard(user: TokenPayload = Depends(get_current_user)):
    if user.role != "admin":
        raise HTTPException(403, "Admin access required")
    return {"admin": True}

# Organization-based access
@app.get("/api/org/data")
async def org_data(user: TokenPayload = Depends(get_current_user)):
    if not user.tenant_id:
        raise HTTPException(403, "Organization membership required")

    # Scope data to tenant
    return db.query(Resource).filter(
        Resource.tenant_id == user.tenant_id
    ).all()

# Organization role check
@app.post("/api/org/settings")
async def update_org_settings(
    settings: OrgSettings,
    user: TokenPayload = Depends(get_current_user)
):
    if user.org_role not in ["owner", "admin"]:
        raise HTTPException(403, "Organization admin access required")
    # Update settings...
```

### Backend (MCP Server)

```python
# Tool visibility based on role
@server.list_tools()
async def list_tools(user: TokenPayload):
    tools = [
        {"name": "get_profile", "description": "Get your profile"}
    ]

    if user.role == "admin":
        tools.append({
            "name": "admin_stats",
            "description": "Get system statistics"
        })

    if user.tenant_id:
        tools.append({
            "name": "org_data",
            "description": "Get organization data"
        })

    return tools
```

## Hardware Tier Access Control

Control access based on user's hardware tier:

| Tier | Equipment | Use Case |
|------|-----------|----------|
| `tier1` | Laptop/Cloud | Browser-based, simulations |
| `tier2` | RTX GPU | Local Isaac Sim, Gazebo |
| `tier3` | Jetson Edge | Edge deployment |
| `tier4` | Physical Robot | Real-world testing |

```python
# Tier-based content gating
@app.get("/api/content/{content_id}")
async def get_content(
    content_id: str,
    user: TokenPayload = Depends(get_current_user)
):
    content = get_content_by_id(content_id)

    # Check tier requirement
    tier_map = {"tier1": 1, "tier2": 2, "tier3": 3, "tier4": 4}
    user_tier = tier_map.get(user.hardware_tier, 1)
    required_tier = tier_map.get(content.min_tier, 1)

    if user_tier < required_tier:
        raise HTTPException(
            403,
            f"This content requires {content.min_tier}. You have {user.hardware_tier}."
        )

    return content
```

## Best Practices

### 1. Defense in Depth

Always validate both:
- Token validity (JWT signature, expiration)
- Authorization (roles, organization membership)

### 2. Least Privilege

Grant minimum required permissions:
- Use `member` role by default
- Promote to `admin` only when needed
- Scope data access by `tenant_id`

### 3. Audit Logging

Log authorization decisions:
```python
logger.info(f"User {user.sub} accessed /api/admin with role {user.role}")
```

### 4. Token Handling

- Never store tokens in localStorage (use httpOnly cookies)
- Implement token refresh before expiration
- Clear tokens on logout

### 5. Organization Isolation

Always filter by `tenant_id` for multi-tenant data:
```python
# Good: Scoped query
resources = db.query(Resource).filter(
    Resource.tenant_id == user.tenant_id
).all()

# Bad: Unscoped query
resources = db.query(Resource).filter(
    Resource.id == resource_id
).first()  # Missing tenant check!
```

## Token Introspection

Decode a token to see claims:

```python
import jwt

# Decode without verification (for debugging only!)
payload = jwt.decode(token, options={"verify_signature": False})
print(payload)
```

Output:
```json
{
  "sub": "user-id",
  "email": "user@example.com",
  "role": "admin",
  "tenant_id": "org-123",
  "organization_ids": ["org-123"],
  "org_role": "owner",
  "software_background": "advanced",
  "hardware_tier": "tier2",
  "exp": 1234567890,
  "iat": 1234560000,
  "iss": "http://localhost:3001"
}
```
