/**
 * Industry-Standard Scopes for Panaversity SSO
 *
 * Format: `domain:resource:action` or `resource:action`
 *
 * Standards followed:
 * - OAuth 2.0 scope format (RFC 6749)
 * - OpenID Connect standard scopes
 * - GitHub/Google-style granular permissions
 */

// =============================================================================
// SCOPE DEFINITIONS
// =============================================================================

export type ScopeAction = "read" | "write" | "delete" | "list" | "verify" | "manage" | "*";

export interface ScopeDefinition {
  /** Unique scope identifier (e.g., "users:read") */
  id: string;
  /** Human-readable name */
  name: string;
  /** Description of what this scope allows */
  description: string;
  /** Category for UI grouping */
  category: ScopeCategory;
  /** Whether this is a sensitive/admin scope */
  sensitive?: boolean;
  /** Resource type this scope applies to */
  resource: string;
  /** Action this scope allows */
  action: ScopeAction;
}

export type ScopeCategory =
  | "users"
  | "organizations"
  | "auth"
  | "profile"
  | "clients"
  | "admin";

// =============================================================================
// STANDARD SCOPES - Core SSO functionality
// =============================================================================

export const SCOPES: Record<string, ScopeDefinition> = {
  // ---------------------------------------------------------------------------
  // User Scopes - Managing user accounts
  // ---------------------------------------------------------------------------
  "users:read": {
    id: "users:read",
    name: "Read Users",
    description: "Read user information by ID or email",
    category: "users",
    resource: "users",
    action: "read",
  },
  "users:public:read": {
    id: "users:public:read",
    name: "Read Public Profiles",
    description: "Read public user profile data (name, username, avatar) - excludes email, phone",
    category: "users",
    resource: "users-public",
    action: "read",
  },
  "users:list": {
    id: "users:list",
    name: "List Users",
    description: "List and search all users",
    category: "users",
    resource: "users",
    action: "list",
    sensitive: true,
  },
  "users:write": {
    id: "users:write",
    name: "Write Users",
    description: "Create and update user accounts",
    category: "users",
    resource: "users",
    action: "write",
    sensitive: true,
  },
  "users:delete": {
    id: "users:delete",
    name: "Delete Users",
    description: "Delete user accounts",
    category: "users",
    resource: "users",
    action: "delete",
    sensitive: true,
  },

  // ---------------------------------------------------------------------------
  // Profile Scopes - User profile data
  // ---------------------------------------------------------------------------
  "profile:read": {
    id: "profile:read",
    name: "Read Profile",
    description: "Read user profile data (name, email, avatar)",
    category: "profile",
    resource: "profile",
    action: "read",
  },
  "profile:write": {
    id: "profile:write",
    name: "Write Profile",
    description: "Update user profile data",
    category: "profile",
    resource: "profile",
    action: "write",
  },

  // ---------------------------------------------------------------------------
  // Organization Scopes - Multi-tenancy
  // ---------------------------------------------------------------------------
  "orgs:read": {
    id: "orgs:read",
    name: "Read Organizations",
    description: "Read organization information",
    category: "organizations",
    resource: "organizations",
    action: "read",
  },
  "orgs:write": {
    id: "orgs:write",
    name: "Write Organizations",
    description: "Create and update organizations",
    category: "organizations",
    resource: "organizations",
    action: "write",
    sensitive: true,
  },
  "orgs:members:read": {
    id: "orgs:members:read",
    name: "Read Members",
    description: "Read organization membership",
    category: "organizations",
    resource: "organization-members",
    action: "read",
  },
  "orgs:members:write": {
    id: "orgs:members:write",
    name: "Manage Members",
    description: "Add/remove organization members",
    category: "organizations",
    resource: "organization-members",
    action: "write",
    sensitive: true,
  },

  // ---------------------------------------------------------------------------
  // Auth Scopes - Token and session operations
  // ---------------------------------------------------------------------------
  "auth:verify": {
    id: "auth:verify",
    name: "Verify Auth",
    description: "Verify tokens and API keys",
    category: "auth",
    resource: "auth",
    action: "verify",
  },
  "auth:introspect": {
    id: "auth:introspect",
    name: "Introspect Tokens",
    description: "Introspect access tokens for claims",
    category: "auth",
    resource: "auth",
    action: "read",
  },
  "auth:sessions:read": {
    id: "auth:sessions:read",
    name: "Read Sessions",
    description: "Read user session information",
    category: "auth",
    resource: "sessions",
    action: "read",
  },
  "auth:sessions:revoke": {
    id: "auth:sessions:revoke",
    name: "Revoke Sessions",
    description: "Revoke user sessions",
    category: "auth",
    resource: "sessions",
    action: "delete",
    sensitive: true,
  },

  // ---------------------------------------------------------------------------
  // OAuth Client Scopes - Client management
  // ---------------------------------------------------------------------------
  "clients:read": {
    id: "clients:read",
    name: "Read Clients",
    description: "Read OAuth client information",
    category: "clients",
    resource: "oauth-clients",
    action: "read",
  },
  "clients:write": {
    id: "clients:write",
    name: "Write Clients",
    description: "Create and update OAuth clients",
    category: "clients",
    resource: "oauth-clients",
    action: "write",
    sensitive: true,
  },
  "clients:delete": {
    id: "clients:delete",
    name: "Delete Clients",
    description: "Delete OAuth clients",
    category: "clients",
    resource: "oauth-clients",
    action: "delete",
    sensitive: true,
  },

  // ---------------------------------------------------------------------------
  // Admin Scopes - Full administrative access
  // ---------------------------------------------------------------------------
  "admin:*": {
    id: "admin:*",
    name: "Full Admin Access",
    description: "Complete administrative access to all resources",
    category: "admin",
    resource: "admin",
    action: "*",
    sensitive: true,
  },
  "admin:api-keys:manage": {
    id: "admin:api-keys:manage",
    name: "Manage API Keys",
    description: "Create, update, and delete API keys",
    category: "admin",
    resource: "api-keys",
    action: "manage",
    sensitive: true,
  },

};

// =============================================================================
// SCOPE PRESETS - Common combinations for different use cases
// =============================================================================

export interface ScopePreset {
  id: string;
  name: string;
  description: string;
  scopes: string[];
  /** Whether this is suitable for M2M tokens */
  forM2M: boolean;
}

export const SCOPE_PRESETS: ScopePreset[] = [
  {
    id: "m2m-public-profiles",
    name: "M2M Public Profiles",
    description: "Read public user profiles (for profile pages, leaderboards)",
    scopes: ["users:public:read"],
    forM2M: true,
  },
  {
    id: "m2m-readonly",
    name: "M2M Read-Only",
    description: "Basic read access for service integrations",
    scopes: ["users:read", "profile:read", "orgs:read", "auth:verify"],
    forM2M: true,
  },
  {
    id: "m2m-user-sync",
    name: "M2M User Sync",
    description: "Sync users between services",
    scopes: ["users:read", "users:list", "profile:read", "orgs:read", "orgs:members:read"],
    forM2M: true,
  },
  {
    id: "m2m-auth-service",
    name: "M2M Auth Service",
    description: "Verify tokens and manage sessions",
    scopes: ["auth:verify", "auth:introspect", "auth:sessions:read"],
    forM2M: true,
  },
  {
    id: "admin-full",
    name: "Full Admin",
    description: "Complete administrative access (use with caution)",
    scopes: ["admin:*"],
    forM2M: true,
  },
];

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/**
 * Get all scopes in a category
 */
export function getScopesByCategory(category: ScopeCategory): ScopeDefinition[] {
  return Object.values(SCOPES).filter((s) => s.category === category);
}

/**
 * Get all categories with their scopes
 */
export function getScopesGroupedByCategory(): Record<ScopeCategory, ScopeDefinition[]> {
  const categories: ScopeCategory[] = [
    "users",
    "profile",
    "organizations",
    "auth",
    "clients",
    "admin",
  ];

  return categories.reduce(
    (acc, cat) => {
      acc[cat] = getScopesByCategory(cat);
      return acc;
    },
    {} as Record<ScopeCategory, ScopeDefinition[]>
  );
}

/**
 * Convert scope IDs to permissions object for Better Auth
 * e.g., ["users:read", "users:write"] → { users: ["read", "write"] }
 */
export function scopesToPermissions(scopeIds: string[]): Record<string, string[]> {
  const permissions: Record<string, string[]> = {};

  for (const scopeId of scopeIds) {
    const scope = SCOPES[scopeId];
    if (scope) {
      if (!permissions[scope.resource]) {
        permissions[scope.resource] = [];
      }
      if (!permissions[scope.resource].includes(scope.action)) {
        permissions[scope.resource].push(scope.action);
      }
    }
  }

  return permissions;
}

/**
 * Convert Better Auth permissions object back to scope IDs
 * e.g., { users: ["read", "write"] } → ["users:read", "users:write"]
 */
export function permissionsToScopes(permissions: Record<string, string[]>): string[] {
  const scopeIds: string[] = [];

  for (const [resource, actions] of Object.entries(permissions)) {
    for (const action of actions) {
      // Find matching scope
      const matchingScope = Object.values(SCOPES).find(
        (s) => s.resource === resource && s.action === action
      );
      if (matchingScope) {
        scopeIds.push(matchingScope.id);
      }
    }
  }

  return scopeIds;
}

/**
 * Get scope display info
 */
export function getScopeInfo(scopeId: string): ScopeDefinition | undefined {
  return SCOPES[scopeId];
}

/**
 * Validate scope IDs
 */
export function validateScopes(scopeIds: string[]): { valid: boolean; invalid: string[] } {
  const invalid = scopeIds.filter((id) => !SCOPES[id]);
  return { valid: invalid.length === 0, invalid };
}

/**
 * Category display names
 */
export const CATEGORY_LABELS: Record<ScopeCategory, string> = {
  users: "User Management",
  profile: "Profile Data",
  organizations: "Organizations",
  auth: "Authentication",
  clients: "OAuth Clients",
  admin: "Administration",
};
