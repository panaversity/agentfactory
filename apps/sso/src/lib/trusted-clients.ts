/**
 * Trusted OAuth Clients Configuration
 *
 * First-party applications that skip the OAuth consent screen.
 * These clients MUST also be seeded into the database for token storage.
 *
 * Architecture:
 * 1. Defined here (auth.ts imports for skipConsent behavior)
 * 2. Seeded into database (for token storage FK constraint)
 * 3. Protected in admin UI (cannot edit/delete via UI)
 *
 * Security:
 * - Localhost URLs are automatically filtered out in production
 * - Only HTTPS URLs allowed in production (except localhost in dev)
 */

/**
 * ==============================================================================
 * ORGANIZATION CONFIGURATION
 * ==============================================================================
 * Default organization for the hybrid multi-tenant model.
 *
 * Architecture:
 * - Panaversity is the default organization for general users
 * - All new users auto-join this organization on signup
 * - Additional organizations can be created for institutions/schools
 * - This ID is hardcoded for performance (no DB lookup on every signup)
 *
 * Setup:
 * - Run `pnpm run seed:setup` to create this organization in database
 * - The seed script uses this same ID to ensure consistency
 */
export const DEFAULT_ORG_ID = "panaversity-default-org-id";
export const DEFAULT_ORG_NAME = "Panaversity";
export const DEFAULT_ORG_SLUG = "panaversity";

/**
 * ==============================================================================
 * OAUTH CLIENT CONFIGURATION
 * ==============================================================================
 */

const ROBOLEARN_INTERFACE_CLIENT_ID = "robolearn-public-client";

/**
 * Helper to filter redirect URLs based on environment
 * - Development: Allow localhost URLs
 * - Production: Remove localhost URLs for security
 */
function getRedirectUrls(urls: string[]): string[] {
  if (process.env.NODE_ENV === "production") {
    return urls.filter((url) => {
      // Remove localhost and 127.0.0.1 URLs in production
      return !url.includes("localhost") && !url.includes("127.0.0.1");
    });
  }
  return urls;
}

export const TRUSTED_CLIENTS = [
  {
    clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
    name: "RoboLearn Book Interface",
    type: "public" as const,
    redirectUrls: getRedirectUrls([
      "http://localhost:3000/auth/callback",
      "https://mjunaidca.github.io/robolearn/auth/callback",
    ]),
    disabled: false,
    skipConsent: true,
    metadata: {},
  },
  {
    clientId: "robolearn-confidential-client",
    name: "RoboLearn Backend Service (Test)",
    type: "web" as const, // "web" type for server-side confidential clients with secrets
    clientSecret: "robolearn-confidential-secret-for-testing-only",
    redirectUrls: getRedirectUrls([
      "http://localhost:8000/auth/callback",
    ]),
    disabled: false,
    skipConsent: true,
    metadata: {},
  },
  {
    clientId: "panaversity-sso-public-client",
    name: "Panaversity SSO",
    type: "public" as const,
    redirectUrls: getRedirectUrls([
      "http://localhost:3000/api/auth/callback",
      "https://panaversity.org/api/auth/callback",
    ]),
    disabled: false,
    skipConsent: true,
    metadata: {},
  },
  {
    clientId: "ai-native-public-client",
    name: "AI Native Platform (Legacy)",
    type: "public" as const,
    redirectUrls: getRedirectUrls([
      "http://localhost:3000/auth/callback",
      "https://ai-native.panaversity.org/auth/callback",
    ]),
    disabled: false,
    skipConsent: true,
    metadata: {},
  },
  {
    clientId: "agent-factory-public-client",
    name: "The AI Agent Factory",
    type: "public" as const,
    redirectUrls: getRedirectUrls([
      "http://localhost:3000/auth/callback",
      "https://agentfactory.panaversity.org/auth/callback",
    ]),
    disabled: false,
    skipConsent: true,
    metadata: {},
  },
  {
    clientId: "assessment-public-client",
    name: "Panaversity Assessment Platform",
    type: "public" as const,
    redirectUrls: getRedirectUrls([
      "http://localhost:3000/api/auth/callback",
      "https://assessment.panaversity.org/api/auth/callback",
    ]),
    disabled: false,
    skipConsent: true,
    metadata: {},
  },
];

/**
 * Array of trusted client IDs for protection checks
 * Used in admin API to prevent editing/deletion
 */
export const TRUSTED_CLIENT_IDS = TRUSTED_CLIENTS.map((c) => c.clientId);

/**
 * Comments explaining each client's purpose
 */
export const CLIENT_DESCRIPTIONS = {
  [ROBOLEARN_INTERFACE_CLIENT_ID]: {
    purpose: "Main RoboLearn book interface",
    audience: "Students and educators using the RoboLearn platform",
    security: "Public client with PKCE, no client secret",
  },
  "panaversity-sso-public-client": {
    purpose: "Panaversity Single Sign-On portal",
    audience: "All Panaversity users across platforms",
    security: "Public client with PKCE, no client secret",
  },
  "ai-native-public-client": {
    purpose: "AI Native development platform (Legacy - use agent-factory-public-client)",
    audience: "Developers building AI applications",
    security: "Public client with PKCE, no client secret",
  },
  "agent-factory-public-client": {
    purpose: "The AI Agent Factory - Spec-Driven Blueprint for Digital FTEs",
    audience: "Developers building and monetizing AI agents",
    security: "Public client with PKCE, no client secret",
  },
  "assessment-public-client": {
    purpose: "Panaversity Assessment Platform",
    audience: "Students taking assessments and quizzes",
    security: "Public client with PKCE, no client secret",
  },
} as const;
