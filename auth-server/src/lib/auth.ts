import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { oidcProvider } from "better-auth/plugins/oidc-provider";
import { admin } from "better-auth/plugins/admin";
import { db } from "./db";
import * as schema from "./db/schema";
import { userProfile } from "./db/schema";
import { eq } from "drizzle-orm";

// Client ID for robolearn-interface (public client - no secret needed with PKCE)
const ROBOLEARN_INTERFACE_CLIENT_ID = "robolearn-interface";

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "pg",
    schema,
  }),

  // Email/password authentication
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // Update session every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },

  // Cookie settings
  advanced: {
    cookiePrefix: "robolearn",
    useSecureCookies: process.env.NODE_ENV === "production",
  },

  // Rate limiting
  rateLimit: {
    window: 60, // 1 minute
    max: 5, // 5 attempts per window
  },

  // Trusted origins for CORS
  // Production: Set ALLOWED_ORIGINS env var (comma-separated list of URLs)
  // Development: Falls back to localhost:3000
  trustedOrigins: process.env.ALLOWED_ORIGINS?.split(",") ||
    (process.env.NODE_ENV === "development" ? ["http://localhost:3000"] : []),

  // Plugins
  plugins: [
    // OIDC Provider - Makes auth-server an OAuth2/OIDC provider
    oidcProvider({
      loginPage: "/auth/sign-in",
      consentPage: "/auth/consent",
      // Pre-register robolearn-interface as a public client (uses PKCE, no secret)
      trustedClients: [
        {
          clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
          // clientSecret is needed for signing ID tokens (HS256), even for public clients
          // Authentication still uses PKCE (no secret transmitted), but ID token needs a signing key
          // Production: Set BETTER_AUTH_SECRET env var (minimum 32 characters)
          // Development: Uses a placeholder key for local testing only
          clientSecret: process.env.BETTER_AUTH_SECRET ||
            (process.env.NODE_ENV === "development"
              ? "dev-only-client-signing-key-min32chars"
              : (() => { throw new Error("BETTER_AUTH_SECRET must be set in production"); })()),
          name: "RoboLearn Book Interface",
          type: "public", // Public client for SPA/browser apps - uses PKCE for auth
          // Redirect URLs for OAuth callback
          // Production: Set ROBOLEARN_INTERFACE_CALLBACK_URL env var
          // Development: Falls back to localhost:3000/auth/callback
          redirectUrls: process.env.ROBOLEARN_INTERFACE_CALLBACK_URL
            ? [process.env.ROBOLEARN_INTERFACE_CALLBACK_URL]
            : (process.env.NODE_ENV === "development"
                ? ["http://localhost:3000/auth/callback"]
                : []),
          disabled: false,
          skipConsent: true, // First-party app, no consent screen needed
          metadata: {},
        },
      ],
      // Allow dynamic client registration for future apps
      allowDynamicClientRegistration: true,
      // Add custom claims to userinfo endpoint
      async getAdditionalUserInfoClaim(user) {
        // Fetch user profile with software_background
        const profile = await db.query.userProfile.findFirst({
          where: eq(userProfile.userId, user.id),
        });

        return {
          software_background: profile?.softwareBackground || null,
          role: user.role || "user",
        };
      },
    }),

    // Admin plugin - User management and admin dashboard
    admin({
      defaultRole: "user",
      adminRoles: ["admin"],
      // You can specify admin user IDs directly
      // adminUserIds: ["your-admin-user-id"],
    }),
  ],
});

// Export client config for use in robolearn-interface (no secret for public client)
export const oauthClientConfig = {
  clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
};

export type Auth = typeof auth;
