import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { oidcProvider } from "better-auth/plugins/oidc-provider";
import { admin } from "better-auth/plugins/admin";
import { db } from "./db";
import * as schema from "./db/schema";

// Generate a stable client secret for robolearn-interface
const ROBOLEARN_INTERFACE_CLIENT_ID = "robolearn-interface";
const ROBOLEARN_INTERFACE_CLIENT_SECRET = process.env.ROBOLEARN_CLIENT_SECRET || "robolearn-interface-secret-change-in-production";

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
  trustedOrigins: process.env.ALLOWED_ORIGINS?.split(",") || [
    "http://localhost:3000",
  ],

  // Plugins
  plugins: [
    // OIDC Provider - Makes auth-server an OAuth2/OIDC provider
    oidcProvider({
      loginPage: "/auth/sign-in",
      consentPage: "/auth/consent",
      // Pre-register robolearn-interface as a trusted client
      trustedClients: [
        {
          clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
          clientSecret: ROBOLEARN_INTERFACE_CLIENT_SECRET,
          name: "RoboLearn Book Interface",
          type: "web",
          redirectUrls: [
            "http://localhost:3000/api/auth/callback",
            "http://localhost:3000/auth/callback",
            process.env.ROBOLEARN_INTERFACE_CALLBACK_URL || "http://localhost:3000/api/auth/callback",
          ],
          disabled: false,
          skipConsent: true, // First-party app, no consent screen needed
          metadata: {},
        },
      ],
      // Allow dynamic client registration for future apps
      allowDynamicClientRegistration: true,
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

// Export client credentials for use in robolearn-interface
export const oauthClientConfig = {
  clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
  clientSecret: ROBOLEARN_INTERFACE_CLIENT_SECRET,
};

export type Auth = typeof auth;
