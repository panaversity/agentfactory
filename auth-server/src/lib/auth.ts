import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { oidcProvider } from "better-auth/plugins/oidc-provider";
import { admin } from "better-auth/plugins/admin";
import { organization } from "better-auth/plugins/organization";
import { jwt } from "better-auth/plugins";
import { db } from "./db";
import * as schema from "./db/schema";
import { userProfile, member } from "./db/schema";
import { eq } from "drizzle-orm";
import { Resend } from "resend";
import * as nodemailer from "nodemailer";

// Client ID for robolearn-interface (public client - no secret needed with PKCE)
const ROBOLEARN_INTERFACE_CLIENT_ID = "robolearn-public-client";

// Email configuration - supports multiple providers
// Priority: SMTP (Google/custom) > Resend
// Required: EMAIL_FROM or provider-specific from address

const EMAIL_FROM = process.env.EMAIL_FROM || process.env.RESEND_FROM_EMAIL || process.env.SMTP_FROM;

// Provider 1: SMTP (Google Gmail, custom SMTP, etc.)
// For Gmail: SMTP_HOST=smtp.gmail.com, SMTP_PORT=587, SMTP_USER=your@gmail.com, SMTP_PASS=app-password
const smtpConfigured = !!(
  process.env.SMTP_HOST &&
  process.env.SMTP_USER &&
  process.env.SMTP_PASS
);

const smtpTransport = smtpConfigured
  ? nodemailer.createTransport({
      host: process.env.SMTP_HOST,
      port: parseInt(process.env.SMTP_PORT || "587"),
      secure: process.env.SMTP_SECURE === "true", // true for 465, false for other ports
      auth: {
        user: process.env.SMTP_USER,
        pass: process.env.SMTP_PASS,
      },
    })
  : null;

// Provider 2: Resend
const resend = process.env.RESEND_API_KEY ? new Resend(process.env.RESEND_API_KEY) : null;

// Email is enabled if we have any provider AND a from address
const emailEnabled = !!(EMAIL_FROM && (smtpTransport || resend));

// Log which provider is active on startup
if (emailEnabled) {
  console.log("[Auth] Email enabled via:", smtpTransport ? "SMTP" : "Resend", "from:", EMAIL_FROM);
} else {
  console.log("[Auth] Email disabled - missing provider or EMAIL_FROM");
}

// Generic email sender - tries SMTP first, then Resend
async function sendEmail({ to, subject, html }: { to: string; subject: string; html: string }) {
  if (!emailEnabled || !EMAIL_FROM) {
    console.warn("[Auth] Email not configured - skipping email to:", to);
    return;
  }

  try {
    // Priority 1: SMTP (Google, custom)
    if (smtpTransport) {
      const result = await smtpTransport.sendMail({
        from: EMAIL_FROM,
        to,
        subject,
        html,
      });
      console.log("[Auth] Email sent via SMTP to:", to, "messageId:", result.messageId);
      return;
    }

    // Priority 2: Resend
    if (resend) {
      const result = await resend.emails.send({
        from: EMAIL_FROM,
        to,
        subject,
        html,
      });
      console.log("[Auth] Email sent via Resend to:", to, "id:", result.data?.id);
      return;
    }
  } catch (error) {
    console.error("[Auth] Failed to send email to:", to, "error:", error);
    throw error; // Re-throw so Better Auth knows it failed
  }
}

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: "pg",
    schema,
  }),

  // Disable /token endpoint when using OIDC Provider (OAuth equivalent is /oauth2/token)
  disabledPaths: ["/token"],


  // Email/password authentication
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    // Always require email verification for security
    requireEmailVerification: true,
    // Password reset (only when email is configured)
    ...(emailEnabled && {
      sendResetPassword: async ({ user, url }) => {
        await sendEmail({
          to: user.email,
          subject: "Reset your RoboLearn password",
          html: `
            <h2>Password Reset Request</h2>
            <p>You requested to reset your password. Click the button below to set a new password:</p>
            <p><a href="${url}" style="background-color: #2563eb; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; display: inline-block;">Reset Password</a></p>
            <p>Or copy and paste this link: ${url}</p>
            <p>This link expires in 1 hour.</p>
            <p>If you didn't request this, you can safely ignore this email.</p>
          `,
        });
      },
    }),
  },

  // Email verification configuration - always required for security
  emailVerification: {
    sendOnSignUp: true,
    autoSignInAfterVerification: true,
    expiresIn: 3600, // 1 hour
    sendVerificationEmail: async ({ user, url }) => {
      await sendEmail({
        to: user.email,
        subject: "Verify your RoboLearn account",
        html: `
          <h2>Welcome to RoboLearn!</h2>
          <p>Please verify your email address by clicking the link below:</p>
          <p><a href="${url}" style="background-color: #2563eb; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; display: inline-block;">Verify Email</a></p>
          <p>Or copy and paste this link: ${url}</p>
          <p>This link expires in 1 hour.</p>
        `,
      });
    },
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
    // JWT Plugin - Enables JWKS endpoint for asymmetric key signing (RS256)
    // This allows client-side token verification, reducing server load
    jwt({
      jwks: {
        keyPairConfig: {
          alg: "RS256", // RSA with SHA-256 - standard for OIDC/JWKS
        },
        disablePrivateKeyEncryption: true, // Disable encryption for simplicity
      },
    }),

    // OIDC Provider - Makes auth-server an OAuth2/OIDC provider
    oidcProvider({
      loginPage: "/auth/sign-in",
      consentPage: "/auth/consent",
      // Enable JWT plugin integration for asymmetric key signing
      // ID tokens will be signed with RS256 using JWKS keys instead of HS256 with secret
      useJWTPlugin: true,
      // OAuth token expiration configuration
      accessTokenExpiresIn: 60 * 60 * 6, // 6 hours (21600 seconds)
      refreshTokenExpiresIn: 60 * 60 * 24 * 7, // 7 days (604800 seconds)
      codeExpiresIn: 600, // 10 minutes (authorization code expiry)
      // Pre-register robolearn-interface as a public client (uses PKCE, no secret)
      trustedClients: [
        {
          clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
          // No clientSecret needed for public clients with PKCE + JWKS
          // ID tokens are signed with asymmetric keys (RS256) from JWKS
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
      // SECURITY: Disable open dynamic client registration
      // Use /api/admin/clients/register (admin auth) or /api/clients/register (API key) instead
      allowDynamicClientRegistration: false,
      // Add custom claims to userinfo endpoint and ID token
      async getAdditionalUserInfoClaim(user) {
        // Fetch user profile with software_background and hardware_tier
        const profile = await db.query.userProfile.findFirst({
          where: eq(userProfile.userId, user.id),
        });

        // Fetch user's organization memberships for tenant_id
        const memberships = await db
          .select()
          .from(member)
          .where(eq(member.userId, user.id));

        // Get all organization IDs the user belongs to
        const organizationIds = memberships.map((m) => m.organizationId);

        // Primary tenant is the first organization (can be extended to support active org)
        const primaryTenantId = organizationIds[0] || null;

        return {
          software_background: profile?.softwareBackground || null,
          hardware_tier: profile?.hardwareTier || null,
          role: user.role || "user",
          // Tenant/organization claims
          tenant_id: primaryTenantId,
          organization_ids: organizationIds,
          // Organization role from primary tenant
          org_role: memberships[0]?.role || null,
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

    // Organization plugin - Multi-tenancy support
    // Enables tenant_id in token claims and organization management
    organization({
      // Allow any user to create organizations (can be restricted later)
      allowUserToCreateOrganization: true,
    }),
  ],
});

// Export client config for use in robolearn-interface (no secret for public client)
export const oauthClientConfig = {
  clientId: ROBOLEARN_INTERFACE_CLIENT_ID,
};

export type Auth = typeof auth;
