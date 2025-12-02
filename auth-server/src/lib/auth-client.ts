import { createAuthClient } from "better-auth/react";
import { oidcClient } from "better-auth/client/plugins";
import { adminClient } from "better-auth/client/plugins";
import { organizationClient } from "better-auth/client/plugins";
import { usernameClient } from "better-auth/client/plugins";
import { inferAdditionalFields } from "better-auth/client/plugins";
import type { auth } from "./auth";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001",
  plugins: [
    inferAdditionalFields<typeof auth>(), // Infer custom fields from server
    oidcClient(),
    adminClient(),
    organizationClient(),
    usernameClient(), // Matches server's username() plugin for profile usernames
  ],
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
  getSession,
  // Password reset methods
  requestPasswordReset,
  resetPassword,
  // Email verification
  sendVerificationEmail,
  // OIDC methods
  oauth2,
  // Admin methods
  admin,
  // Organization methods
  organization,
  useActiveOrganization,
} = authClient;
