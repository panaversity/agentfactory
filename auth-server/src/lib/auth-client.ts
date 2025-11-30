import { createAuthClient } from "better-auth/react";
import { oidcClient } from "better-auth/client/plugins";
import { adminClient } from "better-auth/client/plugins";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001",
  plugins: [
    oidcClient(),
    adminClient(),
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
} = authClient;
