import { createAuthClient } from "better-auth/client";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_SSO_SERVER_URL || "http://localhost:3000",
  fetchOptions: {
    credentials: 'include',
  },
});

export type AuthClient = typeof authClient;
