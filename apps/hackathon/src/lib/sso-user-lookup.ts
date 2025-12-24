import { z } from "zod";

// Schema for user object inside SSO M2M API response
const ssoUserSchema = z.object({
  id: z.string(),
  username: z.string(),
  name: z.string(),
  // Note: email is NOT returned by public profile endpoint (privacy)
  // displayUsername and image are optional
  displayUsername: z.string().optional(),
  image: z.string().nullable().optional(),
  createdAt: z.string().optional(),
});

// SSO wraps user in { user: ... } envelope
const ssoResponseSchema = z.object({
  user: ssoUserSchema,
});

export type SSOUser = z.infer<typeof ssoUserSchema>;

/**
 * Fetch a user by username from the SSO service using M2M API key
 */
export async function getUserByUsername(username: string): Promise<SSOUser | null> {
  // Clean username (remove leading @ if present)
  const cleanUsername = username.startsWith("@") ? username.slice(1) : username;

  const ssoUrl = process.env.NEXT_PUBLIC_SSO_URL;
  const apiKey = process.env.SSO_API_KEY;

  if (!ssoUrl || !apiKey) {
    console.error("Missing SSO configuration: NEXT_PUBLIC_SSO_URL or SSO_API_KEY");
    throw new Error("Server configuration error");
  }

  try {
    const response = await fetch(`${ssoUrl}/api/m2m/users/${cleanUsername}`, {
      headers: {
        "x-api-key": apiKey,
        "Content-Type": "application/json",
      },
      cache: "no-store", // Always fetch fresh data
    });

    if (response.status === 404) {
      return null;
    }

    if (!response.ok) {
      console.error(
        `SSO User lookup failed: ${response.status} ${response.statusText}`
      );
      throw new Error("Failed to fetch user from SSO");
    }

    const data = await response.json();

    // Validate response shape (SSO wraps in { user: ... })
    const parsed = ssoResponseSchema.safeParse(data);
    if (!parsed.success) {
      console.error("Invalid user data from SSO:", parsed.error);
      throw new Error("Received invalid user data from SSO");
    }

    return parsed.data.user;
  } catch (error) {
    console.error("Error in getUserByUsername:", error);
    throw error;
  }
}
