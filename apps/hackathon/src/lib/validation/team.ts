import { z } from "zod";

/**
 * Validation schema for creating a team
 * Note: hackathonId comes from URL params, not the request body
 */
export const createTeamSchema = z.object({
  name: z
    .string()
    .min(2, "Team name must be at least 2 characters")
    .max(50, "Team name must be at most 50 characters"),
  description: z
    .string()
    .max(500, "Description must be at most 500 characters")
    .optional()
    .nullable(),
});

export type CreateTeamInput = z.infer<typeof createTeamSchema>;

/**
 * Validation schema for updating a team
 */
export const updateTeamSchema = z.object({
  name: z
    .string()
    .min(2, "Team name must be at least 2 characters")
    .max(50, "Team name must be at most 50 characters")
    .optional(),
  description: z
    .string()
    .max(500, "Description must be at most 500 characters")
    .optional()
    .nullable(),
});

export type UpdateTeamInput = z.infer<typeof updateTeamSchema>;

/**
 * Validation schema for joining a team via invite code
 */
export const joinTeamSchema = z.object({
  inviteCode: z
    .string()
    .length(8, "Invite code must be 8 characters")
    .regex(/^[A-Z0-9]+$/, "Invalid invite code format"),
});

export type JoinTeamInput = z.infer<typeof joinTeamSchema>;

/**
 * Validation schema for team status
 */
export const teamStatusSchema = z.enum([
  "forming",
  "ready",
  "submitted",
  "disqualified",
]);

export type TeamStatusInput = z.infer<typeof teamStatusSchema>;
