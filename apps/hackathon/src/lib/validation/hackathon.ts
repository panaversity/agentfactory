import { z } from "zod";

/**
 * Base hackathon fields schema (without refinements)
 */
const hackathonFieldsSchema = z.object({
  title: z
    .string()
    .min(3, "Title must be at least 3 characters")
    .max(100, "Title must be at most 100 characters"),
  description: z
    .string()
    .min(10, "Description must be at least 10 characters")
    .max(5000, "Description must be at most 5000 characters"),
  slug: z
    .string()
    .min(3, "Slug must be at least 3 characters")
    .max(50, "Slug must be at most 50 characters")
    .regex(
      /^[a-z0-9-]+$/,
      "Slug must only contain lowercase letters, numbers, and hyphens"
    ),
  startDate: z.coerce.date(),
  endDate: z.coerce.date(),
  registrationDeadline: z.coerce.date(),
  submissionDeadline: z.coerce.date(),
  minTeamSize: z.number().int().min(1).max(10).default(1),
  maxTeamSize: z.number().int().min(1).max(20).default(5),
  prizes: z.string().optional(), // JSON string of prizes array
  organizers: z.string().optional(), // JSON string of organizers array
  sponsors: z.string().optional(), // JSON string of sponsors array
  categories: z.string().optional(), // JSON string of categories array { id, name, description? }
});

/**
 * Validation schema for creating a hackathon
 */
export const createHackathonSchema = hackathonFieldsSchema
  .refine((data) => data.endDate > data.startDate, {
    message: "End date must be after start date",
    path: ["endDate"],
  })
  .refine((data) => data.registrationDeadline < data.startDate, {
    message: "Registration deadline must be before start date",
    path: ["registrationDeadline"],
  })
  .refine((data) => data.submissionDeadline <= data.endDate, {
    message: "Submission deadline must be on or before end date",
    path: ["submissionDeadline"],
  })
  .refine((data) => data.maxTeamSize >= data.minTeamSize, {
    message: "Max team size must be greater than or equal to min team size",
    path: ["maxTeamSize"],
  });

export type CreateHackathonInput = z.infer<typeof createHackathonSchema>;

/**
 * Validation schema for updating a hackathon
 * Uses base fields schema with partial() since refinements don't support partial
 */
export const updateHackathonSchema = hackathonFieldsSchema.partial();

export type UpdateHackathonInput = z.infer<typeof updateHackathonSchema>;

/**
 * Validation schema for hackathon status
 */
export const hackathonStatusSchema = z.enum([
  "draft",
  "open",
  "active",
  "judging",
  "completed",
]);

export type HackathonStatusInput = z.infer<typeof hackathonStatusSchema>;

/**
 * Generate slug from title
 */
export function generateSlug(title: string): string {
  return title
    .toLowerCase()
    .replace(/[^a-z0-9\s-]/g, "")
    .replace(/\s+/g, "-")
    .replace(/-+/g, "-")
    .slice(0, 50);
}
