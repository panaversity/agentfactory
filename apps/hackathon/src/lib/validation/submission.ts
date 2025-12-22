import { z } from "zod";

const urlSchema = z.string().url("Must be a valid URL");

/**
 * Validation schema for creating/updating a submission
 */
export const createSubmissionSchema = z.object({
  teamId: z.string().uuid("Invalid team ID"),
  hackathonId: z.string().uuid("Invalid hackathon ID"),
  projectName: z
    .string()
    .min(3, "Project name must be at least 3 characters")
    .max(100, "Project name must be at most 100 characters"),
  description: z
    .string()
    .min(20, "Description must be at least 20 characters")
    .max(5000, "Description must be at most 5000 characters"),
  repositoryUrl: urlSchema.refine(
    (url) =>
      url.includes("github.com") ||
      url.includes("gitlab.com") ||
      url.includes("bitbucket.org"),
    "Repository URL must be from GitHub, GitLab, or Bitbucket"
  ),
  demoUrl: urlSchema.optional().nullable(),
  presentationUrl: urlSchema.optional().nullable(),
});

export type CreateSubmissionInput = z.infer<typeof createSubmissionSchema>;

/**
 * Validation schema for updating a submission
 */
export const updateSubmissionSchema = createSubmissionSchema
  .omit({ teamId: true, hackathonId: true })
  .partial();

export type UpdateSubmissionInput = z.infer<typeof updateSubmissionSchema>;

/**
 * Validation schema for scoring a submission
 */
export const scoreSubmissionSchema = z.object({
  submissionId: z.string().uuid("Invalid submission ID"),
  scores: z.array(
    z.object({
      criterionId: z.string().uuid("Invalid criterion ID"),
      score: z
        .number()
        .int()
        .min(1, "Score must be at least 1")
        .max(10, "Score must be at most 10"),
      feedback: z.string().max(1000, "Feedback must be at most 1000 characters").optional(),
    })
  ),
});

export type ScoreSubmissionInput = z.infer<typeof scoreSubmissionSchema>;

/**
 * Validation schema for submission status
 */
export const submissionStatusSchema = z.enum([
  "submitted",
  "under_review",
  "scored",
]);

export type SubmissionStatusInput = z.infer<typeof submissionStatusSchema>;
