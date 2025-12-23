import { z } from "zod";

// Field types supported by the form builder
export const fieldTypeSchema = z.enum([
  "text",
  "url",
  "textarea",
  "select",
  "number",
  "email",
]);

export type FieldType = z.infer<typeof fieldTypeSchema>;

// Single field definition
export const submissionFieldSchema = z.object({
  id: z.string().optional(), // Optional for new fields
  name: z
    .string()
    .min(1, "Field name is required")
    .max(50)
    .regex(/^[a-z_][a-z0-9_]*$/i, "Field name must be alphanumeric with underscores"),
  label: z.string().min(1, "Label is required").max(100),
  type: fieldTypeSchema,
  placeholder: z.string().max(200).optional(),
  description: z.string().max(500).optional(),
  options: z.string().optional(), // JSON array for select type
  required: z.boolean().default(false),
  order: z.number().int().min(0),
});

export type SubmissionFieldInput = z.infer<typeof submissionFieldSchema>;

// Bulk update schema (replaces all fields)
export const bulkSubmissionFieldsSchema = z.object({
  fields: z.array(submissionFieldSchema),
});

export type BulkSubmissionFieldsInput = z.infer<typeof bulkSubmissionFieldsSchema>;

// Hackathon submission mode update
export const submissionModeSchema = z.object({
  submissionMode: z.enum(["platform", "external", "hybrid"]),
  externalFormUrl: z.string().url().optional().nullable(),
});

export type SubmissionModeInput = z.infer<typeof submissionModeSchema>;

// CSV sync column mapping
export const syncColumnMappingSchema = z.object({
  email: z.string().min(1, "Email column mapping is required"),
  projectName: z.string().optional(),
  repositoryUrl: z.string().optional(),
  demoUrl: z.string().optional(),
});

export type SyncColumnMapping = z.infer<typeof syncColumnMappingSchema>;

// CSV sync request
export const syncRequestSchema = z.object({
  csvContent: z.string().min(1, "CSV content is required"),
  columnMapping: syncColumnMappingSchema,
  createMissing: z.boolean().default(false), // Create submissions for unmatched emails
  updateExisting: z.boolean().default(true), // Update existing submissions
});

export type SyncRequestInput = z.infer<typeof syncRequestSchema>;
