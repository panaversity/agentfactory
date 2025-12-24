import { db } from "@/db";
import { submissionFields, submissionSyncs } from "@/db/schema";
import { eq, asc } from "drizzle-orm";
import type { NewSubmissionField, NewSubmissionSync } from "@/db/schema";

// =============================================================================
// SUBMISSION FIELDS (Custom Form Builder)
// =============================================================================

export async function getSubmissionFieldsByHackathon(hackathonId: string) {
  return db
    .select()
    .from(submissionFields)
    .where(eq(submissionFields.hackathonId, hackathonId))
    .orderBy(asc(submissionFields.order));
}

export async function getSubmissionFieldById(id: string) {
  const results = await db
    .select()
    .from(submissionFields)
    .where(eq(submissionFields.id, id))
    .limit(1);
  return results[0] || null;
}

export async function createSubmissionField(
  data: Omit<NewSubmissionField, "id" | "createdAt" | "updatedAt">
) {
  const results = await db.insert(submissionFields).values(data).returning();
  return results[0];
}

export async function updateSubmissionField(
  id: string,
  data: Partial<Omit<NewSubmissionField, "id" | "createdAt" | "updatedAt">>
) {
  const results = await db
    .update(submissionFields)
    .set(data)
    .where(eq(submissionFields.id, id))
    .returning();
  return results[0] || null;
}

export async function deleteSubmissionField(id: string) {
  const results = await db
    .delete(submissionFields)
    .where(eq(submissionFields.id, id))
    .returning();
  return results[0] || null;
}

export async function deleteAllSubmissionFields(hackathonId: string) {
  return db
    .delete(submissionFields)
    .where(eq(submissionFields.hackathonId, hackathonId))
    .returning();
}

export async function bulkUpsertSubmissionFields(
  hackathonId: string,
  organizationId: string,
  fields: Array<{
    id?: string;
    name: string;
    label: string;
    type: string;
    placeholder?: string;
    description?: string;
    options?: string;
    required?: boolean;
    order: number;
  }>
) {
  // Delete existing fields
  await deleteAllSubmissionFields(hackathonId);

  // Insert new fields
  if (fields.length === 0) return [];

  const results = await db
    .insert(submissionFields)
    .values(
      fields.map((field) => ({
        hackathonId,
        organizationId,
        name: field.name,
        label: field.label,
        type: field.type,
        placeholder: field.placeholder,
        description: field.description,
        options: field.options,
        required: field.required ?? false,
        order: field.order,
      }))
    )
    .returning();

  return results;
}

// =============================================================================
// SUBMISSION SYNCS (External Form Sync History)
// =============================================================================

export async function getSyncHistoryByHackathon(hackathonId: string) {
  return db
    .select()
    .from(submissionSyncs)
    .where(eq(submissionSyncs.hackathonId, hackathonId))
    .orderBy(asc(submissionSyncs.syncedAt));
}

export async function createSubmissionSync(
  data: Omit<NewSubmissionSync, "id" | "syncedAt">
) {
  const results = await db.insert(submissionSyncs).values(data).returning();
  return results[0];
}
