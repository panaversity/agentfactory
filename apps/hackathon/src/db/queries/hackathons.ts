import { and, desc, eq } from "drizzle-orm";
import { unstable_cache } from "next/cache";
import { db } from "@/db";
import { hackathons, hackathonRoles } from "@/db/schema";
import type { CreateHackathonInput, UpdateHackathonInput } from "@/lib/validation/hackathon";
import { CacheTags } from "@/lib/cache";

/**
 * Get all hackathons for an organization (cached)
 */
export async function getHackathonsByOrg(organizationId: string) {
  const cached = unstable_cache(
    async () => {
      return db
        .select()
        .from(hackathons)
        .where(eq(hackathons.organizationId, organizationId))
        .orderBy(desc(hackathons.createdAt));
    },
    [`hackathons-by-org-${organizationId}`],
    {
      tags: [CacheTags.hackathonsByOrg(organizationId), CacheTags.hackathons()],
      revalidate: false, // Only revalidate via tags
    }
  );
  return cached();
}

/**
 * Get published hackathons (for participants browsing) (cached)
 */
export async function getPublishedHackathons() {
  const cached = unstable_cache(
    async () => {
      return db
        .select()
        .from(hackathons)
        .where(eq(hackathons.published, true))
        .orderBy(desc(hackathons.startDate));
    },
    ["published-hackathons"],
    {
      tags: [CacheTags.publishedHackathons()],
      revalidate: false,
    }
  );
  return cached();
}

/**
 * Get hackathons created by a specific user
 */
export async function getHackathonsByCreator(userId: string) {
  return db
    .select()
    .from(hackathons)
    .where(eq(hackathons.createdBy, userId))
    .orderBy(desc(hackathons.createdAt));
}

/**
 * Get a single hackathon by ID (cached)
 */
export async function getHackathonById(id: string) {
  const cached = unstable_cache(
    async () => {
      const result = await db
        .select()
        .from(hackathons)
        .where(eq(hackathons.id, id))
        .limit(1);
      return result[0] ?? null;
    },
    [`hackathon-${id}`],
    {
      tags: [CacheTags.hackathon(id)],
      revalidate: false,
    }
  );
  return cached();
}

/**
 * Get hackathon by slug (for URL-friendly access)
 */
export async function getHackathonBySlug(slug: string, organizationId: string) {
  const result = await db
    .select()
    .from(hackathons)
    .where(
      and(
        eq(hackathons.slug, slug),
        eq(hackathons.organizationId, organizationId)
      )
    )
    .limit(1);

  return result[0] ?? null;
}

/**
 * Create a new hackathon
 */
export async function createHackathon(
  data: CreateHackathonInput,
  organizationId: string,
  createdBy: string
) {
  const result = await db
    .insert(hackathons)
    .values({
      ...data,
      organizationId,
      createdBy,
      status: "draft",
      published: false,
    })
    .returning();

  return result[0];
}

/**
 * Update an existing hackathon
 */
export async function updateHackathon(id: string, data: UpdateHackathonInput) {
  const result = await db
    .update(hackathons)
    .set({
      ...data,
      updatedAt: new Date(),
    })
    .where(eq(hackathons.id, id))
    .returning();

  return result[0] ?? null;
}

/**
 * Delete a hackathon
 */
export async function deleteHackathon(id: string) {
  await db.delete(hackathons).where(eq(hackathons.id, id));
}

/**
 * Toggle hackathon publish status
 */
export async function toggleHackathonPublished(id: string) {
  const hackathon = await getHackathonById(id);
  if (!hackathon) return null;

  const result = await db
    .update(hackathons)
    .set({
      published: !hackathon.published,
      status: !hackathon.published ? "open" : hackathon.status,
      updatedAt: new Date(),
    })
    .where(eq(hackathons.id, id))
    .returning();

  return result[0] ?? null;
}

/**
 * Update hackathon status
 */
export async function updateHackathonStatus(
  id: string,
  status: "draft" | "open" | "active" | "judging" | "completed"
) {
  const result = await db
    .update(hackathons)
    .set({
      status,
      updatedAt: new Date(),
    })
    .where(eq(hackathons.id, id))
    .returning();

  return result[0] ?? null;
}

/**
 * Get user's role for a hackathon
 */
export async function getUserRole(hackathonId: string, userId: string) {
  const result = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, userId)
      )
    )
    .limit(1);

  return result[0] ?? null;
}

/**
 * Check if user is organizer of hackathon
 */
export async function isHackathonOrganizer(hackathonId: string, userId: string) {
  const hackathon = await getHackathonById(hackathonId);
  if (hackathon?.createdBy === userId) return true;

  const role = await getUserRole(hackathonId, userId);
  return role?.role === "organizer";
}

/**
 * Check if user can manage hackathon (organizer or manager)
 */
export async function canManageHackathon(hackathonId: string, userId: string) {
  const hackathon = await getHackathonById(hackathonId);
  if (hackathon?.createdBy === userId) return true;

  const role = await getUserRole(hackathonId, userId);
  return role?.role === "organizer" || role?.role === "manager";
}
