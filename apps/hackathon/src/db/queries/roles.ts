import { and, eq, desc } from "drizzle-orm";
import { db } from "@/db";
import { hackathonRoles, hackathons } from "@/db/schema";
import type { RoleType } from "@/lib/auth/permissions";

/**
 * User data for denormalization (from session or SSO lookup)
 */
export interface UserData {
  id: string;
  username: string;
  name?: string | null;
  email?: string | null;
  image?: string | null;
}

/**
 * Get all roles for a hackathon
 */
export async function getRolesByHackathon(hackathonId: string) {
  return db
    .select()
    .from(hackathonRoles)
    .where(eq(hackathonRoles.hackathonId, hackathonId))
    .orderBy(desc(hackathonRoles.createdAt));
}

/**
 * Get all roles for a user across hackathons
 */
export async function getRolesByUser(userId: string) {
  return db
    .select({
      id: hackathonRoles.id,
      role: hackathonRoles.role,
      hackathonId: hackathonRoles.hackathonId,
      hackathonTitle: hackathons.title,
      hackathonStatus: hackathons.status,
      createdAt: hackathonRoles.createdAt,
    })
    .from(hackathonRoles)
    .innerJoin(hackathons, eq(hackathonRoles.hackathonId, hackathons.id))
    .where(eq(hackathonRoles.userId, userId))
    .orderBy(desc(hackathonRoles.createdAt));
}

/**
 * Get user's roles for a specific hackathon
 */
export async function getUserRolesForHackathon(
  hackathonId: string,
  userId: string
): Promise<RoleType[]> {
  const roles = await db
    .select({ role: hackathonRoles.role })
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, userId)
      )
    );

  return roles.map((r) => r.role as RoleType);
}

/**
 * Assign a role to a user
 */
export async function assignRole(data: {
  hackathonId: string;
  organizationId: string;
  user: UserData;
  role: RoleType;
  assignedBy: string;
  maxTeamsAssigned?: number;
}) {
  // Check if role already exists
  const existing = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, data.hackathonId),
        eq(hackathonRoles.userId, data.user.id),
        eq(hackathonRoles.role, data.role)
      )
    )
    .limit(1);

  if (existing[0]) {
    // Update if mentor with different max teams
    if (data.role === "mentor" && data.maxTeamsAssigned !== undefined) {
      const result = await db
        .update(hackathonRoles)
        .set({ maxTeamsAssigned: data.maxTeamsAssigned })
        .where(eq(hackathonRoles.id, existing[0].id))
        .returning();
      return result[0];
    }
    return existing[0];
  }

  const result = await db
    .insert(hackathonRoles)
    .values({
      hackathonId: data.hackathonId,
      organizationId: data.organizationId,
      userId: data.user.id,
      username: data.user.username,
      name: data.user.name,
      email: data.user.email,
      image: data.user.image,
      role: data.role,
      assignedBy: data.assignedBy,
      maxTeamsAssigned: data.maxTeamsAssigned,
    })
    .returning();

  return result[0];
}

/**
 * Remove a role from a user
 */
export async function removeRole(
  hackathonId: string,
  userId: string,
  role: RoleType
) {
  await db
    .delete(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, userId),
        eq(hackathonRoles.role, role)
      )
    );
}

/**
 * Get all judges for a hackathon
 */
export async function getJudges(hackathonId: string) {
  return db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.role, "judge")
      )
    );
}

/**
 * Get all mentors for a hackathon
 */
export async function getMentors(hackathonId: string) {
  return db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.role, "mentor")
      )
    );
}

/**
 * Get all participants for a hackathon
 */
export async function getParticipants(hackathonId: string) {
  return db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.role, "participant")
      )
    );
}

/**
 * Check if user has specific role for hackathon
 */
export async function hasRole(
  hackathonId: string,
  userId: string,
  role: RoleType
): Promise<boolean> {
  const result = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, userId),
        eq(hackathonRoles.role, role)
      )
    )
    .limit(1);

  return !!result[0];
}

/**
 * Check if user can manage hackathon (organizer, manager, or creator)
 */
export async function canManage(
  hackathonId: string,
  userId: string
): Promise<boolean> {
  // Check if creator
  const hackathon = await db
    .select({ createdBy: hackathons.createdBy })
    .from(hackathons)
    .where(eq(hackathons.id, hackathonId))
    .limit(1);

  if (hackathon[0]?.createdBy === userId) {
    return true;
  }

  // Check roles
  const roles = await getUserRolesForHackathon(hackathonId, userId);
  return roles.includes("organizer") || roles.includes("manager");
}

/**
 * Get hackathon stats for analytics
 */
export async function getHackathonRoleStats(hackathonId: string) {
  const roles = await getRolesByHackathon(hackathonId);

  const stats = {
    organizers: 0,
    managers: 0,
    judges: 0,
    mentors: 0,
    participants: 0,
    total: roles.length,
  };

  for (const role of roles) {
    switch (role.role) {
      case "organizer":
        stats.organizers++;
        break;
      case "manager":
        stats.managers++;
        break;
      case "judge":
        stats.judges++;
        break;
      case "mentor":
        stats.mentors++;
        break;
      case "participant":
        stats.participants++;
        break;
    }
  }

  return stats;
}
