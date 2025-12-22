import { and, eq, sql, desc } from "drizzle-orm";
import { db } from "@/db";
import {
  teams,
  teamMembers,
  hackathons,
  hackathonRoles,
} from "@/db/schema";
import type { CreateTeamInput, JoinTeamInput } from "@/lib/validation/team";
import type { TeamWithMemberCount } from "@/types";

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
 * Get all teams for a hackathon
 */
export async function getTeamsByHackathon(hackathonId: string) {
  return db
    .select({
      id: teams.id,
      name: teams.name,
      description: teams.description,
      status: teams.status,
      inviteCode: teams.inviteCode,
      leaderId: teams.leaderId,
      hackathonId: teams.hackathonId,
      createdAt: teams.createdAt,
    })
    .from(teams)
    .where(eq(teams.hackathonId, hackathonId))
    .orderBy(desc(teams.createdAt));
}

/**
 * Get teams with member count for a hackathon
 */
export async function getTeamsWithMemberCount(
  hackathonId: string
): Promise<TeamWithMemberCount[]> {
  const hackathon = await db
    .select({
      minTeamSize: hackathons.minTeamSize,
      maxTeamSize: hackathons.maxTeamSize,
    })
    .from(hackathons)
    .where(eq(hackathons.id, hackathonId))
    .limit(1);

  const minSize = hackathon[0]?.minTeamSize ?? 1;
  const maxSize = hackathon[0]?.maxTeamSize ?? 5;

  const teamsData = await db
    .select({
      id: teams.id,
      name: teams.name,
      description: teams.description,
      status: teams.status,
      inviteCode: teams.inviteCode,
      leaderId: teams.leaderId,
      hackathonId: teams.hackathonId,
      memberCount: sql<number>`(
        SELECT COUNT(*)::int
        FROM team_members
        WHERE team_members.team_id = ${teams.id}
      )`,
    })
    .from(teams)
    .where(eq(teams.hackathonId, hackathonId))
    .orderBy(desc(teams.createdAt));

  return teamsData.map((team) => ({
    ...team,
    status: team.status as TeamWithMemberCount["status"],
    maxSize,
    minSize,
    isReady: team.memberCount >= minSize && team.memberCount <= maxSize,
  }));
}

/**
 * Get a single team by ID
 */
export async function getTeamById(id: string) {
  const result = await db.select().from(teams).where(eq(teams.id, id)).limit(1);
  return result[0] ?? null;
}

/**
 * Get team by invite code
 */
export async function getTeamByInviteCode(inviteCode: string) {
  const result = await db
    .select()
    .from(teams)
    .where(eq(teams.inviteCode, inviteCode))
    .limit(1);
  return result[0] ?? null;
}

/**
 * Get user's team for a hackathon
 */
export async function getUserTeam(hackathonId: string, userId: string) {
  const result = await db
    .select({
      team: teams,
      role: teamMembers.role,
    })
    .from(teamMembers)
    .innerJoin(teams, eq(teamMembers.teamId, teams.id))
    .where(
      and(
        eq(teams.hackathonId, hackathonId),
        eq(teamMembers.userId, userId)
      )
    )
    .limit(1);

  if (!result[0]) return null;
  return {
    ...result[0].team,
    memberRole: result[0].role,
  };
}

/**
 * Get all members of a team
 */
export async function getTeamMembers(teamId: string) {
  return db
    .select()
    .from(teamMembers)
    .where(eq(teamMembers.teamId, teamId))
    .orderBy(teamMembers.joinedAt);
}

/**
 * Create a new team
 */
export async function createTeam(
  data: CreateTeamInput,
  hackathonId: string,
  organizationId: string,
  leader: UserData
) {
  // Create team with denormalized leader info
  const result = await db
    .insert(teams)
    .values({
      ...data,
      hackathonId,
      organizationId,
      leaderId: leader.id,
      leaderUsername: leader.username,
      leaderName: leader.name,
      leaderImage: leader.image,
      status: "forming",
    })
    .returning();

  const team = result[0];

  // Add leader as first member with denormalized data
  await db.insert(teamMembers).values({
    teamId: team.id,
    organizationId,
    userId: leader.id,
    username: leader.username,
    name: leader.name,
    email: leader.email,
    image: leader.image,
    role: "leader",
    status: "accepted",
    joinedAt: new Date(),
  });

  // Ensure user has participant role
  const existingRole = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, leader.id),
        eq(hackathonRoles.role, "participant")
      )
    )
    .limit(1);

  if (!existingRole[0]) {
    await db.insert(hackathonRoles).values({
      hackathonId,
      organizationId,
      userId: leader.id,
      username: leader.username,
      name: leader.name,
      email: leader.email,
      image: leader.image,
      role: "participant",
    });
  }

  return team;
}

/**
 * Join an existing team via invite code
 */
export async function joinTeam(
  data: JoinTeamInput,
  hackathonId: string,
  organizationId: string,
  user: UserData
) {
  const team = await getTeamByInviteCode(data.inviteCode);

  if (!team) {
    throw new Error("Invalid invite code");
  }

  if (team.hackathonId !== hackathonId) {
    throw new Error("Team is not part of this hackathon");
  }

  // Check if user is already in a team for this hackathon
  const existingTeam = await getUserTeam(hackathonId, user.id);
  if (existingTeam) {
    throw new Error("You are already in a team for this hackathon");
  }

  // Check if team has reached max size
  const hackathon = await db
    .select({ maxTeamSize: hackathons.maxTeamSize })
    .from(hackathons)
    .where(eq(hackathons.id, hackathonId))
    .limit(1);

  const members = await getTeamMembers(team.id);
  if (members.length >= (hackathon[0]?.maxTeamSize ?? 5)) {
    throw new Error("Team is full");
  }

  // Add user to team with denormalized data
  await db.insert(teamMembers).values({
    teamId: team.id,
    organizationId,
    userId: user.id,
    username: user.username,
    name: user.name,
    email: user.email,
    image: user.image,
    role: "member",
    status: "accepted",
    joinedAt: new Date(),
  });

  // Ensure user has participant role
  const existingRole = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, user.id),
        eq(hackathonRoles.role, "participant")
      )
    )
    .limit(1);

  if (!existingRole[0]) {
    await db.insert(hackathonRoles).values({
      hackathonId,
      organizationId,
      userId: user.id,
      username: user.username,
      name: user.name,
      email: user.email,
      image: user.image,
      role: "participant",
    });
  }

  return team;
}

/**
 * Leave a team
 */
export async function leaveTeam(teamId: string, userId: string) {
  const team = await getTeamById(teamId);
  if (!team) {
    throw new Error("Team not found");
  }

  if (team.leaderId === userId) {
    throw new Error("Team leader cannot leave. Transfer leadership first or delete the team.");
  }

  await db
    .delete(teamMembers)
    .where(
      and(eq(teamMembers.teamId, teamId), eq(teamMembers.userId, userId))
    );
}

/**
 * Remove a member from team (leader only)
 */
export async function removeMember(teamId: string, memberId: string, requesterId: string) {
  const team = await getTeamById(teamId);
  if (!team) {
    throw new Error("Team not found");
  }

  if (team.leaderId !== requesterId) {
    throw new Error("Only team leader can remove members");
  }

  if (team.leaderId === memberId) {
    throw new Error("Cannot remove team leader");
  }

  await db
    .delete(teamMembers)
    .where(
      and(eq(teamMembers.teamId, teamId), eq(teamMembers.userId, memberId))
    );
}

/**
 * Update team details
 */
export async function updateTeam(
  teamId: string,
  data: Partial<CreateTeamInput>,
  userId: string
) {
  const team = await getTeamById(teamId);
  if (!team) {
    throw new Error("Team not found");
  }

  if (team.leaderId !== userId) {
    throw new Error("Only team leader can update team details");
  }

  const result = await db
    .update(teams)
    .set({
      ...data,
      updatedAt: new Date(),
    })
    .where(eq(teams.id, teamId))
    .returning();

  return result[0] ?? null;
}

/**
 * Delete a team (leader only, before submission)
 */
export async function deleteTeam(teamId: string, userId: string) {
  const team = await getTeamById(teamId);
  if (!team) {
    throw new Error("Team not found");
  }

  if (team.leaderId !== userId) {
    throw new Error("Only team leader can delete the team");
  }

  if (team.status === "submitted") {
    throw new Error("Cannot delete team after submission");
  }

  // Delete team (cascade will handle members)
  await db.delete(teams).where(eq(teams.id, teamId));
}

/**
 * Regenerate invite code
 */
export async function regenerateInviteCode(teamId: string, userId: string) {
  const team = await getTeamById(teamId);
  if (!team) {
    throw new Error("Team not found");
  }

  if (team.leaderId !== userId) {
    throw new Error("Only team leader can regenerate invite code");
  }

  const result = await db
    .update(teams)
    .set({
      // inviteCode has $defaultFn, but we need to trigger it manually
      inviteCode: sql`substr(md5(random()::text), 1, 8)`,
      updatedAt: new Date(),
    })
    .where(eq(teams.id, teamId))
    .returning();

  return result[0] ?? null;
}

/**
 * Check if hackathon registration is open
 */
export async function isRegistrationOpen(hackathonId: string) {
  const hackathon = await db
    .select({
      status: hackathons.status,
      published: hackathons.published,
      registrationDeadline: hackathons.registrationDeadline,
    })
    .from(hackathons)
    .where(eq(hackathons.id, hackathonId))
    .limit(1);

  const h = hackathon[0];
  if (!h) return false;

  return (
    h.published &&
    h.status === "open" &&
    new Date() < new Date(h.registrationDeadline)
  );
}

/**
 * Check if user is registered for hackathon
 */
export async function isUserRegistered(hackathonId: string, userId: string) {
  const role = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.hackathonId, hackathonId),
        eq(hackathonRoles.userId, userId),
        eq(hackathonRoles.role, "participant")
      )
    )
    .limit(1);

  return !!role[0];
}

/**
 * Register user for hackathon (creates participant role)
 */
export async function registerForHackathon(
  hackathonId: string,
  organizationId: string,
  user: UserData
) {
  const open = await isRegistrationOpen(hackathonId);
  if (!open) {
    throw new Error("Registration is not open for this hackathon");
  }

  const existing = await isUserRegistered(hackathonId, user.id);
  if (existing) {
    throw new Error("Already registered for this hackathon");
  }

  await db.insert(hackathonRoles).values({
    hackathonId,
    organizationId,
    userId: user.id,
    username: user.username,
    name: user.name,
    email: user.email,
    image: user.image,
    role: "participant",
  });
}

/**
 * Invite a user to a team by username
 * Creates a pending invitation that user must accept
 */
export async function inviteToTeam(
  teamId: string,
  organizationId: string,
  invitee: UserData
) {
  const team = await getTeamById(teamId);
  if (!team) {
    throw new Error("Team not found");
  }

  // Check if user is already in team
  const existingMember = await db
    .select()
    .from(teamMembers)
    .where(
      and(eq(teamMembers.teamId, teamId), eq(teamMembers.userId, invitee.id))
    )
    .limit(1);

  if (existingMember[0]) {
    throw new Error("User is already a member or has pending invitation");
  }

  // Check if user is already in another team for this hackathon
  const existingTeam = await getUserTeam(team.hackathonId, invitee.id);
  if (existingTeam) {
    throw new Error("User is already in another team for this hackathon");
  }

  // Check team capacity
  const hackathon = await db
    .select({ maxTeamSize: hackathons.maxTeamSize })
    .from(hackathons)
    .where(eq(hackathons.id, team.hackathonId))
    .limit(1);

  const members = await getTeamMembers(teamId);
  if (members.length >= (hackathon[0]?.maxTeamSize ?? 5)) {
    throw new Error("Team is full");
  }

  // Create pending invitation
  const result = await db
    .insert(teamMembers)
    .values({
      teamId,
      organizationId,
      userId: invitee.id,
      username: invitee.username,
      name: invitee.name,
      email: invitee.email,
      image: invitee.image,
      role: "member",
      status: "invited",
      // joinedAt is null until accepted
    })
    .returning();

  return result[0];
}

/**
 * Accept a team invitation
 */
export async function acceptTeamInvitation(teamId: string, userId: string) {
  const member = await db
    .select()
    .from(teamMembers)
    .where(
      and(
        eq(teamMembers.teamId, teamId),
        eq(teamMembers.userId, userId),
        eq(teamMembers.status, "invited")
      )
    )
    .limit(1);

  if (!member[0]) {
    throw new Error("No pending invitation found");
  }

  const result = await db
    .update(teamMembers)
    .set({
      status: "accepted",
      joinedAt: new Date(),
    })
    .where(eq(teamMembers.id, member[0].id))
    .returning();

  return result[0];
}

/**
 * Decline a team invitation
 */
export async function declineTeamInvitation(teamId: string, userId: string) {
  const member = await db
    .select()
    .from(teamMembers)
    .where(
      and(
        eq(teamMembers.teamId, teamId),
        eq(teamMembers.userId, userId),
        eq(teamMembers.status, "invited")
      )
    )
    .limit(1);

  if (!member[0]) {
    throw new Error("No pending invitation found");
  }

  await db
    .update(teamMembers)
    .set({ status: "declined" })
    .where(eq(teamMembers.id, member[0].id));
}

/**
 * Get pending invitations for a user
 */
export async function getPendingInvitations(userId: string) {
  return db
    .select({
      id: teamMembers.id,
      teamId: teamMembers.teamId,
      teamName: teams.name,
      hackathonId: teams.hackathonId,
      hackathonTitle: hackathons.title,
      invitedAt: teamMembers.invitedAt,
    })
    .from(teamMembers)
    .innerJoin(teams, eq(teamMembers.teamId, teams.id))
    .innerJoin(hackathons, eq(teams.hackathonId, hackathons.id))
    .where(
      and(eq(teamMembers.userId, userId), eq(teamMembers.status, "invited"))
    )
    .orderBy(desc(teamMembers.invitedAt));
}
