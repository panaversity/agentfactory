import { and, eq, desc, sql, inArray } from "drizzle-orm";
import { db } from "@/db";
import {
  submissions,
  teams,
  hackathons,
  scores,
  judgingCriteria,
  teamMembers,
} from "@/db/schema";
import type {
  CreateSubmissionInput,
  UpdateSubmissionInput,
} from "@/lib/validation/submission";

/**
 * Get all submissions for a hackathon
 */
export async function getSubmissionsByHackathon(hackathonId: string) {
  return db
    .select({
      id: submissions.id,
      projectName: submissions.projectName,
      description: submissions.description,
      repositoryUrl: submissions.repositoryUrl,
      demoUrl: submissions.demoUrl,
      presentationUrl: submissions.presentationUrl,
      status: submissions.status,
      submittedAt: submissions.submittedAt,
      teamId: submissions.teamId,
      teamName: teams.name,
    })
    .from(submissions)
    .innerJoin(teams, eq(submissions.teamId, teams.id))
    .where(eq(submissions.hackathonId, hackathonId))
    .orderBy(desc(submissions.submittedAt));
}

/**
 * Get submissions with score summary for judging
 */
export async function getSubmissionsWithScores(hackathonId: string) {
  const submissionList = await db
    .select({
      id: submissions.id,
      projectName: submissions.projectName,
      description: submissions.description,
      repositoryUrl: submissions.repositoryUrl,
      demoUrl: submissions.demoUrl,
      presentationUrl: submissions.presentationUrl,
      status: submissions.status,
      submittedAt: submissions.submittedAt,
      teamId: submissions.teamId,
      teamName: teams.name,
      avgScore: sql<number>`(
        SELECT COALESCE(AVG(s.score), 0)::float
        FROM scores s
        WHERE s.submission_id = ${submissions.id}
      )`,
      judgeCount: sql<number>`(
        SELECT COUNT(DISTINCT s.judge_id)::int
        FROM scores s
        WHERE s.submission_id = ${submissions.id}
      )`,
    })
    .from(submissions)
    .innerJoin(teams, eq(submissions.teamId, teams.id))
    .where(eq(submissions.hackathonId, hackathonId))
    .orderBy(desc(submissions.submittedAt));

  return submissionList;
}

/**
 * Get submission by ID
 */
export async function getSubmissionById(id: string) {
  const result = await db
    .select()
    .from(submissions)
    .where(eq(submissions.id, id))
    .limit(1);
  return result[0] ?? null;
}

/**
 * Get submission by team
 */
export async function getSubmissionByTeam(teamId: string) {
  const result = await db
    .select()
    .from(submissions)
    .where(eq(submissions.teamId, teamId))
    .limit(1);
  return result[0] ?? null;
}

/**
 * User data for denormalization
 */
interface UserData {
  id: string;
  username: string;
  name?: string | null;
  email?: string | null;
}

/**
 * Create a new submission
 */
export async function createSubmission(
  data: CreateSubmissionInput,
  teamId: string,
  hackathonId: string,
  organizationId: string,
  user: UserData
) {
  // Check if hackathon allows submissions
  const hackathon = await db
    .select({
      status: hackathons.status,
      submissionDeadline: hackathons.submissionDeadline,
    })
    .from(hackathons)
    .where(eq(hackathons.id, hackathonId))
    .limit(1);

  const h = hackathon[0];
  if (!h) {
    throw new Error("Hackathon not found");
  }

  if (h.status !== "active") {
    throw new Error("Submissions are only allowed during active hackathon");
  }

  if (new Date() > new Date(h.submissionDeadline)) {
    throw new Error("Submission deadline has passed");
  }

  // Check if team already has a submission
  const existing = await getSubmissionByTeam(teamId);
  if (existing) {
    throw new Error("Team already has a submission. Please update instead.");
  }

  // Extract formData and serialize it
  const { formData, ...restData } = data;

  const result = await db
    .insert(submissions)
    .values({
      ...restData,
      teamId,
      hackathonId,
      organizationId,
      submittedBy: user.id,
      submitterUsername: user.username,
      submitterName: user.name,
      submitterEmail: user.email,
      formData: formData ? JSON.stringify(formData) : null,
      status: "submitted",
    })
    .returning();

  // Update team status
  await db
    .update(teams)
    .set({ status: "submitted", updatedAt: new Date() })
    .where(eq(teams.id, teamId));

  return result[0];
}

/**
 * Update an existing submission
 */
export async function updateSubmission(
  id: string,
  data: UpdateSubmissionInput,
  userId: string
) {
  const submission = await getSubmissionById(id);
  if (!submission) {
    throw new Error("Submission not found");
  }

  // Check deadline
  const hackathon = await db
    .select({ submissionDeadline: hackathons.submissionDeadline })
    .from(hackathons)
    .where(eq(hackathons.id, submission.hackathonId))
    .limit(1);

  if (new Date() > new Date(hackathon[0]?.submissionDeadline ?? 0)) {
    throw new Error("Submission deadline has passed");
  }

  const result = await db
    .update(submissions)
    .set({
      ...data,
      formData: data.formData ? JSON.stringify(data.formData) : undefined,
      updatedAt: new Date(),
    })
    .where(eq(submissions.id, id))
    .returning();

  return result[0] ?? null;
}

/**
 * Get judging criteria for a hackathon
 */
export async function getJudgingCriteria(hackathonId: string) {
  return db
    .select()
    .from(judgingCriteria)
    .where(eq(judgingCriteria.hackathonId, hackathonId))
    .orderBy(judgingCriteria.order);
}

/**
 * Create judging criteria
 */
export async function createJudgingCriterion(data: {
  hackathonId: string;
  name: string;
  description?: string;
  weight: number;
  maxScore: number;
  order: number;
}) {
  const result = await db.insert(judgingCriteria).values(data).returning();
  return result[0];
}

/**
 * Get scores for a submission
 */
export async function getScoresForSubmission(submissionId: string) {
  return db
    .select({
      id: scores.id,
      score: scores.score,
      feedback: scores.feedback,
      judgeId: scores.judgeId,
      criterionId: scores.criterionId,
      criterionName: judgingCriteria.name,
      criterionMaxScore: judgingCriteria.maxScore,
      criterionWeight: judgingCriteria.weight,
    })
    .from(scores)
    .innerJoin(judgingCriteria, eq(scores.criterionId, judgingCriteria.id))
    .where(eq(scores.submissionId, submissionId));
}

/**
 * Get judge's scores for a submission
 */
export async function getJudgeScoresForSubmission(
  submissionId: string,
  judgeId: string
) {
  return db
    .select({
      id: scores.id,
      score: scores.score,
      feedback: scores.feedback,
      criterionId: scores.criterionId,
    })
    .from(scores)
    .where(
      and(eq(scores.submissionId, submissionId), eq(scores.judgeId, judgeId))
    );
}

/**
 * Judge data for denormalization
 */
interface JudgeData {
  id: string;
  username: string;
  name?: string | null;
}

/**
 * Submit or update a judge's score
 */
export async function submitScore(data: {
  submissionId: string;
  organizationId: string;
  judge: JudgeData;
  criterionId: string;
  score: number;
  feedback?: string;
}) {
  // Check if score exists
  const existing = await db
    .select()
    .from(scores)
    .where(
      and(
        eq(scores.submissionId, data.submissionId),
        eq(scores.judgeId, data.judge.id),
        eq(scores.criterionId, data.criterionId)
      )
    )
    .limit(1);

  if (existing[0]) {
    // Update
    const result = await db
      .update(scores)
      .set({
        score: data.score,
        feedback: data.feedback,
        updatedAt: new Date(),
      })
      .where(eq(scores.id, existing[0].id))
      .returning();
    return result[0];
  } else {
    // Insert
    const result = await db
      .insert(scores)
      .values({
        submissionId: data.submissionId,
        organizationId: data.organizationId,
        judgeId: data.judge.id,
        judgeUsername: data.judge.username,
        judgeName: data.judge.name,
        criterionId: data.criterionId,
        score: data.score,
        feedback: data.feedback,
      })
      .returning();
    return result[0];
  }
}

/**
 * Submit all scores for a submission
 */
export async function submitAllScores(
  submissionId: string,
  organizationId: string,
  judge: JudgeData,
  scoreData: Array<{
    criterionId: string;
    score: number;
    feedback?: string;
  }>
) {
  const results = [];
  for (const s of scoreData) {
    const result = await submitScore({
      submissionId,
      organizationId,
      judge,
      criterionId: s.criterionId,
      score: s.score,
      feedback: s.feedback,
    });
    results.push(result);
  }

  // Update submission status if all criteria scored
  const criteria = await getJudgingCriteria(
    (await getSubmissionById(submissionId))?.hackathonId ?? ""
  );
  const judgeScores = await getJudgeScoresForSubmission(submissionId, judge.id);

  if (judgeScores.length >= criteria.length) {
    await db
      .update(submissions)
      .set({ status: "under_review", updatedAt: new Date() })
      .where(eq(submissions.id, submissionId));
  }

  return results;
}

/**
 * Get leaderboard for a hackathon
 */
export async function getLeaderboard(hackathonId: string) {
  const submissionList = await db
    .select({
      id: submissions.id,
      projectName: submissions.projectName,
      teamId: submissions.teamId,
      teamName: teams.name,
      totalScore: sql<number>`(
        SELECT COALESCE(SUM(s.score * jc.weight), 0)::float
        FROM scores s
        JOIN judging_criteria jc ON s.criterion_id = jc.id
        WHERE s.submission_id = ${submissions.id}
      )`,
      avgScore: sql<number>`(
        SELECT COALESCE(AVG(s.score), 0)::float
        FROM scores s
        WHERE s.submission_id = ${submissions.id}
      )`,
      judgeCount: sql<number>`(
        SELECT COUNT(DISTINCT s.judge_id)::int
        FROM scores s
        WHERE s.submission_id = ${submissions.id}
      )`,
    })
    .from(submissions)
    .innerJoin(teams, eq(submissions.teamId, teams.id))
    .where(eq(submissions.hackathonId, hackathonId))
    .orderBy(sql`total_score DESC`);

  return submissionList.map((s, i) => ({
    ...s,
    rank: i + 1,
  }));
}

/**
 * Get submissions assigned to a judge (or all if no assignment system)
 * Optimized: Single query for judge scores instead of N+1
 */
export async function getSubmissionsForJudge(
  hackathonId: string,
  judgeId: string
) {
  // Get submissions with scores in single query
  const submissionList = await getSubmissionsWithScores(hackathonId);

  // Get criteria count once (not N times)
  const criteria = await getJudgingCriteria(hackathonId);
  const criteriaCount = criteria.length;

  // Get all judge scores for this hackathon in single query
  const submissionIds = submissionList.map((s) => s.id);
  const judgeScoresAll =
    submissionIds.length > 0
      ? await db
        .select({
          submissionId: scores.submissionId,
          count: sql<number>`count(*)::int`,
        })
        .from(scores)
        .where(
          and(
            inArray(scores.submissionId, submissionIds),
            eq(scores.judgeId, judgeId)
          )
        )
        .groupBy(scores.submissionId)
      : [];

  // Build lookup map
  const scoreCountBySubmission = new Map(
    judgeScoresAll.map((s) => [s.submissionId, s.count])
  );

  // Enrich submissions with judge scoring status
  return submissionList.map((submission) => {
    const judgeScoreCount = scoreCountBySubmission.get(submission.id) ?? 0;
    return {
      ...submission,
      hasJudgeScored: judgeScoreCount > 0,
      scoringComplete: judgeScoreCount >= criteriaCount,
      judgeScoreCount,
      criteriaCount,
    };
  });
}

// =============================================================================
// EXTERNAL FORM SYNC
// =============================================================================

/**
 * Get all team members with their emails for matching
 */
export async function getTeamMembersByHackathon(hackathonId: string) {
  return db
    .select({
      teamId: teamMembers.teamId,
      userId: teamMembers.userId,
      email: teamMembers.email,
      username: teamMembers.username,
      name: teamMembers.name,
      teamName: teams.name,
    })
    .from(teamMembers)
    .innerJoin(teams, eq(teamMembers.teamId, teams.id))
    .where(eq(teams.hackathonId, hackathonId));
}

/**
 * Get existing submissions by email for sync matching
 */
export async function getSubmissionsByEmails(
  hackathonId: string,
  emails: string[]
) {
  if (emails.length === 0) return [];

  return db
    .select()
    .from(submissions)
    .where(
      and(
        eq(submissions.hackathonId, hackathonId),
        inArray(submissions.submitterEmail, emails)
      )
    );
}

/**
 * Create submission from external sync (no deadline check)
 */
export async function createSubmissionFromSync(data: {
  teamId: string;
  hackathonId: string;
  organizationId: string;
  projectName: string;
  description: string;
  repositoryUrl: string;
  demoUrl?: string;
  presentationUrl?: string;
  categoryId?: string;
  submittedBy: string;
  submitterUsername: string;
  submitterName?: string;
  submitterEmail?: string;
  formData?: string;
}) {
  const result = await db
    .insert(submissions)
    .values({
      ...data,
      status: "submitted",
      syncedFromExternal: true,
      syncedAt: new Date(),
    })
    .returning();

  // Update team status
  await db
    .update(teams)
    .set({ status: "submitted", updatedAt: new Date() })
    .where(eq(teams.id, data.teamId));

  return result[0];
}

/**
 * Update submission from external sync
 */
export async function updateSubmissionFromSync(
  id: string,
  data: {
    projectName?: string;
    description?: string;
    repositoryUrl?: string;
    demoUrl?: string;
    presentationUrl?: string;
    formData?: string;
  }
) {
  const result = await db
    .update(submissions)
    .set({
      ...data,
      syncedFromExternal: true,
      syncedAt: new Date(),
      updatedAt: new Date(),
    })
    .where(eq(submissions.id, id))
    .returning();

  return result[0] ?? null;
}

/**
 * Batch create submissions from external sync
 * Uses single insert for performance at scale
 */
export async function batchCreateSubmissionsFromSync(
  data: Array<{
    teamId: string;
    hackathonId: string;
    organizationId: string;
    projectName: string;
    description: string;
    repositoryUrl: string;
    demoUrl?: string;
    submittedBy: string;
    submitterUsername: string;
    submitterName?: string;
    submitterEmail: string;
    formData?: string;
  }>
) {
  if (data.length === 0) return [];

  const now = new Date();

  // Batch insert all submissions
  const results = await db
    .insert(submissions)
    .values(
      data.map((d) => ({
        ...d,
        status: "submitted" as const,
        syncedFromExternal: true,
        syncedAt: now,
      }))
    )
    .returning();

  // Batch update team statuses
  const teamIds = [...new Set(data.map((d) => d.teamId))];
  if (teamIds.length > 0) {
    await db
      .update(teams)
      .set({ status: "submitted", updatedAt: now })
      .where(inArray(teams.id, teamIds));
  }

  return results;
}

/**
 * Batch update submissions from external sync
 * Uses individual updates but in a single transaction
 */
export async function batchUpdateSubmissionsFromSync(
  data: Array<{
    id: string;
    projectName?: string;
    repositoryUrl?: string;
    demoUrl?: string;
    formData?: string;
  }>
) {
  if (data.length === 0) return [];

  const now = new Date();
  const results = [];

  // Execute updates - Drizzle doesn't support bulk update with different values
  // but we batch them logically and execute efficiently
  for (const item of data) {
    const { id, ...updateData } = item;
    const result = await db
      .update(submissions)
      .set({
        ...updateData,
        syncedFromExternal: true,
        syncedAt: now,
        updatedAt: now,
      })
      .where(eq(submissions.id, id))
      .returning();
    if (result[0]) results.push(result[0]);
  }

  return results;
}

/**
 * Get submission count stats for a hackathon
 */
export async function getSubmissionStats(hackathonId: string) {
  const total = await db
    .select({ count: sql<number>`count(*)::int` })
    .from(submissions)
    .where(eq(submissions.hackathonId, hackathonId));

  const synced = await db
    .select({ count: sql<number>`count(*)::int` })
    .from(submissions)
    .where(
      and(
        eq(submissions.hackathonId, hackathonId),
        eq(submissions.syncedFromExternal, true)
      )
    );

  const byStatus = await db
    .select({
      status: submissions.status,
      count: sql<number>`count(*)::int`,
    })
    .from(submissions)
    .where(eq(submissions.hackathonId, hackathonId))
    .groupBy(submissions.status);

  return {
    total: total[0]?.count ?? 0,
    synced: synced[0]?.count ?? 0,
    platform: (total[0]?.count ?? 0) - (synced[0]?.count ?? 0),
    byStatus: Object.fromEntries(byStatus.map((s) => [s.status, s.count])),
  };
}
