import { and, eq, desc, sql } from "drizzle-orm";
import { db } from "@/db";
import {
  submissions,
  teams,
  hackathons,
  scores,
  judgingCriteria,
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

  const result = await db
    .insert(submissions)
    .values({
      ...data,
      teamId,
      hackathonId,
      organizationId,
      submittedBy: user.id,
      submitterUsername: user.username,
      submitterName: user.name,
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
 */
export async function getSubmissionsForJudge(
  hackathonId: string,
  judgeId: string
) {
  // For now, judges can see all submissions
  // In the future, could add assignment table
  const submissionList = await getSubmissionsWithScores(hackathonId);

  // Add judge's scoring status to each
  const results = [];
  for (const submission of submissionList) {
    const judgeScores = await getJudgeScoresForSubmission(
      submission.id,
      judgeId
    );
    const criteria = await getJudgingCriteria(hackathonId);
    results.push({
      ...submission,
      hasJudgeScored: judgeScores.length > 0,
      scoringComplete: judgeScores.length >= criteria.length,
      judgeScoreCount: judgeScores.length,
      criteriaCount: criteria.length,
    });
  }

  return results;
}
