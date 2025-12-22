import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { z } from "zod";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  getSubmissionById,
  getJudgeScoresForSubmission,
  submitAllScores,
  getJudgingCriteria,
} from "@/db/queries/submissions";
import { getHackathonById } from "@/db/queries/hackathons";
import { hasRole } from "@/db/queries/roles";
import { invalidateCache } from "@/lib/cache";

type RouteParams = { params: Promise<{ id: string; submissionId: string }> };

const scoreSchema = z.object({
  scores: z.array(
    z.object({
      criterionId: z.string(),
      score: z.number().min(0).max(10),
      feedback: z.string().optional(),
    })
  ),
});

export async function GET(request: NextRequest, { params }: RouteParams) {
  try {
    const { id: hackathonId, submissionId } = await params;

    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Check if user is judge
    const isJudge = await hasRole(hackathonId, session.user.id, "judge");
    if (!isJudge) {
      return NextResponse.json({ error: "Not a judge" }, { status: 403 });
    }

    const submission = await getSubmissionById(submissionId);
    if (!submission || submission.hackathonId !== hackathonId) {
      return NextResponse.json(
        { error: "Submission not found" },
        { status: 404 }
      );
    }

    // Get criteria and judge's existing scores
    const criteria = await getJudgingCriteria(hackathonId);
    const judgeScores = await getJudgeScoresForSubmission(
      submissionId,
      session.user.id
    );

    // Map scores to criteria
    const criteriaWithScores = criteria.map((c) => {
      const score = judgeScores.find((s) => s.criterionId === c.id);
      return {
        ...c,
        judgeScore: score?.score ?? null,
        judgeFeedback: score?.feedback ?? null,
      };
    });

    return NextResponse.json({
      data: {
        submission,
        criteria: criteriaWithScores,
        scoringComplete: judgeScores.length >= criteria.length,
      },
    });
  } catch (error) {
    console.error("Error fetching scores:", error);
    return NextResponse.json(
      { error: "Failed to fetch scores" },
      { status: 500 }
    );
  }
}

export async function POST(request: NextRequest, { params }: RouteParams) {
  try {
    const { id: hackathonId, submissionId } = await params;

    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Check if user is judge
    const isJudge = await hasRole(hackathonId, session.user.id, "judge");
    if (!isJudge) {
      return NextResponse.json({ error: "Not a judge" }, { status: 403 });
    }

    const hackathon = await getHackathonById(hackathonId);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    // Only allow scoring during judging phase
    if (hackathon.status !== "judging" && hackathon.status !== "active") {
      return NextResponse.json(
        { error: "Scoring is only allowed during judging phase" },
        { status: 400 }
      );
    }

    const submission = await getSubmissionById(submissionId);
    if (!submission || submission.hackathonId !== hackathonId) {
      return NextResponse.json(
        { error: "Submission not found" },
        { status: 404 }
      );
    }

    const body = await request.json();
    const parsed = scoreSchema.safeParse(body);

    if (!parsed.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parsed.error.flatten() },
        { status: 400 }
      );
    }

    // Validate all criteria are from this hackathon
    const criteria = await getJudgingCriteria(hackathonId);
    const criteriaIds = new Set(criteria.map((c) => c.id));

    for (const score of parsed.data.scores) {
      if (!criteriaIds.has(score.criterionId)) {
        return NextResponse.json(
          { error: `Invalid criterion: ${score.criterionId}` },
          { status: 400 }
        );
      }

      const criterion = criteria.find((c) => c.id === score.criterionId);
      if (criterion && score.score > criterion.maxScore) {
        return NextResponse.json(
          {
            error: `Score for ${criterion.name} exceeds max of ${criterion.maxScore}`,
          },
          { status: 400 }
        );
      }
    }

    const scores = await submitAllScores(
      submissionId,
      hackathon.organizationId,
      {
        id: session.user.id,
        username: session.user.username,
        name: session.user.name,
      },
      parsed.data.scores
    );

    // Invalidate cache for this submission's scores
    invalidateCache.onSubmissionChange(hackathonId, submissionId);

    return NextResponse.json({ data: scores });
  } catch (error) {
    console.error("Error submitting scores:", error);
    const message =
      error instanceof Error ? error.message : "Failed to submit scores";
    return NextResponse.json({ error: message }, { status: 500 });
  }
}
