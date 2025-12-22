import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  getSubmissionsWithScores,
  createSubmission,
  getSubmissionByTeam,
} from "@/db/queries/submissions";
import { getUserTeam } from "@/db/queries/teams";
import { getHackathonById, canManageHackathon } from "@/db/queries/hackathons";
import { hasRole } from "@/db/queries/roles";
import { createSubmissionSchema } from "@/lib/validation/submission";
import { invalidateCache } from "@/lib/cache";

type RouteParams = { params: Promise<{ id: string }> };

export async function GET(request: NextRequest, { params }: RouteParams) {
  try {
    const { id: hackathonId } = await params;

    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    const hackathon = await getHackathonById(hackathonId);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    // Check access - only organizers, managers, and judges can see all submissions
    const canManage = await canManageHackathon(hackathonId, session.user.id);
    const isJudge = await hasRole(hackathonId, session.user.id, "judge");

    if (!canManage && !isJudge) {
      // Participants can only see their own team's submission
      const userTeam = await getUserTeam(hackathonId, session.user.id);
      if (!userTeam) {
        return NextResponse.json({ data: [] });
      }
      const submission = await getSubmissionByTeam(userTeam.id);
      return NextResponse.json({
        data: submission ? [submission] : [],
      });
    }

    const submissions = await getSubmissionsWithScores(hackathonId);

    return NextResponse.json({ data: submissions });
  } catch (error) {
    console.error("Error fetching submissions:", error);
    return NextResponse.json(
      { error: "Failed to fetch submissions" },
      { status: 500 }
    );
  }
}

export async function POST(request: NextRequest, { params }: RouteParams) {
  try {
    const { id: hackathonId } = await params;

    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    const hackathon = await getHackathonById(hackathonId);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    // Get user's team
    const userTeam = await getUserTeam(hackathonId, session.user.id);
    if (!userTeam) {
      return NextResponse.json(
        { error: "You must be in a team to submit" },
        { status: 400 }
      );
    }

    // Only team leader can submit
    if (userTeam.leaderId !== session.user.id) {
      return NextResponse.json(
        { error: "Only team leader can submit projects" },
        { status: 403 }
      );
    }

    const body = await request.json();
    const parsed = createSubmissionSchema.safeParse(body);

    if (!parsed.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parsed.error.flatten() },
        { status: 400 }
      );
    }

    const submission = await createSubmission(
      parsed.data,
      userTeam.id,
      hackathonId,
      hackathon.organizationId,
      {
        id: session.user.id,
        username: session.user.username,
        name: session.user.name,
      }
    );

    // Invalidate cache tags for submissions
    invalidateCache.onSubmissionChange(hackathonId, submission.id);

    return NextResponse.json({ data: submission }, { status: 201 });
  } catch (error) {
    console.error("Error creating submission:", error);
    const message =
      error instanceof Error ? error.message : "Failed to create submission";
    return NextResponse.json({ error: message }, { status: 400 });
  }
}
