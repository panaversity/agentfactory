import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { z } from "zod";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  getJudgingCriteria,
  createJudgingCriterion,
} from "@/db/queries/submissions";
import { getHackathonById, canManageHackathon } from "@/db/queries/hackathons";
import { db } from "@/db";
import { judgingCriteria } from "@/db/schema";
import { eq } from "drizzle-orm";
import { invalidateCache } from "@/lib/cache";

type RouteParams = { params: Promise<{ id: string }> };

const createCriterionSchema = z.object({
  name: z.string().min(1).max(100),
  description: z.string().optional(),
  weight: z.number().min(1).max(10).default(1),
  maxScore: z.number().min(1).max(100).default(10),
});

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

    const criteria = await getJudgingCriteria(hackathonId);

    return NextResponse.json({ data: criteria });
  } catch (error) {
    console.error("Error fetching criteria:", error);
    return NextResponse.json(
      { error: "Failed to fetch criteria" },
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

    // Only managers/organizers can create criteria
    const canManage = await canManageHackathon(hackathonId, session.user.id);
    if (!canManage) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const body = await request.json();
    const parsed = createCriterionSchema.safeParse(body);

    if (!parsed.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parsed.error.flatten() },
        { status: 400 }
      );
    }

    // Get current criteria count for order
    const existingCriteria = await getJudgingCriteria(hackathonId);
    const nextOrder = existingCriteria.length;

    const criterion = await createJudgingCriterion({
      hackathonId,
      name: parsed.data.name,
      description: parsed.data.description,
      weight: parsed.data.weight,
      maxScore: parsed.data.maxScore,
      order: nextOrder,
    });

    // Invalidate cache for criteria
    invalidateCache.onCriteriaChange(hackathonId);

    return NextResponse.json({ data: criterion }, { status: 201 });
  } catch (error) {
    console.error("Error creating criterion:", error);
    const message =
      error instanceof Error ? error.message : "Failed to create criterion";
    return NextResponse.json({ error: message }, { status: 500 });
  }
}

export async function DELETE(request: NextRequest, { params }: RouteParams) {
  try {
    const { id: hackathonId } = await params;
    const { searchParams } = new URL(request.url);
    const criterionId = searchParams.get("criterionId");

    if (!criterionId) {
      return NextResponse.json(
        { error: "criterionId is required" },
        { status: 400 }
      );
    }

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

    // Only managers/organizers can delete criteria
    const canManage = await canManageHackathon(hackathonId, session.user.id);
    if (!canManage) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    await db
      .delete(judgingCriteria)
      .where(eq(judgingCriteria.id, criterionId));

    // Invalidate cache for criteria
    invalidateCache.onCriteriaChange(hackathonId);

    return NextResponse.json({ success: true });
  } catch (error) {
    console.error("Error deleting criterion:", error);
    return NextResponse.json(
      { error: "Failed to delete criterion" },
      { status: 500 }
    );
  }
}
