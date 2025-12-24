import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import {
  getSubmissionFieldsByHackathon,
  bulkUpsertSubmissionFields,
} from "@/db/queries/submission-fields";
import { bulkSubmissionFieldsSchema } from "@/lib/validation/submission-fields";
import { invalidateCache } from "@/lib/cache";

/**
 * GET /api/hackathons/[id]/submission-fields
 * Get custom submission fields for a hackathon
 */
export async function GET(
  request: Request,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;

    const cookieStore = await cookies();
    const session = await getIronSession<SessionData>(
      cookieStore,
      sessionOptions
    );

    if (!session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Check hackathon exists and user has access
    const hackathon = await getHackathonById(id);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    if (hackathon.organizationId !== session.user.organizationId) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const fields = await getSubmissionFieldsByHackathon(id);
    return NextResponse.json(fields);
  } catch (error) {
    console.error("Error getting submission fields:", error);
    return NextResponse.json(
      { error: "Failed to get submission fields" },
      { status: 500 }
    );
  }
}

/**
 * PUT /api/hackathons/[id]/submission-fields
 * Replace all custom submission fields for a hackathon
 */
export async function PUT(
  request: Request,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;

    const cookieStore = await cookies();
    const session = await getIronSession<SessionData>(
      cookieStore,
      sessionOptions
    );

    if (!session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Check hackathon exists and user has access
    const hackathon = await getHackathonById(id);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    if (hackathon.organizationId !== session.user.organizationId) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const body = await request.json();
    const parseResult = bulkSubmissionFieldsSchema.safeParse(body);

    if (!parseResult.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parseResult.error.flatten() },
        { status: 400 }
      );
    }

    const fields = await bulkUpsertSubmissionFields(
      id,
      session.user.organizationId,
      parseResult.data.fields
    );

    // Invalidate hackathon cache
    invalidateCache.onHackathonUpdate(id, session.user.organizationId);

    return NextResponse.json(fields);
  } catch (error) {
    console.error("Error updating submission fields:", error);
    return NextResponse.json(
      { error: "Failed to update submission fields" },
      { status: 500 }
    );
  }
}
