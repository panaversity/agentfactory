import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import { getSyncHistoryByHackathon } from "@/db/queries/submission-fields";

/**
 * GET /api/hackathons/[id]/submissions/sync-history
 * Get sync history for a hackathon
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

    const history = await getSyncHistoryByHackathon(id);
    return NextResponse.json(history);
  } catch (error) {
    console.error("Error getting sync history:", error);
    return NextResponse.json(
      { error: "Failed to get sync history" },
      { status: 500 }
    );
  }
}
