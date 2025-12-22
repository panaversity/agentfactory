import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  toggleHackathonPublished,
  canManageHackathon,
} from "@/db/queries/hackathons";
import { invalidateCache } from "@/lib/cache";

/**
 * POST /api/hackathons/[id]/publish
 * Toggle hackathon publish status
 */
export async function POST(
  request: Request,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;

    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Check permissions
    const canManage = await canManageHackathon(id, session.user.id);
    if (!canManage) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const updated = await toggleHackathonPublished(id);

    if (!updated) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    // Invalidate cache tags
    invalidateCache.onHackathonPublish(id, updated.organizationId);

    return NextResponse.json(updated);
  } catch (error) {
    console.error("Error toggling hackathon publish status:", error);
    return NextResponse.json(
      { error: "Failed to update hackathon" },
      { status: 500 }
    );
  }
}
