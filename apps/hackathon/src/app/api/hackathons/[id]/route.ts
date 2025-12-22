import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  getHackathonById,
  updateHackathon,
  deleteHackathon,
  canManageHackathon,
} from "@/db/queries/hackathons";
import { updateHackathonSchema } from "@/lib/validation/hackathon";
import { invalidateCache } from "@/lib/cache";

/**
 * GET /api/hackathons/[id]
 * Get hackathon details
 */
export async function GET(
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

    const hackathon = await getHackathonById(id);

    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    // Check access: published hackathons are public, otherwise need org membership
    if (
      !hackathon.published &&
      hackathon.organizationId !== session.user.organizationId
    ) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    return NextResponse.json(hackathon);
  } catch (error) {
    console.error("Error fetching hackathon:", error);
    return NextResponse.json(
      { error: "Failed to fetch hackathon" },
      { status: 500 }
    );
  }
}

/**
 * PATCH /api/hackathons/[id]
 * Update hackathon details
 */
export async function PATCH(
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

    const body = await request.json();
    const parseResult = updateHackathonSchema.safeParse(body);

    if (!parseResult.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parseResult.error.flatten() },
        { status: 400 }
      );
    }

    const updated = await updateHackathon(id, parseResult.data);

    if (!updated) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    // Invalidate cache tags
    invalidateCache.onHackathonUpdate(id, updated.organizationId);

    return NextResponse.json(updated);
  } catch (error) {
    console.error("Error updating hackathon:", error);
    return NextResponse.json(
      { error: "Failed to update hackathon" },
      { status: 500 }
    );
  }
}

/**
 * DELETE /api/hackathons/[id]
 * Delete a hackathon
 */
export async function DELETE(
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

    // Only organizers can delete
    const hackathon = await getHackathonById(id);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    if (hackathon.createdBy !== session.user.id) {
      return NextResponse.json(
        { error: "Only the creator can delete hackathons" },
        { status: 403 }
      );
    }

    const orgId = hackathon.organizationId;
    await deleteHackathon(id);

    // Invalidate cache tags
    invalidateCache.onHackathonDelete(orgId);

    return NextResponse.json({ success: true });
  } catch (error) {
    console.error("Error deleting hackathon:", error);
    return NextResponse.json(
      { error: "Failed to delete hackathon" },
      { status: 500 }
    );
  }
}
