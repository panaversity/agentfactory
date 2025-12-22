import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { z } from "zod";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  getRolesByHackathon,
  assignRole,
  removeRole,
  canManage,
} from "@/db/queries/roles";
import { getHackathonById } from "@/db/queries/hackathons";
import { invalidateCache } from "@/lib/cache";

type RouteParams = { params: Promise<{ id: string }> };

// Accept user info directly (SSO lookup happens on frontend or via separate API)
const assignRoleSchema = z.object({
  userId: z.string().min(1),
  username: z.string().min(1),
  name: z.string().optional(),
  email: z.string().email().optional(),
  image: z.string().url().optional(),
  role: z.enum(["manager", "judge", "mentor", "participant"]),
  maxTeamsAssigned: z.number().optional(),
});

const removeRoleSchema = z.object({
  userId: z.string().min(1),
  role: z.enum(["manager", "judge", "mentor", "participant"]),
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

    // Only managers/organizers can view all roles
    const hasAccess = await canManage(hackathonId, session.user.id);
    if (!hasAccess) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const roles = await getRolesByHackathon(hackathonId);

    return NextResponse.json({ data: roles });
  } catch (error) {
    console.error("Error fetching roles:", error);
    return NextResponse.json(
      { error: "Failed to fetch roles" },
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

    // Only managers/organizers can assign roles
    const hasAccess = await canManage(hackathonId, session.user.id);
    if (!hasAccess) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const body = await request.json();
    const parsed = assignRoleSchema.safeParse(body);

    if (!parsed.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parsed.error.flatten() },
        { status: 400 }
      );
    }

    const role = await assignRole({
      hackathonId,
      organizationId: hackathon.organizationId,
      user: {
        id: parsed.data.userId,
        username: parsed.data.username,
        name: parsed.data.name,
        email: parsed.data.email,
        image: parsed.data.image,
      },
      role: parsed.data.role,
      assignedBy: session.user.id,
      maxTeamsAssigned: parsed.data.maxTeamsAssigned,
    });

    // Invalidate cache for roles
    invalidateCache.onRoleChange(hackathonId);

    return NextResponse.json({ data: role }, { status: 201 });
  } catch (error) {
    console.error("Error assigning role:", error);
    const message =
      error instanceof Error ? error.message : "Failed to assign role";
    return NextResponse.json({ error: message }, { status: 500 });
  }
}

export async function DELETE(request: NextRequest, { params }: RouteParams) {
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

    // Only managers/organizers can remove roles
    const hasAccess = await canManage(hackathonId, session.user.id);
    if (!hasAccess) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const body = await request.json();
    const parsed = removeRoleSchema.safeParse(body);

    if (!parsed.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parsed.error.flatten() },
        { status: 400 }
      );
    }

    await removeRole(hackathonId, parsed.data.userId, parsed.data.role);

    // Invalidate cache for roles
    invalidateCache.onRoleChange(hackathonId);

    return NextResponse.json({ success: true });
  } catch (error) {
    console.error("Error removing role:", error);
    const message =
      error instanceof Error ? error.message : "Failed to remove role";
    return NextResponse.json({ error: message }, { status: 500 });
  }
}
