import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { joinTeam, isRegistrationOpen, getUserTeam } from "@/db/queries/teams";
import { getHackathonById } from "@/db/queries/hackathons";
import { joinTeamSchema } from "@/lib/validation/team";
import { invalidateCache } from "@/lib/cache";

type RouteParams = { params: Promise<{ id: string }> };

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

    // Check if registration is open
    const open = await isRegistrationOpen(hackathonId);
    if (!open) {
      return NextResponse.json(
        { error: "Registration is not open for this hackathon" },
        { status: 400 }
      );
    }

    // Check if user already has a team
    const existingTeam = await getUserTeam(hackathonId, session.user.id);
    if (existingTeam) {
      return NextResponse.json(
        { error: "You are already in a team for this hackathon" },
        { status: 400 }
      );
    }

    const body = await request.json();
    const parsed = joinTeamSchema.safeParse(body);

    if (!parsed.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parsed.error.flatten() },
        { status: 400 }
      );
    }

    const team = await joinTeam(parsed.data, hackathonId, hackathon.organizationId, {
      id: session.user.id,
      username: session.user.username,
      name: session.user.name,
      email: session.user.email,
      image: session.user.image,
    });

    // Invalidate cache tags for teams and hackathon stats
    invalidateCache.onTeamChange(hackathonId);

    return NextResponse.json({ data: team });
  } catch (error) {
    console.error("Error joining team:", error);
    const message =
      error instanceof Error ? error.message : "Failed to join team";
    return NextResponse.json({ error: message }, { status: 400 });
  }
}
