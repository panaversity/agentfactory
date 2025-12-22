import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  registerForHackathon,
  isUserRegistered,
  isRegistrationOpen,
} from "@/db/queries/teams";
import { getHackathonById } from "@/db/queries/hackathons";

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

    const registered = await isUserRegistered(hackathonId, session.user.id);
    const open = await isRegistrationOpen(hackathonId);

    return NextResponse.json({
      data: {
        isRegistered: registered,
        isOpen: open,
        hackathon: {
          id: hackathon.id,
          title: hackathon.title,
          status: hackathon.status,
          registrationDeadline: hackathon.registrationDeadline,
        },
      },
    });
  } catch (error) {
    console.error("Error checking registration:", error);
    return NextResponse.json(
      { error: "Failed to check registration" },
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

    await registerForHackathon(hackathonId, hackathon.organizationId, {
      id: session.user.id,
      username: session.user.username,
      name: session.user.name,
      email: session.user.email,
      image: session.user.image,
    });

    return NextResponse.json({
      data: {
        success: true,
        message: "Successfully registered for hackathon",
      },
    });
  } catch (error) {
    console.error("Error registering:", error);
    const message =
      error instanceof Error ? error.message : "Failed to register";
    return NextResponse.json({ error: message }, { status: 400 });
  }
}
