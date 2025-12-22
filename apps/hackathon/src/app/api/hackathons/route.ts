import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import {
  getHackathonsByOrg,
  getPublishedHackathons,
  createHackathon,
} from "@/db/queries/hackathons";
import { createHackathonSchema } from "@/lib/validation/hackathon";
import { invalidateCache } from "@/lib/cache";

/**
 * GET /api/hackathons
 * List hackathons for the user's organization or published hackathons
 */
export async function GET(request: Request) {
  try {
    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    const { searchParams } = new URL(request.url);
    const onlyPublished = searchParams.get("published") === "true";

    let hackathonsList;
    if (onlyPublished) {
      // Anyone can see published hackathons
      hackathonsList = await getPublishedHackathons();
    } else {
      // Show hackathons from user's organization (from SSO token)
      hackathonsList = await getHackathonsByOrg(session.user.organizationId);
    }

    return NextResponse.json(hackathonsList);
  } catch (error) {
    console.error("Error fetching hackathons:", error);
    return NextResponse.json(
      { error: "Failed to fetch hackathons" },
      { status: 500 }
    );
  }
}

/**
 * POST /api/hackathons
 * Create a new hackathon in user's organization (from SSO token)
 */
export async function POST(request: Request) {
  try {
    const session = await getIronSession<SessionData>(
      await cookies(),
      sessionOptions
    );

    if (!session.isLoggedIn || !session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    const body = await request.json();
    const parseResult = createHackathonSchema.safeParse(body);

    if (!parseResult.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parseResult.error.flatten() },
        { status: 400 }
      );
    }

    // Use organizationId from SSO session (not from request body)
    const organizationId = session.user.organizationId;

    const hackathon = await createHackathon(
      parseResult.data,
      organizationId,
      session.user.id
    );

    // Invalidate cache tags
    invalidateCache.onHackathonCreate(organizationId);

    return NextResponse.json(hackathon, { status: 201 });
  } catch (error) {
    console.error("Error creating hackathon:", error);
    return NextResponse.json(
      { error: "Failed to create hackathon" },
      { status: 500 }
    );
  }
}
