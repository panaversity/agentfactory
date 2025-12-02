import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { user, member } from "@/auth-schema";
import { eq } from "drizzle-orm";
import type { SoftwareBackground, HardwareTier } from "@/types/profile";

// TODO: Migrate softwareBackground and hardwareTier to member.metadata in Proposal 001
// Currently stored in user table via additionalFields

// GET /api/profile - Get current user's profile
export async function GET() {
  try {
    const session = await auth.api.getSession({
      headers: await headers(),
    });

    if (!session) {
      return NextResponse.json(
        { error: "Unauthorized" },
        { status: 401 }
      );
    }

    // Get user with profile fields
    const [userRecord] = await db
      .select()
      .from(user)
      .where(eq(user.id, session.user.id))
      .limit(1);

    if (!userRecord) {
      return NextResponse.json(
        { error: "User not found" },
        { status: 404 }
      );
    }

    // Get user's organization memberships
    const memberships = await db
      .select()
      .from(member)
      .where(eq(member.userId, session.user.id));

    const organizationIds = memberships.map((m: typeof memberships[number]) => m.organizationId);

    // Return flattened structure with organizationIds (expected by tests)
    return NextResponse.json({
      id: userRecord.id,
      email: userRecord.email,
      name: userRecord.name,
      image: userRecord.image,
      softwareBackground: userRecord.softwareBackground,
      hardwareTier: userRecord.hardwareTier,
      organizationIds,
    });
  } catch (error) {
    console.error("Error fetching profile:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
}

// POST /api/profile - Create/update user profile (called after signup or by user)
export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { softwareBackground, hardwareTier, userId } = body;

    let targetUserId: string;

    // Allow creating profile during signup with userId (user not verified yet)
    // OR with authenticated session (user is signed in)
    if (userId) {
      // Validate user exists and was created recently (within 5 minutes)
      const [userRecord] = await db
        .select()
        .from(user)
        .where(eq(user.id, userId))
        .limit(1);

      if (!userRecord) {
        return NextResponse.json(
          { error: "User not found" },
          { status: 404 }
        );
      }

      // Check if user was created recently (within 5 minutes)
      const fiveMinutesAgo = new Date(Date.now() - 5 * 60 * 1000);
      if (userRecord.createdAt < fiveMinutesAgo) {
        return NextResponse.json(
          { error: "Profile creation window expired. Please sign in and update your profile." },
          { status: 403 }
        );
      }

      targetUserId = userId;
    } else {
      // Fallback to session-based authentication
      const session = await auth.api.getSession({
        headers: await headers(),
      });

      if (!session) {
        return NextResponse.json(
          { error: "Unauthorized. Provide userId for signup or sign in first." },
          { status: 401 }
        );
      }

      targetUserId = session.user.id;
    }

    // Validate software background
    const validBackgrounds: SoftwareBackground[] = ["beginner", "intermediate", "advanced"];
    if (softwareBackground && !validBackgrounds.includes(softwareBackground)) {
      return NextResponse.json(
        { error: "Invalid software background. Must be: beginner, intermediate, or advanced" },
        { status: 400 }
      );
    }

    // Validate hardware tier
    const validTiers: HardwareTier[] = ["tier1", "tier2", "tier3", "tier4"];
    if (hardwareTier && !validTiers.includes(hardwareTier)) {
      return NextResponse.json(
        { error: "Invalid hardware tier. Must be: tier1, tier2, tier3, or tier4" },
        { status: 400 }
      );
    }

    // Update user record with profile fields
    const updateData: { softwareBackground?: string; hardwareTier?: string; updatedAt: Date } = {
      updatedAt: new Date(),
    };
    if (softwareBackground) updateData.softwareBackground = softwareBackground;
    if (hardwareTier) updateData.hardwareTier = hardwareTier;

    const updatedUser = await db
      .update(user)
      .set(updateData)
      .where(eq(user.id, targetUserId))
      .returning();

    if (updatedUser.length === 0) {
      return NextResponse.json(
        { error: "Failed to update profile" },
        { status: 500 }
      );
    }

    return NextResponse.json({
      profile: {
        softwareBackground: updatedUser[0].softwareBackground,
        hardwareTier: updatedUser[0].hardwareTier,
      },
    });
  } catch (error) {
    console.error("Error creating/updating profile:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
}

// PUT /api/profile - Update user profile
export async function PUT(request: NextRequest) {
  // Same as POST for now (upsert behavior)
  return POST(request);
}
