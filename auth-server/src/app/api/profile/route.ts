import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { userProfile, user, SoftwareBackground, HardwareTier } from "@/lib/db/schema";
import { eq } from "drizzle-orm";
import { randomUUID } from "crypto";

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

    // Get user profile
    const profile = await db.query.userProfile.findFirst({
      where: eq(userProfile.userId, session.user.id),
    });

    return NextResponse.json({
      user: {
        id: session.user.id,
        email: session.user.email,
        name: session.user.name,
        image: session.user.image,
      },
      profile: profile
        ? {
            softwareBackground: profile.softwareBackground,
            hardwareTier: profile.hardwareTier,
            createdAt: profile.createdAt,
            updatedAt: profile.updatedAt,
          }
        : null,
    });
  } catch (error) {
    console.error("Error fetching profile:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
}

// POST /api/profile - Create user profile (called after signup)
export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { softwareBackground, hardwareTier, userId } = body;

    let targetUserId: string;

    // Allow creating profile during signup with userId (user not verified yet)
    // OR with authenticated session (user is signed in)
    if (userId) {
      // Validate user exists and was created recently (within 5 minutes)
      // This prevents abuse while allowing signup-time profile creation
      const userRecord = await db.query.user.findFirst({
        where: eq(user.id, userId),
      });

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
    if (!validBackgrounds.includes(softwareBackground)) {
      return NextResponse.json(
        { error: "Invalid software background. Must be: beginner, intermediate, or advanced" },
        { status: 400 }
      );
    }

    // Validate hardware tier
    const validTiers: HardwareTier[] = ["tier1", "tier2", "tier3", "tier4"];
    if (!hardwareTier || !validTiers.includes(hardwareTier)) {
      return NextResponse.json(
        { error: "Invalid hardware tier. Must be: tier1, tier2, tier3, or tier4" },
        { status: 400 }
      );
    }

    // Check if profile already exists
    const existingProfile = await db.query.userProfile.findFirst({
      where: eq(userProfile.userId, targetUserId),
    });

    if (existingProfile) {
      return NextResponse.json(
        { error: "Profile already exists. Use PUT to update." },
        { status: 409 }
      );
    }

    // Create profile
    const newProfile = await db
      .insert(userProfile)
      .values({
        id: randomUUID(),
        userId: targetUserId,
        softwareBackground,
        hardwareTier,
      })
      .returning();

    return NextResponse.json({
      profile: {
        softwareBackground: newProfile[0].softwareBackground,
        hardwareTier: newProfile[0].hardwareTier,
        createdAt: newProfile[0].createdAt,
        updatedAt: newProfile[0].updatedAt,
      },
    });
  } catch (error) {
    console.error("Error creating profile:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
}

// PUT /api/profile - Update user profile
export async function PUT(request: NextRequest) {
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

    const body = await request.json();
    const { softwareBackground, hardwareTier } = body;

    // Validate software background (if provided)
    if (softwareBackground) {
      const validBackgrounds: SoftwareBackground[] = ["beginner", "intermediate", "advanced"];
      if (!validBackgrounds.includes(softwareBackground)) {
        return NextResponse.json(
          { error: "Invalid software background. Must be: beginner, intermediate, or advanced" },
          { status: 400 }
        );
      }
    }

    // Validate hardware tier (if provided)
    if (hardwareTier) {
      const validTiers: HardwareTier[] = ["tier1", "tier2", "tier3", "tier4"];
      if (!validTiers.includes(hardwareTier)) {
        return NextResponse.json(
          { error: "Invalid hardware tier. Must be: tier1, tier2, tier3, or tier4" },
          { status: 400 }
        );
      }
    }

    // Build update object with only provided fields
    const updateData: { softwareBackground?: SoftwareBackground; hardwareTier?: HardwareTier; updatedAt: Date } = {
      updatedAt: new Date(),
    };
    if (softwareBackground) updateData.softwareBackground = softwareBackground;
    if (hardwareTier) updateData.hardwareTier = hardwareTier;

    // Update profile
    const updatedProfile = await db
      .update(userProfile)
      .set(updateData)
      .where(eq(userProfile.userId, session.user.id))
      .returning();

    if (updatedProfile.length === 0) {
      return NextResponse.json(
        { error: "Profile not found. Create one first." },
        { status: 404 }
      );
    }

    return NextResponse.json({
      profile: {
        softwareBackground: updatedProfile[0].softwareBackground,
        hardwareTier: updatedProfile[0].hardwareTier,
        createdAt: updatedProfile[0].createdAt,
        updatedAt: updatedProfile[0].updatedAt,
      },
    });
  } catch (error) {
    console.error("Error updating profile:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
}
