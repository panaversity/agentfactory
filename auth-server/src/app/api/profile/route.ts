import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { userProfile, SoftwareBackground } from "@/lib/db/schema";
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
    const { softwareBackground } = body;

    // Validate software background
    const validBackgrounds: SoftwareBackground[] = ["beginner", "intermediate", "advanced"];
    if (!validBackgrounds.includes(softwareBackground)) {
      return NextResponse.json(
        { error: "Invalid software background. Must be: beginner, intermediate, or advanced" },
        { status: 400 }
      );
    }

    // Check if profile already exists
    const existingProfile = await db.query.userProfile.findFirst({
      where: eq(userProfile.userId, session.user.id),
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
        userId: session.user.id,
        softwareBackground,
      })
      .returning();

    return NextResponse.json({
      profile: {
        softwareBackground: newProfile[0].softwareBackground,
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
    const { softwareBackground } = body;

    // Validate software background
    const validBackgrounds: SoftwareBackground[] = ["beginner", "intermediate", "advanced"];
    if (!validBackgrounds.includes(softwareBackground)) {
      return NextResponse.json(
        { error: "Invalid software background. Must be: beginner, intermediate, or advanced" },
        { status: 400 }
      );
    }

    // Update profile
    const updatedProfile = await db
      .update(userProfile)
      .set({
        softwareBackground,
        updatedAt: new Date(),
      })
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
