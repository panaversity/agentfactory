import { NextRequest, NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getUserByUsername } from "@/lib/sso-user-lookup";

export async function GET(request: NextRequest) {
    try {
        // 1. Check authentication
        const session = await getIronSession<SessionData>(
            await cookies(),
            sessionOptions
        );

        if (!session.isLoggedIn || !session.user) {
            return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
        }

        // 2. Get username from query params
        const searchParams = request.nextUrl.searchParams;
        const username = searchParams.get("username");

        if (!username || username.trim() === "") {
            return NextResponse.json(
                { error: "Username is required" },
                { status: 400 }
            );
        }

        // 3. Perform lookup
        const user = await getUserByUsername(username);

        if (!user) {
            return NextResponse.json(
                { error: "User not found" },
                { status: 404 }
            );
        }

        // 4. Return user data (SSO M2M endpoint already returns only public fields)
        return NextResponse.json({ data: user });
    } catch (error) {
        console.error("User lookup API error:", error);
        return NextResponse.json(
            { error: "Internal server error" },
            { status: 500 }
        );
    }
}
