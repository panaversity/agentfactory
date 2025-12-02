import { NextResponse } from "next/server";

/**
 * Health check endpoint for CI/CD and monitoring
 *
 * Returns:
 * - 200 OK: Server is running and database is accessible
 * - 503 Service Unavailable: Server is running but database connection failed
 *
 * Used by:
 * - GitHub Actions CI workflow
 * - Load balancers / health checkers
 * - Monitoring services
 */
export async function GET() {
  try {
    // Check database connection
    const { db } = await import("@/lib/db");

    // Simple query to verify database connectivity
    // This will throw if database is unreachable
    await db.execute("SELECT 1");

    return NextResponse.json(
      {
        status: "healthy",
        timestamp: new Date().toISOString(),
        service: "auth-server",
        database: "connected",
      },
      { status: 200 }
    );
  } catch (error) {
    console.error("Health check failed:", error);

    return NextResponse.json(
      {
        status: "unhealthy",
        timestamp: new Date().toISOString(),
        service: "auth-server",
        database: "disconnected",
        error: error instanceof Error ? error.message : "Unknown error",
      },
      { status: 503 }
    );
  }
}
