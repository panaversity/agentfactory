import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import {
  getTeamMembersByHackathon,
  getSubmissionsByEmails,
  batchCreateSubmissionsFromSync,
  batchUpdateSubmissionsFromSync,
} from "@/db/queries/submissions";
import { createSubmissionSync } from "@/db/queries/submission-fields";
import { syncRequestSchema } from "@/lib/validation/submission-fields";
import { parseCSV, mapRowToSubmission } from "@/lib/csv-parser";
import { invalidateCache } from "@/lib/cache";

// Simple in-memory rate limiter (use Upstash in production)
const rateLimitMap = new Map<string, { count: number; resetAt: number }>();
const RATE_LIMIT_WINDOW_MS = 60000; // 1 minute
const RATE_LIMIT_MAX_REQUESTS = 5; // 5 syncs per minute

function checkRateLimit(userId: string): boolean {
  const now = Date.now();
  const record = rateLimitMap.get(userId);

  if (!record || now > record.resetAt) {
    rateLimitMap.set(userId, { count: 1, resetAt: now + RATE_LIMIT_WINDOW_MS });
    return true;
  }

  if (record.count >= RATE_LIMIT_MAX_REQUESTS) {
    return false;
  }

  record.count++;
  return true;
}

/**
 * POST /api/hackathons/[id]/submissions/sync
 * Sync submissions from external form CSV
 * Uses batch operations for performance at scale
 */
export async function POST(
  request: Request,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;

    const cookieStore = await cookies();
    const session = await getIronSession<SessionData>(
      cookieStore,
      sessionOptions
    );

    if (!session.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Rate limiting
    if (!checkRateLimit(session.user.id)) {
      return NextResponse.json(
        { error: "Rate limit exceeded. Max 5 syncs per minute." },
        { status: 429 }
      );
    }

    // Check hackathon exists and user has access
    const hackathon = await getHackathonById(id);
    if (!hackathon) {
      return NextResponse.json(
        { error: "Hackathon not found" },
        { status: 404 }
      );
    }

    if (hackathon.organizationId !== session.user.organizationId) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const body = await request.json();
    const parseResult = syncRequestSchema.safeParse(body);

    if (!parseResult.success) {
      return NextResponse.json(
        { error: "Invalid input", details: parseResult.error.flatten() },
        { status: 400 }
      );
    }

    const { csvContent, columnMapping, updateExisting } = parseResult.data;

    // Limit CSV size (prevent memory issues)
    if (csvContent.length > 5 * 1024 * 1024) {
      return NextResponse.json(
        { error: "CSV too large. Maximum 5MB." },
        { status: 400 }
      );
    }

    // Parse CSV
    const { headers, rows, errors: parseErrors } = parseCSV(csvContent);

    if (parseErrors.length > 0) {
      return NextResponse.json(
        { error: "CSV parse errors", details: parseErrors },
        { status: 400 }
      );
    }

    // Limit row count
    if (rows.length > 5000) {
      return NextResponse.json(
        { error: "Too many rows. Maximum 5000 per sync." },
        { status: 400 }
      );
    }

    // Validate email column exists
    if (!headers.includes(columnMapping.email)) {
      return NextResponse.json(
        { error: `Email column "${columnMapping.email}" not found in CSV` },
        { status: 400 }
      );
    }

    // Get all team members for matching (single query)
    const teamMembers = await getTeamMembersByHackathon(id);
    const membersByEmail = new Map(
      teamMembers
        .filter((m) => m.email)
        .map((m) => [m.email!.toLowerCase(), m])
    );

    // Get existing submissions (single query)
    const emails = rows
      .map((r) => r[columnMapping.email]?.toLowerCase())
      .filter(Boolean);
    const existingSubmissions = await getSubmissionsByEmails(id, emails);
    const submissionsByEmail = new Map(
      existingSubmissions
        .filter((s) => s.submitterEmail)
        .map((s) => [s.submitterEmail!.toLowerCase(), s])
    );

    // Prepare batch operations
    const toCreate: Array<{
      teamId: string;
      hackathonId: string;
      organizationId: string;
      projectName: string;
      description: string;
      repositoryUrl: string;
      demoUrl?: string;
      submittedBy: string;
      submitterUsername: string;
      submitterName?: string;
      submitterEmail: string;
      formData?: string;
    }> = [];

    const toUpdate: Array<{
      id: string;
      projectName?: string;
      repositoryUrl?: string;
      demoUrl?: string;
      formData?: string;
    }> = [];

    const results = {
      totalRows: rows.length,
      matched: 0,
      unmatched: 0,
      created: 0,
      updated: 0,
      skipped: 0,
      errors: [] as string[],
    };

    // Process rows and collect batch operations
    for (let i = 0; i < rows.length; i++) {
      const row = rows[i];
      const rowNum = i + 2; // 1-indexed + header row

      try {
        const mapped = mapRowToSubmission(row, columnMapping, headers);
        const email = mapped.email.toLowerCase();

        if (!email) {
          results.errors.push(`Row ${rowNum}: Empty email`);
          results.skipped++;
          continue;
        }

        // Find team member by email
        const member = membersByEmail.get(email);

        if (!member) {
          results.unmatched++;
          // Only add first 50 unmatched errors to avoid huge response
          if (results.errors.length < 50) {
            results.errors.push(
              `Row ${rowNum}: Email "${mapped.email}" not found in registered teams`
            );
          }
          continue;
        }

        results.matched++;

        // Check if submission exists
        const existingSubmission = submissionsByEmail.get(email);

        if (existingSubmission) {
          if (updateExisting) {
            toUpdate.push({
              id: existingSubmission.id,
              projectName: mapped.projectName || existingSubmission.projectName,
              repositoryUrl:
                mapped.repositoryUrl || existingSubmission.repositoryUrl,
              demoUrl: mapped.demoUrl || existingSubmission.demoUrl || undefined,
              formData: JSON.stringify(mapped.formData),
            });
          } else {
            results.skipped++;
          }
        } else {
          // Create new submission
          if (!mapped.repositoryUrl) {
            if (results.errors.length < 50) {
              results.errors.push(
                `Row ${rowNum}: Missing repository URL for new submission`
              );
            }
            results.skipped++;
            continue;
          }

          toCreate.push({
            teamId: member.teamId,
            hackathonId: id,
            organizationId: session.user.organizationId,
            projectName: mapped.projectName,
            description: "Imported from external form",
            repositoryUrl: mapped.repositoryUrl,
            demoUrl: mapped.demoUrl,
            submittedBy: member.userId,
            submitterUsername: member.username,
            submitterName: member.name || undefined,
            submitterEmail: email,
            formData: JSON.stringify(mapped.formData),
          });
        }
      } catch (error) {
        if (results.errors.length < 50) {
          results.errors.push(
            `Row ${rowNum}: ${error instanceof Error ? error.message : "Unknown error"}`
          );
        }
        results.skipped++;
      }
    }

    // Execute batch operations
    if (toCreate.length > 0) {
      await batchCreateSubmissionsFromSync(toCreate);
      results.created = toCreate.length;
    }

    if (toUpdate.length > 0) {
      await batchUpdateSubmissionsFromSync(toUpdate);
      results.updated = toUpdate.length;
    }

    // Record sync history
    await createSubmissionSync({
      hackathonId: id,
      organizationId: session.user.organizationId,
      fileName: "upload.csv",
      totalRows: results.totalRows,
      matched: results.matched,
      unmatched: results.unmatched,
      created: results.created,
      updated: results.updated,
      syncedBy: session.user.id,
      syncedByUsername: session.user.username,
      syncedByName: session.user.name,
    });

    // Invalidate caches
    invalidateCache(`hackathon:${id}:submissions`);
    invalidateCache(`hackathon:${id}`);

    // Truncate errors if too many
    if (results.errors.length === 50) {
      results.errors.push("... (additional errors truncated)");
    }

    return NextResponse.json({
      success: true,
      results,
    });
  } catch (error) {
    console.error("Error syncing submissions:", error);
    return NextResponse.json(
      { error: "Failed to sync submissions" },
      { status: 500 }
    );
  }
}
