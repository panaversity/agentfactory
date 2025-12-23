import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import {
  getTeamMembersByHackathon,
  getSubmissionsByEmails,
  createSubmissionFromSync,
  updateSubmissionFromSync,
} from "@/db/queries/submissions";
import { createSubmissionSync } from "@/db/queries/submission-fields";
import { syncRequestSchema } from "@/lib/validation/submission-fields";
import { parseCSV, mapRowToSubmission } from "@/lib/csv-parser";
import { invalidateCache } from "@/lib/cache";

/**
 * POST /api/hackathons/[id]/submissions/sync
 * Sync submissions from external form CSV
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

    // Parse CSV
    const { headers, rows, errors: parseErrors } = parseCSV(csvContent);

    if (parseErrors.length > 0) {
      return NextResponse.json(
        { error: "CSV parse errors", details: parseErrors },
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

    // Get all team members for matching
    const teamMembers = await getTeamMembersByHackathon(id);
    const membersByEmail = new Map(
      teamMembers
        .filter((m) => m.email)
        .map((m) => [m.email!.toLowerCase(), m])
    );

    // Get existing submissions
    const emails = rows
      .map((r) => r[columnMapping.email]?.toLowerCase())
      .filter(Boolean);
    const existingSubmissions = await getSubmissionsByEmails(id, emails);
    const submissionsByEmail = new Map(
      existingSubmissions
        .filter((s) => s.submitterEmail)
        .map((s) => [s.submitterEmail!.toLowerCase(), s])
    );

    // Process rows
    const results = {
      totalRows: rows.length,
      matched: 0,
      unmatched: 0,
      created: 0,
      updated: 0,
      skipped: 0,
      errors: [] as string[],
    };

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
          results.errors.push(
            `Row ${rowNum}: Email "${mapped.email}" not found in registered teams`
          );
          continue;
        }

        results.matched++;

        // Check if submission exists
        const existingSubmission = submissionsByEmail.get(email);

        if (existingSubmission) {
          if (updateExisting) {
            await updateSubmissionFromSync(existingSubmission.id, {
              projectName: mapped.projectName || existingSubmission.projectName,
              repositoryUrl:
                mapped.repositoryUrl || existingSubmission.repositoryUrl,
              demoUrl: mapped.demoUrl || existingSubmission.demoUrl || undefined,
              formData: JSON.stringify(mapped.formData),
            });
            results.updated++;
          } else {
            results.skipped++;
          }
        } else {
          // Create new submission
          if (!mapped.repositoryUrl) {
            results.errors.push(
              `Row ${rowNum}: Missing repository URL for new submission`
            );
            results.skipped++;
            continue;
          }

          await createSubmissionFromSync({
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
          results.created++;
        }
      } catch (error) {
        results.errors.push(
          `Row ${rowNum}: ${error instanceof Error ? error.message : "Unknown error"}`
        );
        results.skipped++;
      }
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
