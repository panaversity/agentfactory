import { notFound, redirect } from "next/navigation";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import Link from "next/link";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import { hasRole } from "@/db/queries/roles";
import {
  getSubmissionById,
  getJudgingCriteria,
  getJudgeScoresForSubmission,
} from "@/db/queries/submissions";
import { getTeamById } from "@/db/queries/teams";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft,
  ExternalLink,
  FileText,
  Video,
  Presentation,
  Github,
} from "lucide-react";
import { ScoringForm } from "./scoring-form";

export async function generateMetadata({
  params,
}: {
  params: Promise<{ id: string; submissionId: string }>;
}) {
  const { submissionId } = await params;
  const submission = await getSubmissionById(submissionId);

  if (!submission) {
    return { title: "Submission Not Found" };
  }

  return {
    title: `Score: ${submission.projectName}`,
    description: `Judge ${submission.projectName}`,
  };
}

export default async function ScoreSubmissionPage({
  params,
}: {
  params: Promise<{ id: string; submissionId: string }>;
}) {
  const { id: hackathonId, submissionId } = await params;

  const session = await getIronSession<SessionData>(
    await cookies(),
    sessionOptions
  );

  if (!session.isLoggedIn || !session.user) {
    redirect("/login");
  }

  const hackathon = await getHackathonById(hackathonId);

  if (!hackathon) {
    notFound();
  }

  // Check if user is a judge
  const isJudge = await hasRole(hackathonId, session.user.id, "judge");
  if (!isJudge) {
    notFound();
  }

  const submission = await getSubmissionById(submissionId);

  if (!submission || submission.hackathonId !== hackathonId) {
    notFound();
  }

  // Fetch remaining data in parallel
  const [team, criteria, existingScores] = await Promise.all([
    getTeamById(submission.teamId),
    getJudgingCriteria(hackathonId),
    getJudgeScoresForSubmission(submissionId, session.user.id),
  ]);

  const canScore =
    hackathon.status === "judging" || hackathon.status === "active";

  // Map existing scores to criteria
  const criteriaWithScores = criteria.map((c) => {
    const score = existingScores.find((s) => s.criterionId === c.id);
    return {
      ...c,
      existingScore: score?.score ?? null,
      existingFeedback: score?.feedback ?? null,
    };
  });

  return (
    <div className="space-y-8">
      {/* Header */}
      <div className="flex items-center gap-4">
        <Button variant="ghost" size="sm" asChild>
          <Link href={`/hackathons/${hackathonId}/judge`}>
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back to Submissions
          </Link>
        </Button>
      </div>

      <div className="flex items-start justify-between">
        <div className="space-y-2">
          <h1 className="text-3xl font-bold tracking-tight">
            {submission.projectName}
          </h1>
          <p className="text-muted-foreground">by {team?.name || "Unknown Team"}</p>
        </div>
        {existingScores.length >= criteria.length && (
          <Badge variant="success" className="text-base px-4 py-1">
            Scored
          </Badge>
        )}
      </div>

      {/* Project Details */}
      <Card>
        <CardHeader>
          <CardTitle className="text-lg flex items-center gap-2">
            <FileText className="h-5 w-5" />
            Project Details
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div>
            <h4 className="text-sm font-medium text-muted-foreground mb-1">
              Description
            </h4>
            <p className="whitespace-pre-wrap">{submission.description}</p>
          </div>

          <Separator />

          <div className="flex flex-wrap gap-4">
            <a
              href={submission.repositoryUrl}
              target="_blank"
              rel="noopener noreferrer"
              className="inline-flex items-center gap-2 px-4 py-2 rounded-lg bg-muted hover:bg-muted/80 transition-colors"
            >
              <Github className="h-5 w-5" />
              <span>Repository</span>
              <ExternalLink className="h-4 w-4" />
            </a>
            {submission.demoUrl && (
              <a
                href={submission.demoUrl}
                target="_blank"
                rel="noopener noreferrer"
                className="inline-flex items-center gap-2 px-4 py-2 rounded-lg bg-muted hover:bg-muted/80 transition-colors"
              >
                <Video className="h-5 w-5" />
                <span>Demo</span>
                <ExternalLink className="h-4 w-4" />
              </a>
            )}
            {submission.presentationUrl && (
              <a
                href={submission.presentationUrl}
                target="_blank"
                rel="noopener noreferrer"
                className="inline-flex items-center gap-2 px-4 py-2 rounded-lg bg-muted hover:bg-muted/80 transition-colors"
              >
                <Presentation className="h-5 w-5" />
                <span>Presentation</span>
                <ExternalLink className="h-4 w-4" />
              </a>
            )}
          </div>
        </CardContent>
      </Card>

      {/* Scoring Form */}
      <Card>
        <CardHeader>
          <CardTitle className="text-lg">Score Submission</CardTitle>
        </CardHeader>
        <CardContent>
          {canScore ? (
            <ScoringForm
              hackathonId={hackathonId}
              submissionId={submissionId}
              criteria={criteriaWithScores}
            />
          ) : (
            <div className="py-8 text-center">
              <p className="text-muted-foreground">
                Scoring is not available at this time.
              </p>
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  );
}
