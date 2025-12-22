import { notFound, redirect } from "next/navigation";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import Link from "next/link";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import { hasRole } from "@/db/queries/roles";
import { getSubmissionsForJudge, getJudgingCriteria } from "@/db/queries/submissions";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { HackathonStatusBadge } from "@/components/hackathons/hackathon-status-badge";
import {
  ArrowLeft,
  Gavel,
  ExternalLink,
  CheckCircle2,
  Clock,
} from "lucide-react";

export async function generateMetadata({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id } = await params;
  const hackathon = await getHackathonById(id);

  if (!hackathon) {
    return { title: "Hackathon Not Found" };
  }

  return {
    title: `Judge - ${hackathon.title}`,
    description: `Judge submissions for ${hackathon.title}`,
  };
}

export default async function JudgePage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id: hackathonId } = await params;

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

  const submissions = await getSubmissionsForJudge(hackathonId, session.user.id);
  const criteria = await getJudgingCriteria(hackathonId);

  const scoredCount = submissions.filter((s) => s.scoringComplete).length;
  const totalCount = submissions.length;
  const progress = totalCount > 0 ? (scoredCount / totalCount) * 100 : 0;

  const canScore =
    hackathon.status === "judging" || hackathon.status === "active";

  return (
    <div className="space-y-8">
      {/* Header */}
      <div className="flex items-center gap-4">
        <Button variant="ghost" size="sm" asChild>
          <Link href={`/hackathons/${hackathonId}`}>
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back
          </Link>
        </Button>
      </div>

      <div className="flex items-start justify-between">
        <div className="space-y-2">
          <div className="flex items-center gap-3">
            <h1 className="text-3xl font-bold tracking-tight">
              Judging Dashboard
            </h1>
            <HackathonStatusBadge
              status={
                hackathon.status as
                  | "draft"
                  | "open"
                  | "active"
                  | "judging"
                  | "completed"
              }
            />
          </div>
          <p className="text-muted-foreground">{hackathon.title}</p>
        </div>
      </div>

      {/* Progress Card */}
      <Card>
        <CardHeader>
          <CardTitle className="text-lg flex items-center gap-2">
            <Gavel className="h-5 w-5" />
            Your Progress
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="flex items-center justify-between">
            <span className="text-sm text-muted-foreground">
              Submissions Scored
            </span>
            <span className="font-medium">
              {scoredCount} / {totalCount}
            </span>
          </div>
          <Progress value={progress} />
          {scoredCount === totalCount && totalCount > 0 ? (
            <div className="flex items-center gap-2 text-emerald-600">
              <CheckCircle2 className="h-4 w-4" />
              <span className="text-sm font-medium">
                All submissions scored!
              </span>
            </div>
          ) : (
            <p className="text-sm text-muted-foreground">
              {totalCount - scoredCount} submissions remaining
            </p>
          )}
        </CardContent>
      </Card>

      {/* Criteria Info */}
      {criteria.length > 0 && (
        <Card>
          <CardHeader>
            <CardTitle className="text-lg">Judging Criteria</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
              {criteria.map((c) => (
                <div
                  key={c.id}
                  className="p-4 rounded-lg bg-muted/30 space-y-1"
                >
                  <div className="flex items-center justify-between">
                    <h4 className="font-medium">{c.name}</h4>
                    <Badge variant="secondary">
                      Max: {c.maxScore}
                    </Badge>
                  </div>
                  {c.description && (
                    <p className="text-sm text-muted-foreground">
                      {c.description}
                    </p>
                  )}
                  <p className="text-xs text-muted-foreground">
                    Weight: {c.weight}x
                  </p>
                </div>
              ))}
            </div>
          </CardContent>
        </Card>
      )}

      {/* Submissions List */}
      <Card>
        <CardHeader>
          <CardTitle className="text-lg">Submissions</CardTitle>
        </CardHeader>
        <CardContent>
          {submissions.length === 0 ? (
            <div className="py-8 text-center">
              <Clock className="mx-auto h-12 w-12 text-muted-foreground mb-4" />
              <h3 className="text-lg font-semibold mb-2">
                No Submissions Yet
              </h3>
              <p className="text-muted-foreground max-w-md mx-auto">
                Teams haven&apos;t submitted their projects yet. Check back
                later.
              </p>
            </div>
          ) : (
            <div className="space-y-4">
              {submissions.map((submission) => (
                <div
                  key={submission.id}
                  className="flex items-center justify-between p-4 rounded-lg border"
                >
                  <div className="space-y-1">
                    <div className="flex items-center gap-2">
                      <h4 className="font-medium">{submission.projectName}</h4>
                      {submission.scoringComplete ? (
                        <Badge variant="success">Scored</Badge>
                      ) : submission.hasJudgeScored ? (
                        <Badge variant="warning">
                          {submission.judgeScoreCount} / {submission.criteriaCount}
                        </Badge>
                      ) : (
                        <Badge variant="secondary">Not Scored</Badge>
                      )}
                    </div>
                    <p className="text-sm text-muted-foreground">
                      by {submission.teamName}
                    </p>
                    <div className="flex items-center gap-4 text-sm">
                      <a
                        href={submission.repositoryUrl}
                        target="_blank"
                        rel="noopener noreferrer"
                        className="flex items-center gap-1 text-primary hover:underline"
                      >
                        Repository
                        <ExternalLink className="h-3 w-3" />
                      </a>
                      {submission.demoUrl && (
                        <a
                          href={submission.demoUrl}
                          target="_blank"
                          rel="noopener noreferrer"
                          className="flex items-center gap-1 text-primary hover:underline"
                        >
                          Demo
                          <ExternalLink className="h-3 w-3" />
                        </a>
                      )}
                    </div>
                  </div>
                  <Button
                    asChild
                    variant={submission.scoringComplete ? "outline" : "default"}
                    disabled={!canScore}
                  >
                    <Link
                      href={`/hackathons/${hackathonId}/judge/${submission.id}`}
                    >
                      {submission.scoringComplete ? "View Score" : "Score"}
                    </Link>
                  </Button>
                </div>
              ))}
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  );
}
