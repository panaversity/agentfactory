import { notFound } from "next/navigation";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import Link from "next/link";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById, canManageHackathon } from "@/db/queries/hackathons";
import { getTeamsWithMemberCount, isUserRegistered, getUserTeam } from "@/db/queries/teams";
import { hasRole } from "@/db/queries/roles";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { HackathonStatusBadge } from "@/components/hackathons/hackathon-status-badge";
import {
  Calendar,
  Users,
  Settings,
  Clock,
  Eye,
  EyeOff,
  ArrowLeft,
  UserPlus,
  Gavel,
  Trophy,
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
    title: hackathon.title,
    description: hackathon.description,
  };
}

export default async function HackathonDetailPage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id } = await params;

  const session = await getIronSession<SessionData>(
    await cookies(),
    sessionOptions
  );

  if (!session.isLoggedIn || !session.user) {
    notFound();
  }

  const hackathon = await getHackathonById(id);

  if (!hackathon) {
    notFound();
  }

  // Check access
  if (
    !hackathon.published &&
    hackathon.organizationId !== session.user.organizationId
  ) {
    notFound();
  }

  // Fetch all user-related data in parallel to avoid N+1 queries
  const [isManager, isJudge, isRegistered, _userTeam, teams] = await Promise.all([
    canManageHackathon(id, session.user.id),
    hasRole(id, session.user.id, "judge"),
    isUserRegistered(id, session.user.id),
    getUserTeam(id, session.user.id),
    getTeamsWithMemberCount(id),
  ]);
  // Note: _userTeam fetched for potential future use (e.g., showing user's team badge)
  void _userTeam;

  const formatDate = (date: string | Date) => {
    const dateObj = typeof date === "string" ? new Date(date) : date;
    return dateObj.toLocaleDateString("en-US", {
      weekday: "long",
      year: "numeric",
      month: "long",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    });
  };

  return (
    <div className="space-y-8">
      <div className="flex items-center gap-4">
        <Button variant="ghost" size="sm" asChild>
          <Link href="/dashboard">
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back
          </Link>
        </Button>
      </div>

      <div className="flex items-start justify-between">
        <div className="space-y-2">
          <div className="flex items-center gap-3">
            <h1 className="text-3xl font-bold tracking-tight">
              {hackathon.title}
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
          <div className="flex items-center gap-2 text-muted-foreground">
            {hackathon.published ? (
              <span className="inline-flex items-center gap-1 text-green-600">
                <Eye className="h-4 w-4" />
                Published
              </span>
            ) : (
              <span className="inline-flex items-center gap-1">
                <EyeOff className="h-4 w-4" />
                Hidden
              </span>
            )}
          </div>
        </div>

        <div className="flex gap-2">
          {hackathon.published && (
            <Button asChild variant={isRegistered ? "default" : "outline"}>
              <Link href={`/hackathons/${hackathon.id}/participate`}>
                <UserPlus className="mr-2 h-4 w-4" />
                {isRegistered ? "My Team" : "Participate"}
              </Link>
            </Button>
          )}
          {isJudge && (
            <Button asChild variant="outline">
              <Link href={`/hackathons/${hackathon.id}/judge`}>
                <Gavel className="mr-2 h-4 w-4" />
                Judge
              </Link>
            </Button>
          )}
          {isManager && (
            <Button asChild>
              <Link href={`/hackathons/${hackathon.id}/manage`}>
                <Settings className="mr-2 h-4 w-4" />
                Manage
              </Link>
            </Button>
          )}
        </div>
      </div>

      <div className="grid gap-6 md:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle className="text-lg">About</CardTitle>
          </CardHeader>
          <CardContent>
            <p className="text-muted-foreground whitespace-pre-wrap">
              {hackathon.description}
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle className="text-lg">Details</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="flex items-start gap-3">
              <Calendar className="h-5 w-5 text-muted-foreground mt-0.5" />
              <div>
                <p className="font-medium">Event Dates</p>
                <p className="text-sm text-muted-foreground">
                  {formatDate(hackathon.startDate)}
                </p>
                <p className="text-sm text-muted-foreground">
                  to {formatDate(hackathon.endDate)}
                </p>
              </div>
            </div>

            <div className="flex items-start gap-3">
              <Clock className="h-5 w-5 text-muted-foreground mt-0.5" />
              <div>
                <p className="font-medium">Deadlines</p>
                <p className="text-sm text-muted-foreground">
                  Registration: {formatDate(hackathon.registrationDeadline)}
                </p>
                <p className="text-sm text-muted-foreground">
                  Submission: {formatDate(hackathon.submissionDeadline)}
                </p>
              </div>
            </div>

            <div className="flex items-start gap-3">
              <Users className="h-5 w-5 text-muted-foreground mt-0.5" />
              <div>
                <p className="font-medium">Team Size</p>
                <p className="text-sm text-muted-foreground">
                  {hackathon.minTeamSize} - {hackathon.maxTeamSize} members
                </p>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Registered Teams */}
      <Card>
        <CardHeader className="flex flex-row items-center justify-between">
          <CardTitle className="text-lg flex items-center gap-2">
            <Trophy className="h-5 w-5" />
            Registered Teams
          </CardTitle>
          <Badge variant="secondary">{teams.length} teams</Badge>
        </CardHeader>
        <CardContent>
          {teams.length === 0 ? (
            <p className="text-muted-foreground text-center py-4">
              No teams have registered yet.
            </p>
          ) : (
            <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
              {teams.slice(0, 6).map((team) => (
                <div
                  key={team.id}
                  className="p-4 rounded-lg border bg-card space-y-2"
                >
                  <div className="flex items-center justify-between">
                    <h4 className="font-medium truncate">{team.name}</h4>
                    <Badge
                      variant={team.isReady ? "success" : "warning"}
                      className="shrink-0"
                    >
                      {team.memberCount}/{team.maxSize}
                    </Badge>
                  </div>
                  {team.description && (
                    <p className="text-sm text-muted-foreground line-clamp-2">
                      {team.description}
                    </p>
                  )}
                </div>
              ))}
            </div>
          )}
          {teams.length > 6 && (
            <p className="text-sm text-muted-foreground text-center mt-4">
              + {teams.length - 6} more teams
            </p>
          )}
        </CardContent>
      </Card>
    </div>
  );
}
