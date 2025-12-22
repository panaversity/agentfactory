import { notFound, redirect } from "next/navigation";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import Link from "next/link";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import {
  getUserTeam,
  getTeamMembers,
  isRegistrationOpen,
  isUserRegistered,
} from "@/db/queries/teams";
import { getSubmissionByTeam } from "@/db/queries/submissions";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Avatar } from "@/components/ui/avatar";
import { Separator } from "@/components/ui/separator";
import { HackathonStatusBadge } from "@/components/hackathons/hackathon-status-badge";
import { CreateTeamDialog } from "@/components/teams/create-team-dialog";
import { JoinTeamDialog } from "@/components/teams/join-team-dialog";
import {
  ArrowLeft,
  Clock,
  Users,
  Send,
  FileText,
  Crown,
  User,
} from "lucide-react";
import { SubmissionForm } from "./submission-form";
import { RegisterButton } from "./register-button";

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
    title: `Participate - ${hackathon.title}`,
    description: `Join and participate in ${hackathon.title}`,
  };
}

export default async function ParticipatePage({
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

  // Check if hackathon is published
  if (!hackathon.published) {
    notFound();
  }

  // First batch: queries that don't depend on each other
  const [isRegistered, registrationOpen, userTeam] = await Promise.all([
    isUserRegistered(hackathonId, session.user.id),
    isRegistrationOpen(hackathonId),
    getUserTeam(hackathonId, session.user.id),
  ]);

  // Second batch: queries that depend on userTeam
  const [teamMembers, submission] = userTeam
    ? await Promise.all([
        getTeamMembers(userTeam.id),
        getSubmissionByTeam(userTeam.id),
      ])
    : [[], null];

  const isLeader = userTeam?.leaderId === session.user.id;
  const canSubmit =
    hackathon.status === "active" &&
    new Date() < new Date(hackathon.submissionDeadline);

  const formatDate = (date: Date) => {
    return date.toLocaleDateString("en-US", {
      month: "short",
      day: "numeric",
      year: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    });
  };

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
          <p className="text-muted-foreground">{hackathon.description}</p>
        </div>
      </div>

      {/* Key Dates */}
      <Card>
        <CardHeader>
          <CardTitle className="text-lg flex items-center gap-2">
            <Clock className="h-5 w-5" />
            Key Dates
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
            <div>
              <p className="text-sm text-muted-foreground">
                Registration Deadline
              </p>
              <p className="font-medium">
                {formatDate(hackathon.registrationDeadline)}
              </p>
            </div>
            <div>
              <p className="text-sm text-muted-foreground">Start Date</p>
              <p className="font-medium">{formatDate(hackathon.startDate)}</p>
            </div>
            <div>
              <p className="text-sm text-muted-foreground">
                Submission Deadline
              </p>
              <p className="font-medium">
                {formatDate(hackathon.submissionDeadline)}
              </p>
            </div>
            <div>
              <p className="text-sm text-muted-foreground">End Date</p>
              <p className="font-medium">{formatDate(hackathon.endDate)}</p>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Not Registered State */}
      {!isRegistered && (
        <Card className="border-dashed">
          <CardContent className="py-12 text-center">
            <Users className="mx-auto h-12 w-12 text-muted-foreground mb-4" />
            <h3 className="text-lg font-semibold mb-2">
              Register for this Hackathon
            </h3>
            <p className="text-muted-foreground mb-6 max-w-md mx-auto">
              {registrationOpen
                ? "Join this hackathon to create or join a team and submit your project."
                : "Registration is closed for this hackathon."}
            </p>
            {registrationOpen && (
              <RegisterButton hackathonId={hackathonId} />
            )}
          </CardContent>
        </Card>
      )}

      {/* Registered but no team */}
      {isRegistered && !userTeam && (
        <Card>
          <CardHeader>
            <CardTitle className="text-lg flex items-center gap-2">
              <Users className="h-5 w-5" />
              Your Team
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-6">
            <div className="text-center py-8">
              <Users className="mx-auto h-12 w-12 text-muted-foreground mb-4" />
              <h3 className="text-lg font-semibold mb-2">
                You&apos;re not in a team yet
              </h3>
              <p className="text-muted-foreground mb-6 max-w-md mx-auto">
                Create a new team or join an existing one using an invite code.
                Team size: {hackathon.minTeamSize} - {hackathon.maxTeamSize}{" "}
                members.
              </p>
              <div className="flex justify-center gap-4">
                <CreateTeamDialog hackathonId={hackathonId} />
                <JoinTeamDialog hackathonId={hackathonId} />
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Has Team */}
      {isRegistered && userTeam && (
        <>
          {/* Team Section */}
          <Card>
            <CardHeader className="flex flex-row items-center justify-between">
              <CardTitle className="text-lg flex items-center gap-2">
                <Users className="h-5 w-5" />
                Your Team
              </CardTitle>
              <Badge
                variant={
                  teamMembers.length >= hackathon.minTeamSize
                    ? "success"
                    : "warning"
                }
              >
                {teamMembers.length} / {hackathon.maxTeamSize} members
              </Badge>
            </CardHeader>
            <CardContent className="space-y-6">
              <div className="flex items-center justify-between">
                <div>
                  <h3 className="text-xl font-semibold">{userTeam.name}</h3>
                  {userTeam.description && (
                    <p className="text-muted-foreground mt-1">
                      {userTeam.description}
                    </p>
                  )}
                </div>
                {isLeader && (
                  <div className="flex items-center gap-2 p-3 bg-muted/50 rounded-lg">
                    <span className="text-sm text-muted-foreground">
                      Invite Code:
                    </span>
                    <code className="font-mono font-bold tracking-widest">
                      {userTeam.inviteCode}
                    </code>
                  </div>
                )}
              </div>

              <Separator />

              <div>
                <h4 className="text-sm font-medium text-muted-foreground mb-3">
                  Team Members
                </h4>
                <div className="grid gap-3 sm:grid-cols-2">
                  {teamMembers.map((member) => (
                    <div
                      key={member.id}
                      className="flex items-center gap-3 p-3 rounded-lg bg-muted/30"
                    >
                      <Avatar fallback={member.userId} size="sm" />
                      <div className="flex-1 min-w-0">
                        <p className="font-medium truncate">
                          {member.userId === session.user?.id
                            ? "You"
                            : `User ${member.userId.slice(0, 8)}...`}
                        </p>
                        <p className="text-xs text-muted-foreground">
                          Joined {member.joinedAt ? formatDate(member.joinedAt) : "recently"}
                        </p>
                      </div>
                      {member.role === "leader" ? (
                        <Crown className="h-4 w-4 text-amber-500" />
                      ) : (
                        <User className="h-4 w-4 text-muted-foreground" />
                      )}
                    </div>
                  ))}
                </div>
              </div>

              {teamMembers.length < hackathon.minTeamSize && (
                <div className="p-4 bg-amber-500/10 border border-amber-500/20 rounded-lg">
                  <p className="text-sm text-amber-700 dark:text-amber-400">
                    Your team needs at least {hackathon.minTeamSize} members to
                    submit. Share your invite code with teammates!
                  </p>
                </div>
              )}
            </CardContent>
          </Card>

          {/* Submission Section */}
          <Card>
            <CardHeader className="flex flex-row items-center justify-between">
              <CardTitle className="text-lg flex items-center gap-2">
                <FileText className="h-5 w-5" />
                Project Submission
              </CardTitle>
              {submission && (
                <Badge variant="success">Submitted</Badge>
              )}
            </CardHeader>
            <CardContent>
              {submission ? (
                <div className="space-y-4">
                  <div className="grid gap-4 sm:grid-cols-2">
                    <div>
                      <p className="text-sm text-muted-foreground">
                        Project Name
                      </p>
                      <p className="font-medium">{submission.projectName}</p>
                    </div>
                    <div>
                      <p className="text-sm text-muted-foreground">
                        Submitted At
                      </p>
                      <p className="font-medium">
                        {formatDate(submission.submittedAt)}
                      </p>
                    </div>
                  </div>
                  <div>
                    <p className="text-sm text-muted-foreground">Description</p>
                    <p className="mt-1">{submission.description}</p>
                  </div>
                  <div className="flex flex-wrap gap-4">
                    <a
                      href={submission.repositoryUrl}
                      target="_blank"
                      rel="noopener noreferrer"
                      className="text-sm text-primary hover:underline"
                    >
                      View Repository →
                    </a>
                    {submission.demoUrl && (
                      <a
                        href={submission.demoUrl}
                        target="_blank"
                        rel="noopener noreferrer"
                        className="text-sm text-primary hover:underline"
                      >
                        View Demo →
                      </a>
                    )}
                    {submission.presentationUrl && (
                      <a
                        href={submission.presentationUrl}
                        target="_blank"
                        rel="noopener noreferrer"
                        className="text-sm text-primary hover:underline"
                      >
                        View Presentation →
                      </a>
                    )}
                  </div>
                  {canSubmit && isLeader && (
                    <div className="pt-4">
                      <p className="text-sm text-muted-foreground mb-2">
                        You can update your submission until the deadline.
                      </p>
                    </div>
                  )}
                </div>
              ) : canSubmit ? (
                isLeader ? (
                  teamMembers.length >= hackathon.minTeamSize ? (
                    <SubmissionForm
                      hackathonId={hackathonId}
                      teamId={userTeam.id}
                    />
                  ) : (
                    <div className="py-8 text-center">
                      <Send className="mx-auto h-12 w-12 text-muted-foreground mb-4" />
                      <h3 className="text-lg font-semibold mb-2">
                        Team Not Ready
                      </h3>
                      <p className="text-muted-foreground max-w-md mx-auto">
                        Your team needs at least {hackathon.minTeamSize} members
                        before you can submit a project.
                      </p>
                    </div>
                  )
                ) : (
                  <div className="py-8 text-center">
                    <Send className="mx-auto h-12 w-12 text-muted-foreground mb-4" />
                    <h3 className="text-lg font-semibold mb-2">
                      Only Team Leaders Can Submit
                    </h3>
                    <p className="text-muted-foreground max-w-md mx-auto">
                      Ask your team leader to submit your project.
                    </p>
                  </div>
                )
              ) : (
                <div className="py-8 text-center">
                  <Clock className="mx-auto h-12 w-12 text-muted-foreground mb-4" />
                  <h3 className="text-lg font-semibold mb-2">
                    Submissions Not Open
                  </h3>
                  <p className="text-muted-foreground max-w-md mx-auto">
                    {hackathon.status === "open"
                      ? "Submissions will open when the hackathon starts."
                      : "The submission deadline has passed."}
                  </p>
                </div>
              )}
            </CardContent>
          </Card>
        </>
      )}
    </div>
  );
}
