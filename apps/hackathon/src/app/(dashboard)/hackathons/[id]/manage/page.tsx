import { notFound, redirect } from "next/navigation";
import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import Link from "next/link";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonById, canManageHackathon } from "@/db/queries/hackathons";
import { getTeamsWithMemberCount } from "@/db/queries/teams";
import { getJudgingCriteria, getSubmissionsWithScores } from "@/db/queries/submissions";
import { getRolesByHackathon, getJudges } from "@/db/queries/roles";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { EditHackathonForm } from "@/components/hackathons/edit-form";
import { PublishToggle } from "./publish-toggle";
import { CriteriaManager } from "./criteria-manager";
import { RoleManager } from "./role-manager";
import { AnalyticsDashboard } from "./analytics-dashboard";
import { ArrowLeft, Settings, Users, Award, BarChart } from "lucide-react";

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
    title: `Manage ${hackathon.title}`,
    description: `Manage ${hackathon.title} hackathon settings`,
  };
}

export default async function ManageHackathonPage({
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
    redirect("/login");
  }

  const hackathon = await getHackathonById(id);

  if (!hackathon) {
    notFound();
  }

  // Check permissions
  const isManager = await canManageHackathon(id, session.user.id);
  if (!isManager) {
    notFound();
  }

  // Fetch data for all tabs in parallel
  const [teams, criteria, roles, submissions, judges] = await Promise.all([
    getTeamsWithMemberCount(id),
    getJudgingCriteria(id),
    getRolesByHackathon(id),
    getSubmissionsWithScores(id),
    getJudges(id),
  ]);

  // Calculate analytics stats
  const teamsReady = teams.filter((t) => t.isReady).length;
  const totalParticipants = teams.reduce((sum, t) => sum + t.memberCount, 0);
  const avgTeamSize = teams.length > 0 ? totalParticipants / teams.length : 0;
  // Count submissions that have at least one judge score
  const submissionsScored = submissions.filter((s) => s.judgeCount > 0).length;

  const stats = {
    totalTeams: teams.length,
    totalParticipants,
    totalSubmissions: submissions.length,
    totalJudges: judges.length,
    teamsReady,
    submissionsScored,
    avgTeamSize,
    registrationProgress: hackathon.maxTeamSize
      ? (teams.length / hackathon.maxTeamSize) * 100
      : 0,
    submissionProgress: teamsReady > 0 ? (submissions.length / teamsReady) * 100 : 0,
    judgingProgress:
      submissions.length > 0 ? (submissionsScored / submissions.length) * 100 : 0,
  };

  return (
    <div className="space-y-8">
      <div className="flex items-center gap-4">
        <Button variant="ghost" size="sm" asChild>
          <Link href={`/hackathons/${id}`}>
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back to Hackathon
          </Link>
        </Button>
      </div>

      <div className="flex items-start justify-between">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">
            Manage: {hackathon.title}
          </h1>
          <p className="text-muted-foreground">
            Configure hackathon settings, roles, and judging criteria
          </p>
        </div>
        <PublishToggle hackathonId={id} published={hackathon.published} />
      </div>

      <Tabs defaultValue="settings" className="space-y-6">
        <TabsList>
          <TabsTrigger value="settings" className="flex items-center gap-2">
            <Settings className="h-4 w-4" />
            Settings
          </TabsTrigger>
          <TabsTrigger value="roles" className="flex items-center gap-2">
            <Users className="h-4 w-4" />
            Roles
          </TabsTrigger>
          <TabsTrigger value="judging" className="flex items-center gap-2">
            <Award className="h-4 w-4" />
            Judging
          </TabsTrigger>
          <TabsTrigger value="analytics" className="flex items-center gap-2">
            <BarChart className="h-4 w-4" />
            Analytics
          </TabsTrigger>
        </TabsList>

        <TabsContent value="settings">
          <Card>
            <CardHeader>
              <CardTitle>Hackathon Settings</CardTitle>
            </CardHeader>
            <CardContent>
              <EditHackathonForm hackathon={hackathon} />
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="roles">
          <Card>
            <CardHeader>
              <CardTitle>Manage Roles</CardTitle>
            </CardHeader>
            <CardContent>
              <RoleManager hackathonId={id} initialRoles={roles} />
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="judging">
          <Card>
            <CardHeader>
              <CardTitle>Judging Criteria</CardTitle>
            </CardHeader>
            <CardContent>
              <CriteriaManager hackathonId={id} initialCriteria={criteria} />
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="analytics">
          <AnalyticsDashboard stats={stats} />
        </TabsContent>
      </Tabs>
    </div>
  );
}
