import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Progress } from "@/components/ui/progress";
import { Users, Trophy, FileText, Gavel } from "lucide-react";

interface AnalyticsDashboardProps {
  stats: {
    totalTeams: number;
    totalParticipants: number;
    totalSubmissions: number;
    totalJudges: number;
    teamsReady: number;
    submissionsScored: number;
    avgTeamSize: number;
    registrationProgress: number;
    submissionProgress: number;
    judgingProgress: number;
  };
}

export function AnalyticsDashboard({ stats }: AnalyticsDashboardProps) {
  return (
    <div className="space-y-6">
      {/* Summary Cards */}
      <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center gap-4">
              <div className="p-3 rounded-full bg-blue-500/15">
                <Users className="h-6 w-6 text-blue-600" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Participants</p>
                <p className="text-2xl font-bold">{stats.totalParticipants}</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center gap-4">
              <div className="p-3 rounded-full bg-purple-500/15">
                <Trophy className="h-6 w-6 text-purple-600" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Teams</p>
                <p className="text-2xl font-bold">{stats.totalTeams}</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center gap-4">
              <div className="p-3 rounded-full bg-emerald-500/15">
                <FileText className="h-6 w-6 text-emerald-600" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Submissions</p>
                <p className="text-2xl font-bold">{stats.totalSubmissions}</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="pt-6">
            <div className="flex items-center gap-4">
              <div className="p-3 rounded-full bg-amber-500/15">
                <Gavel className="h-6 w-6 text-amber-600" />
              </div>
              <div>
                <p className="text-sm text-muted-foreground">Judges</p>
                <p className="text-2xl font-bold">{stats.totalJudges}</p>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Progress Sections */}
      <div className="grid gap-6 md:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle className="text-lg">Team Formation</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span className="text-muted-foreground">Teams Ready</span>
                <span className="font-medium">
                  {stats.teamsReady} / {stats.totalTeams}
                </span>
              </div>
              <Progress
                value={
                  stats.totalTeams > 0
                    ? (stats.teamsReady / stats.totalTeams) * 100
                    : 0
                }
              />
            </div>
            <div className="pt-2 grid grid-cols-2 gap-4 text-sm">
              <div>
                <p className="text-muted-foreground">Avg Team Size</p>
                <p className="text-lg font-medium">
                  {stats.avgTeamSize.toFixed(1)}
                </p>
              </div>
              <div>
                <p className="text-muted-foreground">Teams Forming</p>
                <p className="text-lg font-medium">
                  {stats.totalTeams - stats.teamsReady}
                </p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle className="text-lg">Submission Progress</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span className="text-muted-foreground">
                  Submissions Received
                </span>
                <span className="font-medium">
                  {stats.totalSubmissions} / {stats.teamsReady}
                </span>
              </div>
              <Progress value={stats.submissionProgress} />
            </div>
            <div className="pt-2">
              <p className="text-sm text-muted-foreground">
                {stats.teamsReady - stats.totalSubmissions} teams haven&apos;t
                submitted yet
              </p>
            </div>
          </CardContent>
        </Card>

        <Card className="md:col-span-2">
          <CardHeader>
            <CardTitle className="text-lg">Judging Progress</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span className="text-muted-foreground">Submissions Scored</span>
                <span className="font-medium">
                  {stats.submissionsScored} / {stats.totalSubmissions}
                </span>
              </div>
              <Progress value={stats.judgingProgress} />
            </div>
            <div className="pt-2 grid grid-cols-3 gap-4 text-sm">
              <div>
                <p className="text-muted-foreground">Active Judges</p>
                <p className="text-lg font-medium">{stats.totalJudges}</p>
              </div>
              <div>
                <p className="text-muted-foreground">Fully Scored</p>
                <p className="text-lg font-medium">{stats.submissionsScored}</p>
              </div>
              <div>
                <p className="text-muted-foreground">Pending Review</p>
                <p className="text-lg font-medium">
                  {stats.totalSubmissions - stats.submissionsScored}
                </p>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
