import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { SessionData, sessionOptions } from "@/lib/auth/session";
import { getHackathonsByOrg } from "@/db/queries/hackathons";
import { HackathonList } from "./hackathon-list";
import { CreateHackathonButton } from "@/components/hackathons/create-button";
import { Trophy } from "lucide-react";

export const metadata = {
  title: "Dashboard",
  description: "Manage your hackathons",
};

export default async function DashboardPage() {
  const session = await getIronSession<SessionData>(
    await cookies(),
    sessionOptions
  );

  if (!session.user?.organizationId) {
    return (
      <div className="flex flex-col items-center justify-center py-12">
        <Trophy className="h-12 w-12 text-muted-foreground mb-4" />
        <h2 className="text-xl font-semibold mb-2">No Organization</h2>
        <p className="text-muted-foreground text-center max-w-md">
          You need to be part of an organization to create and manage
          hackathons. Contact your administrator for access.
        </p>
      </div>
    );
  }

  const hackathons = await getHackathonsByOrg(session.user.organizationId);

  return (
    <div className="space-y-8">
      {/* Hero Section */}
      <div className="relative overflow-hidden rounded-2xl bg-gradient-hero p-8 md:p-10 animate-fade-in-up">
        <div className="absolute inset-0 bg-gradient-to-br from-primary/5 via-transparent to-primary/10" />
        <div className="absolute -bottom-20 -right-20 w-80 h-80 bg-primary/10 rounded-full blur-3xl" />
        <div className="relative flex items-center justify-between">
          <div>
            <h1 className="text-3xl md:text-4xl font-bold tracking-tight mb-2">
              Welcome to your <span className="text-gradient-brand">Dashboard</span>
            </h1>
            <p className="text-muted-foreground">
              Create, manage, and monitor your hackathons
            </p>
          </div>
          <div className="hidden sm:block">
            <CreateHackathonButton organizationId={session.user.organizationId} />
          </div>
        </div>
      </div>

      <div className="sm:hidden">
        <CreateHackathonButton organizationId={session.user.organizationId} />
      </div>

      {hackathons.length === 0 ? (
        <div className="flex flex-col items-center justify-center py-12 border rounded-lg bg-muted/20">
          <Trophy className="h-12 w-12 text-muted-foreground mb-4" />
          <h2 className="text-xl font-semibold mb-2">No Hackathons Yet</h2>
          <p className="text-muted-foreground text-center max-w-md mb-4">
            Get started by creating your first hackathon. You can set dates,
            team sizes, and judging criteria.
          </p>
          <CreateHackathonButton organizationId={session.user.organizationId} />
        </div>
      ) : (
        <HackathonList initialHackathons={hackathons} />
      )}
    </div>
  );
}
