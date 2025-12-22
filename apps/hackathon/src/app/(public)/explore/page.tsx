import Link from "next/link";
import { getPublishedHackathons } from "@/db/queries/hackathons";
import { getTeamsWithMemberCount } from "@/db/queries/teams";
import { getHackathonRoleStats } from "@/db/queries/roles";
import { Button } from "@/components/ui/button";
import {
  Calendar,
  Users,
  ArrowRight,
  Trophy,
  Clock,
  Sparkles,
} from "lucide-react";

export const revalidate = 60;

export const metadata = {
  title: "Explore Hackathons | Panaversity",
  description: "Discover and join hackathons on Panaversity",
};

export default async function PublicBrowseHackathonsPage() {
  const hackathons = await getPublishedHackathons();

  // Fetch stats for all hackathons in parallel
  const hackathonsWithStats = await Promise.all(
    hackathons.map(async (hackathon) => {
      const [teams, roleStats] = await Promise.all([
        getTeamsWithMemberCount(hackathon.id),
        getHackathonRoleStats(hackathon.id),
      ]);
      return {
        ...hackathon,
        teamCount: teams.length,
        participantCount: roleStats.participants,
      };
    })
  );

  const formatDate = (date: Date) => {
    return new Date(date).toLocaleDateString("en-US", {
      month: "short",
      day: "numeric",
    });
  };

  const getStatusConfig = (status: string) => {
    switch (status) {
      case "open":
        return {
          label: "Registration Open",
          color: "text-[#1cd98e]",
          bg: "bg-[#1cd98e]/10",
          glow: true,
        };
      case "active":
        return {
          label: "In Progress",
          color: "text-blue-400",
          bg: "bg-blue-400/10",
          glow: false,
        };
      case "judging":
        return {
          label: "Judging",
          color: "text-amber-400",
          bg: "bg-amber-400/10",
          glow: false,
        };
      case "completed":
        return {
          label: "Completed",
          color: "text-gray-400",
          bg: "bg-gray-400/10",
          glow: false,
        };
      default:
        return {
          label: "Coming Soon",
          color: "text-gray-400",
          bg: "bg-gray-400/10",
          glow: false,
        };
    }
  };

  const isRegistrationOpen = (hackathon: (typeof hackathonsWithStats)[0]) =>
    hackathon.status === "open" &&
    new Date() < new Date(hackathon.registrationDeadline);

  return (
    <div className="relative overflow-hidden min-h-screen">
      {/* Background Effects */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute top-0 left-1/4 w-[800px] h-[800px] bg-[#1cd98e]/5 rounded-full blur-[120px] -translate-y-1/2" />
        <div className="absolute top-1/2 right-0 w-[600px] h-[600px] bg-blue-500/5 rounded-full blur-[100px] translate-x-1/2" />
      </div>

      {/* Hero Section */}
      <section className="relative pt-16 pb-12 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          <div className="text-center max-w-3xl mx-auto">
            <div className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-[#1cd98e]/10 text-[#1cd98e] text-sm font-medium mb-6">
              <Sparkles className="h-4 w-4" />
              {hackathonsWithStats.filter((h) => h.status === "open").length} hackathons accepting registrations
            </div>
            <h1 className="text-5xl md:text-6xl font-bold text-white tracking-tight mb-6">
              Build the{" "}
              <span className="bg-gradient-to-r from-[#1cd98e] to-blue-400 bg-clip-text text-transparent">
                Future
              </span>
            </h1>
            <p className="text-xl text-white/60 mb-8">
              Join hackathons, form teams, and compete for amazing prizes.
              Whether you&apos;re a beginner or a pro, there&apos;s a challenge for you.
            </p>
            <Button
              size="lg"
              className="bg-[#1cd98e] hover:bg-[#19c580] text-black font-semibold px-8 h-12 rounded-xl shadow-lg shadow-[#1cd98e]/20"
              asChild
            >
              <Link href="/login">
                Get Started
                <ArrowRight className="ml-2 h-5 w-5" />
              </Link>
            </Button>
          </div>
        </div>
      </section>

      {/* Hackathons Grid */}
      <section className="relative py-12 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          {hackathonsWithStats.length === 0 ? (
            <div className="text-center py-20">
              <div className="inline-flex items-center justify-center w-16 h-16 rounded-full bg-white/5 mb-6">
                <Trophy className="h-8 w-8 text-white/40" />
              </div>
              <h3 className="text-xl font-semibold text-white mb-2">
                No hackathons available
              </h3>
              <p className="text-white/50 max-w-md mx-auto">
                There are no published hackathons at the moment. Check back soon
                for new opportunities!
              </p>
            </div>
          ) : (
            <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
              {hackathonsWithStats.map((hackathon, index) => {
                const status = getStatusConfig(hackathon.status);
                const regOpen = isRegistrationOpen(hackathon);

                return (
                  <Link
                    key={hackathon.id}
                    href={`/h/${hackathon.id}`}
                    className="group block"
                    style={{
                      animationDelay: `${index * 100}ms`,
                    }}
                  >
                    <div
                      className={`relative h-full p-6 rounded-2xl bg-white/[0.02] border border-white/5 hover:border-white/10 hover:bg-white/[0.04] transition-all duration-300 ${
                        status.glow ? "ring-1 ring-[#1cd98e]/20" : ""
                      }`}
                    >
                      {/* Status Badge */}
                      <div className="flex items-center justify-between mb-4">
                        <span
                          className={`inline-flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium ${status.bg} ${status.color}`}
                        >
                          {status.glow && (
                            <span className="relative flex h-2 w-2">
                              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-[#1cd98e] opacity-75"></span>
                              <span className="relative inline-flex rounded-full h-2 w-2 bg-[#1cd98e]"></span>
                            </span>
                          )}
                          {status.label}
                        </span>
                        <ArrowRight className="h-5 w-5 text-white/20 group-hover:text-white/50 group-hover:translate-x-1 transition-all" />
                      </div>

                      {/* Title */}
                      <h3 className="text-xl font-semibold text-white mb-2 line-clamp-2 group-hover:text-[#1cd98e] transition-colors">
                        {hackathon.title}
                      </h3>

                      {/* Description */}
                      <p className="text-white/50 text-sm line-clamp-2 mb-6">
                        {hackathon.description}
                      </p>

                      {/* Stats */}
                      <div className="grid grid-cols-2 gap-4 mb-6">
                        <div className="flex items-center gap-2 text-white/40 text-sm">
                          <Users className="h-4 w-4" />
                          <span>{hackathon.participantCount} participants</span>
                        </div>
                        <div className="flex items-center gap-2 text-white/40 text-sm">
                          <Trophy className="h-4 w-4" />
                          <span>{hackathon.teamCount} teams</span>
                        </div>
                      </div>

                      {/* Dates */}
                      <div className="flex items-center justify-between pt-4 border-t border-white/5">
                        <div className="flex items-center gap-2 text-white/40 text-sm">
                          <Calendar className="h-4 w-4" />
                          <span>
                            {formatDate(hackathon.startDate)} -{" "}
                            {formatDate(hackathon.endDate)}
                          </span>
                        </div>
                        {regOpen && (
                          <div className="flex items-center gap-1 text-[#1cd98e] text-xs">
                            <Clock className="h-3 w-3" />
                            <span>Ends {formatDate(hackathon.registrationDeadline)}</span>
                          </div>
                        )}
                      </div>
                    </div>
                  </Link>
                );
              })}
            </div>
          )}
        </div>
      </section>

      {/* Footer CTA */}
      <section className="relative py-20 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-3xl text-center">
          <h2 className="text-3xl font-bold text-white mb-4">
            Ready to start building?
          </h2>
          <p className="text-white/60 mb-8">
            Sign in to register for hackathons, form teams, and submit your projects.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Button
              size="lg"
              className="bg-[#1cd98e] hover:bg-[#19c580] text-black font-semibold px-8 h-12 rounded-xl"
              asChild
            >
              <Link href="/login">Sign In to Continue</Link>
            </Button>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-white/5 py-8 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl flex flex-col sm:flex-row items-center justify-between gap-4">
          <div className="flex items-center gap-2">
            <div className="h-6 w-6 rounded bg-gradient-to-br from-[#1cd98e] to-[#0ea66a] flex items-center justify-center">
              <span className="text-white font-bold text-xs">P</span>
            </div>
            <span className="text-white/40 text-sm">Panaversity Hackathons</span>
          </div>
          <p className="text-white/30 text-sm">
            &copy; {new Date().getFullYear()} Panaversity. All rights reserved.
          </p>
        </div>
      </footer>
    </div>
  );
}
