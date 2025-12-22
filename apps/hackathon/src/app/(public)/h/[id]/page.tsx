import { notFound } from "next/navigation";
import Link from "next/link";
import { getHackathonById } from "@/db/queries/hackathons";
import { getTeamsWithMemberCount } from "@/db/queries/teams";
import { getHackathonRoleStats } from "@/db/queries/roles";
import { Button } from "@/components/ui/button";
import {
  Calendar,
  Users,
  Clock,
  Trophy,
  Target,
  Zap,
  ArrowRight,
  Timer,
  Award,
  Sparkles,
  Building2,
  ExternalLink,
  Heart,
  Layers,
} from "lucide-react";
import { CountdownTimer } from "./countdown-timer";

export const revalidate = 60;

type Prize = {
  place: string;
  title: string;
  value: string;
};

type Organizer = {
  name: string;
  logo?: string;
  url?: string;
};

type Sponsor = {
  name: string;
  logo?: string;
  url?: string;
  tier?: "platinum" | "gold" | "silver" | "bronze";
};

type Category = {
  id: string;
  name: string;
  description?: string;
};

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
    title: `${hackathon.title} | Panaversity Hackathons`,
    description: hackathon.description,
    openGraph: {
      title: hackathon.title,
      description: hackathon.description,
      type: "website",
    },
  };
}

export default async function PublicHackathonPage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id } = await params;

  const [hackathon, teams, roleStats] = await Promise.all([
    getHackathonById(id),
    getTeamsWithMemberCount(id),
    getHackathonRoleStats(id),
  ]);

  if (!hackathon || !hackathon.published) {
    notFound();
  }

  const prizes: Prize[] = hackathon.prizes
    ? JSON.parse(hackathon.prizes)
    : [
        { place: "1st", title: "Grand Prize", value: "$5,000" },
        { place: "2nd", title: "Runner Up", value: "$2,500" },
        { place: "3rd", title: "Third Place", value: "$1,000" },
      ];

  const organizers: Organizer[] = hackathon.organizers
    ? JSON.parse(hackathon.organizers)
    : [];

  const sponsors: Sponsor[] = hackathon.sponsors
    ? JSON.parse(hackathon.sponsors)
    : [];

  const categories: Category[] = hackathon.categories
    ? JSON.parse(hackathon.categories)
    : [];

  const formatDate = (date: Date) => {
    return new Date(date).toLocaleDateString("en-US", {
      weekday: "short",
      month: "short",
      day: "numeric",
      year: "numeric",
    });
  };

  const formatTime = (date: Date) => {
    return new Date(date).toLocaleTimeString("en-US", {
      hour: "2-digit",
      minute: "2-digit",
      timeZoneName: "short",
    });
  };

  const getStatusLabel = (status: string) => {
    switch (status) {
      case "open":
        return { label: "Registration Open", color: "text-[#1cd98e]", bg: "bg-[#1cd98e]/10" };
      case "active":
        return { label: "In Progress", color: "text-blue-400", bg: "bg-blue-400/10" };
      case "judging":
        return { label: "Judging Phase", color: "text-amber-400", bg: "bg-amber-400/10" };
      case "completed":
        return { label: "Completed", color: "text-gray-400", bg: "bg-gray-400/10" };
      default:
        return { label: "Coming Soon", color: "text-gray-400", bg: "bg-gray-400/10" };
    }
  };

  const status = getStatusLabel(hackathon.status);
  const isRegistrationOpen =
    hackathon.status === "open" &&
    new Date() < new Date(hackathon.registrationDeadline);

  const timelineEvents = [
    {
      label: "Registration Opens",
      date: hackathon.createdAt,
      icon: Zap,
      completed: true,
    },
    {
      label: "Registration Deadline",
      date: hackathon.registrationDeadline,
      icon: Clock,
      completed: new Date() > new Date(hackathon.registrationDeadline),
    },
    {
      label: "Hackathon Starts",
      date: hackathon.startDate,
      icon: Target,
      completed: new Date() > new Date(hackathon.startDate),
    },
    {
      label: "Submission Deadline",
      date: hackathon.submissionDeadline,
      icon: Timer,
      completed: new Date() > new Date(hackathon.submissionDeadline),
    },
    {
      label: "Hackathon Ends",
      date: hackathon.endDate,
      icon: Award,
      completed: new Date() > new Date(hackathon.endDate),
    },
  ];

  return (
    <div className="relative overflow-hidden">
      {/* Background Effects */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute top-0 left-1/4 w-[800px] h-[800px] bg-[#1cd98e]/5 rounded-full blur-[120px] -translate-y-1/2" />
        <div className="absolute top-1/3 right-0 w-[600px] h-[600px] bg-blue-500/5 rounded-full blur-[100px] translate-x-1/2" />
        <div className="absolute bottom-0 left-0 w-[500px] h-[500px] bg-purple-500/5 rounded-full blur-[80px] translate-y-1/2 -translate-x-1/2" />
      </div>

      {/* Hero Section */}
      <section className="relative pt-12 pb-20 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          {/* Status Badge */}
          <div className="flex items-center gap-3 mb-6">
            <span
              className={`inline-flex items-center gap-2 px-4 py-1.5 rounded-full text-sm font-medium ${status.bg} ${status.color}`}
            >
              {hackathon.status === "open" && (
                <span className="relative flex h-2 w-2">
                  <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-[#1cd98e] opacity-75"></span>
                  <span className="relative inline-flex rounded-full h-2 w-2 bg-[#1cd98e]"></span>
                </span>
              )}
              {status.label}
            </span>
          </div>

          {/* Title */}
          <h1 className="text-5xl md:text-7xl font-bold text-white tracking-tight mb-6 max-w-4xl">
            {hackathon.title}
          </h1>

          {/* Description */}
          <p className="text-xl text-white/60 max-w-3xl mb-10 leading-relaxed">
            {hackathon.description}
          </p>

          {/* Quick Stats Row */}
          <div className="flex flex-wrap items-center gap-6 text-white/50 mb-12">
            <div className="flex items-center gap-2">
              <Calendar className="h-5 w-5 text-[#1cd98e]" />
              <span>
                {formatDate(hackathon.startDate)} - {formatDate(hackathon.endDate)}
              </span>
            </div>
            <div className="flex items-center gap-2">
              <Users className="h-5 w-5 text-[#1cd98e]" />
              <span>
                {hackathon.minTeamSize}-{hackathon.maxTeamSize} members per team
              </span>
            </div>
          </div>

          {/* CTA Section */}
          {isRegistrationOpen && (
            <div className="flex flex-col sm:flex-row gap-4 mb-16">
              <Button
                size="lg"
                className="bg-[#1cd98e] hover:bg-[#19c580] text-black font-semibold text-lg px-8 h-14 rounded-xl shadow-lg shadow-[#1cd98e]/20 hover:shadow-[#1cd98e]/30 transition-all group"
                asChild
              >
                <Link href={`/login?returnUrl=${encodeURIComponent(`/hackathons/${id}/participate`)}`}>
                  Register Now
                  <ArrowRight className="ml-2 h-5 w-5 transition-transform group-hover:translate-x-1" />
                </Link>
              </Button>
              <Button
                size="lg"
                variant="outline"
                className="border-white/20 text-white hover:bg-white/5 h-14 px-8 rounded-xl"
                asChild
              >
                <Link href="/explore">View All Hackathons</Link>
              </Button>
            </div>
          )}

          {/* Countdown Timer */}
          {isRegistrationOpen && (
            <div className="mb-16">
              <p className="text-white/40 text-sm uppercase tracking-widest mb-4">
                Registration closes in
              </p>
              <CountdownTimer targetDate={new Date(hackathon.registrationDeadline).toISOString()} />
            </div>
          )}
        </div>
      </section>

      {/* Stats Cards */}
      <section className="relative py-16 px-4 sm:px-6 lg:px-8 border-y border-white/5 bg-white/[0.01]">
        <div className="mx-auto max-w-7xl">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-6">
            <StatCard
              icon={Users}
              value={roleStats.participants.toString()}
              label="Participants"
              highlight
            />
            <StatCard
              icon={Trophy}
              value={teams.length.toString()}
              label="Teams Registered"
            />
            <StatCard
              icon={Award}
              value={(roleStats.judges + roleStats.mentors).toString()}
              label="Judges & Mentors"
            />
            <StatCard
              icon={Sparkles}
              value={prizes.length > 0 ? prizes[0].value : "TBA"}
              label="Grand Prize"
              highlight
            />
          </div>
        </div>
      </section>

      {/* Categories Section */}
      {categories.length > 0 && (
        <section className="relative py-20 px-4 sm:px-6 lg:px-8">
          <div className="mx-auto max-w-7xl">
            <div className="text-center mb-12">
              <div className="inline-flex items-center gap-2 text-[#1cd98e] mb-3">
                <Layers className="h-5 w-5" />
                <span className="text-sm font-medium uppercase tracking-widest">Competition Tracks</span>
              </div>
              <h2 className="text-3xl md:text-4xl font-bold text-white mb-4">Categories</h2>
              <p className="text-white/50 max-w-2xl mx-auto">
                Choose your track and compete with teams in your category
              </p>
            </div>

            <div className="grid sm:grid-cols-2 lg:grid-cols-3 gap-6">
              {categories.map((category) => (
                <div
                  key={category.id}
                  className="group relative p-6 rounded-2xl bg-gradient-to-br from-[#1cd98e]/10 to-transparent border border-[#1cd98e]/20 hover:border-[#1cd98e]/40 transition-all"
                >
                  <div className="flex items-start gap-4">
                    <div className="p-3 rounded-xl bg-[#1cd98e]/20">
                      <Layers className="h-6 w-6 text-[#1cd98e]" />
                    </div>
                    <div>
                      <h3 className="font-semibold text-white text-lg mb-2">{category.name}</h3>
                      {category.description && (
                        <p className="text-sm text-white/50">{category.description}</p>
                      )}
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </section>
      )}

      {/* Prizes Section */}
      <section className="relative py-20 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          <div className="text-center mb-12">
            <h2 className="text-3xl md:text-4xl font-bold text-white mb-4">Prizes & Rewards</h2>
            <p className="text-white/50 max-w-2xl mx-auto">
              Compete for amazing prizes and recognition from industry leaders
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6">
            {prizes.map((prize, index) => (
              <PrizeCard key={index} prize={prize} index={index} />
            ))}
          </div>
        </div>
      </section>

      {/* Timeline Section */}
      <section className="relative py-20 px-4 sm:px-6 lg:px-8 border-t border-white/5">
        <div className="mx-auto max-w-4xl">
          <div className="text-center mb-12">
            <h2 className="text-3xl md:text-4xl font-bold text-white mb-4">Event Timeline</h2>
            <p className="text-white/50">Important dates and milestones</p>
          </div>

          <div className="relative">
            {/* Timeline Line */}
            <div className="absolute left-8 top-0 bottom-0 w-px bg-gradient-to-b from-[#1cd98e]/50 via-white/20 to-transparent" />

            <div className="space-y-8">
              {timelineEvents.map((event, index) => (
                <TimelineItem key={index} event={event} formatDate={formatDate} formatTime={formatTime} />
              ))}
            </div>
          </div>
        </div>
      </section>

      {/* Teams Preview */}
      {teams.length > 0 && (
        <section className="relative py-20 px-4 sm:px-6 lg:px-8 border-t border-white/5">
          <div className="mx-auto max-w-7xl">
            <div className="flex items-center justify-between mb-8">
              <div>
                <h2 className="text-3xl font-bold text-white mb-2">Registered Teams</h2>
                <p className="text-white/50">{teams.length} teams ready to compete</p>
              </div>
            </div>

            <div className="grid sm:grid-cols-2 lg:grid-cols-3 gap-4">
              {teams.slice(0, 6).map((team) => (
                <div
                  key={team.id}
                  className="p-5 rounded-2xl bg-white/[0.02] border border-white/5 hover:border-white/10 hover:bg-white/[0.04] transition-all"
                >
                  <div className="flex items-start justify-between mb-3">
                    <h3 className="font-semibold text-white">{team.name}</h3>
                    <span
                      className={`text-xs px-2 py-1 rounded-full ${
                        team.isReady
                          ? "bg-[#1cd98e]/10 text-[#1cd98e]"
                          : "bg-amber-400/10 text-amber-400"
                      }`}
                    >
                      {team.memberCount}/{team.maxSize}
                    </span>
                  </div>
                  {team.description && (
                    <p className="text-sm text-white/40 line-clamp-2">{team.description}</p>
                  )}
                </div>
              ))}
            </div>

            {teams.length > 6 && (
              <p className="text-center text-white/40 mt-6">
                + {teams.length - 6} more teams
              </p>
            )}
          </div>
        </section>
      )}

      {/* Organizers & Sponsors Section */}
      {(organizers.length > 0 || sponsors.length > 0) && (
        <section className="relative py-20 px-4 sm:px-6 lg:px-8 border-t border-white/5">
          <div className="mx-auto max-w-7xl">
            {/* Organizers */}
            {organizers.length > 0 && (
              <div className="mb-16">
                <div className="text-center mb-10">
                  <div className="inline-flex items-center gap-2 text-[#1cd98e] mb-3">
                    <Building2 className="h-5 w-5" />
                    <span className="text-sm font-medium uppercase tracking-widest">Organized By</span>
                  </div>
                  <h2 className="text-3xl md:text-4xl font-bold text-white">Event Organizers</h2>
                </div>
                <div className="flex flex-wrap justify-center gap-8">
                  {organizers.map((org, index) => (
                    <OrganizerCard key={index} organizer={org} />
                  ))}
                </div>
              </div>
            )}

            {/* Sponsors */}
            {sponsors.length > 0 && (
              <div>
                <div className="text-center mb-10">
                  <div className="inline-flex items-center gap-2 text-[#1cd98e] mb-3">
                    <Heart className="h-5 w-5" />
                    <span className="text-sm font-medium uppercase tracking-widest">Sponsored By</span>
                  </div>
                  <h2 className="text-3xl md:text-4xl font-bold text-white">Our Sponsors</h2>
                </div>

                {/* Group sponsors by tier */}
                {["platinum", "gold", "silver", "bronze"].map((tier) => {
                  const tierSponsors = sponsors.filter((s) => s.tier === tier);
                  if (tierSponsors.length === 0) return null;
                  return (
                    <div key={tier} className="mb-10">
                      <h3 className={`text-center text-sm font-medium uppercase tracking-widest mb-6 ${
                        tier === "platinum" ? "text-purple-400" :
                        tier === "gold" ? "text-amber-400" :
                        tier === "silver" ? "text-gray-300" : "text-orange-400"
                      }`}>
                        {tier} Sponsors
                      </h3>
                      <div className="flex flex-wrap justify-center gap-6">
                        {tierSponsors.map((sponsor, index) => (
                          <SponsorCard key={index} sponsor={sponsor} />
                        ))}
                      </div>
                    </div>
                  );
                })}

                {/* Sponsors without tier */}
                {sponsors.filter((s) => !s.tier).length > 0 && (
                  <div className="flex flex-wrap justify-center gap-6">
                    {sponsors.filter((s) => !s.tier).map((sponsor, index) => (
                      <SponsorCard key={index} sponsor={sponsor} />
                    ))}
                  </div>
                )}
              </div>
            )}
          </div>
        </section>
      )}

      {/* Final CTA */}
      <section className="relative py-24 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-3xl text-center">
          <div className="relative">
            <div className="absolute inset-0 bg-gradient-to-r from-[#1cd98e]/20 via-blue-500/20 to-purple-500/20 rounded-3xl blur-3xl" />
            <div className="relative p-12 rounded-3xl bg-white/[0.02] border border-white/10">
              <h2 className="text-3xl md:text-4xl font-bold text-white mb-4">
                Ready to Build Something Amazing?
              </h2>
              <p className="text-white/60 mb-8 text-lg">
                Join {roleStats.participants}+ participants and compete for{" "}
                {prizes.length > 0 ? prizes[0].value : "amazing prizes"}
              </p>
              {isRegistrationOpen ? (
                <Button
                  size="lg"
                  className="bg-[#1cd98e] hover:bg-[#19c580] text-black font-semibold text-lg px-10 h-14 rounded-xl shadow-lg shadow-[#1cd98e]/20"
                  asChild
                >
                  <Link href={`/login?returnUrl=${encodeURIComponent(`/hackathons/${id}/participate`)}`}>
                    Get Started
                    <ArrowRight className="ml-2 h-5 w-5" />
                  </Link>
                </Button>
              ) : (
                <p className="text-white/40">
                  Registration is currently closed. Check back soon!
                </p>
              )}
            </div>
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

function StatCard({
  icon: Icon,
  value,
  label,
  highlight,
}: {
  icon: React.ComponentType<{ className?: string }>;
  value: string;
  label: string;
  highlight?: boolean;
}) {
  return (
    <div
      className={`p-6 rounded-2xl border transition-all ${
        highlight
          ? "bg-gradient-to-br from-[#1cd98e]/10 to-transparent border-[#1cd98e]/20"
          : "bg-white/[0.02] border-white/5 hover:border-white/10"
      }`}
    >
      <Icon className={`h-6 w-6 mb-4 ${highlight ? "text-[#1cd98e]" : "text-white/40"}`} />
      <p className="text-3xl md:text-4xl font-bold text-white mb-1">{value}</p>
      <p className="text-sm text-white/50">{label}</p>
    </div>
  );
}

function PrizeCard({ prize, index }: { prize: Prize; index: number }) {
  const gradients = [
    "from-amber-400/20 via-yellow-500/10 to-transparent border-amber-400/30",
    "from-gray-300/20 via-gray-400/10 to-transparent border-gray-300/30",
    "from-orange-400/20 via-orange-500/10 to-transparent border-orange-400/30",
  ];

  const icons = [
    "text-amber-400",
    "text-gray-300",
    "text-orange-400",
  ];

  return (
    <div
      className={`relative p-8 rounded-2xl bg-gradient-to-br ${gradients[index] || gradients[2]} border backdrop-blur-sm`}
    >
      <Trophy className={`h-10 w-10 mb-4 ${icons[index] || icons[2]}`} />
      <p className="text-sm text-white/50 uppercase tracking-widest mb-1">
        {prize.place} Place
      </p>
      <h3 className="text-2xl font-bold text-white mb-2">{prize.title}</h3>
      <p className="text-3xl font-bold text-white">{prize.value}</p>
    </div>
  );
}

function TimelineItem({
  event,
  formatDate,
  formatTime,
}: {
  event: {
    label: string;
    date: Date;
    icon: React.ComponentType<{ className?: string }>;
    completed: boolean;
  };
  formatDate: (date: Date) => string;
  formatTime: (date: Date) => string;
}) {
  const Icon = event.icon;

  return (
    <div className="relative flex gap-6 pl-4">
      <div
        className={`relative z-10 flex h-8 w-8 shrink-0 items-center justify-center rounded-full ${
          event.completed
            ? "bg-[#1cd98e]/20 text-[#1cd98e]"
            : "bg-white/10 text-white/50"
        }`}
      >
        <Icon className="h-4 w-4" />
      </div>
      <div className="flex-1 pb-8">
        <h3 className={`font-medium ${event.completed ? "text-white/60" : "text-white"}`}>
          {event.label}
        </h3>
        <p className="text-sm text-white/40">
          {formatDate(event.date)} at {formatTime(event.date)}
        </p>
      </div>
    </div>
  );
}

function OrganizerCard({ organizer }: { organizer: Organizer }) {
  const content = (
    <div className="group relative p-6 rounded-2xl bg-white/[0.03] border border-white/10 hover:border-[#1cd98e]/30 hover:bg-white/[0.05] transition-all min-w-[200px] text-center">
      {organizer.logo ? (
        <div className="w-20 h-20 mx-auto mb-4 rounded-xl bg-white/10 overflow-hidden flex items-center justify-center">
          {/* eslint-disable-next-line @next/next/no-img-element */}
          <img
            src={organizer.logo}
            alt={organizer.name}
            className="w-full h-full object-contain"
          />
        </div>
      ) : (
        <div className="w-20 h-20 mx-auto mb-4 rounded-xl bg-gradient-to-br from-[#1cd98e]/20 to-blue-500/20 flex items-center justify-center">
          <Building2 className="h-8 w-8 text-[#1cd98e]" />
        </div>
      )}
      <h3 className="font-semibold text-white text-lg mb-1">{organizer.name}</h3>
      {organizer.url && (
        <span className="inline-flex items-center gap-1 text-xs text-[#1cd98e] opacity-0 group-hover:opacity-100 transition-opacity">
          Visit website <ExternalLink className="h-3 w-3" />
        </span>
      )}
    </div>
  );

  if (organizer.url) {
    return (
      <a href={organizer.url} target="_blank" rel="noopener noreferrer">
        {content}
      </a>
    );
  }

  return content;
}

function SponsorCard({ sponsor }: { sponsor: Sponsor }) {
  const tierGradients = {
    platinum: "from-purple-500/20 to-purple-600/10 border-purple-400/30",
    gold: "from-amber-400/20 to-amber-500/10 border-amber-400/30",
    silver: "from-gray-300/20 to-gray-400/10 border-gray-300/30",
    bronze: "from-orange-400/20 to-orange-500/10 border-orange-400/30",
  };

  const tierSize = {
    platinum: "min-w-[220px] p-8",
    gold: "min-w-[200px] p-6",
    silver: "min-w-[180px] p-5",
    bronze: "min-w-[160px] p-4",
  };

  const tier = sponsor.tier || "bronze";
  const gradient = tierGradients[tier] || "from-white/5 to-white/[0.02] border-white/10";
  const size = tierSize[tier] || tierSize.bronze;

  const content = (
    <div className={`group relative rounded-2xl bg-gradient-to-br ${gradient} border hover:scale-105 transition-all text-center ${size}`}>
      {sponsor.logo ? (
        <div className={`mx-auto mb-3 rounded-xl bg-white/10 overflow-hidden flex items-center justify-center ${
          tier === "platinum" ? "w-24 h-24" :
          tier === "gold" ? "w-20 h-20" :
          tier === "silver" ? "w-16 h-16" : "w-14 h-14"
        }`}>
          {/* eslint-disable-next-line @next/next/no-img-element */}
          <img
            src={sponsor.logo}
            alt={sponsor.name}
            className="w-full h-full object-contain"
          />
        </div>
      ) : (
        <div className={`mx-auto mb-3 rounded-xl bg-white/10 flex items-center justify-center ${
          tier === "platinum" ? "w-24 h-24" :
          tier === "gold" ? "w-20 h-20" :
          tier === "silver" ? "w-16 h-16" : "w-14 h-14"
        }`}>
          <Heart className={`${
            tier === "platinum" ? "h-10 w-10" :
            tier === "gold" ? "h-8 w-8" :
            tier === "silver" ? "h-6 w-6" : "h-5 w-5"
          } text-white/50`} />
        </div>
      )}
      <h3 className={`font-semibold text-white ${
        tier === "platinum" ? "text-xl" :
        tier === "gold" ? "text-lg" :
        "text-base"
      }`}>{sponsor.name}</h3>
      {sponsor.url && (
        <span className="inline-flex items-center gap-1 text-xs text-white/40 opacity-0 group-hover:opacity-100 transition-opacity mt-1">
          Visit <ExternalLink className="h-3 w-3" />
        </span>
      )}
    </div>
  );

  if (sponsor.url) {
    return (
      <a href={sponsor.url} target="_blank" rel="noopener noreferrer">
        {content}
      </a>
    );
  }

  return content;
}
