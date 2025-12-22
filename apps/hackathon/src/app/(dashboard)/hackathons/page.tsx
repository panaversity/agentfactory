import Link from "next/link";
import { getPublishedHackathons } from "@/db/queries/hackathons";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Calendar, Users, ArrowRight } from "lucide-react";

// Revalidate cached data every 60 seconds for fresh hackathon listings
export const revalidate = 60;

export const metadata = {
  title: "Browse Hackathons",
  description: "Discover and join hackathons",
};

export default async function BrowseHackathonsPage() {
  const hackathons = await getPublishedHackathons();

  const getStatusColor = (status: string) => {
    switch (status) {
      case "open":
        return "bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300";
      case "active":
        return "bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-300";
      case "judging":
        return "bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-300";
      case "completed":
        return "bg-gray-100 text-gray-800 dark:bg-gray-900 dark:text-gray-300";
      default:
        return "bg-gray-100 text-gray-800";
    }
  };

  const formatDate = (date: Date) => {
    return new Date(date).toLocaleDateString("en-US", {
      month: "short",
      day: "numeric",
      year: "numeric",
    });
  };

  return (
    <div className="space-y-8">
      {/* Hero Section */}
      <div className="relative overflow-hidden rounded-2xl bg-gradient-hero p-8 md:p-12 animate-fade-in-up">
        <div className="absolute inset-0 bg-gradient-to-r from-primary/5 to-transparent" />
        <div className="absolute top-0 right-0 w-64 h-64 bg-primary/10 rounded-full blur-3xl -translate-y-1/2 translate-x-1/2" />
        <div className="relative">
          <h1 className="text-4xl md:text-5xl font-bold tracking-tight mb-3">
            Discover <span className="text-gradient-brand">Hackathons</span>
          </h1>
          <p className="text-lg text-muted-foreground max-w-2xl">
            Join exciting challenges, build innovative solutions, and compete with teams worldwide.
          </p>
        </div>
      </div>

      {hackathons.length === 0 ? (
        <Card>
          <CardContent className="flex flex-col items-center justify-center py-16">
            <div className="rounded-full bg-muted p-4 mb-4">
              <Calendar className="h-8 w-8 text-muted-foreground" />
            </div>
            <h3 className="text-lg font-semibold mb-2">No hackathons available</h3>
            <p className="text-muted-foreground text-center max-w-md">
              There are no published hackathons at the moment. Check back later or
              create your own hackathon from the dashboard!
            </p>
            <Button asChild className="mt-4">
              <Link href="/dashboard">Go to Dashboard</Link>
            </Button>
          </CardContent>
        </Card>
      ) : (
        <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
          {hackathons.map((hackathon, index) => (
            <Card
              key={hackathon.id}
              className={`card-hover animate-fade-in-up opacity-0 ${
                hackathon.status === "open" ? "ring-2 ring-primary/20" : ""
              }`}
              style={{ animationDelay: `${index * 100}ms`, animationFillMode: 'forwards' }}
            >
              <CardHeader>
                <div className="flex items-start justify-between">
                  <CardTitle className="text-xl line-clamp-2">
                    {hackathon.title}
                  </CardTitle>
                  <Badge className={`${getStatusColor(hackathon.status)} ${
                    hackathon.status === "open" ? "animate-pulse" : ""
                  }`}>
                    {hackathon.status.charAt(0).toUpperCase() +
                      hackathon.status.slice(1)}
                  </Badge>
                </div>
              </CardHeader>
              <CardContent className="space-y-4">
                <p className="text-muted-foreground line-clamp-3">
                  {hackathon.description}
                </p>

                <div className="space-y-2 text-sm">
                  <div className="flex items-center gap-2 text-muted-foreground">
                    <Calendar className="h-4 w-4 text-primary" />
                    <span>
                      {formatDate(hackathon.startDate)} -{" "}
                      {formatDate(hackathon.endDate)}
                    </span>
                  </div>
                  <div className="flex items-center gap-2 text-muted-foreground">
                    <Users className="h-4 w-4 text-primary" />
                    <span>
                      {hackathon.minTeamSize}-{hackathon.maxTeamSize} members per
                      team
                    </span>
                  </div>
                </div>

                <div className="pt-2">
                  <Button asChild className="w-full group">
                    <Link href={`/hackathons/${hackathon.id}`}>
                      View Details
                      <ArrowRight className="ml-2 h-4 w-4 transition-transform group-hover:translate-x-1" />
                    </Link>
                  </Button>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      )}
    </div>
  );
}
