import Link from "next/link";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { HackathonStatusBadge } from "./hackathon-status-badge";
import { Calendar, Users, Eye, EyeOff, Settings } from "lucide-react";

type HackathonStatus = "draft" | "open" | "active" | "judging" | "completed";

interface Hackathon {
  id: string;
  title: string;
  description: string;
  slug: string;
  startDate: string | Date;
  endDate: string | Date;
  status: string;
  published: boolean;
  minTeamSize: number;
  maxTeamSize: number;
}

interface HackathonCardProps {
  hackathon: Hackathon;
  showManageButton?: boolean;
  onPublishToggle?: (id: string) => void;
}

export function HackathonCard({
  hackathon,
  showManageButton = false,
  onPublishToggle,
}: HackathonCardProps) {
  const startDate = new Date(hackathon.startDate);
  const endDate = new Date(hackathon.endDate);

  const formatDate = (date: Date) => {
    return date.toLocaleDateString("en-US", {
      month: "short",
      day: "numeric",
      year: "numeric",
    });
  };

  return (
    <Card className="hover:shadow-md transition-shadow">
      <CardHeader className="flex flex-row items-start justify-between space-y-0 pb-2">
        <div className="space-y-1">
          <CardTitle className="text-lg font-semibold">
            <Link
              href={`/hackathons/${hackathon.id}`}
              className="hover:underline"
            >
              {hackathon.title}
            </Link>
          </CardTitle>
          <div className="flex items-center gap-2">
            <HackathonStatusBadge status={hackathon.status as HackathonStatus} />
            {hackathon.published ? (
              <span className="inline-flex items-center gap-1 text-xs text-green-600">
                <Eye className="h-3 w-3" />
                Published
              </span>
            ) : (
              <span className="inline-flex items-center gap-1 text-xs text-muted-foreground">
                <EyeOff className="h-3 w-3" />
                Hidden
              </span>
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <p className="text-sm text-muted-foreground line-clamp-2 mb-4">
          {hackathon.description}
        </p>

        <div className="flex flex-wrap gap-4 text-sm text-muted-foreground mb-4">
          <div className="flex items-center gap-1">
            <Calendar className="h-4 w-4" />
            <span>
              {formatDate(startDate)} - {formatDate(endDate)}
            </span>
          </div>
          <div className="flex items-center gap-1">
            <Users className="h-4 w-4" />
            <span>
              {hackathon.minTeamSize}-{hackathon.maxTeamSize} members
            </span>
          </div>
        </div>

        {showManageButton && (
          <div className="flex gap-2">
            <Button asChild variant="outline" size="sm">
              <Link href={`/hackathons/${hackathon.id}/manage`}>
                <Settings className="mr-2 h-4 w-4" />
                Manage
              </Link>
            </Button>
            {onPublishToggle && (
              <Button
                variant="ghost"
                size="sm"
                onClick={() => onPublishToggle(hackathon.id)}
              >
                {hackathon.published ? (
                  <>
                    <EyeOff className="mr-2 h-4 w-4" />
                    Unpublish
                  </>
                ) : (
                  <>
                    <Eye className="mr-2 h-4 w-4" />
                    Publish
                  </>
                )}
              </Button>
            )}
          </div>
        )}
      </CardContent>
    </Card>
  );
}
