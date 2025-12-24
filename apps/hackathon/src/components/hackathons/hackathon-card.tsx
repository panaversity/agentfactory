import Link from "next/link";
import { HackathonStatusBadge } from "./hackathon-status-badge";
import { Calendar, Users, Eye, EyeOff, Settings, Activity } from "lucide-react";
import { Button } from "@/components/ui/button";

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
    <div className="group relative overflow-hidden rounded-none border border-brand-tech-blue/20 bg-background/50 hover:border-brand-neon-cyan/50 hover:shadow-[0_0_20px_-10px_rgba(0,240,255,0.3)] transition-all duration-300">
      {/* Tech Corner Markers */}
      <div className="absolute top-0 left-0 w-2 h-2 border-t border-l border-brand-tech-blue opacity-50 group-hover:opacity-100 transition-opacity" />
      <div className="absolute top-0 right-0 w-2 h-2 border-t border-r border-brand-tech-blue opacity-50 group-hover:opacity-100 transition-opacity" />
      <div className="absolute bottom-0 left-0 w-2 h-2 border-b border-l border-brand-tech-blue opacity-50 group-hover:opacity-100 transition-opacity" />
      <div className="absolute bottom-0 right-0 w-2 h-2 border-b border-r border-brand-tech-blue opacity-50 group-hover:opacity-100 transition-opacity" />

      {/* Header / Status Bar */}
      <div className="flex items-center justify-between border-b border-white/5 bg-white/[0.02] px-4 py-3">
        <div className="flex items-center gap-2">
          <HackathonStatusBadge status={hackathon.status as HackathonStatus} />
          <span className="text-[10px] uppercase font-mono text-muted-foreground tracking-wider">
            ID: {hackathon.id.slice(0, 6)}
          </span>
        </div>
        {hackathon.published ? (
          <div className="flex items-center gap-1 text-[10px] uppercase text-brand-emerald font-mono tracking-wider">
            <Activity className="h-3 w-3" />
            <span>Live</span>
          </div>
        ) : (
          <div className="flex items-center gap-1 text-[10px] uppercase text-muted-foreground font-mono tracking-wider">
            <EyeOff className="h-3 w-3" />
            <span>Encrypted</span>
          </div>
        )}
      </div>

      {/* Main Content */}
      <div className="p-5">
        <h3 className="mb-2 text-xl font-bold uppercase tracking-tight text-foreground group-hover:text-brand-neon-cyan transition-colors">
          <Link href={`/hackathons/${hackathon.id}`} className="hover:underline decoration-brand-neon-cyan/50 underline-offset-4">
            {hackathon.title}
          </Link>
        </h3>
        <p className="mb-6 text-sm text-muted-foreground line-clamp-2 font-light">
          {hackathon.description}
        </p>

        {/* Intel Grid */}
        <div className="grid grid-cols-2 gap-4 mb-6">
          <div className="col-span-1 border border-white/5 bg-white/[0.01] p-2 flex items-center gap-3">
            <Calendar className="h-4 w-4 text-brand-tech-blue" />
            <div className="flex flex-col">
              <span className="text-[10px] uppercase text-muted-foreground font-mono">Timeline</span>
              <span className="text-xs font-bold">{formatDate(startDate)} - {formatDate(endDate)}</span>
            </div>
          </div>
          <div className="col-span-1 border border-white/5 bg-white/[0.01] p-2 flex items-center gap-3">
            <Users className="h-4 w-4 text-brand-tech-blue" />
            <div className="flex flex-col">
              <span className="text-[10px] uppercase text-muted-foreground font-mono">Personnel</span>
              <span className="text-xs font-bold">{hackathon.minTeamSize}-{hackathon.maxTeamSize} / Unit</span>
            </div>
          </div>
        </div>

        {/* Actions */}
        {showManageButton && (
          <div className="flex gap-2">
            <Link href={`/hackathons/${hackathon.id}/manage`} className="w-full">
              <Button variant="outline" size="sm" className="w-full border-brand-tech-blue/30 text-xs font-mono uppercase hover:bg-brand-tech-blue/10 hover:border-brand-neon-cyan">
                <Settings className="mr-2 h-3 w-3" />
                Configure Spec
              </Button>
            </Link>
            {onPublishToggle && (
              <Button
                variant="ghost"
                size="sm"
                className="px-2 border border-transparent hover:border-destructive/50 hover:text-destructive hover:bg-destructive/10"
                onClick={() => onPublishToggle(hackathon.id)}
              >
                {hackathon.published ? (
                  <EyeOff className="h-4 w-4" />
                ) : (
                  <Eye className="h-4 w-4" />
                )}
              </Button>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
