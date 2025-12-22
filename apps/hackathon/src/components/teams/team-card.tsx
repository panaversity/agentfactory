"use client";

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Avatar } from "@/components/ui/avatar";
import { Users, Crown, Copy, Check } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useState } from "react";
import { cn } from "@/lib/utils";
import type { TeamWithMemberCount } from "@/types";

interface TeamCardProps {
  team: TeamWithMemberCount;
  isOwner?: boolean;
  showInviteCode?: boolean;
  onManage?: () => void;
}

export function TeamCard({
  team,
  isOwner = false,
  showInviteCode = false,
  onManage,
}: TeamCardProps) {
  const [copied, setCopied] = useState(false);

  const copyInviteCode = async () => {
    await navigator.clipboard.writeText(team.inviteCode);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  const statusColors: Record<string, string> = {
    forming: "bg-amber-500/15 text-amber-700 dark:text-amber-400",
    ready: "bg-emerald-500/15 text-emerald-700 dark:text-emerald-400",
    submitted: "bg-blue-500/15 text-blue-700 dark:text-blue-400",
    disqualified: "bg-red-500/15 text-red-700 dark:text-red-400",
  };

  return (
    <Card
      className={cn(
        "transition-all hover:shadow-md",
        isOwner && "ring-2 ring-primary/20"
      )}
    >
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div className="flex items-center gap-3">
            <Avatar fallback={team.name} size="md" />
            <div>
              <CardTitle className="text-lg flex items-center gap-2">
                {team.name}
                {isOwner && (
                  <Crown className="h-4 w-4 text-amber-500" />
                )}
              </CardTitle>
              <Badge className={cn("mt-1", statusColors[team.status])}>
                {team.status}
              </Badge>
            </div>
          </div>
          {onManage && (
            <Button size="sm" variant="outline" onClick={onManage}>
              Manage
            </Button>
          )}
        </div>
      </CardHeader>
      <CardContent className="space-y-3">
        {team.description && (
          <p className="text-sm text-muted-foreground line-clamp-2">
            {team.description}
          </p>
        )}

        <div className="flex items-center justify-between text-sm">
          <div className="flex items-center gap-2 text-muted-foreground">
            <Users className="h-4 w-4" />
            <span>
              {team.memberCount} / {team.maxSize} members
            </span>
          </div>
          {team.isReady ? (
            <Badge variant="success">Ready</Badge>
          ) : (
            <Badge variant="warning">
              Need {team.minSize - team.memberCount} more
            </Badge>
          )}
        </div>

        {showInviteCode && (
          <div className="flex items-center gap-2 p-3 bg-muted/50 rounded-lg">
            <code className="flex-1 text-sm font-mono tracking-wider">
              {team.inviteCode}
            </code>
            <Button
              size="sm"
              variant="ghost"
              onClick={copyInviteCode}
              className="shrink-0"
            >
              {copied ? (
                <Check className="h-4 w-4 text-emerald-500" />
              ) : (
                <Copy className="h-4 w-4" />
              )}
            </Button>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
