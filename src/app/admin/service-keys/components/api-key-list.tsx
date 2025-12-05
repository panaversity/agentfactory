"use client";

import { useState } from "react";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import { MoreHorizontal, Ban, Trash2, Shield, ChevronDown, ChevronUp } from "lucide-react";
import { permissionsToScopes, SCOPES } from "@/lib/scopes";

interface ApiKey {
  id: string;
  name: string | null;
  start: string | null;
  prefix: string | null;
  userId: string;
  enabled: boolean;
  expiresAt: Date | null;
  createdAt: Date;
  lastRequest: Date | null;
  metadata: Record<string, unknown> | null;
  permissions: Record<string, string[]> | null;
}

interface ApiKeyListProps {
  apiKeys: ApiKey[];
  isLoading: boolean;
  onRevoke: (key: ApiKey) => void;
  onDelete: (key: ApiKey) => void;
}

function getKeyStatus(key: ApiKey): {
  label: string;
  variant: "default" | "secondary" | "destructive" | "outline";
} {
  if (!key.enabled) {
    return { label: "Revoked", variant: "destructive" };
  }
  if (key.expiresAt && new Date(key.expiresAt) < new Date()) {
    return { label: "Expired", variant: "secondary" };
  }
  return { label: "Active", variant: "default" };
}

function formatDate(date: Date | null): string {
  if (!date) return "Never";
  try {
    const d = new Date(date);
    const now = new Date();
    const diffMs = now.getTime() - d.getTime();
    const diffDays = Math.floor(diffMs / (1000 * 60 * 60 * 24));

    if (diffDays === 0) {
      const diffHours = Math.floor(diffMs / (1000 * 60 * 60));
      if (diffHours === 0) {
        const diffMins = Math.floor(diffMs / (1000 * 60));
        return diffMins <= 1 ? "just now" : `${diffMins} minutes ago`;
      }
      return diffHours === 1 ? "1 hour ago" : `${diffHours} hours ago`;
    }
    if (diffDays === 1) return "yesterday";
    if (diffDays < 7) return `${diffDays} days ago`;
    if (diffDays < 30) return `${Math.floor(diffDays / 7)} weeks ago`;
    if (diffDays < 365) return `${Math.floor(diffDays / 30)} months ago`;
    return `${Math.floor(diffDays / 365)} years ago`;
  } catch {
    return "Invalid date";
  }
}

function PermissionsBadge({ permissions }: { permissions: Record<string, string[]> | null }) {
  const [expanded, setExpanded] = useState(false);

  if (!permissions || Object.keys(permissions).length === 0) {
    return <span className="text-gray-400 text-sm">No scopes</span>;
  }

  const scopeIds = permissionsToScopes(permissions);
  const displayCount = 2;
  const hasMore = scopeIds.length > displayCount;

  return (
    <div className="flex flex-col gap-1">
      <div className="flex flex-wrap gap-1 items-center">
        {scopeIds.slice(0, expanded ? scopeIds.length : displayCount).map((scopeId) => {
          const scope = SCOPES[scopeId];
          return (
            <TooltipProvider key={scopeId}>
              <Tooltip>
                <TooltipTrigger asChild>
                  <span
                    className={`inline-flex items-center px-1.5 py-0.5 rounded text-xs ${
                      scope?.sensitive
                        ? "bg-amber-100 text-amber-800"
                        : "bg-blue-100 text-blue-800"
                    }`}
                  >
                    {scopeId}
                  </span>
                </TooltipTrigger>
                <TooltipContent>
                  <p className="font-medium">{scope?.name || scopeId}</p>
                  <p className="text-xs text-gray-400">{scope?.description}</p>
                </TooltipContent>
              </Tooltip>
            </TooltipProvider>
          );
        })}
        {hasMore && (
          <button
            onClick={() => setExpanded(!expanded)}
            className="text-xs text-blue-600 hover:text-blue-700 flex items-center gap-0.5"
          >
            {expanded ? (
              <>
                <ChevronUp className="h-3 w-3" />
                less
              </>
            ) : (
              <>
                +{scopeIds.length - displayCount} more
                <ChevronDown className="h-3 w-3" />
              </>
            )}
          </button>
        )}
      </div>
    </div>
  );
}

export function ApiKeyList({ apiKeys, isLoading, onRevoke, onDelete }: ApiKeyListProps) {
  if (isLoading) {
    return (
      <div className="flex items-center justify-center py-12">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-pana-500"></div>
      </div>
    );
  }

  if (apiKeys.length === 0) {
    return (
      <div className="text-center py-12">
        <p className="text-gray-500 mb-2">No API keys found</p>
        <p className="text-sm text-gray-400">
          Create your first API key to enable machine-to-machine authentication.
        </p>
      </div>
    );
  }

  return (
    <div className="rounded-md border">
      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>Name</TableHead>
            <TableHead>Key Prefix</TableHead>
            <TableHead>
              <div className="flex items-center gap-1">
                <Shield className="h-4 w-4" />
                Scopes
              </div>
            </TableHead>
            <TableHead>Status</TableHead>
            <TableHead>Created</TableHead>
            <TableHead>Last Used</TableHead>
            <TableHead className="w-[50px]"></TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {apiKeys.map((key) => {
            const status = getKeyStatus(key);
            return (
              <TableRow key={key.id}>
                <TableCell className="font-medium">{key.name || "Unnamed Key"}</TableCell>
                <TableCell>
                  <code className="text-sm bg-gray-100 px-2 py-1 rounded">
                    {key.start || key.prefix || "—"}
                  </code>
                </TableCell>
                <TableCell className="max-w-[200px]">
                  <PermissionsBadge permissions={key.permissions} />
                </TableCell>
                <TableCell>
                  <Badge variant={status.variant}>{status.label}</Badge>
                </TableCell>
                <TableCell className="text-sm text-gray-600">
                  {formatDate(key.createdAt)}
                </TableCell>
                <TableCell className="text-sm text-gray-600">
                  {formatDate(key.lastRequest)}
                </TableCell>
                <TableCell>
                  <DropdownMenu>
                    <DropdownMenuTrigger asChild>
                      <Button variant="ghost" size="sm" className="h-8 w-8 p-0">
                        <MoreHorizontal className="h-4 w-4" />
                      </Button>
                    </DropdownMenuTrigger>
                    <DropdownMenuContent align="end">
                      {key.enabled && (
                        <DropdownMenuItem
                          onClick={() => onRevoke(key)}
                          className="text-orange-600"
                        >
                          <Ban className="h-4 w-4 mr-2" />
                          Revoke
                        </DropdownMenuItem>
                      )}
                      <DropdownMenuItem onClick={() => onDelete(key)} className="text-red-600">
                        <Trash2 className="h-4 w-4 mr-2" />
                        Delete
                      </DropdownMenuItem>
                    </DropdownMenuContent>
                  </DropdownMenu>
                </TableCell>
              </TableRow>
            );
          })}
        </TableBody>
      </Table>
    </div>
  );
}
