"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Avatar } from "@/components/ui/avatar";
import { UserPlus, Loader2, Trash2, Crown, Gavel, BookOpen, User } from "lucide-react";

interface Role {
  id: string;
  userId: string;
  role: string;
  createdAt: Date;
}

interface RoleManagerProps {
  hackathonId: string;
  initialRoles: Role[];
}

const ROLE_INFO = {
  manager: {
    label: "Manager",
    icon: Crown,
    description: "Can manage hackathon settings and roles",
    color: "bg-purple-500/15 text-purple-700 dark:text-purple-400",
  },
  judge: {
    label: "Judge",
    icon: Gavel,
    description: "Can score submissions",
    color: "bg-blue-500/15 text-blue-700 dark:text-blue-400",
  },
  mentor: {
    label: "Mentor",
    icon: BookOpen,
    description: "Can mentor assigned teams",
    color: "bg-emerald-500/15 text-emerald-700 dark:text-emerald-400",
  },
  participant: {
    label: "Participant",
    icon: User,
    description: "Registered participant",
    color: "bg-amber-500/15 text-amber-700 dark:text-amber-400",
  },
};

export function RoleManager({ hackathonId, initialRoles }: RoleManagerProps) {
  const [roles, setRoles] = useState(initialRoles);
  const [open, setOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [deleteLoading, setDeleteLoading] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [userId, setUserId] = useState("");
  const [selectedRole, setSelectedRole] = useState<string>("");
  const router = useRouter();

  const onSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!userId.trim() || !selectedRole) return;

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`/api/hackathons/${hackathonId}/roles`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ userId: userId.trim(), role: selectedRole }),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.error || "Failed to assign role");
      }

      setRoles([...roles, result.data]);
      setOpen(false);
      setUserId("");
      setSelectedRole("");
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Something went wrong");
    } finally {
      setIsLoading(false);
    }
  };

  const handleRemove = async (role: Role) => {
    setDeleteLoading(role.id);

    try {
      const response = await fetch(`/api/hackathons/${hackathonId}/roles`, {
        method: "DELETE",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ userId: role.userId, role: role.role }),
      });

      if (!response.ok) {
        const result = await response.json();
        throw new Error(result.error || "Failed to remove role");
      }

      setRoles(roles.filter((r) => r.id !== role.id));
      router.refresh();
    } catch (err) {
      console.error("Error removing role:", err);
    } finally {
      setDeleteLoading(null);
    }
  };

  // Group roles by type
  const rolesByType = roles.reduce(
    (acc, role) => {
      if (!acc[role.role]) acc[role.role] = [];
      acc[role.role].push(role);
      return acc;
    },
    {} as Record<string, Role[]>
  );

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <p className="text-sm text-muted-foreground">
          Assign roles to users for this hackathon.
        </p>
        <Dialog open={open} onOpenChange={setOpen}>
          <DialogTrigger asChild>
            <Button size="sm" className="gap-2">
              <UserPlus className="h-4 w-4" />
              Assign Role
            </Button>
          </DialogTrigger>
          <DialogContent>
            <DialogHeader>
              <DialogTitle>Assign Role to User</DialogTitle>
            </DialogHeader>
            <form onSubmit={onSubmit} className="space-y-4">
              <div className="space-y-2">
                <Label htmlFor="userId">User ID *</Label>
                <Input
                  id="userId"
                  placeholder="Enter user ID from SSO"
                  value={userId}
                  onChange={(e) => setUserId(e.target.value)}
                />
                <p className="text-xs text-muted-foreground">
                  The user must be registered in the SSO system.
                </p>
              </div>

              <div className="space-y-2">
                <Label>Role *</Label>
                <Select value={selectedRole} onValueChange={setSelectedRole}>
                  <SelectTrigger>
                    <SelectValue placeholder="Select a role" />
                  </SelectTrigger>
                  <SelectContent>
                    {Object.entries(ROLE_INFO).map(([key, info]) => (
                      <SelectItem key={key} value={key}>
                        <div className="flex items-center gap-2">
                          <info.icon className="h-4 w-4" />
                          {info.label}
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
                {selectedRole && (
                  <p className="text-xs text-muted-foreground">
                    {ROLE_INFO[selectedRole as keyof typeof ROLE_INFO]?.description}
                  </p>
                )}
              </div>

              {error && (
                <div className="p-3 text-sm text-destructive bg-destructive/10 rounded-lg">
                  {error}
                </div>
              )}

              <div className="flex justify-end gap-2">
                <Button
                  type="button"
                  variant="outline"
                  onClick={() => setOpen(false)}
                >
                  Cancel
                </Button>
                <Button
                  type="submit"
                  disabled={isLoading || !userId.trim() || !selectedRole}
                >
                  {isLoading && (
                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  )}
                  Assign Role
                </Button>
              </div>
            </form>
          </DialogContent>
        </Dialog>
      </div>

      {roles.length === 0 ? (
        <div className="text-center py-8">
          <p className="text-muted-foreground">
            No roles assigned yet. Assign roles to enable team management and
            judging.
          </p>
        </div>
      ) : (
        <div className="space-y-6">
          {Object.entries(ROLE_INFO).map(([roleType, info]) => {
            const roleUsers = rolesByType[roleType] || [];
            if (roleUsers.length === 0) return null;

            return (
              <div key={roleType} className="space-y-3">
                <h4 className="font-medium flex items-center gap-2">
                  <info.icon className="h-4 w-4" />
                  {info.label}s ({roleUsers.length})
                </h4>
                <div className="grid gap-2 sm:grid-cols-2">
                  {roleUsers.map((role) => (
                    <div
                      key={role.id}
                      className="flex items-center gap-3 p-3 rounded-lg border bg-card"
                    >
                      <Avatar fallback={role.userId} size="sm" />
                      <div className="flex-1 min-w-0">
                        <p className="text-sm font-medium truncate">
                          {role.userId.slice(0, 12)}...
                        </p>
                        <Badge className={info.color}>{info.label}</Badge>
                      </div>
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => handleRemove(role)}
                        disabled={deleteLoading === role.id}
                      >
                        {deleteLoading === role.id ? (
                          <Loader2 className="h-4 w-4 animate-spin" />
                        ) : (
                          <Trash2 className="h-4 w-4 text-destructive" />
                        )}
                      </Button>
                    </div>
                  ))}
                </div>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}
