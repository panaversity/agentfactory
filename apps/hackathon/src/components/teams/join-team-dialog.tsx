"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { toast } from "sonner";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { UserPlus, Loader2, KeyRound } from "lucide-react";
import { joinTeamSchema, type JoinTeamInput } from "@/lib/validation/team";

interface JoinTeamDialogProps {
  hackathonId: string;
  onSuccess?: () => void;
}

export function JoinTeamDialog({ hackathonId, onSuccess }: JoinTeamDialogProps) {
  const [open, setOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const router = useRouter();

  const form = useForm<JoinTeamInput>({
    resolver: zodResolver(joinTeamSchema),
    defaultValues: {
      inviteCode: "",
    },
  });

  const onSubmit = async (data: JoinTeamInput) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(
        `/api/hackathons/${hackathonId}/teams/join`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(data),
        }
      );

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.error || "Failed to join team");
      }

      setOpen(false);
      form.reset();
      toast.success("You've joined the team!", {
        description: `Welcome to ${result.team?.name || "the team"}!`,
      });
      router.refresh();
      onSuccess?.();
    } catch (err) {
      const message = err instanceof Error ? err.message : "Something went wrong";
      setError(message);
      toast.error("Failed to join team", { description: message });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button variant="outline" className="gap-2">
          <UserPlus className="h-4 w-4" />
          Join Team
        </Button>
      </DialogTrigger>
      <DialogContent className="sm:max-w-md">
        <DialogHeader>
          <DialogTitle className="flex items-center gap-2">
            <KeyRound className="h-5 w-5" />
            Join a Team
          </DialogTitle>
          <DialogDescription>
            Enter the invite code shared by your team leader to join their team.
          </DialogDescription>
        </DialogHeader>

        <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
          <div className="space-y-2">
            <Label htmlFor="inviteCode">Invite Code</Label>
            <Input
              id="inviteCode"
              placeholder="Enter 8-character code"
              className="font-mono text-center text-lg tracking-widest uppercase"
              maxLength={8}
              {...form.register("inviteCode")}
            />
            {form.formState.errors.inviteCode && (
              <p className="text-sm text-destructive">
                {form.formState.errors.inviteCode.message}
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
            <Button type="submit" disabled={isLoading}>
              {isLoading && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
              Join Team
            </Button>
          </div>
        </form>
      </DialogContent>
    </Dialog>
  );
}
