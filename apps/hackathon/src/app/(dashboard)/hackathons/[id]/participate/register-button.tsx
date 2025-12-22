"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Loader2, UserPlus } from "lucide-react";

interface RegisterButtonProps {
  hackathonId: string;
}

export function RegisterButton({ hackathonId }: RegisterButtonProps) {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const router = useRouter();

  const handleRegister = async () => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`/api/hackathons/${hackathonId}/register`, {
        method: "POST",
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.error || "Failed to register");
      }

      toast.success("Registration successful!", {
        description: "You can now create or join a team.",
      });
      router.refresh();
    } catch (err) {
      const message = err instanceof Error ? err.message : "Something went wrong";
      setError(message);
      toast.error("Registration failed", { description: message });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="space-y-3">
      <Button onClick={handleRegister} disabled={isLoading} size="lg">
        {isLoading ? (
          <Loader2 className="mr-2 h-4 w-4 animate-spin" />
        ) : (
          <UserPlus className="mr-2 h-4 w-4" />
        )}
        Register Now
      </Button>
      {error && <p className="text-sm text-destructive">{error}</p>}
    </div>
  );
}
