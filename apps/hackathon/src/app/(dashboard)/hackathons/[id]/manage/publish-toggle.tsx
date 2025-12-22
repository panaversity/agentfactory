"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Eye, EyeOff, Loader2 } from "lucide-react";

interface PublishToggleProps {
  hackathonId: string;
  published: boolean;
}

export function PublishToggle({ hackathonId, published }: PublishToggleProps) {
  const router = useRouter();
  const [isLoading, setIsLoading] = useState(false);
  const [isPublished, setIsPublished] = useState(published);

  const handleToggle = async () => {
    setIsLoading(true);
    try {
      const response = await fetch(`/api/hackathons/${hackathonId}/publish`, {
        method: "POST",
      });

      if (!response.ok) {
        throw new Error("Failed to toggle publish status");
      }

      const updated = await response.json();
      setIsPublished(updated.published);
      toast.success(updated.published ? "Hackathon published!" : "Hackathon unpublished", {
        description: updated.published
          ? "Participants can now discover and join."
          : "Hidden from public listings.",
      });
      router.refresh();
    } catch (error) {
      console.error("Error toggling publish status:", error);
      toast.error("Failed to update publish status");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Button
      variant={isPublished ? "secondary" : "default"}
      onClick={handleToggle}
      disabled={isLoading}
    >
      {isLoading ? (
        <Loader2 className="mr-2 h-4 w-4 animate-spin" />
      ) : isPublished ? (
        <EyeOff className="mr-2 h-4 w-4" />
      ) : (
        <Eye className="mr-2 h-4 w-4" />
      )}
      {isPublished ? "Unpublish" : "Publish"}
    </Button>
  );
}
