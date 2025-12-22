"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { HackathonCard } from "@/components/hackathons/hackathon-card";

interface Hackathon {
  id: string;
  title: string;
  description: string;
  slug: string;
  startDate: Date;
  endDate: Date;
  status: string;
  published: boolean;
  minTeamSize: number;
  maxTeamSize: number;
}

interface HackathonListProps {
  initialHackathons: Hackathon[];
}

export function HackathonList({ initialHackathons }: HackathonListProps) {
  const router = useRouter();
  const [hackathons, setHackathons] = useState(initialHackathons);
  const [_isLoading, setIsLoading] = useState<string | null>(null);

  const handlePublishToggle = async (id: string) => {
    setIsLoading(id);
    try {
      const response = await fetch(`/api/hackathons/${id}/publish`, {
        method: "POST",
      });

      if (!response.ok) {
        throw new Error("Failed to toggle publish status");
      }

      const updated = await response.json();
      setHackathons((prev) =>
        prev.map((h) =>
          h.id === id
            ? { ...h, published: updated.published, status: updated.status }
            : h
        )
      );
      router.refresh();
    } catch (error) {
      console.error("Error toggling publish status:", error);
    } finally {
      setIsLoading(null);
    }
  };

  return (
    <div className="space-y-6">
      <h2 className="text-xl font-semibold">Your Hackathons</h2>
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
        {hackathons.map((hackathon) => (
          <HackathonCard
            key={hackathon.id}
            hackathon={hackathon}
            showManageButton
            onPublishToggle={handlePublishToggle}
          />
        ))}
      </div>
    </div>
  );
}
