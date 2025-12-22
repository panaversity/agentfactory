"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { z } from "zod";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { Plus, Loader2, Trash2, GripVertical } from "lucide-react";

const criterionSchema = z.object({
  name: z.string().min(1, "Name is required").max(100),
  description: z.string().optional(),
  weight: z.coerce.number().min(1).max(10),
  maxScore: z.coerce.number().min(1).max(100),
});

type CriterionInput = z.infer<typeof criterionSchema>;

interface Criterion {
  id: string;
  name: string;
  description: string | null;
  weight: number;
  maxScore: number;
  order: number;
}

interface CriteriaManagerProps {
  hackathonId: string;
  initialCriteria: Criterion[];
}

export function CriteriaManager({
  hackathonId,
  initialCriteria,
}: CriteriaManagerProps) {
  const [criteria, setCriteria] = useState(initialCriteria);
  const [open, setOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [deleteLoading, setDeleteLoading] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const router = useRouter();

  const form = useForm<CriterionInput>({
    resolver: zodResolver(criterionSchema),
    defaultValues: {
      name: "",
      description: "",
      weight: 1,
      maxScore: 10,
    },
  });

  const onSubmit = async (data: CriterionInput) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`/api/hackathons/${hackathonId}/criteria`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data),
      });

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.error || "Failed to create criterion");
      }

      setCriteria([...criteria, result.data]);
      setOpen(false);
      form.reset();
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Something went wrong");
    } finally {
      setIsLoading(false);
    }
  };

  const handleDelete = async (criterionId: string) => {
    setDeleteLoading(criterionId);

    try {
      const response = await fetch(
        `/api/hackathons/${hackathonId}/criteria?criterionId=${criterionId}`,
        { method: "DELETE" }
      );

      if (!response.ok) {
        const result = await response.json();
        throw new Error(result.error || "Failed to delete criterion");
      }

      setCriteria(criteria.filter((c) => c.id !== criterionId));
      router.refresh();
    } catch (err) {
      console.error("Error deleting criterion:", err);
    } finally {
      setDeleteLoading(null);
    }
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <p className="text-sm text-muted-foreground">
          Define the criteria judges will use to score submissions.
        </p>
        <Dialog open={open} onOpenChange={setOpen}>
          <DialogTrigger asChild>
            <Button size="sm" className="gap-2">
              <Plus className="h-4 w-4" />
              Add Criterion
            </Button>
          </DialogTrigger>
          <DialogContent>
            <DialogHeader>
              <DialogTitle>Add Judging Criterion</DialogTitle>
            </DialogHeader>
            <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
              <div className="space-y-2">
                <Label htmlFor="name">Name *</Label>
                <Input
                  id="name"
                  placeholder="e.g., Innovation"
                  {...form.register("name")}
                />
                {form.formState.errors.name && (
                  <p className="text-sm text-destructive">
                    {form.formState.errors.name.message}
                  </p>
                )}
              </div>

              <div className="space-y-2">
                <Label htmlFor="description">Description</Label>
                <Textarea
                  id="description"
                  placeholder="Describe what judges should evaluate..."
                  rows={2}
                  {...form.register("description")}
                />
              </div>

              <div className="grid grid-cols-2 gap-4">
                <div className="space-y-2">
                  <Label htmlFor="maxScore">Max Score</Label>
                  <Input
                    id="maxScore"
                    type="number"
                    min={1}
                    max={100}
                    {...form.register("maxScore")}
                  />
                </div>
                <div className="space-y-2">
                  <Label htmlFor="weight">Weight (1-10)</Label>
                  <Input
                    id="weight"
                    type="number"
                    min={1}
                    max={10}
                    {...form.register("weight")}
                  />
                </div>
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
                  {isLoading && (
                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  )}
                  Add Criterion
                </Button>
              </div>
            </form>
          </DialogContent>
        </Dialog>
      </div>

      {criteria.length === 0 ? (
        <div className="text-center py-8">
          <p className="text-muted-foreground">
            No judging criteria defined yet. Add criteria to enable judging.
          </p>
        </div>
      ) : (
        <div className="space-y-3">
          {criteria.map((criterion, index) => (
            <div
              key={criterion.id}
              className="flex items-center gap-4 p-4 rounded-lg border bg-card"
            >
              <GripVertical className="h-5 w-5 text-muted-foreground cursor-grab" />
              <div className="flex-1 min-w-0">
                <div className="flex items-center gap-2">
                  <h4 className="font-medium">{criterion.name}</h4>
                  <Badge variant="secondary">Max: {criterion.maxScore}</Badge>
                  <Badge variant="outline">Weight: {criterion.weight}x</Badge>
                </div>
                {criterion.description && (
                  <p className="text-sm text-muted-foreground mt-1">
                    {criterion.description}
                  </p>
                )}
              </div>
              <Button
                variant="ghost"
                size="icon"
                onClick={() => handleDelete(criterion.id)}
                disabled={deleteLoading === criterion.id}
              >
                {deleteLoading === criterion.id ? (
                  <Loader2 className="h-4 w-4 animate-spin" />
                ) : (
                  <Trash2 className="h-4 w-4 text-destructive" />
                )}
              </Button>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
