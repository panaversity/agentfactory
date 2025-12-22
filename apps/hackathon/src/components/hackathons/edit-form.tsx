"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Label } from "@/components/ui/label";
import {
  updateHackathonSchema,
  type UpdateHackathonInput,
} from "@/lib/validation/hackathon";
import { Loader2 } from "lucide-react";

interface Hackathon {
  id: string;
  title: string;
  description: string;
  slug: string;
  startDate: string | Date;
  endDate: string | Date;
  registrationDeadline: string | Date;
  submissionDeadline: string | Date;
  minTeamSize: number;
  maxTeamSize: number;
}

interface EditHackathonFormProps {
  hackathon: Hackathon;
  onSuccess?: () => void;
}

// Format date for datetime-local input
function formatDateForInput(date: string | Date): string {
  const d = new Date(date);
  return d.toISOString().slice(0, 16);
}

export function EditHackathonForm({
  hackathon,
  onSuccess,
}: EditHackathonFormProps) {
  const router = useRouter();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<UpdateHackathonInput>({
    resolver: zodResolver(updateHackathonSchema),
    defaultValues: {
      title: hackathon.title,
      description: hackathon.description,
      slug: hackathon.slug,
      startDate: new Date(hackathon.startDate),
      endDate: new Date(hackathon.endDate),
      registrationDeadline: new Date(hackathon.registrationDeadline),
      submissionDeadline: new Date(hackathon.submissionDeadline),
      minTeamSize: hackathon.minTeamSize,
      maxTeamSize: hackathon.maxTeamSize,
    },
  });

  const onSubmit = async (data: UpdateHackathonInput) => {
    setIsSubmitting(true);
    setError(null);

    try {
      const response = await fetch(`/api/hackathons/${hackathon.id}`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || "Failed to update hackathon");
      }

      toast.success("Changes saved!", {
        description: "Your hackathon settings have been updated.",
      });
      onSuccess?.();
      router.refresh();
    } catch (err) {
      const message = err instanceof Error ? err.message : "An error occurred";
      setError(message);
      toast.error("Failed to save changes", { description: message });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="space-y-6">
      {error && (
        <div className="rounded-md bg-destructive/10 p-3 text-sm text-destructive">
          {error}
        </div>
      )}

      <div className="space-y-2">
        <Label htmlFor="title">Title</Label>
        <Input id="title" {...register("title")} />
        {errors.title && (
          <p className="text-sm text-destructive">{errors.title.message}</p>
        )}
      </div>

      <div className="space-y-2">
        <Label htmlFor="slug">URL Slug</Label>
        <Input id="slug" {...register("slug")} />
        {errors.slug && (
          <p className="text-sm text-destructive">{errors.slug.message}</p>
        )}
      </div>

      <div className="space-y-2">
        <Label htmlFor="description">Description</Label>
        <Textarea id="description" rows={4} {...register("description")} />
        {errors.description && (
          <p className="text-sm text-destructive">
            {errors.description.message}
          </p>
        )}
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div className="space-y-2">
          <Label htmlFor="startDate">Start Date</Label>
          <Input
            id="startDate"
            type="datetime-local"
            defaultValue={formatDateForInput(hackathon.startDate)}
            {...register("startDate")}
          />
          {errors.startDate && (
            <p className="text-sm text-destructive">
              {errors.startDate.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="endDate">End Date</Label>
          <Input
            id="endDate"
            type="datetime-local"
            defaultValue={formatDateForInput(hackathon.endDate)}
            {...register("endDate")}
          />
          {errors.endDate && (
            <p className="text-sm text-destructive">{errors.endDate.message}</p>
          )}
        </div>
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div className="space-y-2">
          <Label htmlFor="registrationDeadline">Registration Deadline</Label>
          <Input
            id="registrationDeadline"
            type="datetime-local"
            defaultValue={formatDateForInput(hackathon.registrationDeadline)}
            {...register("registrationDeadline")}
          />
          {errors.registrationDeadline && (
            <p className="text-sm text-destructive">
              {errors.registrationDeadline.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="submissionDeadline">Submission Deadline</Label>
          <Input
            id="submissionDeadline"
            type="datetime-local"
            defaultValue={formatDateForInput(hackathon.submissionDeadline)}
            {...register("submissionDeadline")}
          />
          {errors.submissionDeadline && (
            <p className="text-sm text-destructive">
              {errors.submissionDeadline.message}
            </p>
          )}
        </div>
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div className="space-y-2">
          <Label htmlFor="minTeamSize">Min Team Size</Label>
          <Input
            id="minTeamSize"
            type="number"
            min={1}
            max={10}
            {...register("minTeamSize", { valueAsNumber: true })}
          />
          {errors.minTeamSize && (
            <p className="text-sm text-destructive">
              {errors.minTeamSize.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="maxTeamSize">Max Team Size</Label>
          <Input
            id="maxTeamSize"
            type="number"
            min={1}
            max={20}
            {...register("maxTeamSize", { valueAsNumber: true })}
          />
          {errors.maxTeamSize && (
            <p className="text-sm text-destructive">
              {errors.maxTeamSize.message}
            </p>
          )}
        </div>
      </div>

      <div className="flex justify-end gap-2">
        <Button type="submit" disabled={isSubmitting}>
          {isSubmitting && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
          Save Changes
        </Button>
      </div>
    </form>
  );
}
