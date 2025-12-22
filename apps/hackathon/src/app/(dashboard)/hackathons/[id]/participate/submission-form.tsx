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
import { Loader2, Send, ExternalLink } from "lucide-react";
import {
  createSubmissionSchema,
  type CreateSubmissionInput,
} from "@/lib/validation/submission";

interface SubmissionFormProps {
  hackathonId: string;
  teamId: string;
  existingSubmission?: {
    projectName: string;
    description: string;
    repositoryUrl: string;
    demoUrl?: string | null;
    presentationUrl?: string | null;
  };
}

export function SubmissionForm({
  hackathonId,
  teamId,
  existingSubmission,
}: SubmissionFormProps) {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const router = useRouter();

  const form = useForm<CreateSubmissionInput>({
    resolver: zodResolver(createSubmissionSchema),
    defaultValues: {
      projectName: existingSubmission?.projectName || "",
      description: existingSubmission?.description || "",
      repositoryUrl: existingSubmission?.repositoryUrl || "",
      demoUrl: existingSubmission?.demoUrl || "",
      presentationUrl: existingSubmission?.presentationUrl || "",
    },
  });

  const onSubmit = async (data: CreateSubmissionInput) => {
    setIsLoading(true);
    setError(null);
    setSuccess(false);

    try {
      const response = await fetch(
        `/api/hackathons/${hackathonId}/submissions`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(data),
        }
      );

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.error || "Failed to submit project");
      }

      setSuccess(true);
      toast.success("Project submitted!", {
        description: "You can update it until the deadline.",
      });
      router.refresh();
    } catch (err) {
      const message = err instanceof Error ? err.message : "Something went wrong";
      setError(message);
      toast.error("Submission failed", { description: message });
    } finally {
      setIsLoading(false);
    }
  };

  if (success) {
    return (
      <div className="py-8 text-center">
        <div className="mx-auto w-12 h-12 rounded-full bg-emerald-500/15 flex items-center justify-center mb-4">
          <Send className="h-6 w-6 text-emerald-600" />
        </div>
        <h3 className="text-lg font-semibold mb-2">Submission Received!</h3>
        <p className="text-muted-foreground max-w-md mx-auto">
          Your project has been submitted successfully. You can update it until
          the deadline.
        </p>
      </div>
    );
  }

  return (
    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
      <div className="grid gap-6 sm:grid-cols-2">
        <div className="space-y-2">
          <Label htmlFor="projectName">Project Name *</Label>
          <Input
            id="projectName"
            placeholder="Enter your project name"
            {...form.register("projectName")}
          />
          {form.formState.errors.projectName && (
            <p className="text-sm text-destructive">
              {form.formState.errors.projectName.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="repositoryUrl">Repository URL *</Label>
          <div className="relative">
            <Input
              id="repositoryUrl"
              type="url"
              placeholder="https://github.com/..."
              {...form.register("repositoryUrl")}
            />
            <ExternalLink className="absolute right-3 top-1/2 -translate-y-1/2 h-4 w-4 text-muted-foreground" />
          </div>
          {form.formState.errors.repositoryUrl && (
            <p className="text-sm text-destructive">
              {form.formState.errors.repositoryUrl.message}
            </p>
          )}
        </div>
      </div>

      <div className="space-y-2">
        <Label htmlFor="description">Project Description *</Label>
        <Textarea
          id="description"
          placeholder="Describe your project, what problem it solves, and the technology used..."
          rows={4}
          {...form.register("description")}
        />
        {form.formState.errors.description && (
          <p className="text-sm text-destructive">
            {form.formState.errors.description.message}
          </p>
        )}
      </div>

      <div className="grid gap-6 sm:grid-cols-2">
        <div className="space-y-2">
          <Label htmlFor="demoUrl">Demo URL (optional)</Label>
          <div className="relative">
            <Input
              id="demoUrl"
              type="url"
              placeholder="https://your-demo.com"
              {...form.register("demoUrl")}
            />
            <ExternalLink className="absolute right-3 top-1/2 -translate-y-1/2 h-4 w-4 text-muted-foreground" />
          </div>
          {form.formState.errors.demoUrl && (
            <p className="text-sm text-destructive">
              {form.formState.errors.demoUrl.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="presentationUrl">Presentation URL (optional)</Label>
          <div className="relative">
            <Input
              id="presentationUrl"
              type="url"
              placeholder="https://slides.google.com/..."
              {...form.register("presentationUrl")}
            />
            <ExternalLink className="absolute right-3 top-1/2 -translate-y-1/2 h-4 w-4 text-muted-foreground" />
          </div>
          {form.formState.errors.presentationUrl && (
            <p className="text-sm text-destructive">
              {form.formState.errors.presentationUrl.message}
            </p>
          )}
        </div>
      </div>

      {error && (
        <div className="p-4 text-sm text-destructive bg-destructive/10 rounded-lg">
          {error}
        </div>
      )}

      <div className="flex justify-end">
        <Button type="submit" disabled={isLoading} size="lg">
          {isLoading && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
          <Send className="mr-2 h-4 w-4" />
          Submit Project
        </Button>
      </div>
    </form>
  );
}
