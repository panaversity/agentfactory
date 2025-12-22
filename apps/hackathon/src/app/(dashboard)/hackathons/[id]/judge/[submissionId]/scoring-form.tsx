"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Textarea } from "@/components/ui/textarea";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { Loader2, Save, CheckCircle2 } from "lucide-react";

interface Criterion {
  id: string;
  name: string;
  description: string | null;
  weight: number;
  maxScore: number;
  existingScore: number | null;
  existingFeedback: string | null;
}

interface ScoringFormProps {
  hackathonId: string;
  submissionId: string;
  criteria: Criterion[];
}

export function ScoringForm({
  hackathonId,
  submissionId,
  criteria,
}: ScoringFormProps) {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const router = useRouter();

  const [scores, setScores] = useState<
    Record<string, { score: number; feedback: string }>
  >(() => {
    const initial: Record<string, { score: number; feedback: string }> = {};
    for (const c of criteria) {
      initial[c.id] = {
        score: c.existingScore ?? Math.floor(c.maxScore / 2),
        feedback: c.existingFeedback ?? "",
      };
    }
    return initial;
  });

  const handleScoreChange = (criterionId: string, value: number[]) => {
    setScores((prev) => ({
      ...prev,
      [criterionId]: { ...prev[criterionId], score: value[0] },
    }));
  };

  const handleFeedbackChange = (criterionId: string, feedback: string) => {
    setScores((prev) => ({
      ...prev,
      [criterionId]: { ...prev[criterionId], feedback },
    }));
  };

  const onSubmit = async () => {
    setIsLoading(true);
    setError(null);
    setSuccess(false);

    try {
      const scoreData = Object.entries(scores).map(([criterionId, data]) => ({
        criterionId,
        score: data.score,
        feedback: data.feedback || undefined,
      }));

      const response = await fetch(
        `/api/hackathons/${hackathonId}/submissions/${submissionId}/score`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ scores: scoreData }),
        }
      );

      const result = await response.json();

      if (!response.ok) {
        throw new Error(result.error || "Failed to submit scores");
      }

      setSuccess(true);
      toast.success("Scores saved!", {
        description: "Your evaluation has been recorded.",
      });
      router.refresh();
    } catch (err) {
      const message = err instanceof Error ? err.message : "Something went wrong";
      setError(message);
      toast.error("Failed to save scores", { description: message });
    } finally {
      setIsLoading(false);
    }
  };

  // Calculate weighted total
  const totalWeightedScore = criteria.reduce((sum, c) => {
    const score = scores[c.id]?.score ?? 0;
    return sum + score * c.weight;
  }, 0);

  const maxWeightedScore = criteria.reduce(
    (sum, c) => sum + c.maxScore * c.weight,
    0
  );

  if (success) {
    return (
      <div className="py-8 text-center">
        <div className="mx-auto w-12 h-12 rounded-full bg-emerald-500/15 flex items-center justify-center mb-4">
          <CheckCircle2 className="h-6 w-6 text-emerald-600" />
        </div>
        <h3 className="text-lg font-semibold mb-2">Scores Submitted!</h3>
        <p className="text-muted-foreground max-w-md mx-auto mb-4">
          Your scores have been saved successfully.
        </p>
        <Button variant="outline" asChild>
          <a href={`/hackathons/${hackathonId}/judge`}>Back to Submissions</a>
        </Button>
      </div>
    );
  }

  return (
    <div className="space-y-8">
      {criteria.map((criterion, index) => (
        <div key={criterion.id} className="space-y-4">
          {index > 0 && <Separator />}
          <div className="space-y-4 pt-2">
            <div className="flex items-start justify-between">
              <div>
                <h4 className="font-medium flex items-center gap-2">
                  {criterion.name}
                  <Badge variant="secondary" className="text-xs">
                    Weight: {criterion.weight}x
                  </Badge>
                </h4>
                {criterion.description && (
                  <p className="text-sm text-muted-foreground mt-1">
                    {criterion.description}
                  </p>
                )}
              </div>
              <div className="text-right">
                <span className="text-2xl font-bold">
                  {scores[criterion.id]?.score}
                </span>
                <span className="text-muted-foreground">
                  /{criterion.maxScore}
                </span>
              </div>
            </div>

            <div className="space-y-2">
              <Slider
                value={[scores[criterion.id]?.score ?? 0]}
                onValueChange={(value) =>
                  handleScoreChange(criterion.id, value)
                }
                max={criterion.maxScore}
                step={1}
                className="w-full"
              />
              <div className="flex justify-between text-xs text-muted-foreground">
                <span>0</span>
                <span>{criterion.maxScore}</span>
              </div>
            </div>

            <div className="space-y-2">
              <Label
                htmlFor={`feedback-${criterion.id}`}
                className="text-sm text-muted-foreground"
              >
                Feedback (optional)
              </Label>
              <Textarea
                id={`feedback-${criterion.id}`}
                placeholder={`Share feedback about ${criterion.name.toLowerCase()}...`}
                value={scores[criterion.id]?.feedback ?? ""}
                onChange={(e) =>
                  handleFeedbackChange(criterion.id, e.target.value)
                }
                rows={2}
              />
            </div>
          </div>
        </div>
      ))}

      <Separator />

      {/* Summary */}
      <div className="flex items-center justify-between p-4 bg-muted/50 rounded-lg">
        <div>
          <p className="text-sm text-muted-foreground">
            Total Weighted Score
          </p>
          <p className="text-2xl font-bold">
            {totalWeightedScore.toFixed(1)}
            <span className="text-muted-foreground text-base font-normal">
              {" "}
              / {maxWeightedScore}
            </span>
          </p>
        </div>
        <div className="text-right">
          <p className="text-sm text-muted-foreground">Percentage</p>
          <p className="text-2xl font-bold">
            {((totalWeightedScore / maxWeightedScore) * 100).toFixed(0)}%
          </p>
        </div>
      </div>

      {error && (
        <div className="p-4 text-sm text-destructive bg-destructive/10 rounded-lg">
          {error}
        </div>
      )}

      <div className="flex justify-end">
        <Button onClick={onSubmit} disabled={isLoading} size="lg">
          {isLoading && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
          <Save className="mr-2 h-4 w-4" />
          Save Scores
        </Button>
      </div>
    </div>
  );
}
