"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useForm, useFieldArray } from "react-hook-form";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Label } from "@/components/ui/label";
import { generateSlug } from "@/lib/validation/hackathon";
import { Loader2, Plus, Trash2 } from "lucide-react";

interface CreateHackathonFormProps {
  onSuccess?: () => void;
  organizationId: string;
}

interface Prize {
  place: string;
  title: string;
  value: string;
}

interface Organizer {
  name: string;
  logo: string;
  url: string;
}

interface Sponsor {
  name: string;
  logo: string;
  url: string;
  tier: "platinum" | "gold" | "silver" | "bronze" | "";
}

interface Category {
  id: string;
  name: string;
  description: string;
}

// Form values type that matches what the form collects
interface FormValues {
  title: string;
  description: string;
  slug: string;
  startDate: string;
  endDate: string;
  registrationDeadline: string;
  submissionDeadline: string;
  minTeamSize: number;
  maxTeamSize: number;
  prizes: Prize[];
  organizers: Organizer[];
  sponsors: Sponsor[];
  categories: Category[];
}

export function CreateHackathonForm({ onSuccess, organizationId }: CreateHackathonFormProps) {
  const router = useRouter();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const {
    register,
    handleSubmit,
    setValue,
    control,
    formState: { errors },
  } = useForm<FormValues>({
    defaultValues: {
      minTeamSize: 1,
      maxTeamSize: 5,
      prizes: [
        { place: "1st", title: "Grand Prize", value: "$5,000" },
        { place: "2nd", title: "Runner Up", value: "$2,500" },
        { place: "3rd", title: "Third Place", value: "$1,000" },
      ],
      organizers: [{ name: "", logo: "", url: "" }],
      sponsors: [],
      categories: [],
    },
  });

  const { fields: prizeFields, append: appendPrize, remove: removePrize } = useFieldArray({
    control,
    name: "prizes",
  });

  const { fields: organizerFields, append: appendOrganizer, remove: removeOrganizer } = useFieldArray({
    control,
    name: "organizers",
  });

  const { fields: sponsorFields, append: appendSponsor, remove: removeSponsor } = useFieldArray({
    control,
    name: "sponsors",
  });

  const { fields: categoryFields, append: appendCategory, remove: removeCategory } = useFieldArray({
    control,
    name: "categories",
  });

  // Auto-generate slug from title
  const handleTitleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newTitle = e.target.value;
    setValue("title", newTitle);
    setValue("slug", generateSlug(newTitle));
  };

  const onSubmit = async (data: FormValues) => {
    setIsSubmitting(true);
    setError(null);

    try {
      // Filter out empty entries and convert arrays to JSON strings
      const validPrizes = data.prizes.filter(p => p.place || p.title || p.value);
      const validOrganizers = data.organizers.filter(o => o.name);
      const validSponsors = data.sponsors.filter(s => s.name);
      const validCategories = data.categories.filter(c => c.name);

      const submitData = {
        ...data,
        organizationId,
        prizes: validPrizes.length > 0 ? JSON.stringify(validPrizes) : undefined,
        organizers: validOrganizers.length > 0 ? JSON.stringify(validOrganizers) : undefined,
        sponsors: validSponsors.length > 0 ? JSON.stringify(validSponsors) : undefined,
        categories: validCategories.length > 0 ? JSON.stringify(validCategories.map(c => ({
          ...c,
          id: c.id || c.name.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/-+$/, ''),
        }))) : undefined,
      };

      const response = await fetch("/api/hackathons", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(submitData),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || "Failed to create hackathon");
      }

      const hackathon = await response.json();
      toast.success("Hackathon created!", {
        description: "Configure judging criteria and publish when ready.",
      });
      onSuccess?.();
      router.push(`/hackathons/${hackathon.id}`);
      router.refresh();
    } catch (err) {
      const message = err instanceof Error ? err.message : "An error occurred";
      setError(message);
      toast.error("Failed to create hackathon", { description: message });
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
        <Label htmlFor="title">Title *</Label>
        <Input
          id="title"
          placeholder="My Awesome Hackathon"
          {...register("title")}
          onChange={handleTitleChange}
        />
        {errors.title && (
          <p className="text-sm text-destructive">{errors.title.message}</p>
        )}
      </div>

      <div className="space-y-2">
        <Label htmlFor="slug">URL Slug *</Label>
        <Input
          id="slug"
          placeholder="my-awesome-hackathon"
          {...register("slug")}
        />
        {errors.slug && (
          <p className="text-sm text-destructive">{errors.slug.message}</p>
        )}
        <p className="text-xs text-muted-foreground">
          Used in the hackathon URL
        </p>
      </div>

      <div className="space-y-2">
        <Label htmlFor="description">Description *</Label>
        <Textarea
          id="description"
          placeholder="Describe your hackathon..."
          rows={4}
          {...register("description")}
        />
        {errors.description && (
          <p className="text-sm text-destructive">
            {errors.description.message}
          </p>
        )}
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div className="space-y-2">
          <Label htmlFor="startDate">Start Date *</Label>
          <Input id="startDate" type="datetime-local" {...register("startDate")} />
          {errors.startDate && (
            <p className="text-sm text-destructive">
              {errors.startDate.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="endDate">End Date *</Label>
          <Input id="endDate" type="datetime-local" {...register("endDate")} />
          {errors.endDate && (
            <p className="text-sm text-destructive">{errors.endDate.message}</p>
          )}
        </div>
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div className="space-y-2">
          <Label htmlFor="registrationDeadline">Registration Deadline *</Label>
          <Input
            id="registrationDeadline"
            type="datetime-local"
            {...register("registrationDeadline")}
          />
          {errors.registrationDeadline && (
            <p className="text-sm text-destructive">
              {errors.registrationDeadline.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="submissionDeadline">Submission Deadline *</Label>
          <Input
            id="submissionDeadline"
            type="datetime-local"
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
          <Label htmlFor="minTeamSize">Min Team Size *</Label>
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
          <Label htmlFor="maxTeamSize">Max Team Size *</Label>
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

      {/* Prizes Section */}
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <Label>Prizes</Label>
          <Button
            type="button"
            variant="outline"
            size="sm"
            onClick={() => appendPrize({ place: "", title: "", value: "" })}
          >
            <Plus className="h-4 w-4 mr-1" />
            Add Prize
          </Button>
        </div>
        <div className="space-y-3">
          {prizeFields.map((field, index) => (
            <div key={field.id} className="flex gap-2 items-start">
              <div className="flex-1 grid grid-cols-3 gap-2">
                <Input
                  placeholder="Place (e.g., 1st)"
                  {...register(`prizes.${index}.place`)}
                />
                <Input
                  placeholder="Title (e.g., Grand Prize)"
                  {...register(`prizes.${index}.title`)}
                />
                <Input
                  placeholder="Value (e.g., $5,000)"
                  {...register(`prizes.${index}.value`)}
                />
              </div>
              <Button
                type="button"
                variant="ghost"
                size="icon"
                className="shrink-0 text-muted-foreground hover:text-destructive"
                onClick={() => removePrize(index)}
              >
                <Trash2 className="h-4 w-4" />
              </Button>
            </div>
          ))}
          {prizeFields.length === 0 && (
            <p className="text-sm text-muted-foreground text-center py-4 border rounded-md">
              No prizes added. Click &quot;Add Prize&quot; to add prizes.
            </p>
          )}
        </div>
      </div>

      {/* Organizers Section */}
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <Label>Organizers</Label>
          <Button
            type="button"
            variant="outline"
            size="sm"
            onClick={() => appendOrganizer({ name: "", logo: "", url: "" })}
          >
            <Plus className="h-4 w-4 mr-1" />
            Add Organizer
          </Button>
        </div>
        <div className="space-y-3">
          {organizerFields.map((field, index) => (
            <div key={field.id} className="flex gap-2 items-start">
              <div className="flex-1 grid grid-cols-3 gap-2">
                <Input
                  placeholder="Name (e.g., PIAIC)"
                  {...register(`organizers.${index}.name`)}
                />
                <Input
                  placeholder="Logo URL (optional)"
                  {...register(`organizers.${index}.logo`)}
                />
                <Input
                  placeholder="Website URL (optional)"
                  {...register(`organizers.${index}.url`)}
                />
              </div>
              <Button
                type="button"
                variant="ghost"
                size="icon"
                className="shrink-0 text-muted-foreground hover:text-destructive"
                onClick={() => removeOrganizer(index)}
              >
                <Trash2 className="h-4 w-4" />
              </Button>
            </div>
          ))}
          {organizerFields.length === 0 && (
            <p className="text-sm text-muted-foreground text-center py-4 border rounded-md">
              No organizers added. Click &quot;Add Organizer&quot; to add organizers.
            </p>
          )}
        </div>
      </div>

      {/* Sponsors Section */}
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <Label>Sponsors</Label>
          <Button
            type="button"
            variant="outline"
            size="sm"
            onClick={() => appendSponsor({ name: "", logo: "", url: "", tier: "" })}
          >
            <Plus className="h-4 w-4 mr-1" />
            Add Sponsor
          </Button>
        </div>
        <div className="space-y-3">
          {sponsorFields.map((field, index) => (
            <div key={field.id} className="flex gap-2 items-start">
              <div className="flex-1 grid grid-cols-4 gap-2">
                <Input
                  placeholder="Name (e.g., Panaversity)"
                  {...register(`sponsors.${index}.name`)}
                />
                <Input
                  placeholder="Logo URL (optional)"
                  {...register(`sponsors.${index}.logo`)}
                />
                <Input
                  placeholder="Website URL (optional)"
                  {...register(`sponsors.${index}.url`)}
                />
                <select
                  className="flex h-9 w-full rounded-md border border-input bg-transparent px-3 py-1 text-sm shadow-sm transition-colors placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring"
                  {...register(`sponsors.${index}.tier`)}
                >
                  <option value="">Select Tier</option>
                  <option value="platinum">Platinum</option>
                  <option value="gold">Gold</option>
                  <option value="silver">Silver</option>
                  <option value="bronze">Bronze</option>
                </select>
              </div>
              <Button
                type="button"
                variant="ghost"
                size="icon"
                className="shrink-0 text-muted-foreground hover:text-destructive"
                onClick={() => removeSponsor(index)}
              >
                <Trash2 className="h-4 w-4" />
              </Button>
            </div>
          ))}
          {sponsorFields.length === 0 && (
            <p className="text-sm text-muted-foreground text-center py-4 border rounded-md">
              No sponsors added. Click &quot;Add Sponsor&quot; to add sponsors.
            </p>
          )}
        </div>
      </div>

      {/* Categories Section */}
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <div>
            <Label>Categories / Tracks</Label>
            <p className="text-xs text-muted-foreground mt-1">
              Optional: Add categories for different competition tracks (e.g., AI, Web3, Social Impact)
            </p>
          </div>
          <Button
            type="button"
            variant="outline"
            size="sm"
            onClick={() => appendCategory({ id: "", name: "", description: "" })}
          >
            <Plus className="h-4 w-4 mr-1" />
            Add Category
          </Button>
        </div>
        <div className="space-y-3">
          {categoryFields.map((field, index) => (
            <div key={field.id} className="flex gap-2 items-start">
              <div className="flex-1 grid grid-cols-2 gap-2">
                <Input
                  placeholder="Category Name (e.g., AI & ML)"
                  {...register(`categories.${index}.name`)}
                />
                <Input
                  placeholder="Description (optional)"
                  {...register(`categories.${index}.description`)}
                />
              </div>
              <Button
                type="button"
                variant="ghost"
                size="icon"
                className="shrink-0 text-muted-foreground hover:text-destructive"
                onClick={() => removeCategory(index)}
              >
                <Trash2 className="h-4 w-4" />
              </Button>
            </div>
          ))}
          {categoryFields.length === 0 && (
            <p className="text-sm text-muted-foreground text-center py-4 border rounded-md">
              No categories added. Leave empty for a single-track hackathon.
            </p>
          )}
        </div>
      </div>

      <div className="flex justify-end gap-2">
        <Button type="submit" disabled={isSubmitting}>
          {isSubmitting && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
          Create Hackathon
        </Button>
      </div>
    </form>
  );
}
