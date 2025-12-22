"use client";

import { useState } from "react";
import { Button } from "@/components/ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog";
import { CreateHackathonForm } from "./create-form";
import { Plus } from "lucide-react";

interface CreateHackathonButtonProps {
  organizationId: string;
}

export function CreateHackathonButton({ organizationId }: CreateHackathonButtonProps) {
  const [open, setOpen] = useState(false);

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button>
          <Plus className="mr-2 h-4 w-4" />
          Create Hackathon
        </Button>
      </DialogTrigger>
      <DialogContent className="max-w-2xl max-h-[90vh] overflow-y-auto">
        <DialogHeader>
          <DialogTitle>Create New Hackathon</DialogTitle>
          <DialogDescription>
            Set up a new hackathon for your organization. You can edit these
            details later.
          </DialogDescription>
        </DialogHeader>
        <CreateHackathonForm onSuccess={() => setOpen(false)} organizationId={organizationId} />
      </DialogContent>
    </Dialog>
  );
}
