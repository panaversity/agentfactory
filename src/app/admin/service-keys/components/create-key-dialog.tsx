"use client";

import { useState } from "react";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { ChevronDown, Shield, Check, AlertTriangle } from "lucide-react";
import {
  SCOPES,
  SCOPE_PRESETS,
  CATEGORY_LABELS,
  getScopesGroupedByCategory,
  scopesToPermissions,
  type ScopeCategory,
} from "@/lib/scopes";

interface CreateKeyDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  onCreateKey: (
    name: string,
    expiresInDays: number | null,
    permissions: Record<string, string[]> | null
  ) => Promise<void>;
}

const EXPIRATION_OPTIONS = [
  { label: "Never expires", value: null },
  { label: "30 days", value: 30 },
  { label: "90 days", value: 90 },
  { label: "1 year", value: 365 },
];

export function CreateKeyDialog({ open, onOpenChange, onCreateKey }: CreateKeyDialogProps) {
  const [name, setName] = useState("");
  const [expiresInDays, setExpiresInDays] = useState<number | null>(null);
  const [selectedScopes, setSelectedScopes] = useState<string[]>([]);
  const [selectedPreset, setSelectedPreset] = useState<string | null>(null);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const [isCreating, setIsCreating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const selectedExpiration = EXPIRATION_OPTIONS.find((opt) => opt.value === expiresInDays);
  const scopesByCategory = getScopesGroupedByCategory();

  const handlePresetSelect = (presetId: string) => {
    const preset = SCOPE_PRESETS.find((p) => p.id === presetId);
    if (preset) {
      setSelectedPreset(presetId);
      setSelectedScopes(preset.scopes);
    }
  };

  const handleScopeToggle = (scopeId: string) => {
    setSelectedPreset(null); // Clear preset when manually selecting
    setSelectedScopes((prev) =>
      prev.includes(scopeId) ? prev.filter((s) => s !== scopeId) : [...prev, scopeId]
    );
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!name.trim()) {
      setError("Name is required");
      return;
    }

    if (name.length > 100) {
      setError("Name must be 100 characters or less");
      return;
    }

    if (selectedScopes.length === 0) {
      setError("At least one scope is required");
      return;
    }

    setIsCreating(true);
    try {
      const permissions = scopesToPermissions(selectedScopes);
      await onCreateKey(name.trim(), expiresInDays, permissions);
      // Reset form on success
      setName("");
      setExpiresInDays(null);
      setSelectedScopes([]);
      setSelectedPreset(null);
      setShowAdvanced(false);
    } catch (err: unknown) {
      const message = err instanceof Error ? err.message : "Failed to create API key";
      setError(message);
    } finally {
      setIsCreating(false);
    }
  };

  const handleOpenChange = (newOpen: boolean) => {
    if (!newOpen) {
      // Reset form when closing
      setName("");
      setExpiresInDays(null);
      setSelectedScopes([]);
      setSelectedPreset(null);
      setShowAdvanced(false);
      setError(null);
    }
    onOpenChange(newOpen);
  };

  const hasSensitiveScopes = selectedScopes.some((id) => SCOPES[id]?.sensitive);

  return (
    <Dialog open={open} onOpenChange={handleOpenChange}>
      <DialogContent className="sm:max-w-[600px] max-h-[90vh] overflow-y-auto">
        <DialogHeader>
          <DialogTitle>Create Service Key</DialogTitle>
          <DialogDescription>
            Create a new API key for machine-to-machine authentication. Select the appropriate
            scopes for your use case.
          </DialogDescription>
        </DialogHeader>

        <form onSubmit={handleSubmit}>
          <div className="grid gap-4 py-4">
            {/* Name Input */}
            <div className="grid gap-2">
              <Label htmlFor="name">Name</Label>
              <Input
                id="name"
                placeholder="e.g., robolearn-backend-prod"
                value={name}
                onChange={(e) => setName(e.target.value)}
                disabled={isCreating}
              />
              <p className="text-xs text-gray-500">A descriptive name to identify this key</p>
            </div>

            {/* Expiration Dropdown */}
            <div className="grid gap-2">
              <Label>Expiration</Label>
              <DropdownMenu>
                <DropdownMenuTrigger asChild>
                  <Button
                    type="button"
                    variant="outline"
                    className="w-full justify-between"
                    disabled={isCreating}
                  >
                    {selectedExpiration?.label || "Select expiration"}
                    <ChevronDown className="h-4 w-4 ml-2 opacity-50" />
                  </Button>
                </DropdownMenuTrigger>
                <DropdownMenuContent className="w-full">
                  {EXPIRATION_OPTIONS.map((option) => (
                    <DropdownMenuItem
                      key={option.label}
                      onClick={() => setExpiresInDays(option.value)}
                    >
                      {option.label}
                    </DropdownMenuItem>
                  ))}
                </DropdownMenuContent>
              </DropdownMenu>
            </div>

            {/* Scope Presets */}
            <div className="grid gap-2">
              <Label className="flex items-center gap-2">
                <Shield className="h-4 w-4" />
                Permissions Preset
              </Label>
              <div className="grid grid-cols-2 gap-2">
                {SCOPE_PRESETS.filter((p) => p.forM2M).map((preset) => (
                  <button
                    key={preset.id}
                    type="button"
                    onClick={() => handlePresetSelect(preset.id)}
                    disabled={isCreating}
                    className={`p-3 text-left rounded-lg border transition-colors ${
                      selectedPreset === preset.id
                        ? "border-blue-500 bg-blue-50"
                        : "border-gray-200 hover:border-gray-300"
                    }`}
                  >
                    <div className="flex items-center justify-between">
                      <span className="font-medium text-sm">{preset.name}</span>
                      {selectedPreset === preset.id && (
                        <Check className="h-4 w-4 text-blue-500" />
                      )}
                    </div>
                    <p className="text-xs text-gray-500 mt-1">{preset.description}</p>
                  </button>
                ))}
              </div>
            </div>

            {/* Advanced: Manual Scope Selection */}
            <div className="border-t pt-4">
              <button
                type="button"
                onClick={() => setShowAdvanced(!showAdvanced)}
                className="text-sm text-blue-600 hover:text-blue-700 flex items-center gap-1"
              >
                <ChevronDown
                  className={`h-4 w-4 transition-transform ${showAdvanced ? "rotate-180" : ""}`}
                />
                {showAdvanced ? "Hide" : "Show"} advanced scope selection
              </button>

              {showAdvanced && (
                <div className="mt-4 space-y-4">
                  {(Object.keys(scopesByCategory) as ScopeCategory[]).map((category) => {
                    const scopes = scopesByCategory[category];
                    if (scopes.length === 0) return null;

                    return (
                      <div key={category} className="space-y-2">
                        <Label className="text-xs uppercase tracking-wider text-gray-500">
                          {CATEGORY_LABELS[category]}
                        </Label>
                        <div className="grid grid-cols-2 gap-1">
                          {scopes.map((scope) => (
                            <label
                              key={scope.id}
                              className={`flex items-start gap-2 p-2 rounded cursor-pointer hover:bg-gray-50 ${
                                selectedScopes.includes(scope.id) ? "bg-blue-50" : ""
                              }`}
                            >
                              <input
                                type="checkbox"
                                checked={selectedScopes.includes(scope.id)}
                                onChange={() => handleScopeToggle(scope.id)}
                                disabled={isCreating}
                                className="mt-0.5"
                              />
                              <div className="flex-1 min-w-0">
                                <div className="flex items-center gap-1">
                                  <span className="text-sm font-medium">{scope.name}</span>
                                  {scope.sensitive && (
                                    <AlertTriangle className="h-3 w-3 text-amber-500" />
                                  )}
                                </div>
                                <p className="text-xs text-gray-500 truncate">{scope.description}</p>
                              </div>
                            </label>
                          ))}
                        </div>
                      </div>
                    );
                  })}
                </div>
              )}
            </div>

            {/* Selected Scopes Summary */}
            {selectedScopes.length > 0 && (
              <div className="bg-gray-50 rounded-lg p-3">
                <Label className="text-xs uppercase tracking-wider text-gray-500">
                  Selected Scopes ({selectedScopes.length})
                </Label>
                <div className="flex flex-wrap gap-1 mt-2">
                  {selectedScopes.map((scopeId) => {
                    const scope = SCOPES[scopeId];
                    return (
                      <span
                        key={scopeId}
                        className={`inline-flex items-center gap-1 px-2 py-0.5 rounded text-xs ${
                          scope?.sensitive
                            ? "bg-amber-100 text-amber-800"
                            : "bg-blue-100 text-blue-800"
                        }`}
                      >
                        {scopeId}
                        {scope?.sensitive && <AlertTriangle className="h-3 w-3" />}
                      </span>
                    );
                  })}
                </div>
              </div>
            )}

            {/* Warning for sensitive scopes */}
            {hasSensitiveScopes && (
              <div className="flex items-start gap-2 p-3 bg-amber-50 border border-amber-200 rounded-lg">
                <AlertTriangle className="h-5 w-5 text-amber-500 flex-shrink-0 mt-0.5" />
                <div>
                  <p className="text-sm font-medium text-amber-800">Sensitive Permissions</p>
                  <p className="text-xs text-amber-700">
                    This key will have access to sensitive operations. Ensure it&apos;s stored
                    securely and only used in trusted environments.
                  </p>
                </div>
              </div>
            )}

            {error && (
              <div className="text-sm text-red-500 bg-red-50 p-2 rounded">{error}</div>
            )}
          </div>

          <DialogFooter>
            <Button
              type="button"
              variant="outline"
              onClick={() => handleOpenChange(false)}
              disabled={isCreating}
            >
              Cancel
            </Button>
            <Button type="submit" disabled={isCreating || selectedScopes.length === 0}>
              {isCreating ? "Creating..." : "Create Key"}
            </Button>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
}
