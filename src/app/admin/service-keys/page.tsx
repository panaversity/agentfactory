"use client";

import { useState, useEffect, useCallback } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { ApiKeyList } from "./components/api-key-list";
import { CreateKeyDialog } from "./components/create-key-dialog";
import { KeyDisplayDialog } from "./components/key-display-dialog";
import { RevokeKeyDialog } from "./components/revoke-key-dialog";
import { DeleteKeyDialog } from "./components/delete-key-dialog";
import { Plus, Key, RefreshCw } from "lucide-react";

// Type for API key returned from Better Auth
interface ApiKey {
  id: string;
  name: string | null;
  start: string | null;
  prefix: string | null;
  userId: string;
  enabled: boolean;
  expiresAt: Date | null;
  createdAt: Date;
  lastRequest: Date | null;
  metadata: Record<string, unknown> | null;
}

export default function ServiceKeysPage() {
  const [apiKeys, setApiKeys] = useState<ApiKey[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Dialog states
  const [createDialogOpen, setCreateDialogOpen] = useState(false);
  const [displayDialogOpen, setDisplayDialogOpen] = useState(false);
  const [revokeDialogOpen, setRevokeDialogOpen] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);

  // Selected key for actions
  const [newKeyValue, setNewKeyValue] = useState<string>("");
  const [selectedKey, setSelectedKey] = useState<ApiKey | null>(null);

  const fetchApiKeys = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    try {
      // Use admin-only endpoint instead of Better Auth client
      const response = await fetch("/api/admin/api-keys/list", {
        credentials: "include",
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.error || `HTTP ${response.status}`);
      }

      const data = await response.json();
      setApiKeys((data as ApiKey[]) || []);
    } catch (err: any) {
      setError(err.message || "Failed to fetch API keys");
      setApiKeys([]);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchApiKeys();
  }, [fetchApiKeys]);

  const handleCreateKey = async (name: string, expiresInDays: number | null) => {
    // Convert days to seconds (Better Auth expects seconds)
    const expiresInSeconds = expiresInDays ? expiresInDays * 24 * 60 * 60 : undefined;

    // Use admin-only endpoint
    const response = await fetch("/api/admin/api-keys/create", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify({
        name,
        expiresIn: expiresInSeconds,
      }),
    });

    if (!response.ok) {
      const data = await response.json();
      throw new Error(data.error || "Failed to create API key");
    }

    const data = await response.json();

    // Store the key value to display
    setNewKeyValue(data?.key || "");
    setCreateDialogOpen(false);
    setDisplayDialogOpen(true);

    // Refresh the list
    await fetchApiKeys();
  };

  const handleRevokeKey = async (keyId: string) => {
    // Use admin-only endpoint
    const response = await fetch("/api/admin/api-keys/update", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify({ keyId, enabled: false }),
    });

    if (!response.ok) {
      const data = await response.json();
      throw new Error(data.error || "Failed to revoke API key");
    }

    setRevokeDialogOpen(false);
    setSelectedKey(null);
    await fetchApiKeys();
  };

  const handleDeleteKey = async (keyId: string) => {
    // Use admin-only endpoint
    const response = await fetch("/api/admin/api-keys/delete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify({ keyId }),
    });

    if (!response.ok) {
      const data = await response.json();
      throw new Error(data.error || "Failed to delete API key");
    }

    setDeleteDialogOpen(false);
    setSelectedKey(null);
    await fetchApiKeys();
  };

  const openRevokeDialog = (key: ApiKey) => {
    setSelectedKey(key);
    setRevokeDialogOpen(true);
  };

  const openDeleteDialog = (key: ApiKey) => {
    setSelectedKey(key);
    setDeleteDialogOpen(true);
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-2xl font-bold text-gray-900">Service Keys</h1>
          <p className="text-sm text-gray-600 mt-1">
            Manage API keys for machine-to-machine authentication
          </p>
        </div>
        <div className="flex gap-2">
          <Button
            variant="outline"
            size="sm"
            onClick={fetchApiKeys}
            disabled={isLoading}
          >
            <RefreshCw className={`h-4 w-4 mr-2 ${isLoading ? 'animate-spin' : ''}`} />
            Refresh
          </Button>
          <Button onClick={() => setCreateDialogOpen(true)}>
            <Plus className="h-4 w-4 mr-2" />
            Create New Key
          </Button>
        </div>
      </div>

      {/* Info Card */}
      <Card>
        <CardHeader className="pb-3">
          <div className="flex items-center gap-2">
            <Key className="h-5 w-5 text-pana-500" />
            <CardTitle className="text-lg">API Keys</CardTitle>
          </div>
          <CardDescription>
            API keys allow external services to authenticate with your SSO server.
            Keys are hashed and cannot be retrieved after creation.
          </CardDescription>
        </CardHeader>
        <CardContent>
          {error ? (
            <div className="text-center py-8">
              <p className="text-red-500 mb-4">{error}</p>
              <Button variant="outline" onClick={fetchApiKeys}>
                Try Again
              </Button>
            </div>
          ) : (
            <ApiKeyList
              apiKeys={apiKeys}
              isLoading={isLoading}
              onRevoke={openRevokeDialog}
              onDelete={openDeleteDialog}
            />
          )}
        </CardContent>
      </Card>

      {/* Dialogs */}
      <CreateKeyDialog
        open={createDialogOpen}
        onOpenChange={setCreateDialogOpen}
        onCreateKey={handleCreateKey}
      />

      <KeyDisplayDialog
        open={displayDialogOpen}
        onOpenChange={setDisplayDialogOpen}
        apiKey={newKeyValue}
      />

      {selectedKey && (
        <>
          <RevokeKeyDialog
            open={revokeDialogOpen}
            onOpenChange={setRevokeDialogOpen}
            apiKey={selectedKey}
            onConfirm={() => handleRevokeKey(selectedKey.id)}
          />

          <DeleteKeyDialog
            open={deleteDialogOpen}
            onOpenChange={setDeleteDialogOpen}
            apiKey={selectedKey}
            onConfirm={() => handleDeleteKey(selectedKey.id)}
          />
        </>
      )}
    </div>
  );
}
