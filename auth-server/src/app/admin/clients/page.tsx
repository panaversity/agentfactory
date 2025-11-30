"use client";

import { useState, useEffect } from "react";

interface OAuthClient {
  id: string;
  clientId: string;
  clientSecret?: string;
  name: string | null;
  redirectUrls: string[];
  type: string | null;
  disabled: boolean | null;
  isTrusted?: boolean;
  metadata?: {
    token_endpoint_auth_method?: string;
  };
}

export default function ClientsPage() {
  const [clients, setClients] = useState<OAuthClient[]>([]);
  const [loading, setLoading] = useState(true);
  const [showCreateForm, setShowCreateForm] = useState(false);
  const [newClient, setNewClient] = useState({
    name: "",
    redirectUrls: "",
    scope: "openid profile email",
    clientType: "public" as "public" | "confidential",
  });
  const [createdClient, setCreatedClient] = useState<{
    clientId: string;
    clientSecret: string;
    isPublic: boolean;
  } | null>(null);
  const [isCreating, setIsCreating] = useState(false);
  const [deletingClientId, setDeletingClientId] = useState<string | null>(null);

  useEffect(() => {
    loadClients();
  }, []);

  const loadClients = async () => {
    try {
      const response = await fetch("/api/admin/clients");
      if (response.ok) {
        const data = await response.json();
        setClients(data.clients || []);
      }
    } catch (error) {
      console.error("Failed to load clients:", error);
    } finally {
      setLoading(false);
    }
  };

  const handleCreateClient = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsCreating(true);

    const isPublic = newClient.clientType === "public";
    const redirectUrls = newClient.redirectUrls.split("\n").filter(Boolean);

    try {
      // Use our custom registration endpoint that properly handles public clients
      const response = await fetch("/api/admin/clients/register", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          name: newClient.name,
          redirectUrls,
          scope: newClient.scope,
          clientType: newClient.clientType,
        }),
      });

      const result = await response.json();

      if (response.ok && result.success) {
        setCreatedClient({
          clientId: result.client_id,
          clientSecret: result.client_secret || "",
          isPublic,
        });

        setNewClient({ name: "", redirectUrls: "", scope: "openid profile email", clientType: "public" });
        setShowCreateForm(false);

        // Reload clients from database
        await loadClients();
      } else {
        alert(result.error || "Failed to create OAuth client");
      }
    } catch (error) {
      console.error("Failed to create client:", error);
      alert("Failed to create OAuth client. Please try again.");
    } finally {
      setIsCreating(false);
    }
  };

  const handleDeleteClient = async (clientId: string) => {
    if (!confirm(`Are you sure you want to delete the client "${clientId}"? This action cannot be undone.`)) {
      return;
    }

    setDeletingClientId(clientId);

    try {
      const response = await fetch(`/api/admin/clients?clientId=${encodeURIComponent(clientId)}`, {
        method: "DELETE",
      });

      if (response.ok) {
        await loadClients();
      } else {
        const data = await response.json();
        alert(data.error || "Failed to delete client");
      }
    } catch (error) {
      console.error("Failed to delete client:", error);
      alert("Failed to delete client. Please try again.");
    } finally {
      setDeletingClientId(null);
    }
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
      </div>
    );
  }

  return (
    <div>
      <div className="flex justify-between items-center mb-8">
        <h1 className="text-2xl font-bold text-gray-900">OAuth Clients</h1>
        <button
          onClick={() => setShowCreateForm(true)}
          className="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 text-sm font-medium"
        >
          Register New Client
        </button>
      </div>

      {/* Client Created Success Modal */}
      {createdClient && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg p-6 max-w-lg w-full mx-4 max-h-[90vh] overflow-y-auto">
            <h3 className="text-lg font-semibold text-gray-900 mb-4">
              {createdClient.isPublic ? "Public" : "Confidential"} OAuth Client Created!
            </h3>

            {createdClient.isPublic ? (
              <div className="bg-green-50 border border-green-200 rounded-lg p-4 mb-4">
                <p className="text-sm text-green-800 font-medium mb-1">
                  Public Client (PKCE)
                </p>
                <p className="text-xs text-green-700">
                  No client secret needed. Uses PKCE for security. Perfect for SPAs, mobile apps, and browser-based apps.
                </p>
              </div>
            ) : (
              <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4 mb-4">
                <p className="text-sm text-yellow-800 font-medium mb-1">
                  Confidential Client
                </p>
                <p className="text-xs text-yellow-700">
                  Save the client secret securely - it will not be shown again! Use for server-side apps only.
                </p>
              </div>
            )}

            <div className="space-y-3">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Client ID
                </label>
                <code className="block w-full p-2 bg-gray-100 rounded text-sm break-all select-all">
                  {createdClient.clientId}
                </code>
              </div>

              {!createdClient.isPublic && createdClient.clientSecret && (
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">
                    Client Secret
                  </label>
                  <code className="block w-full p-2 bg-gray-100 rounded text-sm break-all select-all">
                    {createdClient.clientSecret}
                  </code>
                </div>
              )}
            </div>

            {/* Usage Guide */}
            <div className="mt-4 p-4 bg-gray-50 rounded-lg">
              <h4 className="text-sm font-medium text-gray-900 mb-2">Quick Start</h4>
              {createdClient.isPublic ? (
                <div className="text-xs text-gray-600 space-y-2">
                  <p><strong>1. Authorization Request (with PKCE):</strong></p>
                  <code className="block p-2 bg-white rounded text-xs overflow-x-auto">
                    GET /api/auth/oauth2/authorize?<br/>
                    &nbsp;&nbsp;client_id={createdClient.clientId}<br/>
                    &nbsp;&nbsp;&redirect_uri=YOUR_CALLBACK<br/>
                    &nbsp;&nbsp;&response_type=code<br/>
                    &nbsp;&nbsp;&scope=openid profile email<br/>
                    &nbsp;&nbsp;&code_challenge=GENERATED_CHALLENGE<br/>
                    &nbsp;&nbsp;&code_challenge_method=S256
                  </code>
                  <p><strong>2. Token Exchange (no secret):</strong></p>
                  <code className="block p-2 bg-white rounded text-xs overflow-x-auto">
                    POST /api/auth/oauth2/token<br/>
                    grant_type=authorization_code<br/>
                    &code=AUTH_CODE<br/>
                    &redirect_uri=YOUR_CALLBACK<br/>
                    &client_id={createdClient.clientId}<br/>
                    &code_verifier=YOUR_VERIFIER
                  </code>
                </div>
              ) : (
                <div className="text-xs text-gray-600 space-y-2">
                  <p><strong>1. Authorization Request:</strong></p>
                  <code className="block p-2 bg-white rounded text-xs overflow-x-auto">
                    GET /api/auth/oauth2/authorize?<br/>
                    &nbsp;&nbsp;client_id={createdClient.clientId}<br/>
                    &nbsp;&nbsp;&redirect_uri=YOUR_CALLBACK<br/>
                    &nbsp;&nbsp;&response_type=code<br/>
                    &nbsp;&nbsp;&scope=openid profile email
                  </code>
                  <p><strong>2. Token Exchange (with secret):</strong></p>
                  <code className="block p-2 bg-white rounded text-xs overflow-x-auto">
                    POST /api/auth/oauth2/token<br/>
                    grant_type=authorization_code<br/>
                    &code=AUTH_CODE<br/>
                    &redirect_uri=YOUR_CALLBACK<br/>
                    &client_id={createdClient.clientId}<br/>
                    &client_secret=YOUR_SECRET
                  </code>
                </div>
              )}
            </div>

            <button
              onClick={() => setCreatedClient(null)}
              className="mt-6 w-full py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
            >
              I've saved my credentials
            </button>
          </div>
        </div>
      )}

      {/* Create Client Form Modal */}
      {showCreateForm && !createdClient && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg p-6 max-w-md w-full mx-4">
            <h3 className="text-lg font-semibold text-gray-900 mb-4">
              Register OAuth Client
            </h3>
            <form onSubmit={handleCreateClient} className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Application Name
                </label>
                <input
                  type="text"
                  value={newClient.name}
                  onChange={(e) =>
                    setNewClient({ ...newClient, name: e.target.value })
                  }
                  required
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
                  placeholder="My Application"
                />
              </div>

              {/* Client Type Selector */}
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Client Type
                </label>
                <div className="grid grid-cols-2 gap-3">
                  <button
                    type="button"
                    onClick={() => setNewClient({ ...newClient, clientType: "public" })}
                    className={`p-3 rounded-lg border-2 text-left transition-all ${
                      newClient.clientType === "public"
                        ? "border-green-500 bg-green-50"
                        : "border-gray-200 hover:border-gray-300"
                    }`}
                  >
                    <div className="flex items-center gap-2 mb-1">
                      <div className={`w-3 h-3 rounded-full ${
                        newClient.clientType === "public" ? "bg-green-500" : "bg-gray-300"
                      }`} />
                      <span className="font-medium text-sm">Public</span>
                    </div>
                    <p className="text-xs text-gray-500">
                      SPAs, mobile apps, browser-based. Uses PKCE, no secret.
                    </p>
                  </button>
                  <button
                    type="button"
                    onClick={() => setNewClient({ ...newClient, clientType: "confidential" })}
                    className={`p-3 rounded-lg border-2 text-left transition-all ${
                      newClient.clientType === "confidential"
                        ? "border-blue-500 bg-blue-50"
                        : "border-gray-200 hover:border-gray-300"
                    }`}
                  >
                    <div className="flex items-center gap-2 mb-1">
                      <div className={`w-3 h-3 rounded-full ${
                        newClient.clientType === "confidential" ? "bg-blue-500" : "bg-gray-300"
                      }`} />
                      <span className="font-medium text-sm">Confidential</span>
                    </div>
                    <p className="text-xs text-gray-500">
                      Server-side apps. Uses client secret.
                    </p>
                  </button>
                </div>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Redirect URLs (one per line)
                </label>
                <textarea
                  value={newClient.redirectUrls}
                  onChange={(e) =>
                    setNewClient({ ...newClient, redirectUrls: e.target.value })
                  }
                  required
                  rows={3}
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
                  placeholder="https://myapp.com/callback"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Scopes
                </label>
                <input
                  type="text"
                  value={newClient.scope}
                  onChange={(e) =>
                    setNewClient({ ...newClient, scope: e.target.value })
                  }
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
                  placeholder="openid profile email"
                />
              </div>
              <div className="flex gap-3 pt-4">
                <button
                  type="button"
                  onClick={() => setShowCreateForm(false)}
                  className="flex-1 py-2 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={isCreating}
                  className="flex-1 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 disabled:opacity-50"
                >
                  {isCreating ? "Creating..." : "Create Client"}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}

      {/* Clients List */}
      <div className="bg-white rounded-lg shadow overflow-hidden">
        <table className="w-full">
          <thead className="bg-gray-50">
            <tr>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Application
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Client ID
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Type
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Redirect URLs
              </th>
              <th className="px-6 py-3 text-right text-xs font-medium text-gray-500 uppercase tracking-wider">
                Actions
              </th>
            </tr>
          </thead>
          <tbody className="divide-y divide-gray-200">
            {clients.length === 0 ? (
              <tr>
                <td colSpan={5} className="px-6 py-8 text-center text-gray-500">
                  No OAuth clients registered yet. Click "Register New Client" to create one.
                </td>
              </tr>
            ) : (
              clients.map((client) => (
                <tr key={client.id} className="hover:bg-gray-50">
                  <td className="px-6 py-4 whitespace-nowrap">
                    <div className="flex items-center">
                      <div className="flex-shrink-0 h-10 w-10 bg-blue-100 rounded-lg flex items-center justify-center">
                        <svg
                          className="w-5 h-5 text-blue-600"
                          fill="none"
                          stroke="currentColor"
                          viewBox="0 0 24 24"
                        >
                          <path
                            strokeLinecap="round"
                            strokeLinejoin="round"
                            strokeWidth={2}
                            d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z"
                          />
                        </svg>
                      </div>
                      <div className="ml-4">
                        <div className="text-sm font-medium text-gray-900 flex items-center gap-2">
                          {client.name || "Unnamed Client"}
                          {client.isTrusted && (
                            <span className="px-1.5 py-0.5 text-xs font-medium bg-purple-100 text-purple-800 rounded">
                              Trusted
                            </span>
                          )}
                        </div>
                        {client.disabled && (
                          <span className="text-xs text-red-500">Disabled</span>
                        )}
                      </div>
                    </div>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap">
                    <code className="text-sm bg-gray-100 px-2 py-1 rounded">
                      {client.clientId}
                    </code>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap">
                    {(() => {
                      // Determine if public or confidential based on metadata or type
                      const isPublic = client.type === "public" ||
                        client.metadata?.token_endpoint_auth_method === "none";
                      return (
                        <span
                          className={`px-2 py-1 text-xs font-medium rounded ${
                            isPublic
                              ? "bg-green-100 text-green-800"
                              : "bg-blue-100 text-blue-800"
                          }`}
                        >
                          {isPublic ? "Public (PKCE)" : "Confidential"}
                        </span>
                      );
                    })()}
                  </td>
                  <td className="px-6 py-4">
                    <div className="text-sm text-gray-500 max-w-xs truncate">
                      {Array.isArray(client.redirectUrls)
                        ? client.redirectUrls.join(", ")
                        : client.redirectUrls}
                    </div>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap text-right">
                    {client.isTrusted ? (
                      <span className="text-xs text-gray-400">Pre-configured</span>
                    ) : (
                      <button
                        onClick={() => handleDeleteClient(client.clientId)}
                        disabled={deletingClientId === client.clientId}
                        className="text-red-600 hover:text-red-800 text-sm font-medium disabled:opacity-50"
                      >
                        {deletingClientId === client.clientId ? "Deleting..." : "Delete"}
                      </button>
                    )}
                  </td>
                </tr>
              ))
            )}
          </tbody>
        </table>
      </div>

      {/* OAuth Endpoints Info */}
      <div className="mt-8 bg-white rounded-lg shadow p-6">
        <h2 className="text-lg font-semibold text-gray-900 mb-4">
          OAuth 2.0 / OIDC Endpoints
        </h2>
        <div className="space-y-3">
          <EndpointRow
            name="Authorization Endpoint"
            url={`${process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001"}/api/auth/oauth2/authorize`}
          />
          <EndpointRow
            name="Token Endpoint"
            url={`${process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001"}/api/auth/oauth2/token`}
          />
          <EndpointRow
            name="UserInfo Endpoint"
            url={`${process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001"}/api/auth/oauth2/userinfo`}
          />
          <EndpointRow
            name="OIDC Discovery"
            url={`${process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001"}/.well-known/openid-configuration`}
          />
        </div>
      </div>
    </div>
  );
}

function EndpointRow({ name, url }: { name: string; url: string }) {
  return (
    <div className="flex items-center justify-between py-2 border-b border-gray-100 last:border-0">
      <span className="text-sm font-medium text-gray-700">{name}</span>
      <code className="text-sm bg-gray-100 px-2 py-1 rounded text-gray-600">
        {url}
      </code>
    </div>
  );
}
