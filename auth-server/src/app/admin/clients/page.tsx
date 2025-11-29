"use client";

import { useState } from "react";
import { oauth2 } from "@/lib/auth-client";

interface OAuthClient {
  clientId: string;
  clientSecret?: string;
  name: string;
  redirectURLs: string[];
  type: string;
}

export default function ClientsPage() {
  const [clients, setClients] = useState<OAuthClient[]>([
    {
      clientId: "robolearn-interface",
      name: "RoboLearn Book Interface",
      redirectURLs: [
        "http://localhost:3000/api/auth/callback",
        "http://localhost:3000/auth/callback",
      ],
      type: "trusted",
    },
  ]);
  const [showCreateForm, setShowCreateForm] = useState(false);
  const [newClient, setNewClient] = useState({
    name: "",
    redirectUrls: "",
    scope: "openid profile email",
  });
  const [createdClient, setCreatedClient] = useState<{
    clientId: string;
    clientSecret: string;
  } | null>(null);
  const [isCreating, setIsCreating] = useState(false);

  const handleCreateClient = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsCreating(true);

    try {
      const result = await oauth2.register({
        redirect_uris: newClient.redirectUrls.split("\n").filter(Boolean),
        client_name: newClient.name,
        scope: newClient.scope,
      });

      if (result.data) {
        setCreatedClient({
          clientId: result.data.client_id,
          clientSecret: result.data.client_secret,
        });

        // Add to local list
        setClients([
          ...clients,
          {
            clientId: result.data.client_id,
            clientSecret: result.data.client_secret,
            name: newClient.name,
            redirectURLs: newClient.redirectUrls.split("\n").filter(Boolean),
            type: "dynamic",
          },
        ]);

        setNewClient({ name: "", redirectUrls: "", scope: "openid profile email" });
      }
    } catch (error) {
      console.error("Failed to create client:", error);
      alert("Failed to create OAuth client. Please try again.");
    } finally {
      setIsCreating(false);
    }
  };

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
          <div className="bg-white rounded-lg p-6 max-w-md w-full mx-4">
            <h3 className="text-lg font-semibold text-gray-900 mb-4">
              OAuth Client Created!
            </h3>
            <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4 mb-4">
              <p className="text-sm text-yellow-800 font-medium mb-2">
                Save these credentials - the secret will not be shown again!
              </p>
            </div>
            <div className="space-y-3">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Client ID
                </label>
                <code className="block w-full p-2 bg-gray-100 rounded text-sm break-all">
                  {createdClient.clientId}
                </code>
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Client Secret
                </label>
                <code className="block w-full p-2 bg-gray-100 rounded text-sm break-all">
                  {createdClient.clientSecret}
                </code>
              </div>
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
            </tr>
          </thead>
          <tbody className="divide-y divide-gray-200">
            {clients.map((client) => (
              <tr key={client.clientId} className="hover:bg-gray-50">
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
                      <div className="text-sm font-medium text-gray-900">
                        {client.name}
                      </div>
                    </div>
                  </div>
                </td>
                <td className="px-6 py-4 whitespace-nowrap">
                  <code className="text-sm bg-gray-100 px-2 py-1 rounded">
                    {client.clientId}
                  </code>
                </td>
                <td className="px-6 py-4 whitespace-nowrap">
                  <span
                    className={`px-2 py-1 text-xs font-medium rounded ${
                      client.type === "trusted"
                        ? "bg-green-100 text-green-800"
                        : "bg-blue-100 text-blue-800"
                    }`}
                  >
                    {client.type === "trusted" ? "Trusted (First-party)" : "Dynamic"}
                  </span>
                </td>
                <td className="px-6 py-4">
                  <div className="text-sm text-gray-500 max-w-xs truncate">
                    {client.redirectURLs.join(", ")}
                  </div>
                </td>
              </tr>
            ))}
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
