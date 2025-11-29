"use client";

import { useSearchParams } from "next/navigation";
import { useState, useEffect, Suspense } from "react";
import { oauth2, useSession } from "@/lib/auth-client";

function ConsentContent() {
  const searchParams = useSearchParams();
  const { data: session, isPending } = useSession();
  const [isLoading, setIsLoading] = useState(false);
  const [clientInfo, setClientInfo] = useState<{
    name: string;
    scopes: string[];
  } | null>(null);

  const consentCode = searchParams.get("consent_code");
  const clientName = searchParams.get("client_name") || "Unknown Application";
  const scopeParam = searchParams.get("scope") || "openid profile email";
  const scopes = scopeParam.split(" ").filter(Boolean);

  useEffect(() => {
    setClientInfo({
      name: clientName,
      scopes: scopes,
    });
  }, [clientName, scopeParam]);

  const handleConsent = async (accept: boolean) => {
    if (!consentCode) return;

    setIsLoading(true);
    try {
      const result = await oauth2.consent({
        accept,
        consent_code: consentCode,
      });

      if (result.data?.redirectTo) {
        window.location.href = result.data.redirectTo;
      }
    } catch (error) {
      console.error("Consent error:", error);
    } finally {
      setIsLoading(false);
    }
  };

  if (isPending) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
      </div>
    );
  }

  if (!session) {
    // Redirect to sign-in if not authenticated
    window.location.href = `/auth/sign-in?redirect=${encodeURIComponent(window.location.href)}`;
    return null;
  }

  const scopeDescriptions: Record<string, string> = {
    openid: "Verify your identity",
    profile: "Access your name and profile picture",
    email: "Access your email address",
    offline_access: "Access your data when you're not using the app",
  };

  return (
    <div className="max-w-md mx-auto">
      <div className="bg-white rounded-2xl shadow-lg p-8">
        <h2 className="text-2xl font-semibold text-gray-900 text-center mb-2">
          Authorization Request
        </h2>
        <p className="text-gray-600 text-center mb-6">
          <span className="font-medium text-blue-600">{clientInfo?.name}</span> wants
          to access your account
        </p>

        <div className="bg-gray-50 rounded-lg p-4 mb-6">
          <p className="text-sm font-medium text-gray-700 mb-3">
            This application is requesting permission to:
          </p>
          <ul className="space-y-2">
            {clientInfo?.scopes.map((scope) => (
              <li key={scope} className="flex items-center gap-2 text-sm text-gray-600">
                <svg
                  className="w-4 h-4 text-blue-500"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M5 13l4 4L19 7"
                  />
                </svg>
                {scopeDescriptions[scope] || scope}
              </li>
            ))}
          </ul>
        </div>

        <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-3 mb-6">
          <p className="text-xs text-yellow-800">
            By authorizing, you allow this application to access the
            information listed above. You can revoke access at any time.
          </p>
        </div>

        <div className="flex gap-3">
          <button
            onClick={() => handleConsent(false)}
            disabled={isLoading}
            className="flex-1 py-2.5 px-4 border border-gray-300 rounded-lg text-sm font-medium text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-gray-500 disabled:opacity-50"
          >
            Deny
          </button>
          <button
            onClick={() => handleConsent(true)}
            disabled={isLoading}
            className="flex-1 py-2.5 px-4 border border-transparent rounded-lg text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 disabled:opacity-50"
          >
            {isLoading ? "Authorizing..." : "Authorize"}
          </button>
        </div>

        <p className="text-xs text-gray-500 text-center mt-4">
          Signed in as {session.user.email}
        </p>
      </div>
    </div>
  );
}

export default function ConsentPage() {
  return (
    <Suspense
      fallback={
        <div className="flex items-center justify-center min-h-screen">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
        </div>
      }
    >
      <ConsentContent />
    </Suspense>
  );
}
