"use client";

import { useEffect, useState } from "react";
import { useSearchParams } from "next/navigation";
import { Suspense } from "react";

function VerifyCallbackContent() {
  const searchParams = useSearchParams();
  const error = searchParams.get("error");
  const [status, setStatus] = useState<"loading" | "success" | "error">("loading");
  const [errorMessage, setErrorMessage] = useState<string | null>(null);

  useEffect(() => {
    // Handle verification error
    if (error === "invalid_token") {
      setStatus("error");
      setErrorMessage("The verification link is invalid or has expired.");
      return;
    }

    // Check for stored OAuth params
    const storedOAuthParams = localStorage.getItem("oauth_pending_params");

    if (storedOAuthParams) {
      try {
        const oauthParams = JSON.parse(storedOAuthParams);
        // Clear stored params
        localStorage.removeItem("oauth_pending_params");

        // Continue OAuth flow
        const params = new URLSearchParams({
          client_id: oauthParams.client_id,
          redirect_uri: oauthParams.redirect_uri,
          response_type: oauthParams.response_type,
          ...(oauthParams.scope && { scope: oauthParams.scope }),
          ...(oauthParams.state && { state: oauthParams.state }),
          ...(oauthParams.code_challenge && { code_challenge: oauthParams.code_challenge }),
          ...(oauthParams.code_challenge_method && { code_challenge_method: oauthParams.code_challenge_method }),
        });

        setStatus("success");
        // Redirect to OAuth authorize endpoint
        window.location.href = `/api/auth/oauth2/authorize?${params.toString()}`;
        return;
      } catch (e) {
        // Invalid JSON, just redirect home
        console.error("Failed to parse OAuth params:", e);
      }
    }

    // No OAuth flow pending, redirect to home
    setStatus("success");
    window.location.href = "/";
  }, [error]);

  if (status === "error") {
    return (
      <div className="space-y-6">
        <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
          <h3 className="font-medium text-red-800 mb-2">Verification Failed</h3>
          <p className="text-sm text-red-700">{errorMessage}</p>
        </div>
        <a
          href="/auth/sign-in"
          className="block w-full text-center py-2.5 px-4 border border-transparent rounded-lg shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700"
        >
          Back to Sign In
        </a>
      </div>
    );
  }

  return (
    <div className="flex flex-col items-center justify-center py-8 space-y-4">
      <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
      <p className="text-sm text-gray-600">
        Email verified! Redirecting...
      </p>
    </div>
  );
}

export default function VerifyCallbackPage() {
  return (
    <div>
      <h2 className="text-2xl font-semibold text-gray-900 text-center mb-6">
        Email Verification
      </h2>
      <Suspense
        fallback={
          <div className="flex items-center justify-center py-8">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
          </div>
        }
      >
        <VerifyCallbackContent />
      </Suspense>
    </div>
  );
}
