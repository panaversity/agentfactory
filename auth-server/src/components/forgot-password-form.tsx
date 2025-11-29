"use client";

import { useState } from "react";
import { authClient } from "@/lib/auth-client";

export function ForgotPasswordForm() {
  const [email, setEmail] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const [emailNotConfigured, setEmailNotConfigured] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!email) {
      setError("Email is required");
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const result = await authClient.requestPasswordReset({
        email,
        redirectTo: "/auth/reset-password",
      });

      if (result.error) {
        // Check if it's because email/reset is not configured
        if (result.error.message?.includes("sendResetPassword") ||
            result.error.message?.includes("not configured") ||
            result.error.message?.includes("isn't enabled") ||
            result.error.message?.includes("Reset password")) {
          setEmailNotConfigured(true);
        } else {
          setError(result.error.message || "Failed to send reset email");
        }
        return;
      }

      setSuccess(true);
    } catch (err) {
      // If the server doesn't have email configured, show appropriate message
      setEmailNotConfigured(true);
    } finally {
      setIsLoading(false);
    }
  };

  if (emailNotConfigured) {
    return (
      <div className="space-y-6">
        <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
          <h3 className="font-medium text-yellow-800 mb-2">Email Not Configured</h3>
          <p className="text-sm text-yellow-700">
            Password reset emails are not currently configured on this server.
            Please contact the administrator if you need to reset your password.
          </p>
        </div>
        <a
          href="/auth/sign-in"
          className="block text-center text-blue-600 hover:text-blue-500 font-medium"
        >
          Back to sign in
        </a>
      </div>
    );
  }

  if (success) {
    return (
      <div className="space-y-6">
        <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
          <h3 className="font-medium text-green-800 mb-2">Check your email</h3>
          <p className="text-sm text-green-700">
            If an account exists for <strong>{email}</strong>, we've sent a password reset link.
            Please check your inbox and spam folder.
          </p>
        </div>
        <a
          href="/auth/sign-in"
          className="block text-center text-blue-600 hover:text-blue-500 font-medium"
        >
          Back to sign in
        </a>
      </div>
    );
  }

  return (
    <form onSubmit={handleSubmit} className="space-y-6">
      {error && (
        <div className="p-3 bg-red-50 border border-red-200 rounded-lg">
          <p className="text-sm text-red-600">{error}</p>
        </div>
      )}

      <p className="text-sm text-gray-600">
        Enter your email address and we'll send you a link to reset your password.
      </p>

      <div>
        <label htmlFor="email" className="block text-sm font-medium text-gray-700">
          Email
        </label>
        <input
          id="email"
          type="email"
          required
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          className="mt-1 block w-full px-3 py-2 border border-gray-300 rounded-lg shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500"
          placeholder="you@example.com"
        />
      </div>

      <button
        type="submit"
        disabled={isLoading}
        className="w-full flex justify-center py-2.5 px-4 border border-transparent rounded-lg shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
      >
        {isLoading ? "Sending..." : "Send reset link"}
      </button>

      <a
        href="/auth/sign-in"
        className="block text-center text-sm text-blue-600 hover:text-blue-500 font-medium"
      >
        Back to sign in
      </a>
    </form>
  );
}
