"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { authClient } from "@/lib/auth-client";

interface LogoutButtonProps {
  className?: string;
}

export function LogoutButton({ className }: LogoutButtonProps) {
  const router = useRouter();
  const [isLoading, setIsLoading] = useState(false);

  const handleLogout = async () => {
    setIsLoading(true);
    try {
      // Use the authClient.signOut method directly with fetchOptions
      await authClient.signOut({
        fetchOptions: {
          onSuccess: () => {
            // Clear any client-side state and redirect
            window.location.href = "/auth/sign-in";
          },
        },
      });
    } catch (error) {
      console.error("Logout failed:", error);
      // Force redirect even on error
      window.location.href = "/auth/sign-in";
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <button
      onClick={handleLogout}
      disabled={isLoading}
      className={`text-sm font-medium text-gray-600 hover:text-gray-900 disabled:opacity-50 ${className || ""}`}
    >
      {isLoading ? "Signing out..." : "Sign Out"}
    </button>
  );
}
