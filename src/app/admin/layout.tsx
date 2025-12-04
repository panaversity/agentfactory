"use client";

import { useSession } from "@/lib/auth-client";
import { useRouter } from "next/navigation";
import { useEffect } from "react";
import Link from "next/link";

export default function AdminLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const { data: session, isPending } = useSession();
  const router = useRouter();

  useEffect(() => {
    if (!isPending && !session) {
      router.push("/auth/sign-in?redirect=/admin");
    }
    // Check if user is admin
    if (!isPending && session && (session.user as any).role !== "admin") {
      router.push("/?error=unauthorized");
    }
  }, [session, isPending, router]);

  if (isPending) {
    return (
      <div className="min-h-screen bg-gray-50 flex items-center justify-center">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-pana-500"></div>
      </div>
    );
  }

  if (!session || (session.user as any).role !== "admin") {
    return (
      <div className="min-h-screen bg-gray-50 flex items-center justify-center">
        <div className="text-center">
          <h1 className="text-2xl font-bold text-gray-900 mb-2">Access Denied</h1>
          <p className="text-gray-600 mb-4">You need admin privileges to access this page.</p>
          <Link href="/" className="text-pana-500 hover:text-pana-500">
            Go back home
          </Link>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-gray-50">
      {/* Admin Header */}
      <header className="bg-white shadow-sm border-b">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center h-16">
            <div className="flex items-center gap-8">
              <Link href="/admin" className="text-xl font-bold text-gray-900">
                Panaversity Admin
              </Link>
              <nav className="hidden md:flex gap-6">
                <Link
                  href="/admin"
                  className="text-sm font-medium text-gray-600 hover:text-gray-900"
                >
                  Dashboard
                </Link>
                <Link
                  href="/admin/users"
                  className="text-sm font-medium text-gray-600 hover:text-gray-900"
                >
                  Users
                </Link>
                <Link
                  href="/admin/clients"
                  className="text-sm font-medium text-gray-600 hover:text-gray-900"
                >
                  OAuth Clients
                </Link>
              </nav>
            </div>
            <div className="flex items-center gap-4">
              <span className="text-sm text-gray-600">
                {session.user.name || session.user.email}
              </span>
              <span className="px-2 py-1 text-xs font-medium bg-pana-100 text-pana-700 rounded">
                Admin
              </span>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        {children}
      </main>
    </div>
  );
}
