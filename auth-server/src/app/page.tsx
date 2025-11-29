import { redirect } from "next/navigation";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { LogoutButton } from "@/components/logout-button";

export default async function HomePage() {
  const session = await auth.api.getSession({
    headers: await headers(),
  });

  if (!session) {
    redirect("/auth/sign-in");
  }

  // If logged in, show a simple dashboard or redirect to book
  return (
    <div className="min-h-screen flex items-center justify-center py-12 px-4">
      <div className="max-w-md w-full text-center">
        <div className="bg-white py-8 px-6 shadow-lg rounded-xl">
          <h1 className="text-2xl font-bold text-gray-900 mb-4">
            Welcome, {session.user.name || session.user.email}!
          </h1>
          <p className="text-gray-600 mb-6">
            You are signed in to RoboLearn.
          </p>
          <div className="space-y-4">
            <a
              href={process.env.NEXT_PUBLIC_BOOK_URL || "http://localhost:3000"}
              className="block w-full py-2.5 px-4 border border-transparent rounded-lg shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700"
            >
              Go to Book
            </a>
            <LogoutButton className="block w-full py-2.5 px-4 border border-gray-300 rounded-lg text-sm font-medium text-gray-700 hover:bg-gray-50" />
          </div>
        </div>
      </div>
    </div>
  );
}
