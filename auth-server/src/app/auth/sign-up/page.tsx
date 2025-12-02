import { SignUpForm } from "@/components/sign-up-form";
import { Suspense } from "react";
import { redirect } from "next/navigation";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";

export default async function SignUpPage({
  searchParams,
}: {
  searchParams: Promise<{ redirect?: string }>;
}) {
  // Redirect authenticated users away from sign-up page
  const headersList = await headers();
  const session = await auth.api.getSession({
    headers: headersList,
  });

  if (session) {
    // If user is already logged in and there's an OAuth redirect, go to OAuth flow
    // Otherwise redirect to home
    const params = await searchParams;
    const redirectUrl = params.redirect || "/";
    redirect(redirectUrl);
  }

  return (
    <Suspense
      fallback={
        <div className="flex items-center justify-center py-8">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-indigo-600"></div>
        </div>
      }
    >
      <SignUpForm />
    </Suspense>
  );
}
