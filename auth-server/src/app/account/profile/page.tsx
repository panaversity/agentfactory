import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { redirect } from "next/navigation";
import ProfileForm from "./ProfileForm";

export default async function ProfilePage({
  searchParams,
}: {
  searchParams: Promise<{ redirect?: string }>;
}) {
  const session = await auth.api.getSession({
    headers: await headers(),
  });

  if (!session) {
    redirect("/auth/sign-in");
  }

  const params = await searchParams;
  const redirectUrl = params.redirect || null;

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 via-white to-indigo-50/30 py-8 px-4">
      <div className="max-w-3xl mx-auto">
        {/* Refined Header Section */}
        <div className="mb-8">
          {/* Back Button - Floating Above */}
          {redirectUrl && (
            <a
              href={redirectUrl}
              className="inline-flex items-center gap-2 mb-6 group text-slate-600 hover:text-slate-900 transition-colors duration-200"
            >
              <div className="flex items-center justify-center w-8 h-8 rounded-lg bg-white border border-slate-200 group-hover:border-slate-300 group-hover:shadow-sm transition-all duration-200">
                <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 19l-7-7m0 0l7-7m-7 7h18" />
                </svg>
              </div>
              <span className="text-sm font-medium">Back</span>
            </a>
          )}

          {/* Hero Header */}
          <div className="relative">
            {/* Decorative accent */}
            <div className="absolute -left-3 top-0 bottom-0 w-1 bg-gradient-to-b from-indigo-500 to-indigo-600 rounded-full" />

            <div className="pl-6">
              <h1 className="text-4xl font-bold text-slate-900 tracking-tight mb-3 bg-gradient-to-r from-slate-900 to-slate-700 bg-clip-text">
                Profile Settings
              </h1>
              <p className="text-base text-slate-600 leading-relaxed max-w-xl">
                Manage your account details and personalize your learning experience
              </p>
            </div>
          </div>
        </div>

        {/* Form Card with Enhanced Shadow */}
        <div className="relative">
          {/* Subtle glow effect */}
          <div className="absolute inset-0 bg-gradient-to-r from-indigo-100/20 via-transparent to-purple-100/20 rounded-2xl blur-2xl -z-10" />

          <div className="bg-white/80 backdrop-blur-sm rounded-2xl shadow-xl shadow-slate-200/50 border border-slate-200/50 p-8 md:p-10">
            <ProfileForm user={session.user} redirectUrl={redirectUrl} />
          </div>
        </div>
      </div>
    </div>
  );
}
