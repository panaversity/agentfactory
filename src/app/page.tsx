import { redirect } from "next/navigation";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { LogoutButton } from "@/components/logout-button";
import { HomePageStyles } from "@/components/home-page-styles";
import Image from "next/image";

export default async function HomePage() {
  const session = await auth.api.getSession({
    headers: await headers(),
  });

  if (!session) {
    redirect("/auth/sign-in");
  }

  const appName = process.env.NEXT_PUBLIC_APP_NAME || "Panaversity SSO";
  const appDescription = process.env.NEXT_PUBLIC_APP_DESCRIPTION || "Secure Single Sign-On";
  const orgName = process.env.NEXT_PUBLIC_ORG_NAME || "Panaversity";
  const continueUrl = process.env.NEXT_PUBLIC_CONTINUE_URL || "http://localhost:3000";

  const firstName = session.user.name?.split(" ")[0] || session.user.email?.split("@")[0];

  return (
    <>
      <HomePageStyles />
      <div className="min-h-screen flex items-center justify-center py-12 px-4 relative overflow-hidden">
        {/* Animated background mesh - matches auth pages */}
        <div className="absolute inset-0 gradient-mesh opacity-60" />

        {/* Subtle grid pattern - matches auth pages */}
        <div
          className="absolute inset-0 opacity-[0.03]"
          style={{
            backgroundImage: `
              linear-gradient(to right, #0f172a 1px, transparent 1px),
              linear-gradient(to bottom, #0f172a 1px, transparent 1px)
            `,
            backgroundSize: '48px 48px'
          }}
        />

        <div className="max-w-lg w-full relative z-10">
          {/* Logo/Brand - matches auth pages */}
          <div className="text-center mb-10 animate-in slide-in-from-top">
            <div className="flex justify-center mb-4">
              <Image
                src="/logo.webp"
                alt="Panaversity"
                width={280}
                height={70}
                className="h-16 w-auto"
                priority
              />
            </div>
            <p className="text-sm text-slate-600 font-medium tracking-wide">
              {appDescription}
            </p>
            <div className="mt-3 flex items-center justify-center gap-2">
              <div className="h-px w-12 bg-gradient-to-r from-transparent via-slate-300 to-transparent" />
              <div className="w-1.5 h-1.5 rounded-full bg-pana-500" />
              <div className="h-px w-12 bg-gradient-to-r from-transparent via-slate-300 to-transparent" />
            </div>
          </div>

          {/* Main Card - glass effect matching auth pages */}
          <div className="glass-effect rounded-2xl shadow-2xl shadow-pana-500/10 p-8 md:p-10 animate-in scale-in">
            {/* Status Badge */}
            <div className="inline-flex items-center gap-2 px-3 py-1.5 rounded-full bg-pana-50 border border-pana-200 mb-6">
              <div className="w-2 h-2 rounded-full bg-pana-500 animate-pulse" />
              <span className="text-xs font-semibold text-pana-700 tracking-wide uppercase">
                Authenticated
              </span>
            </div>

            {/* User Welcome */}
            <div className="mb-8">
              <h2 className="text-2xl font-semibold text-slate-900 mb-2">
                Welcome back, {firstName}
              </h2>
              <p className="text-sm text-slate-600">
                {session.user.email}
              </p>
            </div>

            {/* Session Info */}
            <div className="bg-gradient-to-br from-pana-50 to-slate-50 rounded-xl p-5 mb-8 border border-pana-100">
              <div className="flex items-start gap-4">
                <div className="w-10 h-10 rounded-lg bg-gradient-to-br from-pana-500 to-pana-600 flex items-center justify-center flex-shrink-0 shadow-md shadow-pana-500/30">
                  <svg
                    className="w-5 h-5 text-white"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M9 12l2 2 4-4m5.618-4.016A11.955 11.955 0 0112 2.944a11.955 11.955 0 01-8.618 3.04A12.02 12.02 0 003 9c0 5.591 3.824 10.29 9 11.622 5.176-1.332 9-6.03 9-11.622 0-1.042-.133-2.052-.382-3.016z"
                    />
                  </svg>
                </div>
                <div className="flex-1">
                  <h3 className="font-semibold text-slate-900 mb-1 text-sm">
                    Secure Session Active
                  </h3>
                  <p className="text-xs text-slate-600 leading-relaxed">
                    Your identity is verified and protected by {orgName}. Access your connected applications securely.
                  </p>
                </div>
              </div>
            </div>

            {/* Action Buttons */}
            <div className="space-y-3">
              <a
                href={continueUrl}
                className="block w-full py-3.5 px-6 rounded-xl text-sm font-semibold text-white bg-gradient-to-r from-pana-500 to-pana-600 shadow-lg shadow-pana-500/30 text-center transition-all duration-200 hover:shadow-xl hover:shadow-pana-500/40 hover:-translate-y-0.5"
              >
                Continue to Application →
              </a>

              <LogoutButton className="block w-full py-3.5 px-6 rounded-xl text-sm font-medium text-slate-700 bg-white border-2 border-slate-200 text-center transition-all duration-200 hover:border-slate-300 hover:bg-slate-50" />
            </div>
          </div>

          {/* Footer - matches auth pages */}
          <div className="mt-8 text-center animate-in slide-in-from-bottom">
            <p className="text-xs text-slate-500">
              Powered by <span className="font-medium text-slate-700">{orgName}</span>
            </p>
          </div>
        </div>
      </div>
    </>
  );
}
