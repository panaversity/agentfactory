import { redirect } from "next/navigation";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { LogoutButton } from "@/components/logout-button";
import { HomePageStyles } from "@/components/home-page-styles";

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
      <div className="security-grid-bg">
        <div className="min-h-screen flex items-center justify-center py-12 px-4 relative">
          <div className="max-w-lg w-full">
            {/* Main Card */}
            <div className="card-glass rounded-3xl shadow-2xl overflow-hidden">
              {/* Header Section */}
              <div className="bg-gradient-to-br from-slate-800 to-slate-900 px-8 py-10 relative overflow-hidden">
                <div className="absolute inset-0 opacity-10">
                  <svg className="w-full h-full" xmlns="http://www.w3.org/2000/svg">
                    <defs>
                      <pattern id="dots" x="0" y="0" width="30" height="30" patternUnits="userSpaceOnUse">
                        <circle cx="2" cy="2" r="1" fill="currentColor" className="text-emerald-400" />
                      </pattern>
                    </defs>
                    <rect width="100%" height="100%" fill="url(#dots)" />
                  </svg>
                </div>

                <div className="relative">
                  {/* Status Badge */}
                  <div className="status-badge inline-flex items-center gap-2 px-4 py-2 rounded-full bg-emerald-500/20 border border-emerald-500/30 mb-6">
                    <div className="w-2 h-2 rounded-full bg-emerald-400 animate-pulse" />
                    <span className="text-xs font-medium text-emerald-200 tracking-wide uppercase">
                      Authenticated
                    </span>
                  </div>

                  {/* App Name */}
                  <h1
                    className="welcome-text text-3xl font-bold text-white mb-2"
                    style={{ fontFamily: "'Outfit', sans-serif" }}
                  >
                    {appName}
                  </h1>
                  <p className="welcome-text text-slate-300 text-sm">
                    {appDescription}
                  </p>
                </div>
              </div>

              {/* Content Section */}
              <div className="px-8 py-10">
                {/* User Welcome */}
                <div className="user-info mb-8">
                  <h2
                    className="text-2xl font-semibold text-slate-800 mb-2"
                    style={{ fontFamily: "'Outfit', sans-serif" }}
                  >
                    Welcome back, {firstName}
                  </h2>
                  <p className="text-slate-600 text-sm">
                    {session.user.email}
                  </p>
                </div>

                {/* Session Info */}
                <div className="user-info bg-slate-50 rounded-2xl p-6 mb-8 border border-slate-200">
                  <div className="flex items-start gap-4">
                    <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-emerald-500 to-teal-600 flex items-center justify-center flex-shrink-0 shadow-lg">
                      <svg
                        className="w-6 h-6 text-white"
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
                      <h3 className="font-semibold text-slate-800 mb-1">
                        Secure Session Active
                      </h3>
                      <p className="text-xs text-slate-600 leading-relaxed">
                        Your identity is verified and protected by {orgName}.
                        You can now access your connected applications securely.
                      </p>
                    </div>
                  </div>
                </div>

                {/* Action Buttons */}
                <div className="action-buttons space-y-3">
                  <a
                    href={continueUrl}
                    className="primary-button block w-full py-4 px-6 rounded-xl text-base font-semibold text-white bg-gradient-to-r from-emerald-500 to-teal-600 shadow-lg text-center relative z-10"
                    style={{ fontFamily: "'Outfit', sans-serif" }}
                  >
                    Continue to Application →
                  </a>

                  <LogoutButton className="secondary-button block w-full py-4 px-6 rounded-xl text-base font-medium text-slate-700 bg-white text-center" />
                </div>
              </div>
            </div>

            {/* Footer */}
            <div className="footer-text text-center mt-8">
              <p className="text-slate-400 text-sm">
                Powered by <span className="font-semibold text-slate-300">{orgName}</span>
              </p>
              <p className="text-slate-500 text-xs mt-2">
                Enterprise-grade authentication & authorization
              </p>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
