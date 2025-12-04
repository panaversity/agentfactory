import Image from "next/image";

export default function AuthLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <div className="min-h-screen flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8 relative overflow-hidden">
      {/* Animated background mesh */}
      <div className="absolute inset-0 gradient-mesh opacity-60" />

      {/* Subtle grid pattern */}
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

      <div className="max-w-md lg:max-w-lg w-full relative z-10">
        {/* Logo/Brand */}
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
            {process.env.NEXT_PUBLIC_APP_DESCRIPTION || "Secure Single Sign-On"}
          </p>
          <div className="mt-3 flex items-center justify-center gap-2">
            <div className="h-px w-12 bg-gradient-to-r from-transparent via-slate-300 to-transparent" />
            <div className="w-1.5 h-1.5 rounded-full bg-pana-500" />
            <div className="h-px w-12 bg-gradient-to-r from-transparent via-slate-300 to-transparent" />
          </div>
        </div>

        {/* Form container with glass effect */}
        <div className="glass-effect rounded-2xl shadow-2xl shadow-pana-500/10 p-8 md:p-10 animate-in scale-in">
          {children}
        </div>

        {/* Footer */}
        <div className="mt-8 text-center animate-in slide-in-from-bottom">
          <p className="text-xs text-slate-400">
            © {new Date().getFullYear()} {process.env.NEXT_PUBLIC_ORG_NAME || "Panaversity"}
          </p>
        </div>
      </div>
    </div>
  );
}
