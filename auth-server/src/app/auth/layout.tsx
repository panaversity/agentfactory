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

      <div className="max-w-sm lg:max-w-md w-full relative z-10">
        {/* Logo/Brand */}
        <div className="text-center mb-10 animate-in slide-in-from-top">
          <h1 className="text-4xl font-bold mb-2">
            <span className="text-gradient">RoboLearn</span>
          </h1>
          <p className="text-sm text-slate-600 font-medium tracking-wide uppercase">
            Physical AI & Humanoid Robotics
          </p>
          <div className="mt-3 flex items-center justify-center gap-2">
            <div className="h-px w-12 bg-gradient-to-r from-transparent via-slate-300 to-transparent" />
            <div className="w-1.5 h-1.5 rounded-full bg-indigo-500" />
            <div className="h-px w-12 bg-gradient-to-r from-transparent via-slate-300 to-transparent" />
          </div>
        </div>

        {/* Form container with glass effect */}
        <div className="glass-effect rounded-2xl shadow-2xl shadow-indigo-500/10 p-8 md:p-10 animate-in scale-in">
          {children}
        </div>

        {/* Footer */}
        <div className="mt-8 text-center animate-in slide-in-from-bottom">
          <p className="text-xs text-slate-500">
            Secure authentication powered by{' '}
            <span className="font-medium text-slate-700">Better Auth</span>
          </p>
        </div>
      </div>
    </div>
  );
}
