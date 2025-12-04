import Image from "next/image";
import Link from "next/link";

export default function AccountLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 via-white to-pana-50/30">
      {/* Header with Logo */}
      <header className="sticky top-0 z-40 bg-white/80 backdrop-blur-md border-b border-slate-200/50">
        <div className="max-w-5xl mx-auto px-4 py-2">
          <div className="flex items-center justify-between">
            <Link href="/" className="flex items-center hover:opacity-80 transition-opacity">
              <Image
                src="/logo.webp"
                alt="Panaversity"
                width={240}
                height={60}
                className="h-14 w-auto"
                priority
              />
            </Link>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main>
        {children}
      </main>

      {/* Footer */}
      <footer className="py-6 text-center border-t border-slate-200/50 mt-auto">
        <p className="text-xs text-slate-400">
          © {new Date().getFullYear()} {process.env.NEXT_PUBLIC_ORG_NAME || "Panaversity"}. All rights reserved.
        </p>
      </footer>
    </div>
  );
}
