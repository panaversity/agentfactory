import Link from "next/link";
import { Button } from "@/components/ui/button";

export default function PublicLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <div className="min-h-screen bg-[#0a0a0f]">
      {/* Minimal Public Navbar */}
      <nav className="fixed top-0 left-0 right-0 z-50 border-b border-white/5 bg-[#0a0a0f]/80 backdrop-blur-xl">
        <div className="mx-auto max-w-7xl px-4 sm:px-6 lg:px-8">
          <div className="flex h-16 items-center justify-between">
            <Link href="/" className="flex items-center gap-2">
              <div className="h-8 w-8 rounded-lg bg-gradient-to-br from-[#1cd98e] to-[#0ea66a] flex items-center justify-center">
                <span className="text-white font-bold text-sm">P</span>
              </div>
              <span className="font-semibold text-white">Panaversity</span>
            </Link>
            <div className="flex items-center gap-3">
              <Button variant="ghost" className="text-white/70 hover:text-white hover:bg-white/5" asChild>
                <Link href="/explore">Browse All</Link>
              </Button>
              <Button className="bg-[#1cd98e] hover:bg-[#19c580] text-black font-medium" asChild>
                <Link href="/login">Sign In</Link>
              </Button>
            </div>
          </div>
        </div>
      </nav>
      <main className="pt-16">{children}</main>
    </div>
  );
}
