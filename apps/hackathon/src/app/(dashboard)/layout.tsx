import { getCurrentUser } from "@/lib/auth/session";
import { redirect } from "next/navigation";
import { Navbar } from "@/components/layout/navbar";
import { Sidebar } from "@/components/layout/sidebar";

export default async function DashboardLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const user = await getCurrentUser();

  if (!user) {
    redirect("/login");
  }

  return (
    <div className="min-h-screen bg-background relative overflow-hidden">
      {/* Dashboard Grid Background */}
      <div className="absolute inset-0 z-0 bg-grid-black/[0.02] dark:bg-grid-white/[0.02] pointer-events-none" />

      <div className="relative z-10 flex min-h-screen flex-col">
        <Navbar user={user} />
        <div className="flex flex-1">
          <Sidebar />
          <main className="flex-1 p-6 overflow-y-auto">{children}</main>
        </div>
      </div>
    </div>
  );
}
