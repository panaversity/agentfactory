"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { cn } from "@/lib/utils";
import { LayoutDashboard, Trophy, Users, Terminal } from "lucide-react";

const navigation = [
  {
    name: "Command Center",
    href: "/dashboard",
    icon: LayoutDashboard,
  },
  {
    name: "Agent Specs",
    href: "/hackathons",
    icon: Trophy,
  },
  {
    name: "Engineering Units",
    href: "/teams",
    icon: Users,
  },
];

export function Sidebar() {
  const pathname = usePathname();

  return (
    <aside className="hidden w-64 shrink-0 border-r border-brand-tech-blue/20 bg-background/50 backdrop-blur-xl md:block relative">
      {/* Tech Decorators */}
      <div className="absolute top-0 right-0 w-[1px] h-20 bg-gradient-to-b from-brand-tech-blue to-transparent opacity-50" />
      <div className="absolute bottom-0 left-0 w-full h-[1px] bg-gradient-to-r from-transparent via-brand-tech-blue/20 to-transparent" />

      <nav className="flex flex-col gap-2 p-4 mt-6">
        <div className="px-3 py-2">
          <h2 className="text-xs font-mono text-muted-foreground uppercase tracking-widest mb-4">
            Factory Navigation //
          </h2>
        </div>

        {navigation.map((item) => {
          const isActive =
            pathname === item.href ||
            (item.href !== "/dashboard" && pathname.startsWith(item.href));

          return (
            <Link
              key={item.name}
              href={item.href}
              className={cn(
                "group flex items-center gap-3 rounded-none border-l-2 px-3 py-3 text-sm font-medium transition-all duration-200",
                isActive
                  ? "bg-brand-brand-tech-blue/5 border-brand-neon-cyan text-brand-neon-cyan shadow-[inset_10px_0_20px_-10px_rgba(0,240,255,0.1)]"
                  : "border-transparent text-muted-foreground hover:bg-white/5 hover:text-foreground hover:border-white/20"
              )}
            >
              <item.icon className={cn("h-4 w-4 transition-transform group-hover:scale-110", isActive ? "text-brand-neon-cyan" : "text-muted-foreground")} />
              <span className="font-mono tracking-tight uppercase text-xs">{item.name}</span>
            </Link>
          );
        })}
      </nav>

      <div className="absolute bottom-4 left-4 right-4 p-4 border border-white/5 bg-black/20 backdrop-blur-sm">
        <div className="flex items-center gap-3 text-xs font-mono text-muted-foreground">
          <Terminal className="w-3 h-3 text-brand-emerald" />
          <span>SYSTEM: ONLINE</span>
        </div>
        <div className="mt-2 h-1 w-full bg-white/5 overflow-hidden">
          <div className="h-full bg-brand-emerald/50 w-2/3 animate-pulse"></div>
        </div>
      </div>
    </aside>
  );
}
