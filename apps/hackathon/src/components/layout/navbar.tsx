"use client";

import Link from "next/link";
import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { LogOut, User, Cpu } from "lucide-react";

interface NavbarProps {
  user: {
    id: string;
    name: string;
    email: string;
    organizationName?: string;
  };
}

export function Navbar({ user }: NavbarProps) {
  return (
    <header className="sticky top-0 z-50 w-full border-b border-brand-tech-blue/20 bg-background/80 backdrop-blur-md supports-[backdrop-filter]:bg-background/60">
      <div className="flex h-14 items-center px-6">
        {/* Logo */}
        <Link href="/dashboard" className="flex items-center space-x-2 mr-8">
          <div className="h-8 w-8 bg-brand-obsidian border border-brand-neon-cyan/50 flex items-center justify-center">
            <Cpu className="h-5 w-5 text-brand-neon-cyan animate-pulse-slow" />
          </div>
          <div className="flex flex-col">
            <span className="font-bold tracking-widest uppercase text-sm leading-none">
              AI Agent
            </span>
            <span className="text-[10px] text-brand-tech-blue font-mono tracking-[0.2em] leading-none">
              FACTORY
            </span>
          </div>
        </Link>

        {/* Organization badge -> Unit Indicator */}
        {user.organizationName && (
          <div className="hidden md:flex items-center gap-2 px-3 py-1 border-l border-white/10 ml-4">
            <span className="text-[10px] uppercase text-muted-foreground font-mono">Unit:</span>
            <span className="text-xs font-mono text-brand-neon-cyan tracking-wider">
              {user.organizationName}
            </span>
          </div>
        )}

        {/* Spacer */}
        <div className="flex-1" />

        {/* User menu */}
        <div className="flex items-center gap-4">
          <div className="hidden md:block text-right">
            <div className="text-xs font-bold uppercase tracking-wider">{user.name}</div>
            <div className="text-[10px] font-mono text-muted-foreground">ID: {user.id.slice(0, 8)}</div>
          </div>

          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="ghost" className="relative h-9 w-9 rounded-none border border-brand-tech-blue/30 hover:bg-brand-tech-blue/10 hover:border-brand-neon-cyan transition-colors">
                <div className="flex h-full w-full items-center justify-center bg-transparent">
                  <span className="font-mono text-sm font-bold text-brand-neon-cyan">
                    {user.name.charAt(0).toUpperCase()}
                  </span>
                </div>
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent className="w-56 border-brand-tech-blue/30 bg-background/95 backdrop-blur-xl" align="end" forceMount>
              <DropdownMenuLabel className="font-normal">
                <div className="flex flex-col space-y-1">
                  <p className="text-sm font-medium leading-none font-mono">OPERATOR_PROFILE</p>
                  <p className="text-xs leading-none text-muted-foreground">
                    {user.email}
                  </p>
                </div>
              </DropdownMenuLabel>
              <DropdownMenuSeparator className="bg-white/10" />
              <DropdownMenuItem asChild className="focus:bg-brand-tech-blue/20 focus:text-brand-neon-cyan cursor-pointer">
                <Link href="/profile" className="flex items-center">
                  <User className="mr-2 h-4 w-4" />
                  Identity Specs
                </Link>
              </DropdownMenuItem>
              <DropdownMenuSeparator className="bg-white/10" />
              <DropdownMenuItem asChild className="focus:bg-destructive/20 focus:text-destructive cursor-pointer">
                <a href="/api/auth/logout" className="flex items-center text-destructive">
                  <LogOut className="mr-2 h-4 w-4" />
                  Disconnect
                </a>
              </DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </div>
    </header>
  );
}
