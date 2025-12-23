import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import NavbarAuth from '@/components/NavbarAuth';
import { ModeToggle } from '@/components/ModeToggle';
import { SearchBar } from '@/components/SearchBar';
import { useLocation } from '@docusaurus/router';
import { Sheet, SheetContent, SheetTrigger } from "@/components/ui/sheet";
import { Button } from "@/components/ui/button";
import { Menu, BookOpen, Layers, Lightbulb, Github } from "lucide-react";
import { cn } from "@/lib/utils";

export default function Navbar() {
    const { siteConfig } = useDocusaurusContext();
    const location = useLocation();
    const isDocPage = location.pathname.startsWith('/docs');
    const isHomepage = location.pathname === '/';
    const [isScrolled, setIsScrolled] = useState(false);

    // Handle scroll state for glass effect intensity
    useEffect(() => {
        const handleScroll = () => setIsScrolled(window.scrollY > 20);
        window.addEventListener('scroll', handleScroll);
        return () => window.removeEventListener('scroll', handleScroll);
    }, []);

    return (
        // Outer wrapper with Docusaurus-required classes (for TOC height calculations)
        // We reset inherited styles so they don't affect our layout
        <nav className="navbar navbar--fixed-top !p-0 !m-0 !bg-transparent !border-none !shadow-none !min-h-0 !block !z-40">
            <header
                className={cn(
                    "sticky top-0 z-40 w-full border-b transition-all duration-300",
                    isScrolled
                        ? "bg-background/80 backdrop-blur-xl border-border shadow-sm"
                        : "bg-background/0 border-transparent"
                )}
            >
                <div className="max-w-[1800px] flex h-16 items-center justify-between mx-auto px-4">

                    {/* LEFT: Logo */}
                    <div className="flex items-center gap-2">
                        <Link to="/" className="flex items-center gap-2 font-bold text-xl tracking-tight text-foreground hover:no-underline hover:text-primary transition-colors">
                            <div className="h-8 w-8 bg-primary rounded-none flex items-center justify-center text-primary-foreground text-sm font-bold">
                                AI
                            </div>
                            <span>Native</span>
                        </Link>
                    </div>

                    {/* CENTER: Search Bar - Wider and centered on docs pages */}
                    {!isHomepage && (
                        <div className="hidden md:flex flex-1 justify-center max-w-lg mx-8">
                            <div className="w-full">
                                <SearchBar />
                            </div>
                        </div>
                    )}

                    {/* CENTER: Desktop Navigation - Commented out for now
                    {!isHomepage && (
                        <nav className="hidden md:flex items-center gap-1">
                            <Button variant="ghost" asChild>
                                <Link to="/docs/preface-agent-native">
                                    <BookOpen className="w-4 h-4" />
                                    Read Book
                                </Link>
                            </Button>
                            <Button variant="ghost" asChild>
                                <Link to="/docs/preface-agent-native">
                                    <Layers className="w-4 h-4" />
                                    Chapters
                                </Link>
                            </Button>
                            <Button variant="ghost" asChild>
                                <Link to="/docs/preface-agent-native">
                                    <Lightbulb className="w-4 h-4" />
                                    Resources
                                </Link>
                            </Button>
                        </nav>
                    )}
                    */}

                    {/* RIGHT: Actions */}
                    <div className="flex items-center gap-2">

                        {/* GitHub - Hidden on Homepage */}
                        {!isHomepage && (
                            <Button variant="ghost" size="icon" asChild className="hidden md:inline-flex">
                                <Link to="https://github.com/panaversity/ai-native-software-development">
                                    <Github className="w-5 h-5" />
                                    <span className="sr-only">GitHub</span>
                                </Link>
                            </Button>
                        )}

                        {/* Theme Toggle - Already uses Button variant="ghost" size="icon" */}
                        <ModeToggle />

                        {/* Auth - Already uses Button variants */}
                        <NavbarAuth />

                        {/* Mobile Menu Trigger - Button variant="ghost" size="icon" */}
                        <Sheet>
                            <SheetTrigger asChild>
                                <Button variant="ghost" size="icon" className="md:hidden">
                                    <Menu className="w-6 h-6" />
                                    <span className="sr-only">Toggle menu</span>
                                </Button>
                            </SheetTrigger>
                            <SheetContent side="right" className="w-[300px] sm:w-[400px] flex flex-col gap-6 pt-10">
                                {/* Mobile Search */}
                                <div className="w-full">
                                    <SearchBar enableShortcut={false} />
                                </div>

                                {/* Mobile Nav - Using Button variant="ghost" with full width */}
                                <nav className="flex flex-col gap-1">
                                    <Button variant="ghost" asChild className="justify-start h-12">
                                        <Link to="/docs/preface-agent-native">
                                            <BookOpen className="w-5 h-5" />
                                            Read Book
                                        </Link>
                                    </Button>
                                    <Button variant="ghost" asChild className="justify-start h-12">
                                        <Link to="/docs/preface-agent-native">
                                            <Layers className="w-5 h-5" />
                                            Chapters
                                        </Link>
                                    </Button>
                                    <Button variant="ghost" asChild className="justify-start h-12">
                                        <Link to="/docs/preface-agent-native">
                                            <Lightbulb className="w-5 h-5" />
                                            Resources
                                        </Link>
                                    </Button>

                                    <div className="h-px bg-border my-2" />

                                    <Button variant="ghost" asChild className="justify-start h-12 text-muted-foreground">
                                        <Link to="https://github.com/panaversity/ai-native-software-development">
                                            <Github className="w-5 h-5" />
                                            Source Code
                                        </Link>
                                    </Button>
                                </nav>
                            </SheetContent>
                        </Sheet>
                    </div>
                </div>

                {/* Reading Progress Removed (Redundant with nprogress) */}
            </header>
        </nav>
    );
}
