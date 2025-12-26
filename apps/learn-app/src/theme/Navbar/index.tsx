import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useNavbarSecondaryMenu } from '@docusaurus/theme-common/internal';
import NavbarAuth from '@/components/NavbarAuth';
import { ModeToggle } from '@/components/ModeToggle';
import { SearchBar } from '@/components/SearchBar';
import { useLocation } from '@docusaurus/router';
import { Sheet, SheetContent, SheetTrigger, SheetHeader, SheetTitle } from "@/components/ui/sheet";
import { Button } from "@/components/ui/button";
import { Menu, BookOpen, Layers, Lightbulb, Github, X } from "lucide-react";
import { cn } from "@/lib/utils";

export default function Navbar() {
    const { siteConfig } = useDocusaurusContext();
    const location = useLocation();
    const secondaryMenu = useNavbarSecondaryMenu();
    const isDocPage = location.pathname.startsWith('/docs');
    const isHomepage = location.pathname === '/';
    const [isScrolled, setIsScrolled] = useState(false);
    const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

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
                    <div className="flex items-center gap-2 shrink-0">
                        <Link to="/" className="flex items-center gap-2 font-bold text-base sm:text-lg md:text-xl tracking-tight text-foreground hover:no-underline hover:text-primary transition-colors whitespace-nowrap">
                            <div className="h-7 w-7 sm:h-8 sm:w-8 bg-primary rounded-none flex items-center justify-center text-primary-foreground text-xs sm:text-sm font-bold shrink-0">
                                AI
                            </div>
                            <span>Agent Factory</span>
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

                        {/* Mobile Menu Trigger */}
                        <Sheet open={mobileMenuOpen} onOpenChange={setMobileMenuOpen}>
                            <SheetTrigger asChild>
                                <Button variant="ghost" size="icon" className="md:hidden">
                                    <Menu className="w-6 h-6" />
                                    <span className="sr-only">Toggle menu</span>
                                </Button>
                            </SheetTrigger>
                            <SheetContent side="right" className="w-[300px] sm:w-[350px] flex flex-col p-0 overflow-hidden">
                                <SheetHeader className="px-4 py-3 border-b border-border shrink-0">
                                    <SheetTitle className="text-left text-base font-semibold">
                                        {isDocPage ? 'Book Navigation' : 'Menu'}
                                    </SheetTitle>
                                </SheetHeader>

                                {/* Mobile Search */}
                                <div className="px-4 py-3 border-b border-border shrink-0">
                                    <SearchBar enableShortcut={false} />
                                </div>

                                {/* Content - either doc sidebar or generic nav */}
                                <div className="flex-1 overflow-y-auto">
                                    {isDocPage && secondaryMenu.content ? (
                                        // On doc pages, show the doc sidebar content
                                        <div className="doc-sidebar-mobile p-2">
                                            {secondaryMenu.content}
                                        </div>
                                    ) : (
                                        // On non-doc pages, show generic navigation
                                        <nav className="flex flex-col gap-1 p-4">
                                            <Button variant="ghost" asChild className="justify-start h-12" onClick={() => setMobileMenuOpen(false)}>
                                                <Link to="/docs/preface-agent-native">
                                                    <BookOpen className="w-5 h-5" />
                                                    Read Book
                                                </Link>
                                            </Button>
                                            <Button variant="ghost" asChild className="justify-start h-12" onClick={() => setMobileMenuOpen(false)}>
                                                <Link to="/docs/preface-agent-native">
                                                    <Layers className="w-5 h-5" />
                                                    Chapters
                                                </Link>
                                            </Button>
                                            <Button variant="ghost" asChild className="justify-start h-12" onClick={() => setMobileMenuOpen(false)}>
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
                                    )}
                                </div>
                            </SheetContent>
                        </Sheet>
                    </div>
                </div>

                {/* Reading Progress Removed (Redundant with nprogress) */}
            </header>
        </nav>
    );
}
