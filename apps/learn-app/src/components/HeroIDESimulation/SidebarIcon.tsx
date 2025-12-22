'use client';

import React from 'react';

export type SidebarIconType = 'files' | 'search' | 'git' | 'extensions';

export interface SidebarIconProps {
    /** The type of icon to display */
    type: SidebarIconType;
    /** Whether this icon is currently active/selected */
    isActive?: boolean;
    /** Click handler */
    onClick?: () => void;
}

const ICON_PATHS: Record<SidebarIconType, React.ReactNode> = {
    files: (
        <path
            d="M3 7V17C3 18.1046 3.89543 19 5 19H19C20.1046 19 21 18.1046 21 17V9C21 7.89543 20.1046 7 19 7H12L10 5H5C3.89543 5 3 5.89543 3 7Z"
            stroke="currentColor"
            strokeWidth="1.5"
            strokeLinecap="round"
            strokeLinejoin="round"
        />
    ),
    search: (
        <>
            <circle cx="11" cy="11" r="7" stroke="currentColor" strokeWidth="1.5" />
            <path d="M21 21L16.5 16.5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
        </>
    ),
    git: (
        <>
            <circle cx="6" cy="6" r="2.5" stroke="currentColor" strokeWidth="1.5" />
            <circle cx="18" cy="6" r="2.5" stroke="currentColor" strokeWidth="1.5" />
            <circle cx="6" cy="18" r="2.5" stroke="currentColor" strokeWidth="1.5" />
            <path d="M6 8.5V15.5" stroke="currentColor" strokeWidth="1.5" />
            <path d="M8.5 6H15.5" stroke="currentColor" strokeWidth="1.5" />
        </>
    ),
    extensions: (
        <>
            <rect x="3" y="3" width="8" height="8" rx="1" stroke="currentColor" strokeWidth="1.5" />
            <rect x="13" y="3" width="8" height="8" rx="1" stroke="currentColor" strokeWidth="1.5" />
            <rect x="3" y="13" width="8" height="8" rx="1" stroke="currentColor" strokeWidth="1.5" />
            <rect x="13" y="13" width="8" height="8" rx="1" stroke="currentColor" strokeWidth="1.5" />
        </>
    ),
};

/**
 * A reusable sidebar icon component for VS Code-style activity bar.
 */
export function SidebarIcon({ type, isActive = false, onClick }: SidebarIconProps) {
    return (
        <div
            className={`
                w-10 h-9 flex items-center justify-center rounded-md cursor-pointer
                ${isActive ? 'bg-white/[0.08] border-l-2 border-[#007acc]' : ''}
            `}
            onClick={onClick}
        >
            <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
                className={isActive ? 'text-[#c5c5c5]' : 'text-[#858585]'}
            >
                {ICON_PATHS[type]}
            </svg>
        </div>
    );
}

export interface SidebarProps {
    /** Currently active icon */
    activeIcon?: SidebarIconType;
}

/**
 * The complete sidebar with all icons.
 */
export function Sidebar({ activeIcon = 'files' }: SidebarProps) {
    const icons: SidebarIconType[] = ['files', 'search', 'git', 'extensions'];

    return (
        <div className="macos-sidebar w-[50px] border-r border-black/30 flex flex-col items-center pt-2 gap-1 shrink-0">
            {icons.map((icon) => (
                <SidebarIcon key={icon} type={icon} isActive={icon === activeIcon} />
            ))}
        </div>
    );
}
