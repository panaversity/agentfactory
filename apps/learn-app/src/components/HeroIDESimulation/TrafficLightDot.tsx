'use client';

import React from 'react';

export interface TrafficLightDotProps {
    /** The color variant of the traffic light */
    variant: 'close' | 'minimize' | 'maximize';
    /** Size in pixels (default: 14) */
    size?: number;
    /** Whether to show the icon on hover (handled by parent .traffic-lights container) */
    showIcon?: boolean;
}

const VARIANTS = {
    close: {
        gradient: 'from-[#ff6058] to-[#ff5047]',
        border: 'border-[#d93c35]',
        iconPath: 'M0.5 0.5L5.5 5.5M5.5 0.5L0.5 5.5',
        iconSize: 6,
    },
    minimize: {
        gradient: 'from-[#ffbd2e] to-[#f5a300]',
        border: 'border-[#cf8d00]',
        iconPath: 'M0 1H8',
        iconSize: 8,
    },
    maximize: {
        gradient: 'from-[#28c940] to-[#1aab29]',
        border: 'border-[#148c1d]',
        iconPath: 'M7 4L7 1L4 1M1 4L1 7L4 7',
        iconSize: 8,
    },
} as const;

/**
 * A simple, configurable traffic light dot component.
 * Mimics authentic macOS Sonoma window controls.
 */
export function TrafficLightDot({ variant, size = 14, showIcon = true }: TrafficLightDotProps) {
    const config = VARIANTS[variant];
    const iconViewBox = variant === 'minimize' ? '0 0 8 2' : `0 0 ${config.iconSize} ${config.iconSize}`;

    return (
        <div
            className={`
                rounded-full bg-gradient-to-b ${config.gradient} ${config.border}
                border-[0.5px] flex items-center justify-center cursor-default relative
                [box-shadow:inset_0_0_0_0.5px_rgba(255,255,255,0.15)]
            `}
            style={{ width: size, height: size }}
        >
            {showIcon && (
                <span className="traffic-light-icon flex items-center justify-center">
                    <svg
                        width={config.iconSize}
                        height={variant === 'minimize' ? 2 : config.iconSize}
                        viewBox={iconViewBox}
                        fill="none"
                        xmlns="http://www.w3.org/2000/svg"
                    >
                        <path
                            d={config.iconPath}
                            stroke="rgba(0,0,0,0.5)"
                            strokeWidth={variant === 'minimize' ? 1.5 : 1.2}
                            strokeLinecap="round"
                            strokeLinejoin="round"
                        />
                    </svg>
                </span>
            )}
        </div>
    );
}

export interface TrafficLightsProps {
    /** Size of each dot (default: 14) */
    size?: number;
    /** Gap between dots (default: 8) */
    gap?: number;
}

/**
 * A group of three traffic light dots (close, minimize, maximize).
 * Wraps in a container that shows icons on hover.
 */
export function TrafficLights({ size = 14, gap = 8 }: TrafficLightsProps) {
    return (
        <div className="traffic-lights flex pl-1" style={{ gap }}>
            <TrafficLightDot variant="close" size={size} />
            <TrafficLightDot variant="minimize" size={size} />
            <TrafficLightDot variant="maximize" size={size} />
        </div>
    );
}
