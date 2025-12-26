import React from 'react'
import { cn } from '@/lib/utils'

interface GlassCardProps extends React.HTMLAttributes<HTMLDivElement> {
    children: React.ReactNode
    glow?: 'cyan' | 'red' | 'green' | 'yellow' | 'none'
    pulse?: boolean
    scanLines?: boolean
}

export function GlassCard({
    children,
    className,
    glow = 'none',
    pulse = false,
    scanLines = false,
    ...props
}: GlassCardProps) {
    const glowClass = {
        cyan: 'glow-cyan',
        red: 'glow-red',
        green: 'glow-green',
        yellow: 'text-neon-yellow',
        none: '',
    }[glow]

    const pulseClass = pulse
        ? glow === 'red'
            ? 'glow-pulse-red'
            : 'glow-pulse-cyan'
        : ''

    return (
        <div
            className={cn(
                'glass-card',
                glowClass,
                pulseClass,
                scanLines && 'scan-lines',
                className
            )}
            {...props}
        >
            {children}
        </div>
    )
}
