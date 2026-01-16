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
    // Map glow colors to CSS variables
    const glowColorMap = {
        cyan: 'var(--primary)',
        red: 'var(--destructive)',
        green: 'var(--success)',
        yellow: 'var(--warning)',
        none: 'transparent',
    }

    const glowColor = glowColorMap[glow]

    return (
        <div
            className={cn(
                'card-hud',
                scanLines && 'scan-lines',
                className
            )}
            style={{
                backgroundColor: 'var(--bg-surface)',
                borderColor: glow !== 'none' ? glowColor : 'var(--border-subtle)',
                boxShadow: glow !== 'none' && pulse 
                    ? `0 0 20px ${glowColor}40` 
                    : 'var(--shadow-soft)',
            }}
            {...props}
        >
            {children}
        </div>
    )
}
