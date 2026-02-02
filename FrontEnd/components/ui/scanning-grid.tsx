import React from 'react'
import { cn } from '@/lib/utils'

interface ScanningGridProps {
    className?: string
    message?: string
}

export function ScanningGrid({ className, message = 'INITIALIZING...' }: ScanningGridProps) {
    return (
        <div className={cn('relative w-full h-full bg-bg-secondary grid-overlay', className)}>
            {/* Animated scan line */}
            <div className="absolute inset-0 scan-lines" />

            {/* Center message */}
            <div className="absolute inset-0 flex flex-col items-center justify-center">
                <div className="text-neon-cyan text-xl font-bold tracking-wider animate-pulse">
                    {message}
                </div>

                {/* Pulsing radar circle */}
                <div className="mt-8 relative w-32 h-32">
                    <div className="absolute inset-0 border-2 border-neon-cyan rounded-full animate-ping opacity-75" />
                    <div className="absolute inset-4 border-2 border-neon-cyan rounded-full animate-ping opacity-50" style={{ animationDelay: '0.5s' }} />
                    <div className="absolute inset-8 border-2 border-neon-cyan rounded-full animate-ping opacity-25" style={{ animationDelay: '1s' }} />
                </div>

                {/* Status text */}
                <div className="mt-8 text-fg-secondary text-sm tracking-wide">
                    AWAITING SIGNAL...
                </div>
            </div>

            {/* Corner brackets */}
            <div className="absolute top-4 left-4 w-12 h-12 border-l-2 border-t-2 border-neon-cyan" />
            <div className="absolute top-4 right-4 w-12 h-12 border-r-2 border-t-2 border-neon-cyan" />
            <div className="absolute bottom-4 left-4 w-12 h-12 border-l-2 border-b-2 border-neon-cyan" />
            <div className="absolute bottom-4 right-4 w-12 h-12 border-r-2 border-b-2 border-neon-cyan" />
        </div>
    )
}
