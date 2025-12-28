import React from 'react'
import { cn } from '@/lib/utils'

interface NeonProgressProps {
    value: number
    max?: number
    className?: string
    showLabel?: boolean
    status?: 'safe' | 'warning' | 'danger'
}

export function NeonProgress({
    value,
    max = 100,
    className,
    showLabel = true,
    status,
}: NeonProgressProps) {
    const percentage = Math.min((value / max) * 100, 100)

    // Auto-determine status if not provided
    const finalStatus = status || (percentage < 40 ? 'safe' : percentage < 70 ? 'warning' : 'danger')

    return (
        <div className={cn('w-full', className)}>
            <div className="progress-neon">
                <div
                    className={cn('progress-neon-fill', finalStatus)}
                    style={{ width: `${percentage}%` }}
                />
            </div>
            {showLabel && (
                <div className="flex justify-between items-center mt-1">
                    <span className="text-xs text-fg-secondary">
                        {finalStatus === 'safe' && '✓ Normal'}
                        {finalStatus === 'warning' && '⚠ Elevated'}
                        {finalStatus === 'danger' && '⚠ Critical'}
                    </span>
                    <span className="text-xs digital-number text-fg-primary">
                        {Math.round(percentage)}%
                    </span>
                </div>
            )}
        </div>
    )
}
