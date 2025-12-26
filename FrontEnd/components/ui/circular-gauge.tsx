import React from 'react'
import { cn } from '@/lib/utils'

interface CircularGaugeProps {
    value: number
    max?: number
    size?: number
    strokeWidth?: number
    label?: string
    color?: 'cyan' | 'red' | 'green' | 'yellow'
    showValue?: boolean
}

export function CircularGauge({
    value,
    max = 100,
    size = 120,
    strokeWidth = 8,
    label,
    color = 'cyan',
    showValue = true,
}: CircularGaugeProps) {
    const percentage = Math.min((value / max) * 100, 100)
    const radius = (size - strokeWidth) / 2
    const circumference = 2 * Math.PI * radius
    const offset = circumference - (percentage / 100) * circumference

    const colorMap = {
        cyan: '#00E5FF',
        red: '#FF3B3B',
        green: '#00FFA3',
        yellow: '#FFD700',
    }

    const glowColor = colorMap[color]

    return (
        <div className="relative inline-flex items-center justify-center">
            <svg width={size} height={size} className="transform -rotate-90">
                {/* Background circle */}
                <circle
                    cx={size / 2}
                    cy={size / 2}
                    r={radius}
                    stroke="rgba(255, 255, 255, 0.1)"
                    strokeWidth={strokeWidth}
                    fill="none"
                />
                {/* Progress circle */}
                <circle
                    cx={size / 2}
                    cy={size / 2}
                    r={radius}
                    stroke={glowColor}
                    strokeWidth={strokeWidth}
                    fill="none"
                    strokeDasharray={circumference}
                    strokeDashoffset={offset}
                    strokeLinecap="round"
                    style={{
                        transition: 'stroke-dashoffset 0.5s ease',
                        filter: `drop-shadow(0 0 8px ${glowColor})`,
                    }}
                />
            </svg>
            <div className="absolute inset-0 flex flex-col items-center justify-center">
                {showValue && (
                    <span className="digital-number text-2xl font-bold" style={{ color: glowColor }}>
                        {Math.round(value)}
                    </span>
                )}
                {label && (
                    <span className="text-xs text-fg-secondary mt-1">{label}</span>
                )}
            </div>
        </div>
    )
}
