// ADAS Detection Optimizations
// This file contains optimized functions for smoother, faster detection

import { translateObjectClass } from './translations'

// Frame counter for skipping
let frameCounter = 0
const FRAME_SKIP = 1 // Send every frame for 40 FPS (no skipping)
const MIN_CONFIDENCE = 0.45 // Filter out low-confidence detections
const FPS_TARGET = 40 // Target FPS for smooth performance

// Previous detections for smoothing
let previousDetections: any[] = []

/**
 * Optimized frame sending with skipping
 */
export function shouldSendFrame(): boolean {
    frameCounter++
    return frameCounter % FRAME_SKIP === 0
}

/**
 * Filter detections by confidence threshold
 */
export function filterDetections(detections: any[]): any[] {
    if (!detections || !Array.isArray(detections)) return []

    return detections.filter(det => {
        // Keep high confidence detections
        if (det.conf >= MIN_CONFIDENCE) return true

        // Keep danger objects even with lower confidence
        if (det.danger && det.conf >= 0.35) return true

        return false
    })
}

/**
 * Smooth bounding boxes using previous frame data
 * Prevents flickering by averaging with previous position
 */
export function smoothDetections(current: any[], previous: any[]): any[] {
    if (!previous || previous.length === 0) return current

    const SMOOTH_FACTOR = 0.3 // 30% current, 70% previous

    return current.map(det => {
        // Find matching detection in previous frame (by class and proximity)
        const prev = previous.find(p =>
            p.class === det.class &&
            p.track_id === det.track_id
        )

        if (!prev || !prev.bbox || !det.bbox) return det

        // Smooth bounding box coordinates
        const smoothedBbox = det.bbox.map((coord: number, idx: number) => {
            return coord * SMOOTH_FACTOR + prev.bbox[idx] * (1 - SMOOTH_FACTOR)
        })

        return {
            ...det,
            bbox: smoothedBbox
        }
    })
}

/**
 * Get Vietnamese label for detection
 */
export function getVietnameseLabel(detection: any): string {
    const className = detection.cls || detection.class || 'unknown'
    const vietnameseName = translateObjectClass(className)
    const confidence = (detection.conf * 100).toFixed(0)
    const trackId = detection.track_id !== undefined ? `#${detection.track_id}` : ''

    let label = `${trackId} ${vietnameseName} ${confidence}%`

    // Add distance if available
    if (detection.distance_m !== undefined && detection.distance_m !== null) {
        label += ` ${detection.distance_m.toFixed(1)}m`
    }

    // Add TTC if critical
    if (detection.ttc !== undefined && detection.ttc !== null && detection.ttc < 10) {
        label += ` TTC:${detection.ttc.toFixed(1)}s`
    }

    // Add new tag
    if (detection.is_new) {
        label += ' 🆕'
    }

    return label
}

/**
 * Get color based on danger level
 */
export function getDetectionColor(detection: any): string {
    // New objects - magenta
    if (detection.is_new) {
        return '#ff00ff'
    }

    // Danger objects with TTC
    if (detection.danger) {
        if (detection.ttc !== undefined && detection.ttc !== null) {
            if (detection.ttc < 2.0) return '#ff0000' // Red (critical)
            if (detection.ttc < 3.5) return '#ffaa00' // Orange (warning)
        }
        return '#00ff00' // Green (safe)
    }

    // Non-danger objects
    return '#00ccff' // Cyan (neutral)
}

/**
 * Professional drawing with shadows and smooth lines
 */
export function drawProfessionalBox(
    ctx: CanvasRenderingContext2D,
    detection: any,
    color: string
) {
    if (!detection.bbox || detection.bbox.length < 4) return

    const [x1, y1, x2, y2] = detection.bbox
    const width = x2 - x1
    const height = y2 - y1

    // Shadow for depth
    ctx.shadowBlur = 8
    ctx.shadowColor = 'rgba(0, 0, 0, 0.4)'
    ctx.shadowOffsetX = 2
    ctx.shadowOffsetY = 2

    // Draw bounding box
    ctx.strokeStyle = color
    ctx.lineWidth = detection.is_new ? 4 : 3
    ctx.strokeRect(x1, y1, width, height)

    // Reset shadow
    ctx.shadowBlur = 0
    ctx.shadowOffsetX = 0
    ctx.shadowOffsetY = 0

    // Draw label
    const label = getVietnameseLabel(detection)
    const labelWidth = ctx.measureText(label).width + 16
    const labelHeight = 28

    // Label background with gradient
    const gradient = ctx.createLinearGradient(x1, y1 - labelHeight, x1, y1)
    gradient.addColorStop(0, color + 'dd')
    gradient.addColorStop(1, color + 'aa')

    ctx.fillStyle = gradient
    ctx.fillRect(x1, y1 - labelHeight, labelWidth, labelHeight)

    // Label border
    ctx.strokeStyle = color
    ctx.lineWidth = 2
    ctx.strokeRect(x1, y1 - labelHeight, labelWidth, labelHeight)

    // Label text
    ctx.fillStyle = '#ffffff'
    ctx.font = detection.is_new ? 'bold 15px Inter, Arial' : '14px Inter, Arial'
    ctx.textBaseline = 'middle'
    ctx.fillText(label, x1 + 8, y1 - labelHeight / 2)
}

/**
 * Calculate optimal interval for frame sending
 */
export function getOptimalInterval(): number {
    return Math.floor(1000 / FPS_TARGET)
}

export {
    frameCounter,
    previousDetections,
    FRAME_SKIP,
    MIN_CONFIDENCE,
    FPS_TARGET
}
