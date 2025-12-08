"use client"

import { useEffect, useRef } from "react"

interface ADASOverlayProps {
    width: number
    height: number
    data: {
        detections: any[]
        tracks: any[]
        lanes: {
            left_lane: number[][]
            right_lane: number[][]
        }
        ldw: {
            is_departing: boolean
            side: string
        }
        alerts: any[]
        stats: any
    } | null
}

export function ADASOverlay({ width, height, data }: ADASOverlayProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null)

    useEffect(() => {
        const canvas = canvasRef.current
        if (!canvas || !data) return

        const ctx = canvas.getContext("2d")
        if (!ctx) return

        // Clear canvas
        ctx.clearRect(0, 0, width, height)

        // 1. Draw Lanes
        drawLanes(ctx, data.lanes, data.ldw)

        // 2. Draw Path Prediction (Simple straight line or curve based on lanes)
        drawPathPrediction(ctx, data.lanes, width, height)

        // 3. Draw Tracks/Detections
        // Prefer tracks if available, else detections
        const objects = data.tracks && data.tracks.length > 0 ? data.tracks : data.detections
        if (objects) {
            objects.forEach((obj: any) => {
                drawObject(ctx, obj)
            })
        }

        // 4. Draw Alerts
        if (data.alerts && data.alerts.length > 0) {
            drawAlerts(ctx, data.alerts, width, height)
        }

    }, [data, width, height])

    const drawLanes = (ctx: CanvasRenderingContext2D, lanes: any, ldw: any) => {
        if (!lanes) return

        ctx.lineWidth = 4

        // Left Lane
        if (lanes.left_lane && lanes.left_lane.length > 0) {
            ctx.beginPath()
            ctx.strokeStyle = ldw.is_departing && ldw.side === "left" ? "red" : "#00ff00" // Green or Red

            const pts = lanes.left_lane
            ctx.moveTo(pts[0][0], pts[0][1])
            for (let i = 1; i < pts.length; i++) {
                ctx.lineTo(pts[i][0], pts[i][1])
            }
            ctx.stroke()

            // Fill polygon if both lanes exist
            if (lanes.right_lane && lanes.right_lane.length > 0) {
                ctx.fillStyle = "rgba(0, 255, 0, 0.1)"
                ctx.beginPath()
                ctx.moveTo(pts[0][0], pts[0][1])
                for (let i = 1; i < pts.length; i++) {
                    ctx.lineTo(pts[i][0], pts[i][1])
                }

                const r_pts = lanes.right_lane
                for (let i = r_pts.length - 1; i >= 0; i--) {
                    ctx.lineTo(r_pts[i][0], r_pts[i][1])
                }
                ctx.closePath()
                ctx.fill()
            }
        }

        // Right Lane
        if (lanes.right_lane && lanes.right_lane.length > 0) {
            ctx.beginPath()
            ctx.strokeStyle = ldw.is_departing && ldw.side === "right" ? "red" : "#00ff00"

            const pts = lanes.right_lane
            ctx.moveTo(pts[0][0], pts[0][1])
            for (let i = 1; i < pts.length; i++) {
                ctx.lineTo(pts[i][0], pts[i][1])
            }
            ctx.stroke()
        }
    }

    const drawPathPrediction = (ctx: CanvasRenderingContext2D, lanes: any, w: number, h: number) => {
        // Draw a semi-transparent blue path in the center
        ctx.beginPath()
        ctx.strokeStyle = "rgba(0, 100, 255, 0.3)"
        ctx.lineWidth = 20
        ctx.moveTo(w / 2, h)
        ctx.lineTo(w / 2, h * 0.6)
        ctx.stroke()
    }

    const drawObject = (ctx: CanvasRenderingContext2D, obj: any) => {
        const [x1, y1, x2, y2] = obj.bbox
        const w = x2 - x1
        const h = y2 - y1

        // Color based on danger/type
        let color = "#00ffff" // Cyan default
        if (obj.class_id === 0) color = "#ffff00" // Person Yellow
        if (obj.ttc && obj.ttc < 3.0) color = "#ff0000" // Danger Red

        // Draw Box
        ctx.strokeStyle = color
        ctx.lineWidth = 2
        ctx.strokeRect(x1, y1, w, h)

        // Draw Corners (Tesla style)
        const cornerLen = Math.min(w, h) * 0.2
        ctx.lineWidth = 4

        // Top Left
        ctx.beginPath(); ctx.moveTo(x1, y1 + cornerLen); ctx.lineTo(x1, y1); ctx.lineTo(x1 + cornerLen, y1); ctx.stroke()
        // Top Right
        ctx.beginPath(); ctx.moveTo(x2 - cornerLen, y1); ctx.lineTo(x2, y1); ctx.lineTo(x2, y1 + cornerLen); ctx.stroke()
        // Bottom Left
        ctx.beginPath(); ctx.moveTo(x1, y2 - cornerLen); ctx.lineTo(x1, y2); ctx.lineTo(x1 + cornerLen, y2); ctx.stroke()
        // Bottom Right
        ctx.beginPath(); ctx.moveTo(x2 - cornerLen, y2); ctx.lineTo(x2, y2); ctx.lineTo(x2, y2 - cornerLen); ctx.stroke()

        // Draw Label & Info
        ctx.fillStyle = color
        ctx.font = "bold 14px Inter, sans-serif"

        const label = `${obj.class_id !== undefined ? getClass(obj.class_id) : obj.cls} ${obj.track_id ? '#' + obj.track_id : ''}`
        ctx.fillText(label, x1, y1 - 20)

        if (obj.distance && obj.distance > 0) {
            ctx.font = "12px Inter, sans-serif"
            ctx.fillText(`${obj.distance.toFixed(1)}m`, x1, y1 - 5)
        }

        if (obj.ttc && obj.ttc < 10) {
            ctx.fillStyle = "red"
            ctx.font = "bold 14px Inter, sans-serif"
            ctx.fillText(`TTC: ${obj.ttc.toFixed(1)}s`, x1, y2 + 15)
        }
    }

    const drawAlerts = (ctx: CanvasRenderingContext2D, alerts: any[], w: number, h: number) => {
        // Draw critical alerts in center
        const critical = alerts.find((a: any) => a.level === "CRITICAL" || a.level === "danger")

        if (critical) {
            ctx.fillStyle = "rgba(255, 0, 0, 0.3)"
            ctx.fillRect(0, 0, w, h)

            ctx.fillStyle = "red"
            ctx.font = "bold 48px Inter, sans-serif"
            ctx.textAlign = "center"
            ctx.fillText("WARNING!", w / 2, h / 2 - 30)

            ctx.font = "bold 32px Inter, sans-serif"
            ctx.fillStyle = "white"
            ctx.fillText(critical.message, w / 2, h / 2 + 20)
        }

        // Draw other alerts (BSM)
        alerts.forEach((alert: any) => {
            if (alert.type === "BSM") {
                ctx.fillStyle = "rgba(255, 165, 0, 0.5)"
                if (alert.side === "LEFT") {
                    ctx.beginPath()
                    ctx.moveTo(0, h / 2)
                    ctx.lineTo(50, h / 2 - 50)
                    ctx.lineTo(50, h / 2 + 50)
                    ctx.fill()
                } else if (alert.side === "RIGHT") {
                    ctx.beginPath()
                    ctx.moveTo(w, h / 2)
                    ctx.lineTo(w - 50, h / 2 - 50)
                    ctx.lineTo(w - 50, h / 2 + 50)
                    ctx.fill()
                }
            }
        })
    }

    const getClass = (id: number) => {
        const classes: { [key: number]: string } = {
            0: 'Person', 1: 'Bicycle', 2: 'Car', 3: 'Motorcycle', 5: 'Bus', 7: 'Truck', 9: 'Traffic Light', 11: 'Stop Sign'
        }
        return classes[id] || 'Object'
    }

    return (
        <canvas
            ref={canvasRef}
            width={width}
            height={height}
            className="absolute top-0 left-0 w-full h-full pointer-events-none"
        />
    )
}
