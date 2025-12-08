"use client"

import { Card } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { TrendingUp, AlertTriangle, Zap } from "lucide-react"

interface Detection {
  id: string
  class: string
  confidence: number
  x: number
  y: number
  width: number
  height: number
  color?: string
  danger?: boolean
  ttc?: number // Time to collision
}

interface DetectionStatsProps {
  detections: Detection[]
  fps?: number
  modelName?: string
  processingTime?: number
  dangerDetected?: boolean
}

export function DetectionStats({
  detections,
  fps = 0,
  modelName = "Unknown",
  processingTime = 0,
  dangerDetected = false,
}: DetectionStatsProps) {
  const classCount: Record<string, number> = {}
  let totalConfidence = 0

  detections.forEach((det) => {
    classCount[det.class] = (classCount[det.class] || 0) + 1
    totalConfidence += det.confidence
  })

  const avgConfidence =
    detections.length > 0 ? (totalConfidence / detections.length * 100).toFixed(1) : 0

  const dangerDetections = detections.filter((d) => d.danger)

  return (
    <div className="space-y-3">
      {/* FPS & Performance */}
      <Card className="bg-card border-border p-3">
        <div className="grid grid-cols-3 gap-3">
          <div>
            <p className="text-xs text-foreground/60">FPS</p>
            <p className="text-lg font-bold text-cyan-400">{fps.toFixed(1)}</p>
          </div>
          <div>
            <p className="text-xs text-foreground/60">Xử Lý (ms)</p>
            <p className="text-lg font-bold text-purple-400">{processingTime.toFixed(0)}</p>
          </div>
          <div>
            <p className="text-xs text-foreground/60">Model</p>
            <p className="text-xs font-mono">{modelName}</p>
          </div>
        </div>
      </Card>

      {/* Detection Count & Avg Confidence */}
      <Card className="bg-card border-border p-3">
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-sm text-foreground/60">Tổng Phát Hiện</span>
            <Badge variant="outline" className="text-base font-semibold">
              {detections.length}
            </Badge>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-sm text-foreground/60">Độ Tin Cậy Trung Bình</span>
            <Badge variant="outline" className="text-base font-semibold">
              {avgConfidence}%
            </Badge>
          </div>
        </div>
      </Card>

      {/* Detection Classes */}
      {Object.keys(classCount).length > 0 && (
        <Card className="bg-card border-border p-3">
          <p className="text-xs font-semibold text-foreground/60 mb-2">PHÁT HIỆN THEO LỚP</p>
          <div className="space-y-1.5">
            {Object.entries(classCount)
              .sort((a, b) => b[1] - a[1])
              .map(([className, count]) => (
                <div key={className} className="flex items-center justify-between">
                  <span className="text-sm">{className}</span>
                  <Badge variant="secondary">{count}</Badge>
                </div>
              ))}
          </div>
        </Card>
      )}

      {/* Safety Warnings */}
      {dangerDetections.length > 0 && (
        <Card className="bg-destructive/10 border-destructive/30 p-3">
          <div className="flex items-center gap-2 mb-2">
            <AlertTriangle className="w-4 h-4 text-destructive" />
            <p className="text-sm font-semibold text-destructive">Cảnh Báo An Toàn</p>
          </div>
          <div className="space-y-1.5">
            {dangerDetections.map((det, idx) => (
              <div key={idx} className="flex items-center justify-between text-sm">
                <span className="text-foreground/80">{det.class}</span>
                {det.ttc && (
                  <span className="text-xs font-mono bg-red-500/20 text-red-400 px-2 py-1 rounded">
                    <Zap className="w-3 h-3 inline mr-1" />
                    {det.ttc.toFixed(1)}s
                  </span>
                )}
              </div>
            ))}
          </div>
        </Card>
      )}
    </div>
  )
}
