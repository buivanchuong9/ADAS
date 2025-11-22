"use client"

import { useState } from "react"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { Label } from "@/components/ui/label"
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select"
import { Slider } from "@/components/ui/slider"
import { AlertCircle, Download, Settings } from "lucide-react"
import { Alert, AlertDescription } from "@/components/ui/alert"

interface DetectionConfig {
  model: string
  confidence: number
  cameraSource: string
  iouThreshold: number
  maxDetections: number
}

interface DetectionConfigPanelProps {
  config: DetectionConfig
  onConfigChange: (config: DetectionConfig) => void
  availableModels: Array<{
    id: string
    name: string
    size: string
    downloaded: boolean
    accuracy: number
  }>
  availableCameras: Array<{
    id: string
    name: string
    type: "smartphone" | "webcam" | "stream"
  }>
  onModelDownload?: (modelId: string) => void
  isProcessing?: boolean
}

export function DetectionConfigPanel({
  config,
  onConfigChange,
  availableModels,
  availableCameras,
  onModelDownload,
  isProcessing = false,
}: DetectionConfigPanelProps) {
  const [expandSettings, setExpandSettings] = useState(false)
  const selectedModel = availableModels.find((m) => m.id === config.model)

  return (
    <div className="space-y-4">
      {/* Camera Source Selection */}
      <Card className="bg-card border-border p-4">
        <div className="space-y-3">
          <div className="flex items-center justify-between">
            <Label htmlFor="camera-select" className="text-base font-semibold">
              📹 Nguồn Camera
            </Label>
            <span className="text-xs bg-blue-500/20 text-blue-400 px-2 py-1 rounded">
              {availableCameras.length} khả dụng
            </span>
          </div>
          <Select value={config.cameraSource} onValueChange={(value) => {
            onConfigChange({ ...config, cameraSource: value })
          }}>
            <SelectTrigger id="camera-select">
              <SelectValue placeholder="Chọn nguồn camera" />
            </SelectTrigger>
            <SelectContent>
              {availableCameras.map((camera) => (
                <SelectItem key={camera.id} value={camera.id}>
                  {camera.type === "smartphone" ? "📱" : "🖥️"} {camera.name}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          <p className="text-xs text-foreground/50">
            {availableCameras.find((c) => c.id === config.cameraSource)?.type === "smartphone"
              ? "💡 Kết nối từ điện thoại qua IP Camera hoặc WebRTC"
              : "💡 Sử dụng webcam máy tính"}
          </p>
        </div>
      </Card>

      {/* Model Selection */}
      <Card className="bg-card border-border p-4">
        <div className="space-y-3">
          <div className="flex items-center justify-between">
            <Label htmlFor="model-select" className="text-base font-semibold">
              🤖 Model Phát Hiện
            </Label>
            <span className="text-xs bg-green-500/20 text-green-400 px-2 py-1 rounded">
              {availableModels.filter((m) => m.downloaded).length}/{availableModels.length} tải về
            </span>
          </div>
          <Select value={config.model} onValueChange={(value) => {
            onConfigChange({ ...config, model: value })
          }}>
            <SelectTrigger id="model-select">
              <SelectValue placeholder="Chọn model" />
            </SelectTrigger>
            <SelectContent>
              {availableModels.map((model) => (
                <SelectItem key={model.id} value={model.id}>
                  {model.downloaded ? "✓" : "○"} {model.name} ({model.size})
                </SelectItem>
              ))}
            </SelectContent>
          </Select>

          {selectedModel && (
            <div className="bg-secondary/50 p-2 rounded text-sm space-y-1">
              <p>
                <span className="text-foreground/60">Độ chính xác:</span>{" "}
                <span className="font-semibold">{selectedModel.accuracy}%</span>
              </p>
              <p>
                <span className="text-foreground/60">Kích thước:</span>{" "}
                <span className="font-semibold">{selectedModel.size}</span>
              </p>
              {!selectedModel.downloaded && onModelDownload && (
                <Button
                  onClick={() => onModelDownload(selectedModel.id)}
                  disabled={isProcessing}
                  size="sm"
                  className="w-full mt-2"
                  variant="outline"
                >
                  <Download className="w-3 h-3 mr-2" />
                  {isProcessing ? "Đang tải..." : "Tải Model"}
                </Button>
              )}
            </div>
          )}
        </div>
      </Card>

      {/* Detection Parameters */}
      <Card className="bg-card border-border p-4">
        <button
          onClick={() => setExpandSettings(!expandSettings)}
          className="w-full flex items-center justify-between font-semibold text-sm hover:bg-secondary/50 p-2 -m-2 rounded transition-colors"
        >
          <span className="flex items-center gap-2">
            <Settings className="w-4 h-4" />
            ⚙️ Tham số Phát Hiện
          </span>
          <span className="text-xs text-foreground/50">{expandSettings ? "▼" : "▶"}</span>
        </button>

        {expandSettings && (
          <div className="space-y-4 mt-4 pt-4 border-t border-border">
            {/* Confidence Threshold */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <Label className="text-sm">Ngưỡng Tự Tin (Confidence)</Label>
                <span className="text-sm font-mono bg-primary/20 text-primary px-2 py-1 rounded">
                  {(config.confidence * 100).toFixed(0)}%
                </span>
              </div>
              <Slider
                value={[config.confidence]}
                onValueChange={(value) =>
                  onConfigChange({ ...config, confidence: value[0] })
                }
                min={0.1}
                max={0.99}
                step={0.05}
                className="w-full"
              />
              <p className="text-xs text-foreground/50 mt-1">
                Chỉ phát hiện đối tượng có độ tin cậy cao hơn ngưỡng này
              </p>
            </div>

            {/* IOU Threshold */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <Label className="text-sm">Ngưỡng IoU (NMS)</Label>
                <span className="text-sm font-mono bg-primary/20 text-primary px-2 py-1 rounded">
                  {(config.iouThreshold * 100).toFixed(0)}%
                </span>
              </div>
              <Slider
                value={[config.iouThreshold]}
                onValueChange={(value) =>
                  onConfigChange({ ...config, iouThreshold: value[0] })
                }
                min={0.1}
                max={0.95}
                step={0.05}
                className="w-full"
              />
              <p className="text-xs text-foreground/50 mt-1">
                Giảm box dư thừa từ cùng một đối tượng
              </p>
            </div>

            {/* Max Detections */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <Label className="text-sm">Số Lượng Phát Hiện Tối Đa</Label>
                <span className="text-sm font-mono bg-primary/20 text-primary px-2 py-1 rounded">
                  {config.maxDetections}
                </span>
              </div>
              <Slider
                value={[config.maxDetections]}
                onValueChange={(value) =>
                  onConfigChange({ ...config, maxDetections: value[0] })
                }
                min={1}
                max={300}
                step={10}
                className="w-full"
              />
              <p className="text-xs text-foreground/50 mt-1">
                Giới hạn số lượng phát hiện để tối ưu hóa hiệu suất
              </p>
            </div>
          </div>
        )}
      </Card>

      {/* Info Alert */}
      <Alert>
        <AlertCircle className="h-4 w-4" />
        <AlertDescription>
          💡 <strong>Mẹo:</strong> Tăng Confidence để giảm false positive, hạ IOU để tách các đối tượng gần nhau
        </AlertDescription>
      </Alert>
    </div>
  )
}
