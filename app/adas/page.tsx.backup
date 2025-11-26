"use client"

import { useState, useRef, useEffect, useCallback } from "react"
import { Sidebar } from "@/components/sidebar"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { AlertTriangle, Play, Square, Wifi, WifiOff, Download, RefreshCw } from "lucide-react"
import { DetectionConfigPanel } from "@/components/detection-config-panel"
import { DetectionStats } from "@/components/detection-stats"
import { Alert, AlertDescription } from "@/components/ui/alert"

interface DetectionConfig {
  model: string
  confidence: number
  cameraSource: string
  iouThreshold: number
  maxDetections: number
}

interface Detection {
  id: number
  cls: string
  conf: number
  bbox: [number, number, number, number] // x, y, width, height
  distance_m: number
  color?: string
  danger?: boolean
  ttc?: number
  timestamp?: string
}

interface InferenceResponse {
  detections: Array<{
    id: number
    cls: string
    conf: number
    bbox: number[]
    distance_m: number
  }>
  stats: {
    infer_ms: number
    model: string
  }
}

const AVAILABLE_MODELS = [
  // Object Detection - Vehicles & Objects
  {
    id: "yolov8n",
    name: "YOLOv8 Nano (Fast)",
    size: "6.3 MB",
    downloaded: true,
    accuracy: 80.4,
  },
  {
    id: "yolov8s",
    name: "YOLOv8 Small",
    size: "22.5 MB",
    downloaded: false,
    accuracy: 86.6,
  },
  {
    id: "yolov8m",
    name: "YOLOv8 Medium",
    size: "49.2 MB",
    downloaded: false,
    accuracy: 88.3,
  },
  {
    id: "yolov8l",
    name: "YOLOv8 Large (High Accuracy)",
    size: "83.6 MB",
    downloaded: false,
    accuracy: 90.1,
  },
  {
    id: "yolov10s",
    name: "YOLOv10 Small (Latest)",
    size: "20.1 MB",
    downloaded: false,
    accuracy: 87.2,
  },
  {
    id: "yolov5s",
    name: "YOLOv5 Small",
    size: "27.6 MB",
    downloaded: false,
    accuracy: 85.2,
  },
  // Specialized Models
  {
    id: "custom-adas",
    name: "Custom ADAS (Vehicles Only)",
    size: "35.8 MB",
    downloaded: false,
    accuracy: 92.1,
  },
  {
    id: "faster-rcnn",
    name: "Faster RCNN (Accurate)",
    size: "130 MB",
    downloaded: false,
    accuracy: 93.5,
  },
  // Pose Detection
  {
    id: "yolov8-pose",
    name: "YOLOv8 Pose (Skeleton)",
    size: "57.2 MB",
    downloaded: false,
    accuracy: 88.7,
  },
  // Face Detection & Recognition
  {
    id: "retinaface",
    name: "RetinaFace (Face Detection)",
    size: "102 MB",
    downloaded: false,
    accuracy: 91.2,
  },
  {
    id: "mtcnn",
    name: "MTCNN (Multi-task Cascade)",
    size: "18.5 MB",
    downloaded: false,
    accuracy: 86.8,
  },
  // Traffic-specific
  {
    id: "traffic-sign-detector",
    name: "Traffic Sign Detector",
    size: "42.3 MB",
    downloaded: false,
    accuracy: 89.5,
  },
  {
    id: "license-plate-ocr",
    name: "License Plate Recognition",
    size: "55.6 MB",
    downloaded: false,
    accuracy: 94.2,
  },
  // Semantic Segmentation
  {
    id: "deeplabv3",
    name: "DeepLabv3 (Segmentation)",
    size: "175 MB",
    downloaded: false,
    accuracy: 91.8,
  },
  {
    id: "yolact",
    name: "YOLACT (Instance Segmentation)",
    size: "120 MB",
    downloaded: false,
    accuracy: 89.3,
  },
]

const AVAILABLE_CAMERAS = [
  {
    id: "webcam",
    name: "Web Camera (máy tính)",
    type: "webcam" as const,
  },
  {
    id: "smartphone-rtmp",
    name: "Smartphone (RTMP Stream)",
    type: "smartphone" as const,
  },
  {
    id: "smartphone-webrtc",
    name: "Smartphone (WebRTC)",
    type: "smartphone" as const,
  },
  {
    id: "ip-camera",
    name: "IP Camera",
    type: "stream" as const,
  },
]

// Color mapping for class names
const CLASS_COLORS: { [key: string]: string } = {
  car: "#ff6b35",
  truck: "#ff6b35",
  bus: "#ff6b35",
  person: "#ffd166",
  bicycle: "#06d6a0",
  motorcycle: "#ff006e",
  traffic_light: "#06ffa5",
  stop_sign: "#ff006e",
  dog: "#8338ec",
  cat: "#8338ec",
  default: "#00ff00",
}

export default function ADASLiveDetection() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const animationFrameRef = useRef<number | undefined>(undefined)

  const [isRunning, setIsRunning] = useState(false)
  const [isConnected, setIsConnected] = useState(false)
  const [detections, setDetections] = useState<Detection[]>([])
  const [fps, setFps] = useState(0)
  const [processingTime, setProcessingTime] = useState(0)
  const [isDownloading, setIsDownloading] = useState(false)
  const [downloadingModel, setDownloadingModel] = useState<string | null>(null)
  const [modelWorkerStatus, setModelWorkerStatus] = useState(false)
  const [recentDetections, setRecentDetections] = useState<Detection[]>([])

  const [config, setConfig] = useState<DetectionConfig>({
    model: "yolov8n",
    confidence: 0.5,
    cameraSource: "webcam",
    iouThreshold: 0.45,
    maxDetections: 100,
  })

  // Check if model worker is running
  useEffect(() => {
    const checkWorker = async () => {
      try {
        const res = await fetch("http://localhost:8000/health", { method: "GET" })
        setModelWorkerStatus(res.ok)
      } catch {
        setModelWorkerStatus(false)
      }
    }

    checkWorker()
    const interval = setInterval(checkWorker, 5000)
    return () => clearInterval(interval)
  }, [])

  // Fetch recent detections from database
  useEffect(() => {
    const fetchRecentDetections = async () => {
      try {
        const res = await fetch("http://localhost:8000/api/detections/recent?limit=20")
        if (res.ok) {
          const data = await res.json()
          setRecentDetections(data.detections || [])
        }
      } catch (err) {
        console.error("Error fetching recent detections:", err)
      }
    }

    fetchRecentDetections()
    const interval = setInterval(fetchRecentDetections, 3000) // Refresh every 3s
    return () => clearInterval(interval)
  }, [])

  const startDetection = async () => {
    try {
      if (config.cameraSource === "webcam") {
        const stream = await navigator.mediaDevices.getUserMedia({ 
          video: { width: 640, height: 480 } 
        })

        if (videoRef.current) {
          videoRef.current.srcObject = stream
          setIsRunning(true)
          setIsConnected(true)
        }
      } else if (config.cameraSource === "smartphone-webrtc") {
        alert("💡 WebRTC Mode:\n\n1. Cài app IP Camera trên điện thoại\n2. Nhập URL:\n   ws://localhost:5000/ws/camera\n3. Kết nối từ app")
        return
      } else if (config.cameraSource === "ip-camera") {
        alert("💡 IP Camera Mode:\n\n1. Nhập URL RTMP của camera IP\n2. Ví dụ: rtmp://192.168.1.100:1935/live")
        return
      }
    } catch (err) {
      console.error("Lỗi truy cập camera:", err)
      alert("❌ Không thể truy cập camera. Vui lòng kiểm tra quyền truy cập.")
    }
  }

  const stopDetection = () => {
    if (videoRef.current?.srcObject) {
      const tracks = (videoRef.current.srcObject as MediaStream).getTracks()
      tracks.forEach((track) => track.stop())
    }
    setIsRunning(false)
    setIsConnected(false)
    setDetections([])

    if (animationFrameRef.current) {
      cancelAnimationFrame(animationFrameRef.current)
    }
  }

  const handleModelDownload = async (modelId: string) => {
    setDownloadingModel(modelId)
    setIsDownloading(true)

    try {
      const res = await fetch(`http://localhost:5000/api/models/${modelId}/download`, {
        method: "POST",
      })

      if (res.ok) {
        alert(`✓ Model ${modelId} tải về thành công!`)
      } else {
        throw new Error("Download failed")
      }
    } catch (err) {
      console.error("Lỗi tải model:", err)
      alert(`❌ Lỗi tải model: ${modelId}`)
    } finally {
      setDownloadingModel(null)
      setIsDownloading(false)
    }
  }

  // Main inference loop
  useEffect(() => {
    if (!isRunning || !videoRef.current || !canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    let frameCount = 0
    let lastFpsTime = performance.now()
    let processingTimeSum = 0
    let skipFrames = 0

    const inferenceLoop = async () => {
      try {
        if (videoRef.current && videoRef.current.readyState === HTMLMediaElement.HAVE_ENOUGH_DATA) {
          // Draw video frame
          ctx.drawImage(videoRef.current, 0, 0, canvas.width, canvas.height)

          // Skip frames to reduce load (process every 2nd frame)
          skipFrames++
          if (skipFrames % 2 !== 0) {
            frameCount++
            animationFrameRef.current = requestAnimationFrame(inferenceLoop)
            return
          }

          // Capture frame as base64
          const frameBase64 = canvas.toDataURL("image/jpeg", 0.7).split(",")[1]

          // Send to model worker with timeout
          const controller = new AbortController()
          const timeoutId = setTimeout(() => controller.abort(), 5000)

          const startTime = performance.now()
          
          try {
            const response = await fetch(`http://localhost:8000/infer/${config.model}`, {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({ frame_b64: frameBase64 }),
              signal: controller.signal,
            })

            clearTimeout(timeoutId)

            if (response.ok) {
              const result: InferenceResponse = await response.json()
              const inferTime = performance.now() - startTime

              // Convert detections format
              const processedDetections: Detection[] = result.detections
                .filter((d) => d.conf >= config.confidence)
                .slice(0, config.maxDetections)
                .map((d, idx) => {
                  const [x, y, w, h] = d.bbox
                  const cls_lower = d.cls.toLowerCase()
                  
                  // Calculate danger (person closer than 2 meters or other dangerous objects)
                  const danger = (cls_lower.includes("person") && d.distance_m < 2) || 
                                cls_lower.includes("obstacle")
                  const ttc = danger ? Math.max(0.5, d.distance_m * 1.5) : undefined

                  return {
                    id: idx,
                    cls: d.cls,
                    conf: d.conf,
                    bbox: [x, y, w, h],
                    distance_m: d.distance_m,
                    color: CLASS_COLORS[cls_lower] || CLASS_COLORS.default,
                    danger,
                    ttc,
                  }
                })

              setDetections(processedDetections)
              processingTimeSum += inferTime

              console.log("🔍 Detections:", processedDetections.length, processedDetections)

              // ✅ LƯU DETECTIONS VÀO DATABASE (BACKEND)
              if (processedDetections.length > 0) {
                try {
                  await fetch("http://localhost:8000/api/detections/save", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({
                      detections: processedDetections,
                      camera_id: 1,
                      trip_id: null
                    })
                  })
                  console.log("✅ Saved", processedDetections.length, "detections to database")
                } catch (saveErr) {
                  console.error("❌ Error saving to DB:", saveErr)
                }
              }

              // Draw detections on canvas
              processedDetections.forEach((det, idx) => {
                const [x, y, w, h] = det.bbox

                console.log(`Detection ${idx}:`, { x, y, w, h, cls: det.cls, conf: det.conf })

                // Draw bounding box (thick orange for visibility)
                ctx.strokeStyle = det.danger ? "#ff0000" : "#ff6b35"
                ctx.lineWidth = det.danger ? 4 : 3
                ctx.strokeRect(x, y, w, h)

                // Draw label background
                const labelHeight = 24
                ctx.fillStyle = det.danger ? "#ff0000" : "#ff6b35"
                ctx.globalAlpha = 0.9
                ctx.fillRect(x, Math.max(0, y - labelHeight), Math.max(w, 250), labelHeight)
                ctx.globalAlpha = 1.0

                // Draw label text
                ctx.fillStyle = "#fff"
                ctx.font = "bold 13px monospace"
                const label = `${det.cls} ${(det.conf * 100).toFixed(0)}% | ${det.distance_m.toFixed(1)}m`
                ctx.fillText(label, x + 4, Math.max(15, y - 6))

                // Draw danger indicator
                if (det.danger) {
                  ctx.fillStyle = "#ffff00"
                  ctx.font = "bold 16px Arial"
                  ctx.fillText("⚠️", x + w - 24, y + 20)
                }
              })
            }
          } catch (fetchErr) {
            clearTimeout(timeoutId)
            if (fetchErr instanceof Error && fetchErr.name !== 'AbortError') {
              console.warn("Inference request failed:", fetchErr.message)
            }
          }

          frameCount++
          const now = performance.now()
          if (now - lastFpsTime >= 1000) {
            setFps(frameCount)
            setProcessingTime(processingTimeSum / frameCount)
            frameCount = 0
            processingTimeSum = 0
            lastFpsTime = now
          }
        }
      } catch (err) {
        console.error("Loop error:", err)
      }

      animationFrameRef.current = requestAnimationFrame(inferenceLoop)
    }

    inferenceLoop()

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current)
      }
    }
  }, [isRunning, config.model, config.confidence, config.maxDetections])

  return (
    <div className="flex h-screen bg-background">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-6 space-y-6">
          {/* Header */}
          <div>
            <div className="flex items-center justify-between mb-2">
              <h1 className="text-3xl font-bold text-foreground flex items-center gap-2">
                🚗 ADAS Live Detection
              </h1>
              <div className="flex items-center gap-3">
                {modelWorkerStatus ? (
                  <div className="flex items-center gap-1 px-3 py-1 rounded-full bg-green-500/20 text-green-400 text-sm">
                    <Wifi className="w-4 h-4 animate-pulse" />
                    Model Worker
                  </div>
                ) : (
                  <div className="flex items-center gap-1 px-3 py-1 rounded-full bg-red-500/20 text-red-400 text-sm">
                    <WifiOff className="w-4 h-4" />
                    No Model Worker
                  </div>
                )}
                {isConnected ? (
                  <div className="flex items-center gap-1 px-3 py-1 rounded-full bg-green-500/20 text-green-400 text-sm">
                    <Wifi className="w-4 h-4 animate-pulse" />
                    Camera Connected
                  </div>
                ) : (
                  <div className="flex items-center gap-1 px-3 py-1 rounded-full bg-gray-500/20 text-gray-400 text-sm">
                    <WifiOff className="w-4 h-4" />
                    No Camera
                  </div>
                )}
              </div>
            </div>
            <p className="text-foreground/60">
              Hệ thống ADAS thực tế - Dữ liệu từ camera thực, xử lý bởi YOLOv8
            </p>
          </div>

          {/* Warnings */}
          {!modelWorkerStatus && (
            <Alert>
              <AlertTriangle className="h-4 w-4" />
              <AlertDescription>
                ⚠️ <strong>Model Worker không hoạt động!</strong> Chạy: <code className="bg-black/20 px-2 py-1 rounded">python model-worker/app.py</code>
              </AlertDescription>
            </Alert>
          )}

          {config.cameraSource !== "webcam" && (
            <Alert>
              <AlertTriangle className="h-4 w-4" />
              <AlertDescription>
                ℹ️ <strong>Chế độ {AVAILABLE_CAMERAS.find((c) => c.id === config.cameraSource)?.name}:</strong> Xem hướng dẫn cài đặt
              </AlertDescription>
            </Alert>
          )}

          <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
            {/* Main Video Canvas */}
            <div className="lg:col-span-3 space-y-4">
              <Card className="bg-card border-border overflow-hidden">
                <div className="relative bg-black aspect-video">
                  <video
                    ref={videoRef}
                    className="w-full h-full object-cover hidden"
                    autoPlay
                    playsInline
                  />
                  <canvas
                    ref={canvasRef}
                    width={640}
                    height={480}
                    className="w-full h-full"
                  />

                  {!isRunning && (
                    <div className="absolute inset-0 flex items-center justify-center bg-black/70">
                      <div className="text-center">
                        <div className="text-5xl mb-4">📹</div>
                        <p className="text-foreground/80 font-semibold mb-4">
                          Sẵn sàng để phát hiện
                        </p>
                        <p className="text-foreground/50 text-sm">
                          {modelWorkerStatus
                            ? "Nhấn nút bên dưới để bắt đầu"
                            : "Chạy Model Worker trước"}
                        </p>
                      </div>
                    </div>
                  )}

                  {/* Status Bar */}
                  {isRunning && (
                    <div className="absolute bottom-0 left-0 right-0 bg-linear-to-t from-black/80 to-transparent p-3">
                      <div className="flex items-center justify-between text-white text-sm">
                        <div className="flex gap-4">
                          <span>📊 Model: {config.model}</span>
                          <span>🎯 Confidence: {(config.confidence * 100).toFixed(0)}%</span>
                          <span>🚗 Detections: {detections.length}</span>
                        </div>
                        <span className="text-cyan-400 font-mono">
                          {fps.toFixed(1)} FPS | {processingTime.toFixed(0)}ms
                        </span>
                      </div>
                    </div>
                  )}
                </div>

                {/* Controls */}
                <div className="p-4 border-t border-border flex gap-3">
                  {!isRunning ? (
                    <Button
                      onClick={startDetection}
                      disabled={isDownloading || !modelWorkerStatus}
                      className="flex-1 bg-primary hover:bg-primary/90 text-primary-foreground"
                    >
                      <Play className="w-4 h-4 mr-2" />
                      Bắt Đầu Phát Hiện
                    </Button>
                  ) : (
                    <Button
                      onClick={stopDetection}
                      className="flex-1 bg-destructive hover:bg-destructive/90 text-destructive-foreground"
                    >
                      <Square className="w-4 h-4 mr-2" />
                      Dừng Lại
                    </Button>
                  )}
                  <Button
                    onClick={() => window.location.reload()}
                    variant="outline"
                    size="icon"
                  >
                    <RefreshCw className="w-4 h-4" />
                  </Button>
                </div>
              </Card>
            </div>

            {/* Right Sidebar - Config & Stats */}
            <div className="space-y-4">
              {/* Detection Config */}
              <div>
                <h2 className="text-sm font-semibold mb-3 text-foreground/80">⚙️ CẤU HÌNH</h2>
                <DetectionConfigPanel
                  config={config}
                  onConfigChange={setConfig}
                  availableModels={AVAILABLE_MODELS}
                  availableCameras={AVAILABLE_CAMERAS}
                  onModelDownload={handleModelDownload}
                  isProcessing={isDownloading}
                />
              </div>

              {/* Detection Stats */}
              <div>
                <h2 className="text-sm font-semibold mb-3 text-foreground/80">📈 THỐNG KÊ</h2>
                <DetectionStats
                  detections={detections.map((d) => ({
                    id: d.id.toString(),
                    class: d.cls,
                    confidence: d.conf,
                    x: d.bbox[0],
                    y: d.bbox[1],
                    width: d.bbox[2],
                    height: d.bbox[3],
                    color: d.color,
                    danger: d.danger,
                    ttc: d.ttc,
                  }))}
                  fps={fps}
                  modelName={config.model}
                  processingTime={processingTime}
                  dangerDetected={detections.some((d) => d.danger)}
                />
              </div>

              {/* Recent Detections from Database */}
              <div>
                <h2 className="text-sm font-semibold mb-3 text-foreground/80">
                  💾 DỮ LIỆU ĐÃ LƯU ({recentDetections.length})
                </h2>
                <Card className="bg-card border-border">
                  <div className="p-3 max-h-64 overflow-y-auto space-y-2">
                    {recentDetections.length === 0 ? (
                      <div className="text-center py-8 text-muted-foreground text-sm">
                        Chưa có dữ liệu detection
                      </div>
                    ) : (
                      recentDetections.map((det, idx) => (
                        <div
                          key={idx}
                          className="flex items-center justify-between p-2 bg-accent/50 rounded border border-border text-xs"
                        >
                          <div className="flex-1">
                            <div className="font-semibold text-foreground">
                              {det.cls}
                            </div>
                            <div className="text-muted-foreground">
                              {(det.conf * 100).toFixed(0)}% • {det.distance_m?.toFixed(1)}m
                            </div>
                          </div>
                          <div className="text-muted-foreground text-xs">
                            {det.timestamp ? new Date(det.timestamp).toLocaleTimeString() : ''}
                          </div>
                        </div>
                      ))
                    )}
                  </div>
                </Card>
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  )
}
