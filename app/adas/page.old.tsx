"use client"

import { useState, useRef, useEffect } from "react"
import { Sidebar } from "@/components/sidebar"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Alert, AlertDescription } from "@/components/ui/alert"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Slider } from "@/components/ui/slider"
import { Label } from "@/components/ui/label"
import { 
  Play, 
  Square, 
  Wifi, 
  WifiOff, 
  Download, 
  Loader2, 
  CheckCircle, 
  AlertCircle,
  Camera
} from "lucide-react"

interface ModelInfo {
  id: string
  name: string
  description: string
  type: string
  size_mb: number
  accuracy: number
  speed_ms: number
  downloaded: boolean
}

interface Detection {
  bbox: [number, number, number, number]
  confidence: number
  class: string
  class_id: number
}

const BACKEND_URL = process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8000'
const WS_URL = BACKEND_URL.replace('http', 'ws')

export default function ADASPage() {
  const [models, setModels] = useState<ModelInfo[]>([])
  const [selectedModel, setSelectedModel] = useState<string>('yolov8n')
  const [confidence, setConfidence] = useState(0.20)
  const [isDownloading, setIsDownloading] = useState<string | null>(null)
  const [isStreaming, setIsStreaming] = useState(false)
  const [detections, setDetections] = useState<Detection[]>([])
  const [fps, setFps] = useState(0)
  const [latency, setLatency] = useState(0)
  const [error, setError] = useState<string | null>(null)
  const [backendStatus, setBackendStatus] = useState(false)
  const [recentDetections, setRecentDetections] = useState<any[]>([])
  
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const streamRef = useRef<MediaStream | null>(null)
  const frameIntervalRef = useRef<NodeJS.Timeout | null>(null)
  const fpsCounterRef = useRef({ frames: 0, lastTime: Date.now() })

  // Check backend status
  useEffect(() => {
    const checkBackend = async () => {
      try {
        const res = await fetch(`${BACKEND_URL}/health`)
        setBackendStatus(res.ok)
      } catch {
        setBackendStatus(false)
      }
    }
    checkBackend()
    const interval = setInterval(checkBackend, 5000)
    return () => clearInterval(interval)
  }, [])

  // Load models
  useEffect(() => {
    fetchModels()
  }, [])

  // Fetch recent detections from DB
  useEffect(() => {
    const fetchRecent = async () => {
      try {
        const res = await fetch(`${BACKEND_URL}/api/detections/recent?limit=10`)
        if (res.ok) {
          const data = await res.json()
          if (data.success && data.detections) {
            setRecentDetections(data.detections)
          }
        }
      } catch (err) {
        console.error('Error fetching recent detections:', err)
      }
    }
    
    fetchRecent()
    const interval = setInterval(fetchRecent, 5000)
    return () => clearInterval(interval)
  }, [])

  const fetchModels = async () => {
    try {
      const res = await fetch(`${BACKEND_URL}/api/models/available`)
      const data = await res.json()
      if (data.success) {
        setModels(data.models)
        
        // Auto-download YOLOv8n if not downloaded
        const yolov8n = data.models.find((m: ModelInfo) => m.id === 'yolov8n')
        if (yolov8n && !yolov8n.downloaded) {
          downloadModel('yolov8n')
        }
      }
    } catch (err) {
      console.error('Failed to fetch models:', err)
      setError('Không thể tải danh sách models từ backend')
    }
  }

  const downloadModel = async (modelId: string) => {
    setIsDownloading(modelId)
    setError(null)
    
    try {
      const res = await fetch(`${BACKEND_URL}/api/models/download/${modelId}`, {
        method: 'POST'
      })
      const data = await res.json()
      
      if (data.success) {
        console.log(`✅ Downloaded ${modelId}`)
        fetchModels()
      } else {
        throw new Error(data.error || 'Download failed')
      }
    } catch (err) {
      console.error(`Download error:`, err)
      setError(`Lỗi tải model: ${err instanceof Error ? err.message : 'Unknown'}`)
    } finally {
      setIsDownloading(null)
    }
  }

  const startWebcam = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { width: 1280, height: 720 }
      })
      
      if (videoRef.current) {
        videoRef.current.srcObject = stream
        streamRef.current = stream
      }
      
      setError(null)
    } catch (err) {
      console.error('Webcam error:', err)
      setError('Không thể truy cập camera. Vui lòng cấp quyền.')
    }
  }

  const stopWebcam = () => {
    if (streamRef.current) {
      streamRef.current.getTracks().forEach(track => track.stop())
      streamRef.current = null
    }
    if (videoRef.current) {
      videoRef.current.srcObject = null
    }
  }

  const startInference = async () => {
    const model = models.find(m => m.id === selectedModel)
    if (!model?.downloaded) {
      setError(`Model ${selectedModel} chưa tải. Đang tải...`)
      await downloadModel(selectedModel)
      return
    }

    if (!videoRef.current || !canvasRef.current) return

    await startWebcam()

    const ws = new WebSocket(`${WS_URL}/ws/stream`)
    wsRef.current = ws

    ws.onopen = () => {
      console.log('WebSocket connected')
      setIsStreaming(true)
      setError(null)

      frameIntervalRef.current = setInterval(() => {
        sendFrame()
      }, 100) // 10 FPS - Giảm để model xử lý kịp
    }

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data)
      
      if (data.success && data.detections) {
        const filtered = data.detections.filter((d: Detection) => d.confidence >= confidence)
        setDetections(filtered)
        setLatency(data.latency_ms || 0)
        
        fpsCounterRef.current.frames++
        const now = Date.now()
        if (now - fpsCounterRef.current.lastTime >= 1000) {
          setFps(fpsCounterRef.current.frames)
          fpsCounterRef.current.frames = 0
          fpsCounterRef.current.lastTime = now
        }
        
        drawDetections(filtered)
        
        // Save to database
        if (filtered.length > 0) {
          saveDetections(filtered)
        }
      } else if (data.error) {
        console.error('Inference error:', data.error)
      }
    }

    ws.onerror = (err) => {
      console.error('WebSocket error:', err)
      setError('Lỗi kết nối WebSocket')
    }

    ws.onclose = () => {
      console.log('WebSocket closed')
      setIsStreaming(false)
    }
  }

  const stopInference = () => {
    if (frameIntervalRef.current) {
      clearInterval(frameIntervalRef.current)
      frameIntervalRef.current = null
    }
    
    if (wsRef.current) {
      wsRef.current.close()
      wsRef.current = null
    }
    
    stopWebcam()
    setIsStreaming(false)
    setDetections([])
    setFps(0)
    setLatency(0)
  }

  const sendFrame = () => {
    if (!videoRef.current || !wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return

    const canvas = document.createElement('canvas')
    canvas.width = videoRef.current.videoWidth
    canvas.height = videoRef.current.videoHeight
    const ctx = canvas.getContext('2d')
    
    if (!ctx) return
    
    ctx.drawImage(videoRef.current, 0, 0)
    const frameData = canvas.toDataURL('image/jpeg', 0.8)
    
    wsRef.current.send(JSON.stringify({
      model_id: selectedModel,
      frame: frameData
    }))
  }

  const drawDetections = (dets: Detection[]) => {
    const canvas = canvasRef.current
    const video = videoRef.current
    
    if (!canvas || !video) return
    
    canvas.width = video.videoWidth
    canvas.height = video.videoHeight
    
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    
    dets.forEach(det => {
      const [x1, y1, x2, y2] = det.bbox
      
      ctx.strokeStyle = '#00ff00'
      ctx.lineWidth = 3
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1)
      
      ctx.fillStyle = '#00ff00'
      ctx.font = '16px Arial'
      const label = `${det.class} ${(det.confidence * 100).toFixed(1)}%`
      ctx.fillText(label, x1, y1 - 5)
    })
  }

  const saveDetections = async (dets: Detection[]) => {
    try {
      await fetch(`${BACKEND_URL}/api/detections/save`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          detections: dets.map(d => ({
            class_name: d.class,
            confidence: d.confidence,
            bounding_box: JSON.stringify(d.bbox),
            camera_id: 1
          })),
          camera_id: 1
        })
      })
    } catch (err) {
      console.error('Error saving detections:', err)
    }
  }

  useEffect(() => {
    return () => stopInference()
  }, [])

  return (
    <div className="flex h-screen bg-background">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-6 space-y-6">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold">🚗 ADAS Live Detection</h1>
              <p className="text-muted-foreground mt-2">
                Real-time detection với YOLOv8 và webcam
              </p>
            </div>
            <div className="flex gap-2">
              {backendStatus ? (
                <Badge variant="default" className="gap-1">
                  <Wifi className="h-3 w-3 animate-pulse" />
                  Backend Online
                </Badge>
              ) : (
                <Badge variant="destructive" className="gap-1">
                  <WifiOff className="h-3 w-3" />
                  Backend Offline
                </Badge>
              )}
              {isStreaming && (
                <Badge variant="default" className="gap-1 bg-red-500">
                  <div className="h-2 w-2 rounded-full bg-white animate-pulse" />
                  LIVE
                </Badge>
              )}
            </div>
          </div>

          {error && (
            <Alert variant="destructive">
              <AlertCircle className="h-4 w-4" />
              <AlertDescription>{error}</AlertDescription>
            </Alert>
          )}

          {!backendStatus && (
            <Alert>
              <AlertCircle className="h-4 w-4" />
              <AlertDescription>
                Backend không hoạt động. Chạy: <code className="bg-muted px-2 py-1 rounded">python3 run.py</code>
              </AlertDescription>
            </Alert>
          )}

          {backendStatus && !isStreaming && (
            <Alert>
              <AlertCircle className="h-4 w-4" />
              <AlertDescription>
                <strong>💡 Tips:</strong> Giảm Confidence (hiện tại: {(confidence * 100).toFixed(0)}%) để nhạy hơn. 
                Đưa xe/người vào camera từ từ để model kịp phát hiện. Model đang chạy ở {selectedModel} với FPS: 10.
              </AlertDescription>
            </Alert>
          )}

          <div className="grid gap-6 lg:grid-cols-4">
            {/* Main Video */}
            <div className="lg:col-span-3 space-y-4">
              <Card>
                <div className="relative aspect-video bg-black">
                  <video
                    ref={videoRef}
                    autoPlay
                    playsInline
                    muted
                    className="absolute inset-0 w-full h-full object-cover"
                  />
                  <canvas
                    ref={canvasRef}
                    className="absolute inset-0 w-full h-full"
                  />
                  
                  {isStreaming && (
                    <div className="absolute top-4 left-4 space-y-2">
                      <Badge variant="secondary">{fps} FPS</Badge>
                      <Badge variant="secondary">{latency.toFixed(1)}ms</Badge>
                      <Badge variant="secondary">{detections.length} objects</Badge>
                    </div>
                  )}
                  
                  {!isStreaming && (
                    <div className="absolute inset-0 flex items-center justify-center">
                      <Camera className="h-16 w-16 text-muted-foreground" />
                    </div>
                  )}
                </div>

                <div className="p-4 border-t flex gap-3">
                  {!isStreaming ? (
                    <Button
                      onClick={startInference}
                      disabled={!backendStatus || isDownloading !== null}
                      className="flex-1"
                    >
                      <Play className="h-4 w-4 mr-2" />
                      Bắt đầu
                    </Button>
                  ) : (
                    <Button
                      onClick={stopInference}
                      variant="destructive"
                      className="flex-1"
                    >
                      <Square className="h-4 w-4 mr-2" />
                      Dừng
                    </Button>
                  )}
                </div>
              </Card>

              {/* Detection List */}
              <Card>
                <div className="p-4">
                  <h3 className="font-semibold mb-3">Phát hiện hiện tại ({detections.length})</h3>
                  <div className="grid grid-cols-2 md:grid-cols-4 gap-2 max-h-32 overflow-y-auto">
                    {detections.map((det, idx) => (
                      <div key={idx} className="flex items-center justify-between p-2 bg-muted rounded">
                        <span className="font-medium text-sm">{det.class}</span>
                        <Badge variant="outline" className="text-xs">
                          {(det.confidence * 100).toFixed(0)}%
                        </Badge>
                      </div>
                    ))}
                  </div>
                </div>
              </Card>
            </div>

            {/* Sidebar */}
            <div className="space-y-4">
              {/* Model Selection */}
              <Card>
                <div className="p-4 space-y-4">
                  <div>
                    <Label>Model</Label>
                    <Select value={selectedModel} onValueChange={setSelectedModel}>
                      <SelectTrigger>
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        {models.map(model => (
                          <SelectItem key={model.id} value={model.id}>
                            {model.name} {model.downloaded && '✓'}
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  </div>

                  <div>
                    <Label>Confidence: {(confidence * 100).toFixed(0)}%</Label>
                    <Slider
                      value={[confidence * 100]}
                      onValueChange={([v]) => setConfidence(v / 100)}
                      min={15}
                      max={90}
                      step={5}
                      className="mt-2"
                    />
                    <p className="text-xs text-muted-foreground mt-1">Thấp hơn = nhạy hơn</p>
                  </div>
                </div>
              </Card>

              {/* Available Models */}
              <Card>
                <div className="p-4">
                  <h3 className="font-semibold mb-3">Models ({models.length})</h3>
                  <div className="space-y-2 max-h-96 overflow-y-auto">
                    {models.map(model => (
                      <div key={model.id} className="flex items-center justify-between p-2 bg-muted rounded text-sm">
                        <div className="flex-1 min-w-0">
                          <div className="font-medium truncate">{model.name}</div>
                          <div className="text-xs text-muted-foreground">
                            {model.size_mb}MB • {model.accuracy}%
                          </div>
                        </div>
                        {model.downloaded ? (
                          <CheckCircle className="h-4 w-4 text-green-500 shrink-0" />
                        ) : (
                          <Button
                            size="sm"
                            variant="ghost"
                            onClick={() => downloadModel(model.id)}
                            disabled={isDownloading === model.id}
                          >
                            {isDownloading === model.id ? (
                              <Loader2 className="h-4 w-4 animate-spin" />
                            ) : (
                              <Download className="h-4 w-4" />
                            )}
                          </Button>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              </Card>

              {/* Recent from DB */}
              <Card>
                <div className="p-4">
                  <h3 className="font-semibold mb-3">Đã lưu ({recentDetections.length})</h3>
                  <div className="space-y-2 max-h-64 overflow-y-auto">
                    {recentDetections.map((det, idx) => (
                      <div key={idx} className="p-2 bg-muted rounded text-xs">
                        <div className="font-medium">{det.class_name}</div>
                        <div className="text-muted-foreground">
                          {(det.confidence * 100).toFixed(0)}% • {new Date(det.timestamp).toLocaleTimeString()}
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              </Card>
            </div>
          </div>
        </div>
      </main>
    </div>
  )
}
