"use client"

import { useEffect, useRef, useState } from 'react'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Progress } from "@/components/ui/progress"
import { Alert, AlertDescription } from "@/components/ui/alert"
import { Camera, Download, PlayCircle, StopCircle, Loader2, CheckCircle, AlertCircle } from "lucide-react"
import { getWebSocketUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

interface ModelInfo {
  id: string
  name: string
  description: string
  type: string
  size_mb: number
  accuracy: number
  speed_ms: number
  downloaded: boolean
  file_path?: string
}

interface Detection {
  bbox: [number, number, number, number]
  confidence: number
  class: string
  class_id: number
}

export default function ModelWebcamPage() {
  const [models, setModels] = useState<ModelInfo[]>([])
  const [selectedModel, setSelectedModel] = useState<string>('yolo11n')
  const [isDownloading, setIsDownloading] = useState<string | null>(null)
  const [isStreaming, setIsStreaming] = useState(false)
  const [detections, setDetections] = useState<Detection[]>([])
  const [fps, setFps] = useState(0)
  const [latency, setLatency] = useState(0)
  const [error, setError] = useState<string | null>(null)
  
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const streamRef = useRef<MediaStream | null>(null)
  const frameIntervalRef = useRef<NodeJS.Timeout | null>(null)
  const fpsCounterRef = useRef<{ frames: number; lastTime: number }>({ frames: 0, lastTime: Date.now() })

  // Load available models
  useEffect(() => {
    fetchModels()
  }, [])

  const fetchModels = async () => {
    try {
      const response = await fetch('/api/models')
      const data = await response.json()
      if (data.success) {
        setModels(data.models)
        
        // Auto-download essential models if not downloaded
        const essentialModels = ['yolo11n', 'yolop', 'midas_small']
        for (const modelId of essentialModels) {
          const model = data.models.find((m: ModelInfo) => m.id === modelId)
          if (model && !model.downloaded) {
            console.log(`Auto-downloading ${modelId}...`)
            downloadModel(modelId)
          }
        }
      }
    } catch (err) {
      console.error('Failed to fetch models:', err)
      setError('Không thể tải danh sách models')
    }
  }

  const downloadModel = async (modelId: string) => {
    setIsDownloading(modelId)
    setError(null)
    
    try {
      const response = await fetch(`/api/models/${modelId}`, {
        method: 'POST'
      })
      const data = await response.json()
      
      if (data.success) {
        console.log(`✅ Downloaded ${modelId}`)
        fetchModels() // Refresh list
      } else {
        throw new Error(data.error || 'Download failed')
      }
    } catch (err) {
      console.error(`Download error for ${modelId}:`, err)
      setError(`Lỗi tải model ${modelId}: ${err instanceof Error ? err.message : 'Unknown error'}`)
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
      setError('Không thể truy cập camera. Vui lòng cấp quyền camera.')
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
    // Check if model is downloaded
    const model = models.find(m => m.id === selectedModel)
    if (!model?.downloaded) {
      setError(`Model ${selectedModel} chưa được tải xuống. Đang tải...`)
      await downloadModel(selectedModel)
      return
    }

    if (!videoRef.current || !canvasRef.current) return

    // Start webcam
    await startWebcam()

    // Connect WebSocket
    // Connect to ADAS streaming WebSocket (assumed equivalent endpoint)
    const ws = new WebSocket(getWebSocketUrl(API_ENDPOINTS.WS_STREAM))
    wsRef.current = ws

    ws.onopen = () => {
      console.log('WebSocket connected')
      setIsStreaming(true)
      setError(null)

      // Start sending frames
      frameIntervalRef.current = setInterval(() => {
        sendFrame()
      }, 50) // 20 FPS
    }

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data)
      
      if (data.success && data.detections) {
        setDetections(data.detections)
        setLatency(data.latency_ms || 0)
        
        // Update FPS counter
        fpsCounterRef.current.frames++
        const now = Date.now()
        if (now - fpsCounterRef.current.lastTime >= 1000) {
          setFps(fpsCounterRef.current.frames)
          fpsCounterRef.current.frames = 0
          fpsCounterRef.current.lastTime = now
        }
        
        drawDetections(data.detections)
      } else if (data.error) {
        console.error('Inference error:', data.error)
      }
    }

    ws.onerror = (err) => {
      console.error('WebSocket error:', err)
      setError('Lỗi kết nối WebSocket')
    }

    ws.onclose = () => {
      console.log('WebSocket disconnected')
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
      
      // Draw bounding box
      ctx.strokeStyle = '#00ff00'
      ctx.lineWidth = 3
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1)
      
      // Draw label
      ctx.fillStyle = '#00ff00'
      ctx.font = '16px Arial'
      const label = `${det.class} ${(det.confidence * 100).toFixed(1)}%`
      ctx.fillText(label, x1, y1 - 5)
    })
  }

  useEffect(() => {
    return () => {
      stopInference()
    }
  }, [])

  return (
    <div className="container mx-auto p-6 space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold">AI Models & Webcam Detection</h1>
          <p className="text-muted-foreground mt-2">
            Tải xuống và chạy models AI với camera real-time
          </p>
        </div>
      </div>

      {error && (
        <Alert variant="destructive">
          <AlertCircle className="h-4 w-4" />
          <AlertDescription>{error}</AlertDescription>
        </Alert>
      )}

      {/* Models List */}
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
        {models.map((model) => (
          <Card key={model.id} className={selectedModel === model.id ? 'border-primary' : ''}>
            <CardHeader>
              <div className="flex items-center justify-between">
                <CardTitle className="text-lg">{model.name}</CardTitle>
                {model.downloaded ? (
                  <Badge variant="default" className="gap-1">
                    <CheckCircle className="h-3 w-3" />
                    Đã tải
                  </Badge>
                ) : (
                  <Badge variant="outline">Chưa tải</Badge>
                )}
              </div>
              <CardDescription>{model.description}</CardDescription>
            </CardHeader>
            <CardContent className="space-y-3">
              <div className="text-sm space-y-1">
                <div className="flex justify-between">
                  <span className="text-muted-foreground">Kích thước:</span>
                  <span className="font-medium">{model.size_mb} MB</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-muted-foreground">Độ chính xác:</span>
                  <span className="font-medium">{model.accuracy}%</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-muted-foreground">Tốc độ:</span>
                  <span className="font-medium">{model.speed_ms}ms</span>
                </div>
              </div>
              
              <div className="flex gap-2">
                {!model.downloaded ? (
                  <Button
                    size="sm"
                    className="w-full"
                    onClick={() => downloadModel(model.id)}
                    disabled={isDownloading === model.id}
                  >
                    {isDownloading === model.id ? (
                      <>
                        <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                        Đang tải...
                      </>
                    ) : (
                      <>
                        <Download className="h-4 w-4 mr-2" />
                        Tải xuống
                      </>
                    )}
                  </Button>
                ) : (
                  <Button
                    size="sm"
                    variant={selectedModel === model.id ? 'default' : 'outline'}
                    className="w-full"
                    onClick={() => setSelectedModel(model.id)}
                  >
                    {selectedModel === model.id ? 'Đang chọn' : 'Chọn model'}
                  </Button>
                )}
              </div>
            </CardContent>
          </Card>
        ))}
      </div>

      {/* Webcam & Detection */}
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle>Real-time Detection</CardTitle>
              <CardDescription>
                Model: {models.find(m => m.id === selectedModel)?.name || selectedModel}
              </CardDescription>
            </div>
            <div className="flex gap-2">
              {!isStreaming ? (
                <Button onClick={startInference} disabled={!models.find(m => m.id === selectedModel)?.downloaded}>
                  <PlayCircle className="h-4 w-4 mr-2" />
                  Bắt đầu
                </Button>
              ) : (
                <Button variant="destructive" onClick={stopInference}>
                  <StopCircle className="h-4 w-4 mr-2" />
                  Dừng
                </Button>
              )}
            </div>
          </div>
        </CardHeader>
        <CardContent>
          <div className="grid gap-4 md:grid-cols-2">
            {/* Video Display */}
            <div className="relative aspect-video bg-black rounded-lg overflow-hidden">
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
                  <Badge variant="default" className="bg-red-500">
                    <div className="h-2 w-2 rounded-full bg-white mr-2 animate-pulse" />
                    LIVE
                  </Badge>
                  <Badge variant="secondary">{fps} FPS</Badge>
                  <Badge variant="secondary">{latency.toFixed(1)}ms</Badge>
                </div>
              )}
              
              {!isStreaming && (
                <div className="absolute inset-0 flex items-center justify-center">
                  <Camera className="h-16 w-16 text-muted-foreground" />
                </div>
              )}
            </div>

            {/* Detection Stats */}
            <div className="space-y-4">
              <div className="space-y-2">
                <h3 className="font-semibold">Phát hiện ({detections.length})</h3>
                <div className="max-h-96 overflow-y-auto space-y-2">
                  {detections.map((det, idx) => (
                    <div key={idx} className="flex items-center justify-between p-2 bg-muted rounded">
                      <span className="font-medium">{det.class}</span>
                      <Badge variant="outline">
                        {(det.confidence * 100).toFixed(1)}%
                      </Badge>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
