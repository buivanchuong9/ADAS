'use client'

import { useEffect, useRef, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { AlertCircle, Video, VideoOff, Activity, Zap, Shield, AlertTriangle, Upload } from 'lucide-react'
import { Alert, AlertDescription } from '@/components/ui/alert'
import { Input } from '@/components/ui/input'

interface Detection {
  cls: string
  class: string
  conf: number
  bbox: number[]
  distance_m?: number
  ttc?: number
  danger?: boolean
  is_new?: boolean
  class_id?: number
}

interface LaneInfo {
  left_lane: number[][]
  right_lane: number[][]
  center_offset: number
}

interface AlertInfo {
  level: string
  message: string
  distance?: number
  ttc?: number
}

interface UnifiedResult {
  detections: Detection[]
  lanes?: LaneInfo
  alerts: AlertInfo[]
  stats: {
    fps: number
    inference_time: number
    total_objects: number
    unique_classes?: string[]
    new_objects_count?: number
  }
  collection_stats?: {
    total_collected: number
    new_objects_learned: number
    last_collection: string | null
  }
  new_objects?: Array<{class: string, reason: string}>
}

export default function ADASUnifiedPage() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const fileInputRef = useRef<HTMLInputElement>(null)
  
  const [isStreaming, setIsStreaming] = useState(false)
  const [stream, setStream] = useState<MediaStream | null>(null)
  const [videoFile, setVideoFile] = useState<File | null>(null)
  const [isVideoMode, setIsVideoMode] = useState(false)
  const [fps, setFps] = useState(0)
  const [detectionCount, setDetectionCount] = useState(0)
  const [alerts, setAlerts] = useState<AlertInfo[]>([])
  const [stats, setStats] = useState<{fps: number, inference_time: number, total_objects: number, unique_classes: string[], new_objects_count: number}>({ 
    fps: 0, 
    inference_time: 0, 
    total_objects: 0, 
    unique_classes: [], 
    new_objects_count: 0 
  })
  const [collectionStats, setCollectionStats] = useState({ total_collected: 0, new_objects_learned: 0, last_collection: null })
  const [newObjects, setNewObjects] = useState<Array<{class: string, reason: string}>>([])
  const [enableAutoCollection, setEnableAutoCollection] = useState(true)

  // Start webcam
  const startWebcam = async () => {
    try {
      const mediaStream = await navigator.mediaDevices.getUserMedia({
        video: { width: 1280, height: 720, facingMode: 'environment' }
      })
      
      if (videoRef.current) {
        videoRef.current.srcObject = mediaStream
        setStream(mediaStream)
      }
    } catch (error) {
      console.error('Error accessing webcam:', error)
      alert('Không thể truy cập camera. Vui lòng kiểm tra quyền truy cập.')
    }
  }

  // Stop webcam
  const stopWebcam = () => {
    if (stream) {
      stream.getTracks().forEach(track => track.stop())
      setStream(null)
    }
    if (videoRef.current) {
      videoRef.current.srcObject = null
      videoRef.current.src = ''
    }
    setIsVideoMode(false)
    setVideoFile(null)
  }

  // Handle video file upload
  const handleVideoUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    if (file && file.type.startsWith('video/')) {
      setVideoFile(file)
      
      // Stop any existing stream
      if (stream) {
        stream.getTracks().forEach(track => track.stop())
        setStream(null)
      }

      // Load video file
      if (videoRef.current) {
        const url = URL.createObjectURL(file)
        videoRef.current.src = url
        videoRef.current.srcObject = null
        setIsVideoMode(true)
      }
    } else {
      alert('Vui lòng chọn file video hợp lệ!')
    }
  }

  // Connect to WebSocket and start inference
  const startInference = () => {
    if (!stream && !videoFile) {
      alert('Vui lòng bật camera hoặc upload video trước!')
      return
    }

    const ws = new WebSocket('ws://localhost:8000/ws/adas-unified')
    wsRef.current = ws

    ws.onopen = () => {
      console.log('WebSocket connected to unified ADAS endpoint')
      setIsStreaming(true)
    }

    ws.onmessage = (event) => {
      try {
        const result: UnifiedResult = JSON.parse(event.data)
        
        console.log('📦 Received:', result)
        console.log('🔍 Detections:', result.detections?.length || 0)
        
        // Update stats (with fallbacks for undefined values)
        setStats({
          fps: result.stats?.fps || 0,
          inference_time: result.stats?.inference_time || 0,
          total_objects: result.stats?.total_objects || 0,
          unique_classes: result.stats?.unique_classes || [],
          new_objects_count: result.stats?.new_objects_count || 0
        })
        setFps(result.stats?.fps || 0)
        setDetectionCount(result.detections?.length || 0)
        setAlerts(result.alerts || [])
        
        // Update collection stats
        if (result.collection_stats) {
          setCollectionStats(result.collection_stats as any)
        }
        
        // Update new objects
        if (result.new_objects && result.new_objects.length > 0) {
          setNewObjects(result.new_objects)
        }

        // Draw on canvas
        drawDetections(result)

        // Save to database
        if (result.detections && Array.isArray(result.detections) && result.detections.length > 0) {
          saveDetections(result.detections)
        }
      } catch (error) {
        console.error('Error parsing WebSocket message:', error)
      }
    }

    ws.onerror = (error) => {
      console.error('WebSocket error:', error)
    }

    ws.onclose = () => {
      console.log('WebSocket disconnected')
      setIsStreaming(false)
    }

    // Send frames at 10 FPS
    const interval = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN && videoRef.current) {
        sendFrame(ws)
      }
    }, 100) // 10 FPS

    return () => clearInterval(interval)
  }

  // Stop inference
  const stopInference = () => {
    if (wsRef.current) {
      wsRef.current.close()
      wsRef.current = null
    }
    setIsStreaming(false)
    setAlerts([])
    
    // Clear canvas
    if (canvasRef.current) {
      const ctx = canvasRef.current.getContext('2d')
      if (ctx) {
        ctx.clearRect(0, 0, canvasRef.current.width, canvasRef.current.height)
      }
    }
  }

  // Send video frame to WebSocket
  const sendFrame = (ws: WebSocket) => {
    if (!videoRef.current || !canvasRef.current) return

    const video = videoRef.current
    const canvas = canvasRef.current

    // Set canvas size to match video
    canvas.width = video.videoWidth
    canvas.height = video.videoHeight

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Draw video frame to canvas
    ctx.drawImage(video, 0, 0)

    // Convert to base64
    canvas.toBlob((blob) => {
      if (blob) {
        const reader = new FileReader()
        reader.onloadend = () => {
          const base64 = (reader.result as string).split(',')[1]
          ws.send(JSON.stringify({ frame: base64 }))
        }
        reader.readAsDataURL(blob)
      }
    }, 'image/jpeg', 0.8)
  }

  // Draw detections, lanes, and alerts on canvas
  const drawDetections = (result: UnifiedResult) => {
    if (!canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Clear previous drawings
    ctx.clearRect(0, 0, canvas.width, canvas.height)

    // Draw lanes (green lines) - lanes is an array of line segments
    if (result.lanes && Array.isArray(result.lanes) && result.lanes.length > 0) {
      ctx.strokeStyle = '#00ff00'
      ctx.lineWidth = 3
      
      result.lanes.forEach(line => {
        if (line && Array.isArray(line) && line.length === 2 && line[0] && line[1]) {
          const [start, end] = line
          if (Array.isArray(start) && Array.isArray(end) && start.length >= 2 && end.length >= 2) {
            ctx.beginPath()
            ctx.moveTo(start[0], start[1])
            ctx.lineTo(end[0], end[1])
            ctx.stroke()
          }
        }
      })
    }

    // Draw detections with bounding boxes
    if (!result.detections || !Array.isArray(result.detections)) return
    
    result.detections.forEach(det => {
      // Validate bbox array exists and has correct length
      if (!det.bbox || !Array.isArray(det.bbox) || det.bbox.length < 4) return
      const [x1, y1, x2, y2] = det.bbox
      const width = x2 - x1
      const height = y2 - y1

      // Color based on danger level and TTC
      let color = '#00ff00' // Green (safe)
      
      if (det.danger) {
        // Danger objects (vehicles, pedestrians)
        if (det.ttc !== undefined && det.ttc !== null) {
          if (det.ttc < 2.0) color = '#ff0000' // Red (critical)
          else if (det.ttc < 3.5) color = '#ffaa00' // Orange (warning)
        }
      } else {
        // Non-danger objects (trees, animals, objects)
        color = '#00ccff' // Cyan (neutral)
      }
      
      // Highlight new objects
      if (det.is_new) {
        color = '#ff00ff' // Magenta (new discovery!)
      }

      // Draw bounding box
      ctx.strokeStyle = color
      ctx.lineWidth = det.is_new ? 4 : 3
      ctx.strokeRect(x1, y1, width, height)

      // Draw label background
      const className = det.cls || det.class || 'unknown'
      const label = `${className} ${(det.conf * 100).toFixed(0)}%`
      const distance = (det.distance_m !== undefined && det.distance_m !== null) ? ` ${det.distance_m.toFixed(1)}m` : ''
      const ttc = det.ttc !== undefined && det.ttc !== null && det.ttc < 10 ? ` TTC:${det.ttc.toFixed(1)}s` : ''
      const newTag = det.is_new ? ' 🆕' : ''
      const fullLabel = label + distance + ttc + newTag

      ctx.fillStyle = color
      ctx.fillRect(x1, y1 - 30, ctx.measureText(fullLabel).width + 10, 25)

      // Draw label text
      ctx.fillStyle = '#ffffff'
      ctx.font = det.is_new ? 'bold 16px Arial' : '16px Arial'
      ctx.fillText(fullLabel, x1 + 5, y1 - 10)
    })
  }

  // Save detections to backend
  const saveDetections = async (detections: Detection[]) => {
    try {
      await fetch('http://localhost:8000/api/detections/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ detections })
      })
    } catch (error) {
      console.error('Error saving detections:', error)
    }
  }

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      stopWebcam()
      stopInference()
    }
  }, [])

  return (
    <div className="container mx-auto p-6 space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold flex items-center gap-2">
            <Shield className="w-8 h-8 text-primary" />
            ADAS Unified System
          </h1>
          <p className="text-muted-foreground mt-1">
            Hệ thống ADAS với AI tự học: Phát hiện MỌI đối tượng (80+ loại) + Tự động học từ dữ liệu mới + Cảnh báo thông minh
          </p>
        </div>
      </div>

      <Alert>
        <Zap className="h-4 w-4" />
        <AlertDescription>
          <strong>AI Model:</strong> YOLOv8n phát hiện TẤT CẢ 80 loại đối tượng COCO (xe, người, cây cối, động vật, vật thể, v.v.) + Lane Detection + Distance + TTC + Auto-Learning từ dữ liệu mới
        </AlertDescription>
      </Alert>

      {/* Camera Control */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Video className="w-5 h-5" />
            Camera & Live Stream
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="flex gap-2 flex-wrap">
            <Button
              onClick={startWebcam}
              disabled={!!stream || isVideoMode}
              variant={stream ? 'secondary' : 'default'}
            >
              <Video className="w-4 h-4 mr-2" />
              Bật Camera
            </Button>
            <Button
              onClick={() => fileInputRef.current?.click()}
              disabled={!!stream || isVideoMode}
              variant={videoFile ? 'secondary' : 'default'}
            >
              <Upload className="w-4 h-4 mr-2" />
              Upload Video
            </Button>
            <Input
              ref={fileInputRef}
              type="file"
              accept="video/*"
              onChange={handleVideoUpload}
              className="hidden"
            />
            <Button
              onClick={stopWebcam}
              disabled={!stream && !videoFile}
              variant="destructive"
            >
              <VideoOff className="w-4 h-4 mr-2" />
              {isVideoMode ? 'Xóa Video' : 'Tắt Camera'}
            </Button>
            <div className="flex-1" />
            <Button
              onClick={isStreaming ? stopInference : startInference}
              disabled={!stream && !videoFile}
              variant={isStreaming ? 'destructive' : 'default'}
            >
              <Activity className="w-4 h-4 mr-2" />
              {isStreaming ? 'Dừng Phát Hiện' : 'Bắt Đầu Phát Hiện'}
            </Button>
          </div>
          
          {/* Video Source Info */}
          {(stream || videoFile) && (
            <div className="text-sm text-muted-foreground">
              {isVideoMode ? (
                <Badge variant="secondary">📹 Video File: {videoFile?.name}</Badge>
              ) : (
                <Badge variant="secondary">🎥 Live Camera</Badge>
              )}
            </div>
          )}

          {/* Video Display */}
          <div className="relative w-full bg-black rounded-lg overflow-hidden" style={{ aspectRatio: '16/9' }}>
            <video
              ref={videoRef}
              autoPlay
              playsInline
              muted
              className="w-full h-full object-contain"
            />
            <canvas
              ref={canvasRef}
              className="absolute top-0 left-0 w-full h-full pointer-events-none"
            />
          </div>

          {/* Stats */}
          <div className="grid grid-cols-3 gap-4">
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-primary">{fps}</div>
                  <div className="text-sm text-muted-foreground">FPS</div>
                </div>
              </CardContent>
            </Card>
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-primary">{detectionCount}</div>
                  <div className="text-sm text-muted-foreground">Detections</div>
                </div>
              </CardContent>
            </Card>
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-primary">{stats.inference_time.toFixed(0)}ms</div>
                  <div className="text-sm text-muted-foreground">Inference Time</div>
                </div>
              </CardContent>
            </Card>
          </div>
        </CardContent>
      </Card>

      {/* Alerts Panel */}
      {alerts.length > 0 && (
        <Card className="border-red-500">
          <CardHeader>
            <CardTitle className="flex items-center gap-2 text-red-600">
              <AlertTriangle className="w-5 h-5" />
              Cảnh Báo An Toàn
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-2">
            {alerts.map((alert, index) => (
              <Alert key={index} variant={alert.level === 'critical' ? 'destructive' : 'default'}>
                <AlertCircle className="h-4 w-4" />
                <AlertDescription>
                  <strong className="uppercase">{alert.level}:</strong> {alert.message}
                  {alert.distance && ` (${alert.distance.toFixed(1)}m)`}
                  {alert.ttc && ` - TTC: ${alert.ttc.toFixed(1)}s`}
                </AlertDescription>
              </Alert>
            ))}
          </CardContent>
        </Card>
      )}

      {/* Auto-Learning Panel */}
      {newObjects.length > 0 && (
        <Card className="border-purple-500">
          <CardHeader>
            <CardTitle className="flex items-center gap-2 text-purple-600">
              <Zap className="w-5 h-5" />
              🆕 Phát Hiện Đối Tượng Mới - Đang Học!
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="space-y-2">
              <p className="text-sm text-muted-foreground">
                Hệ thống đã phát hiện và đang học các đối tượng mới:
              </p>
              <div className="flex flex-wrap gap-2">
                {newObjects.map((obj, idx) => (
                  <Badge key={idx} variant="outline" className="text-purple-600 border-purple-500">
                    {obj.class} ({obj.reason === 'new_class' ? 'Loại mới' : 'Chất lượng cao'})
                  </Badge>
                ))}
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Auto-Collection Stats */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Activity className="w-5 h-5" />
            Thống Kê Auto-Learning
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-primary">{stats.unique_classes?.length || 0}</div>
                  <div className="text-sm text-muted-foreground">Loại đối tượng</div>
                </div>
              </CardContent>
            </Card>
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-purple-600">{collectionStats.total_collected}</div>
                  <div className="text-sm text-muted-foreground">Đã thu thập</div>
                </div>
              </CardContent>
            </Card>
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-purple-600">{collectionStats.new_objects_learned}</div>
                  <div className="text-sm text-muted-foreground">Đối tượng mới học</div>
                </div>
              </CardContent>
            </Card>
            <Card>
              <CardContent className="pt-6">
                <div className="text-center">
                  <div className="text-2xl font-bold text-primary">{stats.new_objects_count || 0}</div>
                  <div className="text-sm text-muted-foreground">Mới (frame hiện tại)</div>
                </div>
              </CardContent>
            </Card>
          </div>
          
          {stats.unique_classes && stats.unique_classes.length > 0 && (
            <div className="mt-4">
              <p className="text-sm font-semibold mb-2">Đối tượng đang phát hiện:</p>
              <div className="flex flex-wrap gap-2">
                {stats.unique_classes.map((cls, idx) => (
                  <Badge key={idx} variant="secondary">{cls}</Badge>
                ))}
              </div>
            </div>
          )}
        </CardContent>
      </Card>

      {/* System Info */}
      <Card>
        <CardHeader>
          <CardTitle>System Performance & Capabilities</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 gap-4 text-sm">
            <div>
              <span className="font-semibold">Model:</span> YOLOv8n Unified (80 COCO Classes)
            </div>
            <div>
              <span className="font-semibold">Backend:</span> {isStreaming ? <Badge variant="default">Connected</Badge> : <Badge variant="secondary">Disconnected</Badge>}
            </div>
            <div>
              <span className="font-semibold">Features:</span> ALL Objects, Lanes, Distance, TTC, Auto-Learning
            </div>
            <div>
              <span className="font-semibold">Target FPS:</span> 10
            </div>
            <div className="col-span-2">
              <span className="font-semibold">Detectable Objects:</span> 
              <Badge variant="outline" className="ml-2">Vehicles</Badge>
              <Badge variant="outline" className="ml-2">Pedestrians</Badge>
              <Badge variant="outline" className="ml-2">Animals</Badge>
              <Badge variant="outline" className="ml-2">Trees/Plants</Badge>
              <Badge variant="outline" className="ml-2">Traffic Signs</Badge>
              <Badge variant="outline" className="ml-2">And 75+ more!</Badge>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Color Legend */}
      <Card>
        <CardHeader>
          <CardTitle>Bảng Màu Phát Hiện</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-red-500"></div>
              <span>Nguy hiểm (TTC &lt; 2s)</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-orange-500"></div>
              <span>Cảnh báo (TTC &lt; 3.5s)</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-green-500"></div>
              <span>An toàn (phương tiện)</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-cyan-500"></div>
              <span>Đối tượng thường (cây, vật)</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-purple-500"></div>
              <span>🆕 Đối tượng mới học</span>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
