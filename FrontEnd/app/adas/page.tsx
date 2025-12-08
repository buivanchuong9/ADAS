"use client"

import { useState, useEffect, useRef } from "react"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { ADASOverlay } from "@/components/adas/ADASOverlay"
import { ArrowLeft, Camera, Settings, Zap, AlertTriangle, Activity } from "lucide-react"
import Link from "next/link"
import { useToast } from "@/components/ui/use-toast"
import { getWebSocketUrl } from "@/lib/api-config"

export default function ADASPage() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const { toast } = useToast()

  const [isStreaming, setIsStreaming] = useState(false)
  const [modelStatus, setModelStatus] = useState("disconnected") // disconnected, connecting, ready
  const [fps, setFps] = useState(0)
  const [inferenceTime, setInferenceTime] = useState(0)
  const [adasData, setAdasData] = useState<any>(null)
  const [selectedCamera, setSelectedCamera] = useState<string>("")
  const [cameras, setCameras] = useState<MediaDeviceInfo[]>([])

  // Get available cameras
  useEffect(() => {
    async function getCameras() {
      try {
        const devices = await navigator.mediaDevices.enumerateDevices()
        const videoDevices = devices.filter(device => device.kind === 'videoinput')
        setCameras(videoDevices)
        if (videoDevices.length > 0) {
          setSelectedCamera(videoDevices[0].deviceId)
        }
      } catch (err) {
        console.error("Error getting cameras", err)
      }
    }
    getCameras()
  }, [])

  const connectWebSocket = () => {
    setModelStatus("connecting")
    // 🆕 Connect to NEW ADAS WebSocket endpoint with cross-platform URL support
    const wsUrl = getWebSocketUrl("/ws/adas/stream")
    console.log(`🔌 Connecting to ADAS WebSocket: ${wsUrl}`)
    const ws = new WebSocket(wsUrl)

    ws.onopen = () => {
      console.log("🔌 Connected to ADAS WebSocket")
      setModelStatus("ready")
      
      // Send initial config
      ws.send(JSON.stringify({
        type: 'config',
        config: {
          enable_tsr: true,
          enable_fcw: true,
          enable_ldw: true,
          enable_audio: false
        }
      }))
      
      toast({
        title: "ADAS System Ready",
        description: "Connected to ADAS Detection Backend",
      })
    }

    ws.onmessage = (event) => {
      try {
        const response = JSON.parse(event.data)
        const msgType = response.type || ''

        // Handle ADAS WebSocket response format
        if (msgType === 'adas_result') {
          // ADAS result from /ws/adas/stream
          const data = response.data || {}
          setAdasData({
            detections: data.fcw_detections || [],
            tracks: data.tracks || [],
            lanes: data.ldw_data?.lanes || [],
            ldw: {
              active: data.ldw_data?.is_departing || false,
              is_departing: data.ldw_data?.is_departing || false,
              departure_side: data.ldw_data?.departure_side || null
            },
            alerts: data.alerts || [],
            tsr: data.tsr_detections || [],
            vehicle_speed: data.vehicle_speed || 0,
            speed_limit: data.speed_limit || null,
            stats: {
              fps: response.fps || 0,
              process_time_ms: response.process_time_ms || 0
            }
          })
          setFps(response.fps || 0)
          setInferenceTime(response.process_time_ms || 0)
        } else if (msgType === 'config_updated') {
          console.log("✅ ADAS config updated:", response.config)
        } else if (msgType === 'ping') {
          // Heartbeat - no action needed
        } else if (msgType === 'error') {
          console.error("ADAS WebSocket error:", response.message)
          toast({
            title: "ADAS Error",
            description: response.message || "Unknown error",
            variant: "destructive"
          })
        } else {
          console.warn("Unknown message type:", msgType, response)
        }
      } catch (error) {
        console.error("Failed to parse WebSocket message:", error)
      }
    }

    ws.onerror = (error) => {
      console.error("WebSocket error:", error)
      setModelStatus("error")
      toast({
        title: "Connection Error",
        description: "Failed to connect to ADAS backend. Retrying...",
        variant: "destructive"
      })
    }

    ws.onclose = (event) => {
      console.log(`WebSocket closed (code: ${event.code}, reason: ${event.reason || 'none'})`)
      if (isStreaming && event.code !== 1000) {
        // Not a normal closure - attempt reconnection with exponential backoff
        setModelStatus("reconnecting")
        const reconnectDelay = Math.min(2000 * Math.pow(2, 0), 10000) // Max 10s delay
        setTimeout(() => {
          if (isStreaming) {
            console.log("🔄 Reconnecting to ADAS WebSocket...")
            connectWebSocket()
          }
        }, reconnectDelay)
      } else {
        setModelStatus("disconnected")
      }
    }

    wsRef.current = ws
  }

  const startStream = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: {
          deviceId: selectedCamera ? { exact: selectedCamera } : undefined,
          width: { ideal: 1280 },
          height: { ideal: 720 }
        }
      })

      if (videoRef.current) {
        videoRef.current.srcObject = stream
        videoRef.current.play()
      }

      setIsStreaming(true)
      connectWebSocket()

    } catch (err) {
      console.error("Error accessing camera:", err)
      toast({
        title: "Camera Error",
        description: "Could not access camera. Please check permissions.",
        variant: "destructive"
      })
    }
  }

  const stopStream = () => {
    if (videoRef.current && videoRef.current.srcObject) {
      const tracks = (videoRef.current.srcObject as MediaStream).getTracks()
      tracks.forEach(track => track.stop())
      videoRef.current.srcObject = null
    }

    if (wsRef.current) {
      wsRef.current.close()
      wsRef.current = null
    }

    setIsStreaming(false)
    setModelStatus("disconnected")
  }

  // Frame processing loop
  useEffect(() => {
    if (!isStreaming || modelStatus !== "ready") return

    const processFrame = () => {
      if (!videoRef.current || !canvasRef.current || !wsRef.current) return

      if (wsRef.current.readyState === WebSocket.OPEN) {
        const video = videoRef.current
        const canvas = canvasRef.current
        const ctx = canvas.getContext("2d")

        if (ctx && video.videoWidth > 0) {
          // Draw video to canvas (hidden)
          canvas.width = video.videoWidth
          canvas.height = video.videoHeight
          ctx.drawImage(video, 0, 0, canvas.width, canvas.height)

          // Convert to base64
          // Reduce quality slightly for speed if needed (0.7)
          const base64Frame = canvas.toDataURL("image/jpeg", 0.7)

          // Send to ADAS WebSocket endpoint with proper format
          wsRef.current.send(JSON.stringify({
            type: "frame",
            data: base64Frame,
            vehicle_speed: 0.0, // TODO: Get from GPS/OBD if available
            config: {
              enable_tsr: true,
              enable_fcw: true,
              enable_ldw: true,
              enable_audio: false
            }
          }))
        }
      }

      // Schedule next frame (limit to ~30 FPS to avoid flooding)
      requestAnimationFrame(processFrame)
    }

    const animationId = requestAnimationFrame(processFrame)
    return () => cancelAnimationFrame(animationId)
  }, [isStreaming, modelStatus])

  return (
    <div className="flex flex-col h-screen bg-background text-foreground overflow-hidden">
      {/* Premium Header with glass morphism */}
      <header className="flex items-center justify-between p-4 bg-card/60 border-b border-border/40 backdrop-blur-xl z-10">
        <div className="flex items-center gap-4">
          <Link href="/">
            <Button variant="ghost" size="icon" className="hover:bg-primary/10 hover:text-primary transition-all">
              <ArrowLeft className="w-6 h-6" />
            </Button>
          </Link>
          <div>
            <h1 className="text-xl font-bold flex items-center gap-2 bg-gradient-to-r from-primary via-accent to-primary bg-clip-text text-transparent">
              <Zap className="w-5 h-5 text-primary drop-shadow-[0_0_8px_hsl(189_94%_55%_/_0.5)]" />
              ADAS Pro v3.1 Drive Mode
            </h1>
            <div className="flex items-center gap-2 text-xs">
              <span className={`w-2 h-2 rounded-full animate-pulse ${
                modelStatus === 'ready' ? 'bg-success shadow-[0_0_8px_hsl(158_64%_52%)]' : 
                modelStatus === 'connecting' ? 'bg-warning shadow-[0_0_8px_hsl(38_92%_58%)]' :
                'bg-destructive'
              }`} />
              <span className={modelStatus === 'ready' ? 'text-success' : 'text-muted-foreground'}>
                {modelStatus.toUpperCase()}
              </span>
            </div>
          </div>
        </div>

        <div className="flex items-center gap-4">
          <div className="text-right hidden md:block bg-muted/50 px-4 py-2 rounded-lg border border-border/50">
            <div className="text-sm font-mono text-primary font-bold">{fps} FPS</div>
            <div className="text-xs text-muted-foreground">{inferenceTime}ms latency</div>
          </div>

          {!isStreaming ? (
            <Button onClick={startStream} className="bg-primary hover:bg-primary/90 text-primary-foreground shadow-lg glow-primary transition-all">
              <Camera className="w-4 h-4 mr-2" />
              Start System
            </Button>
          ) : (
            <Button onClick={stopStream} variant="destructive" className="shadow-lg">
              Stop System
            </Button>
          )}

          <Button variant="ghost" size="icon" className="hover:bg-muted">
            <Settings className="w-5 h-5" />
          </Button>
        </div>
      </header>

      {/* Main Viewport with premium gradient */}
      <main className="flex-1 relative bg-gradient-to-br from-background via-background to-muted/20 flex items-center justify-center overflow-hidden">
        {/* Video Feed Container */}
        <div className="relative w-full h-full max-w-[1920px] aspect-video">
          <video
            ref={videoRef}
            className="absolute top-0 left-0 w-full h-full object-contain rounded-lg"
            playsInline
            muted
          />

          {/* Overlay */}
          {isStreaming && videoRef.current && (
            <ADASOverlay
              width={videoRef.current.videoWidth || 1280}
              height={videoRef.current.videoHeight || 720}
              data={adasData}
            />
          )}

          {/* Hidden Canvas for processing */}
          <canvas ref={canvasRef} className="hidden" />

          {/* Premium placeholder when not streaming */}
          {!isStreaming && (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-card/30 backdrop-blur-2xl rounded-lg border border-border/50">
              <div className="relative">
                <Activity className="w-20 h-20 text-primary/40 mb-4 animate-pulse" />
                <div className="absolute inset-0 blur-xl bg-primary/20 rounded-full" />
              </div>
              <h2 className="text-3xl font-bold bg-gradient-to-r from-primary to-accent bg-clip-text text-transparent">
                System Standby
              </h2>
              <p className="text-muted-foreground mt-2 text-lg">Connect camera to activate ADAS</p>
            </div>
          )}
        </div>

        {/* Premium Dashboard Overlay */}
        <div className="absolute bottom-0 left-0 right-0 p-6 bg-gradient-to-t from-background via-background/95 to-transparent pt-24 pointer-events-none">
          <div className="max-w-7xl mx-auto grid grid-cols-3 gap-8 pointer-events-auto">
            {/* Left: Speed & Status with glow */}
            <div className="flex items-end gap-4 bg-card/60 backdrop-blur-xl border border-border/50 rounded-2xl p-4 shadow-xl">
              <div>
                <div className="text-7xl font-bold font-mono bg-gradient-to-br from-primary to-accent bg-clip-text text-transparent drop-shadow-2xl">
                  0
                </div>
                <div className="text-sm text-muted-foreground font-semibold tracking-wider">KM/H</div>
              </div>
              <div className="mb-3">
                <Badge className="bg-success/20 text-success border-success/50 shadow-lg glow-accent">
                  AUTOPILOT READY
                </Badge>
              </div>
            </div>

            {/* Center: Alerts with premium styling */}
            <div className="flex flex-col items-center justify-end pb-2">
              {adasData?.ldw?.is_departing && (
                <div className="flex items-center gap-3 px-6 py-3 bg-destructive/20 backdrop-blur-xl border-2 border-destructive/60 rounded-2xl text-destructive animate-pulse shadow-2xl glow-accent">
                  <AlertTriangle className="w-6 h-6" />
                  <span className="font-bold text-lg tracking-wide">LANE DEPARTURE</span>
                </div>
              )}
            </div>

            {/* Right: Detection stats with premium card */}
            <div className="flex justify-end items-end">
              <div className="text-right bg-card/60 backdrop-blur-xl border border-border/50 rounded-2xl p-4 shadow-xl">
                <div className="text-sm text-muted-foreground font-medium">Objects Detected</div>
                <div className="text-4xl font-bold bg-gradient-to-r from-accent to-primary bg-clip-text text-transparent">
                  {adasData?.tracks?.length || 0}
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  )
}
