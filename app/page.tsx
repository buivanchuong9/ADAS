"use client"

import { useState, useRef, useEffect } from "react"
import { Sidebar } from "@/components/sidebar"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { AlertTriangle, Play, Square } from "lucide-react"

export default function LiveDetection() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [isRunning, setIsRunning] = useState(false)
  const [detections, setDetections] = useState<any[]>([])

  const startDetection = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true })
      if (videoRef.current) {
        videoRef.current.srcObject = stream
        setIsRunning(true)
      }
    } catch (err) {
      console.error("Lỗi truy cập camera:", err)
    }
  }

  const stopDetection = () => {
    if (videoRef.current?.srcObject) {
      const tracks = (videoRef.current.srcObject as MediaStream).getTracks()
      tracks.forEach((track) => track.stop())
      setIsRunning(false)
    }
  }

  useEffect(() => {
    if (!isRunning || !videoRef.current || !canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    const drawFrame = () => {
      if (videoRef.current && ctx) {
        ctx.drawImage(videoRef.current, 0, 0, canvas.width, canvas.height)

        // TODO: Integrate YOLOv8 model for real object detection
        // Mock detection boxes for demo
        ctx.strokeStyle = "#ff6b35"
        ctx.lineWidth = 2
        ctx.strokeRect(100, 80, 150, 200)
        ctx.fillStyle = "#ff6b35"
        ctx.font = "14px Arial"
        ctx.fillText("Xe (0.95)", 105, 75)
      }
      requestAnimationFrame(drawFrame)
    }

    drawFrame()
  }, [isRunning])

  return (
    <div className="flex h-screen bg-background">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-8">
          <div className="mb-8">
            <h1 className="text-3xl font-bold text-foreground mb-2">Phát Hiện Trực Tiếp</h1>
            <p className="text-foreground/60">Giám sát thời gian thực các vật thể và nguy hiểm trên đường</p>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <div className="lg:col-span-2">
              <Card className="bg-card border-border overflow-hidden">
                <div className="relative bg-black aspect-video">
                  <video ref={videoRef} className="w-full h-full object-cover hidden" autoPlay playsInline />
                  <canvas ref={canvasRef} width={640} height={480} className="w-full h-full" />
                  {!isRunning && (
                    <div className="absolute inset-0 flex items-center justify-center bg-black/50">
                      <div className="text-center">
                        <p className="text-foreground/60 mb-4">Nhấn nút bên dưới để bắt đầu</p>
                      </div>
                    </div>
                  )}
                </div>

                <div className="p-4 border-t border-border flex gap-3">
                  {!isRunning ? (
                    <Button
                      onClick={startDetection}
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
                      Dừng Phát Hiện
                    </Button>
                  )}
                </div>
              </Card>
            </div>

            <div className="space-y-4">
              <Card className="bg-card border-border p-4">
                <h3 className="font-semibold text-foreground mb-4">Thống Kê Phát Hiện</h3>
                <div className="space-y-3">
                  <div className="flex justify-between items-center">
                    <span className="text-foreground/70">Xe Phát Hiện</span>
                    <span className="text-lg font-bold text-primary">3</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-foreground/70">Người Phát Hiện</span>
                    <span className="text-lg font-bold text-primary">2</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-foreground/70">Cảnh Báo</span>
                    <span className="text-lg font-bold text-destructive">1</span>
                  </div>
                </div>
              </Card>

              <Card className="bg-card border-border p-4 border-destructive/50 bg-destructive/5">
                <div className="flex gap-3">
                  <AlertTriangle className="w-5 h-5 text-destructive flex-shrink-0 mt-0.5" />
                  <div>
                    <h4 className="font-semibold text-foreground mb-1">Cảnh Báo Nguy Hiểm</h4>
                    <p className="text-sm text-foreground/70">Phát hiện xe phía trước quá gần (TTC: 1.2s)</p>
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
