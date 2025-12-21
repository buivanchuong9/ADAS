'use client'

import { useEffect, useRef, useState } from 'react'
import { Sidebar } from '@/components/sidebar'
import { MobileNav } from '@/components/mobile-nav'
import { Card } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { AlertTriangle, Play, Square, Eye, Zap } from 'lucide-react'

export default function DriverMonitorPage() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [isMonitoring, setIsMonitoring] = useState(false)
  const [fatigueLevel, setFatigueLevel] = useState(0)
  const [distractionLevel, setDistractionLevel] = useState(0)
  const [eyesClosed, setEyesClosed] = useState(false)

  const startMonitoring = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true })
      if (videoRef.current) {
        videoRef.current.srcObject = stream
        setIsMonitoring(true)
      }
    } catch (err) {
      console.error("Lỗi truy cập camera:", err)
    }
  }

  const stopMonitoring = () => {
    if (videoRef.current?.srcObject) {
      const tracks = (videoRef.current.srcObject as MediaStream).getTracks()
      tracks.forEach((track) => track.stop())
      setIsMonitoring(false)
    }
  }

  useEffect(() => {
    if (!isMonitoring || !videoRef.current || !canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    const drawFrame = () => {
      if (videoRef.current && ctx) {
        ctx.drawImage(videoRef.current, 0, 0, canvas.width, canvas.height)

        // Mock fatigue and distraction detection
        const mockFatigue = Math.random() * 100
        const mockDistraction = Math.random() * 100
        const mockEyesClosed = Math.random() > 0.7

        setFatigueLevel(Math.round(mockFatigue))
        setDistractionLevel(Math.round(mockDistraction))
        setEyesClosed(mockEyesClosed)

        // Draw face detection box
        ctx.strokeStyle = mockFatigue > 60 ? "#ff6b35" : "#4ade80"
        ctx.lineWidth = 3
        ctx.strokeRect(150, 100, 200, 250)

        // Draw eye indicators
        ctx.fillStyle = mockEyesClosed ? "#ff6b35" : "#4ade80"
        ctx.beginPath()
        ctx.arc(200, 150, 8, 0, Math.PI * 2)
        ctx.fill()

        ctx.beginPath()
        ctx.arc(300, 150, 8, 0, Math.PI * 2)
        ctx.fill()

        // Draw status text
        ctx.fillStyle = "#ffffff"
        ctx.font = "14px Arial"
        ctx.fillText(`Mệt Mỏi: ${Math.round(mockFatigue)}%`, 160, 380)
        ctx.fillText(`Phân Tán: ${Math.round(mockDistraction)}%`, 160, 400)
      }
      requestAnimationFrame(drawFrame)
    }

    drawFrame()
  }, [isMonitoring])

  return (
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8">
          <div className="mb-6 sm:mb-8">
            <h1 className="text-2xl sm:text-3xl font-bold text-gray-900 mb-2">Giám Sát Tài Xế</h1>
            <p className="text-sm sm:text-base text-gray-600">Theo dõi tình trạng tài xế và phát hiện mệt mỏi, phân tán</p>
          </div>

          <div className="grid grid-cols-1 xl:grid-cols-3 gap-4 sm:gap-6">
            <div className="xl:col-span-2">
              <Card className="bg-card border-border overflow-hidden">
                <div className="relative bg-black aspect-video">
                  <video ref={videoRef} className="w-full h-full object-cover hidden" autoPlay playsInline />
                  <canvas ref={canvasRef} width={640} height={480} className="w-full h-full" />
                  {!isMonitoring && (
                    <div className="absolute inset-0 flex items-center justify-center bg-black/50">
                      <div className="text-center">
                        <p className="text-foreground/60 mb-4">Nhấn nút bên dưới để bắt đầu giám sát</p>
                      </div>
                    </div>
                  )}
                </div>

                <div className="p-4 border-t border-border flex gap-3">
                  {!isMonitoring ? (
                    <Button
                      onClick={startMonitoring}
                      className="flex-1 bg-primary hover:bg-primary/90 text-primary-foreground"
                    >
                      <Play className="w-4 h-4 mr-2" />
                      Bắt Đầu Giám Sát
                    </Button>
                  ) : (
                    <Button
                      onClick={stopMonitoring}
                      className="flex-1 bg-destructive hover:bg-destructive/90 text-destructive-foreground"
                    >
                      <Square className="w-4 h-4 mr-2" />
                      Dừng Giám Sát
                    </Button>
                  )}
                </div>
              </Card>
            </div>

            <div className="space-y-4">
              <Card className="bg-card border-border p-4">
                <h3 className="font-semibold text-foreground mb-4">Chỉ Số Tài Xế</h3>
                <div className="space-y-4">
                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <span className="text-sm text-foreground/70 flex items-center gap-2">
                        <Zap className="w-4 h-4" />
                        Mệt Mỏi
                      </span>
                      <span className="text-lg font-bold text-primary">{fatigueLevel}%</span>
                    </div>
                    <div className="w-full bg-muted rounded-full h-2">
                      <div
                        className={`h-2 rounded-full transition-all ${fatigueLevel > 60 ? "bg-destructive" : "bg-primary"
                          }`}
                        style={{ width: `${fatigueLevel}%` }}
                      />
                    </div>
                  </div>

                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <span className="text-sm text-foreground/70 flex items-center gap-2">
                        <Eye className="w-4 h-4" />
                        Phân Tán
                      </span>
                      <span className="text-lg font-bold text-primary">{distractionLevel}%</span>
                    </div>
                    <div className="w-full bg-muted rounded-full h-2">
                      <div
                        className={`h-2 rounded-full transition-all ${distractionLevel > 60 ? "bg-destructive" : "bg-primary"
                          }`}
                        style={{ width: `${distractionLevel}%` }}
                      />
                    </div>
                  </div>
                </div>
              </Card>

              {(eyesClosed || fatigueLevel > 60 || distractionLevel > 60) && (
                <Card className="p-4 border-destructive/50 bg-destructive/5">
                  <div className="flex gap-3">
                    <AlertTriangle className="w-5 h-5 text-destructive shrink-0 mt-0.5" />
                    <div>
                      <h4 className="font-semibold text-foreground mb-1">Cảnh Báo</h4>
                      <p className="text-sm text-foreground/70">
                        {eyesClosed && "Phát hiện mắt đóng. "}
                        {fatigueLevel > 60 && "Mức mệt mỏi cao. "}
                        {distractionLevel > 60 && "Phát hiện phân tán."}
                      </p>
                    </div>
                  </div>
                </Card>
              )}

              <Card className="bg-card border-border p-4">
                <h3 className="font-semibold text-foreground mb-3">Trạng Thái</h3>
                <div className="space-y-2 text-sm">
                  <div className="flex justify-between">
                    <span className="text-foreground/70">Mắt:</span>
                    <span className={eyesClosed ? "text-destructive font-semibold" : "text-green-400 font-semibold"}>
                      {eyesClosed ? "Đóng" : "Mở"}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-foreground/70">Tình Trạng:</span>
                    <span
                      className={fatigueLevel > 60 ? "text-destructive font-semibold" : "text-green-400 font-semibold"}
                    >
                      {fatigueLevel > 60 ? "Mệt Mỏi" : "Bình Thường"}
                    </span>
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
