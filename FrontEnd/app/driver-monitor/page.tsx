'use client'

import { useEffect, useRef, useState } from 'react'
import { Sidebar } from '@/components/sidebar'
import { MobileNav } from '@/components/mobile-nav'
import { GlassCard } from '@/components/ui/glass-card'
import { ScanningGrid } from '@/components/ui/scanning-grid'
import { NeonProgress } from '@/components/ui/neon-progress'
import { Button } from '@/components/ui/button'
import { AlertTriangle, Play, Square, Eye, Zap, Activity } from 'lucide-react'

export default function DriverMonitorPage() {
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [isMonitoring, setIsMonitoring] = useState(false)
  const [fatigueLevel, setFatigueLevel] = useState(0)
  const [distractionLevel, setDistractionLevel] = useState(0)
  const [eyesClosed, setEyesClosed] = useState(false)
  const [blinkRate, setBlinkRate] = useState(0)

  const startMonitoring = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true })
      if (videoRef.current) {
        videoRef.current.srcObject = stream
        setIsMonitoring(true)
      }
    } catch (err) {
      console.error("Camera access error:", err)
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

        // Mock detection data
        const mockFatigue = Math.random() * 100
        const mockDistraction = Math.random() * 100
        const mockEyesClosed = Math.random() > 0.7
        const mockBlinkRate = Math.floor(Math.random() * 30) + 10

        setFatigueLevel(Math.round(mockFatigue))
        setDistractionLevel(Math.round(mockDistraction))
        setEyesClosed(mockEyesClosed)
        setBlinkRate(mockBlinkRate)

        // Draw face detection box
        ctx.strokeStyle = mockFatigue > 60 ? "#FF3B3B" : "#00E5FF"
        ctx.lineWidth = 3
        ctx.strokeRect(150, 100, 200, 250)
        ctx.shadowBlur = 15
        ctx.shadowColor = mockFatigue > 60 ? "#FF3B3B" : "#00E5FF"

        // Draw eye indicators
        ctx.fillStyle = mockEyesClosed ? "#FF3B3B" : "#00FFA3"
        ctx.shadowBlur = 10
        ctx.shadowColor = mockEyesClosed ? "#FF3B3B" : "#00FFA3"

        ctx.beginPath()
        ctx.arc(200, 150, 8, 0, Math.PI * 2)
        ctx.fill()

        ctx.beginPath()
        ctx.arc(300, 150, 8, 0, Math.PI * 2)
        ctx.fill()
      }
      requestAnimationFrame(drawFrame)
    }

    drawFrame()
  }, [isMonitoring])

  const fatigueStatus = fatigueLevel < 40 ? 'safe' : fatigueLevel < 70 ? 'warning' : 'danger'
  const distractionStatus = distractionLevel < 40 ? 'safe' : distractionLevel < 70 ? 'warning' : 'danger'

  return (
    <div className="flex h-screen bg-bg-primary">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8">
          {/* Header */}
          <div className="mb-6">
            <h1 className="text-3xl font-bold text-neon-cyan tracking-wider">DRIVER MONITOR</h1>
            <p className="text-sm text-fg-secondary mt-1">
              Real-time fatigue and distraction detection system
            </p>
          </div>

          <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
            {/* Video Feed - Takes 2 columns */}
            <div className="xl:col-span-2">
              <GlassCard
                glow={fatigueLevel > 60 ? "red" : "cyan"}
                pulse={fatigueLevel > 60}
                className="overflow-hidden"
              >
                {/* Cyberpunk Frame */}
                <div className="relative bg-bg-secondary aspect-video cyberpunk-frame">
                  <video ref={videoRef} className="w-full h-full object-cover hidden" autoPlay playsInline />
                  <canvas ref={canvasRef} width={640} height={480} className="w-full h-full" />

                  {!isMonitoring && (
                    <div className="absolute inset-0">
                      <ScanningGrid message="CAMERA OFFLINE" />
                    </div>
                  )}

                  {/* AR Overlays - Only show when monitoring */}
                  {isMonitoring && (
                    <>
                      {/* Top-left: Fatigue Indicator */}
                      <div className="absolute top-4 left-4 glass-overlay p-3 rounded-lg min-w-[200px]">
                        <div className="flex items-center gap-2 mb-2">
                          <Zap className="w-4 h-4 text-neon-yellow" />
                          <span className="text-xs text-fg-secondary uppercase tracking-wider">Fatigue Level</span>
                        </div>
                        <div className="digital-number text-2xl font-bold mb-2" style={{
                          color: fatigueStatus === 'danger' ? '#FF3B3B' : fatigueStatus === 'warning' ? '#FFD700' : '#00FFA3'
                        }}>
                          {fatigueLevel}%
                        </div>
                        <NeonProgress value={fatigueLevel} status={fatigueStatus} showLabel={false} />
                      </div>

                      {/* Top-right: Distraction Indicator */}
                      <div className="absolute top-4 right-4 glass-overlay p-3 rounded-lg min-w-[200px]">
                        <div className="flex items-center gap-2 mb-2">
                          <Eye className="w-4 h-4 text-neon-cyan" />
                          <span className="text-xs text-fg-secondary uppercase tracking-wider">Distraction</span>
                        </div>
                        <div className="digital-number text-2xl font-bold mb-2" style={{
                          color: distractionStatus === 'danger' ? '#FF3B3B' : distractionStatus === 'warning' ? '#FFD700' : '#00FFA3'
                        }}>
                          {distractionLevel}%
                        </div>
                        <NeonProgress value={distractionLevel} status={distractionStatus} showLabel={false} />
                      </div>

                      {/* Bottom-left: Eye Status */}
                      <div className="absolute bottom-4 left-4 glass-overlay p-3 rounded-lg">
                        <div className="flex items-center gap-3">
                          <div className={`w-3 h-3 rounded-full ${eyesClosed ? 'bg-neon-red' : 'bg-neon-green'}`}
                            style={{ boxShadow: `0 0 10px ${eyesClosed ? 'var(--neon-red)' : 'var(--neon-green)'}` }}
                          />
                          <span className="text-xs text-fg-primary font-semibold">
                            {eyesClosed ? 'EYES CLOSED' : 'EYES OPEN'}
                          </span>
                        </div>
                      </div>

                      {/* Bottom-right: Blink Rate */}
                      <div className="absolute bottom-4 right-4 glass-overlay p-3 rounded-lg">
                        <div className="text-xs text-fg-secondary uppercase tracking-wider mb-1">Blink Rate</div>
                        <div className="digital-number text-lg font-bold text-neon-cyan">
                          {blinkRate} /min
                        </div>
                      </div>
                    </>
                  )}
                </div>

                {/* Controls */}
                <div className="p-4 border-t border-white/10 flex gap-3">
                  {!isMonitoring ? (
                    <button
                      onClick={startMonitoring}
                      className="btn-neon flex-1 flex items-center justify-center gap-2"
                    >
                      <Play className="w-4 h-4" />
                      START MONITORING
                    </button>
                  ) : (
                    <button
                      onClick={stopMonitoring}
                      className="btn-neon-red flex-1 flex items-center justify-center gap-2"
                    >
                      <Square className="w-4 h-4" />
                      STOP MONITORING
                    </button>
                  )}
                </div>
              </GlassCard>
            </div>

            {/* Right Panel - Stats */}
            <div className="space-y-4">
              {/* Alert Card */}
              {(eyesClosed || fatigueLevel > 60 || distractionLevel > 60) && (
                <GlassCard glow="red" pulse className="p-4">
                  <div className="flex gap-3">
                    <AlertTriangle className="w-6 h-6 text-neon-red shrink-0 mt-0.5" />
                    <div>
                      <h4 className="font-bold text-neon-red mb-2 tracking-wide">⚠ ALERT</h4>
                      <div className="text-sm text-fg-primary space-y-1">
                        {eyesClosed && <p>• Eyes closed detected</p>}
                        {fatigueLevel > 60 && <p>• High fatigue level</p>}
                        {distractionLevel > 60 && <p>• Driver distracted</p>}
                      </div>
                    </div>
                  </div>
                </GlassCard>
              )}

              {/* Status Overview */}
              <GlassCard scanLines className="p-4">
                <h3 className="font-bold text-neon-cyan mb-4 tracking-wide">SYSTEM STATUS</h3>
                <div className="space-y-3 text-sm">
                  <div className="flex justify-between items-center">
                    <span className="text-fg-secondary">Camera:</span>
                    <span className={`font-semibold ${isMonitoring ? 'text-neon-green' : 'text-neon-red'}`}>
                      {isMonitoring ? 'ACTIVE' : 'OFFLINE'}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-fg-secondary">Eyes:</span>
                    <span className={`font-semibold ${eyesClosed ? 'text-neon-red' : 'text-neon-green'}`}>
                      {eyesClosed ? 'CLOSED' : 'OPEN'}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-fg-secondary">Condition:</span>
                    <span className={`font-semibold ${fatigueLevel > 60 ? 'text-neon-red' : 'text-neon-green'}`}>
                      {fatigueLevel > 60 ? 'FATIGUED' : 'NORMAL'}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-fg-secondary">Focus:</span>
                    <span className={`font-semibold ${distractionLevel > 60 ? 'text-neon-red' : 'text-neon-green'}`}>
                      {distractionLevel > 60 ? 'DISTRACTED' : 'FOCUSED'}
                    </span>
                  </div>
                </div>
              </GlassCard>

              {/* Metrics */}
              <GlassCard className="p-4">
                <h3 className="font-bold text-neon-cyan mb-4 tracking-wide">METRICS</h3>
                <div className="space-y-4">
                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <span className="text-xs text-fg-secondary uppercase">Fatigue</span>
                      <span className="digital-number text-sm font-bold text-fg-primary">{fatigueLevel}%</span>
                    </div>
                    <NeonProgress value={fatigueLevel} status={fatigueStatus} showLabel={false} />
                  </div>

                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <span className="text-xs text-fg-secondary uppercase">Distraction</span>
                      <span className="digital-number text-sm font-bold text-fg-primary">{distractionLevel}%</span>
                    </div>
                    <NeonProgress value={distractionLevel} status={distractionStatus} showLabel={false} />
                  </div>

                  <div>
                    <div className="flex justify-between items-center mb-2">
                      <span className="text-xs text-fg-secondary uppercase">Blink Rate</span>
                      <span className="digital-number text-sm font-bold text-neon-cyan">{blinkRate} /min</span>
                    </div>
                    <div className="text-xs text-fg-muted">
                      Normal: 15-20 /min
                    </div>
                  </div>
                </div>
              </GlassCard>
            </div>
          </div>
        </div>
      </main>
    </div>
  )
}
