"use client"

import { useEffect, useRef, useState } from "react"
import { AlertCircle, Play, Square } from "lucide-react"

export default function Live() {
  const videoRef = useRef(null)
  const canvasRef = useRef(null)
  const wsRef = useRef(null)
  const [isStreaming, setIsStreaming] = useState(false)
  const [detections, setDetections] = useState([])
  const [ttc, setTtc] = useState(null)
  const [stats, setStats] = useState({ infer_ms: 0, fps: 0 })
  const [error, setError] = useState("")
  const frameCountRef = useRef(0)
  const lastTimeRef = useRef(Date.now())

  useEffect(() => {
    const initWebSocket = () => {
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:"
      wsRef.current = new WebSocket(`${protocol}//${window.location.host}/ws/infer`)

      wsRef.current.onmessage = (event) => {
        const data = JSON.parse(event.data)
        setDetections(data.detections || [])
        setTtc(data.ttc)
        setStats(data.stats || {})
      }

      wsRef.current.onerror = () => {
        setError("Lỗi kết nối WebSocket")
      }
    }

    initWebSocket()
    return () => {
      if (wsRef.current) wsRef.current.close()
    }
  }, [])

  const startStream = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: "environment", width: { ideal: 1280 }, height: { ideal: 720 } },
      })

      if (videoRef.current) {
        videoRef.current.srcObject = stream
        videoRef.current.onloadedmetadata = () => {
          videoRef.current.play()
          setIsStreaming(true)
          captureFrames()
        }
      }
    } catch (err) {
      setError("Không thể truy cập camera: " + err.message)
    }
  }

  const stopStream = () => {
    if (videoRef.current && videoRef.current.srcObject) {
      videoRef.current.srcObject.getTracks().forEach((track) => track.stop())
      setIsStreaming(false)
      setDetections([])
    }
  }

  const captureFrames = () => {
    if (!isStreaming || !videoRef.current || !canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    const video = videoRef.current

    canvas.width = video.videoWidth
    canvas.height = video.videoHeight

    ctx.drawImage(video, 0, 0)

    // Vẽ bounding boxes
    detections.forEach((det) => {
      const [x, y, w, h] = det.bbox
      const color = det.cls === "car" ? "#ff0000" : "#00ff00"

      ctx.strokeStyle = color
      ctx.lineWidth = 2
      ctx.strokeRect(x, y, w, h)

      ctx.fillStyle = color
      ctx.font = "14px Arial"
      ctx.fillText(`${det.cls} ${(det.conf * 100).toFixed(0)}%`, x, y - 5)
    })

    // Hiển thị TTC
    if (ttc !== null && ttc < 1.5) {
      ctx.fillStyle = "rgba(255, 0, 0, 0.3)"
      ctx.fillRect(0, 0, canvas.width, canvas.height)
      ctx.fillStyle = "#ff0000"
      ctx.font = "bold 32px Arial"
      ctx.fillText(`⚠️ TTC: ${ttc.toFixed(2)}s`, 20, 50)
    }

    // Gửi frame qua WebSocket
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      canvas.toBlob(
        (blob) => {
          const reader = new FileReader()
          reader.onload = () => {
            wsRef.current.send(
              JSON.stringify({
                frame_b64: reader.result.split(",")[1],
              }),
            )
          }
          reader.readAsDataURL(blob)
        },
        "image/jpeg",
        0.8,
      )
    }

    frameCountRef.current++
    const now = Date.now()
    if (now - lastTimeRef.current >= 1000) {
      setStats((prev) => ({ ...prev, fps: frameCountRef.current }))
      frameCountRef.current = 0
      lastTimeRef.current = now
    }

    setTimeout(captureFrames, 100)
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 p-6">
      <div className="max-w-6xl mx-auto">
        <h1 className="text-4xl font-bold text-white mb-8">Nhận diện trực tiếp</h1>

        {error && (
          <div className="bg-red-500/20 border border-red-500 rounded-lg p-4 mb-6 flex items-center gap-3">
            <AlertCircle className="w-5 h-5 text-red-500" />
            <span className="text-red-200">{error}</span>
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <div className="lg:col-span-2">
            <div className="relative bg-black rounded-lg overflow-hidden shadow-2xl">
              <video ref={videoRef} className="w-full h-auto hidden" />
              <canvas ref={canvasRef} className="w-full h-auto" />
              {!isStreaming && (
                <div className="absolute inset-0 flex items-center justify-center bg-black/50">
                  <p className="text-white text-lg">Nhấn "Bắt đầu" để khởi động camera</p>
                </div>
              )}
            </div>

            <div className="flex gap-4 mt-6">
              <button
                onClick={startStream}
                disabled={isStreaming}
                className="flex items-center gap-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold transition"
              >
                <Play className="w-5 h-5" />
                Bắt đầu
              </button>
              <button
                onClick={stopStream}
                disabled={!isStreaming}
                className="flex items-center gap-2 bg-red-600 hover:bg-red-700 disabled:bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold transition"
              >
                <Square className="w-5 h-5" />
                Dừng
              </button>
            </div>
          </div>

          <div className="space-y-4">
            <div className="bg-slate-700/50 rounded-lg p-4 border border-slate-600">
              <h3 className="text-white font-semibold mb-3">Thống kê</h3>
              <div className="space-y-2 text-sm">
                <div className="flex justify-between text-gray-300">
                  <span>FPS:</span>
                  <span className="font-mono">{stats.fps}</span>
                </div>
                <div className="flex justify-between text-gray-300">
                  <span>Suy luận:</span>
                  <span className="font-mono">{stats.infer_ms}ms</span>
                </div>
                <div className="flex justify-between text-gray-300">
                  <span>Phát hiện:</span>
                  <span className="font-mono">{detections.length}</span>
                </div>
              </div>
            </div>

            {ttc !== null && (
              <div
                className={`rounded-lg p-4 border ${ttc < 1.5 ? "bg-red-500/20 border-red-500" : "bg-yellow-500/20 border-yellow-500"}`}
              >
                <h3 className={`font-semibold mb-2 ${ttc < 1.5 ? "text-red-300" : "text-yellow-300"}`}>
                  Thời gian đến va chạm
                </h3>
                <p className={`text-2xl font-bold ${ttc < 1.5 ? "text-red-400" : "text-yellow-400"}`}>
                  {ttc.toFixed(2)}s
                </p>
              </div>
            )}

            <div className="bg-slate-700/50 rounded-lg p-4 border border-slate-600">
              <h3 className="text-white font-semibold mb-3">Phát hiện</h3>
              <div className="space-y-2 max-h-48 overflow-y-auto">
                {detections.map((det, idx) => (
                  <div key={idx} className="text-xs bg-slate-600/50 p-2 rounded">
                    <div className="text-gray-300">{det.cls}</div>
                    <div className="text-gray-400">
                      {(det.conf * 100).toFixed(0)}% - {det.distance_m?.toFixed(1)}m
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
