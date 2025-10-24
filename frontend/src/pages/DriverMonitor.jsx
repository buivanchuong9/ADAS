"use client"

import { useEffect, useRef, useState } from "react"
import { AlertTriangle, Eye, EyeOff } from "lucide-react"

export default function DriverMonitor() {
  const videoRef = useRef(null)
  const canvasRef = useRef(null)
  const [isMonitoring, setIsMonitoring] = useState(false)
  const [fatigueLevel, setFatigueLevel] = useState(0)
  const [eyesClosed, setEyesClosed] = useState(false)
  const [alert, setAlert] = useState("")
  const eyeClosureTimeRef = useRef(0)

  useEffect(() => {
    if (!isMonitoring) return

    const startMonitoring = async () => {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({
          video: { facingMode: "user", width: { ideal: 640 }, height: { ideal: 480 } },
        })

        if (videoRef.current) {
          videoRef.current.srcObject = stream
          videoRef.current.onloadedmetadata = () => {
            videoRef.current.play()
            detectFatigue()
          }
        }
      } catch (err) {
        setAlert("Không thể truy cập camera: " + err.message)
      }
    }

    startMonitoring()

    return () => {
      if (videoRef.current && videoRef.current.srcObject) {
        videoRef.current.srcObject.getTracks().forEach((track) => track.stop())
      }
    }
  }, [isMonitoring])

  const detectFatigue = () => {
    if (!isMonitoring || !videoRef.current || !canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    const video = videoRef.current

    canvas.width = video.videoWidth
    canvas.height = video.videoHeight

    ctx.drawImage(video, 0, 0)

    // TODO: Tích hợp MediaPipe FaceMesh để phát hiện mắt
    // Hiện tại sử dụng mô phỏng
    const isClosed = Math.random() > 0.7

    if (isClosed) {
      eyeClosureTimeRef.current += 100
    } else {
      eyeClosureTimeRef.current = 0
    }

    setEyesClosed(isClosed)
    setFatigueLevel(Math.min(100, (eyeClosureTimeRef.current / 2000) * 100))

    if (eyeClosureTimeRef.current > 2000) {
      setAlert("⚠️ Cảnh báo: Bạn đang mệt mỏi!")
      // TODO: Phát âm thanh cảnh báo
    } else {
      setAlert("")
    }

    setTimeout(detectFatigue, 100)
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 p-6">
      <div className="max-w-4xl mx-auto">
        <h1 className="text-4xl font-bold text-white mb-8">Giám sát tài xế</h1>

        {alert && (
          <div className="bg-red-500/20 border border-red-500 rounded-lg p-4 mb-6 flex items-center gap-3">
            <AlertTriangle className="w-5 h-5 text-red-500" />
            <span className="text-red-200">{alert}</span>
          </div>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          <div className="lg:col-span-2">
            <div className="relative bg-black rounded-lg overflow-hidden shadow-2xl">
              <video ref={videoRef} className="w-full h-auto hidden" />
              <canvas ref={canvasRef} className="w-full h-auto" />
              {!isMonitoring && (
                <div className="absolute inset-0 flex items-center justify-center bg-black/50">
                  <p className="text-white text-lg">Nhấn "Bắt đầu giám sát" để khởi động</p>
                </div>
              )}
            </div>

            <button
              onClick={() => setIsMonitoring(!isMonitoring)}
              className={`mt-6 w-full py-3 rounded-lg font-semibold text-white transition ${
                isMonitoring ? "bg-red-600 hover:bg-red-700" : "bg-green-600 hover:bg-green-700"
              }`}
            >
              {isMonitoring ? "Dừng giám sát" : "Bắt đầu giám sát"}
            </button>
          </div>

          <div className="space-y-4">
            <div className="bg-slate-700/50 rounded-lg p-4 border border-slate-600">
              <h3 className="text-white font-semibold mb-4">Mức độ mệt mỏi</h3>
              <div className="w-full bg-slate-600 rounded-full h-4 overflow-hidden">
                <div
                  className={`h-full transition-all ${
                    fatigueLevel > 70 ? "bg-red-500" : fatigueLevel > 40 ? "bg-yellow-500" : "bg-green-500"
                  }`}
                  style={{ width: `${fatigueLevel}%` }}
                />
              </div>
              <p className="text-gray-300 mt-2 text-center font-mono">{fatigueLevel.toFixed(0)}%</p>
            </div>

            <div className="bg-slate-700/50 rounded-lg p-4 border border-slate-600">
              <h3 className="text-white font-semibold mb-3">Trạng thái mắt</h3>
              <div className="flex items-center gap-3">
                {eyesClosed ? (
                  <>
                    <EyeOff className="w-6 h-6 text-red-500" />
                    <span className="text-red-300">Mắt đóng</span>
                  </>
                ) : (
                  <>
                    <Eye className="w-6 h-6 text-green-500" />
                    <span className="text-green-300">Mắt mở</span>
                  </>
                )}
              </div>
            </div>

            <div className="bg-slate-700/50 rounded-lg p-4 border border-slate-600">
              <h3 className="text-white font-semibold mb-3">Khuyến nghị</h3>
              <ul className="text-sm text-gray-300 space-y-2">
                <li>• Nghỉ ngơi nếu mệt mỏi</li>
                <li>• Uống nước</li>
                <li>• Dừng xe an toàn</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
