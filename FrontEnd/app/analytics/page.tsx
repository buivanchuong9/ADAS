"use client"

import { Sidebar } from "@/components/sidebar"
import { MobileNav } from "@/components/mobile-nav"
import { GlassCard } from "@/components/ui/glass-card"
import { Badge } from "@/components/ui/badge"
import {
  LineChart,
  Line,
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts"
import { TrendingUp, Clock, Gauge, AlertTriangle } from "lucide-react"

const speedData = [
  { time: "0:00", speed: 0 },
  { time: "0:15", speed: 45 },
  { time: "0:30", speed: 60 },
  { time: "0:45", speed: 55 },
  { time: "1:00", speed: 70 },
  { time: "1:15", speed: 65 },
  { time: "1:30", speed: 50 },
  { time: "1:45", speed: 40 },
  { time: "2:00", speed: 0 },
]

const fatigueData = [
  { time: "0:00", fatigue: 10 },
  { time: "0:30", fatigue: 15 },
  { time: "1:00", fatigue: 25 },
  { time: "1:30", fatigue: 35 },
  { time: "2:00", fatigue: 40 },
]

const tripComparisonData = [
  { trip: "Hôm Nay", score: 85 },
  { trip: "Hôm Qua", score: 78 },
  { trip: "3 Ngày Trước", score: 82 },
  { trip: "1 Tuần Trước", score: 75 },
]

export default function Analytics() {
  return (
    <div className="flex h-screen bg-bg-primary">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8 space-y-4 sm:space-y-6">
          {/* Header */}
          <div className="mb-6 sm:mb-8">
            <h1 className="text-3xl font-bold text-neon-cyan tracking-wider">ANALYTICS</h1>
            <p className="text-sm text-fg-secondary mt-1">Xem chi tiết thống kê và phân tích về chuyến đi của bạn</p>
          </div>

          {/* Stats Cards */}
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3 sm:gap-4 mb-6 sm:mb-8">
            <GlassCard glow="cyan" className="p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-xs text-fg-secondary uppercase tracking-wider mb-1">Quãng Đường</p>
                  <p className="text-2xl font-bold text-neon-cyan digital-number">42.5 km</p>
                </div>
                <TrendingUp className="w-8 h-8 text-neon-cyan" />
              </div>
            </GlassCard>

            <GlassCard glow="green" className="p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-xs text-fg-secondary uppercase tracking-wider mb-1">Thời Gian Lái</p>
                  <p className="text-2xl font-bold text-neon-green digital-number">2h 15m</p>
                </div>
                <Clock className="w-8 h-8 text-neon-green" />
              </div>
            </GlassCard>

            <GlassCard glow="yellow" className="p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-xs text-fg-secondary uppercase tracking-wider mb-1">Tốc Độ Trung Bình</p>
                  <p className="text-2xl font-bold text-neon-yellow digital-number">54 km/h</p>
                </div>
                <Gauge className="w-8 h-8 text-neon-yellow" />
              </div>
            </GlassCard>

            <GlassCard glow="green" className="p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-xs text-fg-secondary uppercase tracking-wider mb-1">Điểm An Toàn</p>
                  <p className="text-2xl font-bold text-neon-green digital-number">85/100</p>
                </div>
                <AlertTriangle className="w-8 h-8 text-neon-green" />
              </div>
            </GlassCard>
          </div>

          {/* Charts */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            <GlassCard scanLines className="p-6">
              <h3 className="text-xl font-bold text-neon-cyan mb-4 tracking-wide">TỐC ĐỘ THEO THỜI GIAN</h3>
              <ResponsiveContainer width="100%" height={300}>
                <LineChart data={speedData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.1)" />
                  <XAxis dataKey="time" stroke="var(--fg-secondary)" />
                  <YAxis stroke="var(--fg-secondary)" />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: "rgba(0, 0, 0, 0.8)",
                      border: "1px solid var(--neon-cyan)",
                      borderRadius: "8px",
                      backdropFilter: "blur(10px)"
                    }}
                    labelStyle={{ color: "var(--neon-cyan)" }}
                    itemStyle={{ color: "var(--fg-primary)" }}
                  />
                  <Legend wrapperStyle={{ color: "var(--fg-primary)" }} />
                  <Line
                    type="monotone"
                    dataKey="speed"
                    stroke="var(--neon-cyan)"
                    strokeWidth={3}
                    dot={{ fill: "var(--neon-cyan)", r: 5, strokeWidth: 2, stroke: "var(--bg-primary)" }}
                    name="Tốc Độ (km/h)"
                  />
                </LineChart>
              </ResponsiveContainer>
            </GlassCard>

            <GlassCard scanLines className="p-6">
              <h3 className="text-xl font-bold text-neon-red mb-4 tracking-wide">DIỄN BIẾN TRẠNG THÁI MỆT MỎI</h3>
              <ResponsiveContainer width="100%" height={300}>
                <LineChart data={fatigueData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.1)" />
                  <XAxis dataKey="time" stroke="var(--fg-secondary)" />
                  <YAxis stroke="var(--fg-secondary)" />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: "rgba(0, 0, 0, 0.8)",
                      border: "1px solid var(--neon-red)",
                      borderRadius: "8px",
                      backdropFilter: "blur(10px)"
                    }}
                    labelStyle={{ color: "var(--neon-red)" }}
                    itemStyle={{ color: "var(--fg-primary)" }}
                  />
                  <Legend wrapperStyle={{ color: "var(--fg-primary)" }} />
                  <Line
                    type="monotone"
                    dataKey="fatigue"
                    stroke="var(--neon-red)"
                    strokeWidth={3}
                    dot={{ fill: "var(--neon-red)", r: 5, strokeWidth: 2, stroke: "var(--bg-primary)" }}
                    name="Mệt Mỏi (%)"
                  />
                </LineChart>
              </ResponsiveContainer>
            </GlassCard>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <GlassCard scanLines className="p-6">
              <h3 className="text-xl font-bold text-neon-green mb-4 tracking-wide">SO SÁNH ĐIỂM AN TOÀN</h3>
              <ResponsiveContainer width="100%" height={300}>
                <BarChart data={tripComparisonData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.1)" />
                  <XAxis dataKey="trip" stroke="var(--fg-secondary)" />
                  <YAxis stroke="var(--fg-secondary)" />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: "rgba(0, 0, 0, 0.8)",
                      border: "1px solid var(--neon-green)",
                      borderRadius: "8px",
                      backdropFilter: "blur(10px)"
                    }}
                    labelStyle={{ color: "var(--neon-green)" }}
                    itemStyle={{ color: "var(--fg-primary)" }}
                  />
                  <Bar dataKey="score" fill="var(--neon-green)" name="Điểm An Toàn" radius={[8, 8, 0, 0]} />
                </BarChart>
              </ResponsiveContainer>
            </GlassCard>

            <GlassCard className="p-6">
              <h3 className="text-xl font-bold text-neon-yellow mb-4 tracking-wide">KHUYẾN NGHỊ</h3>
              <div className="space-y-3">
                <div className="p-3 glass-card border-l-4 border-neon-cyan rounded-lg">
                  <p className="text-sm font-semibold text-neon-cyan mb-1">Tăng Khoảng Cách An Toàn</p>
                  <p className="text-xs text-fg-secondary">
                    Bạn đã có 2 cảnh báo va chạm. Hãy tăng khoảng cách với xe phía trước.
                  </p>
                </div>
                <div className="p-3 glass-card border-l-4 border-neon-green rounded-lg">
                  <p className="text-sm font-semibold text-neon-green mb-1">Nghỉ Ngơi Thường Xuyên</p>
                  <p className="text-xs text-fg-secondary">
                    Mức mệt mỏi tăng nhanh sau 1.5 giờ lái. Hãy nghỉ ngơi 15 phút.
                  </p>
                </div>
                <div className="p-3 glass-card border-l-4 border-neon-yellow rounded-lg">
                  <p className="text-sm font-semibold text-neon-yellow mb-1">Tuân Thủ Giới Hạn Tốc Độ</p>
                  <p className="text-xs text-fg-secondary">
                    Bạn đã vượt tốc độ 3 lần. Hãy tuân thủ giới hạn tốc độ để an toàn hơn.
                  </p>
                </div>
              </div>
            </GlassCard>
          </div>
        </div>
      </main>
    </div>
  )
}
