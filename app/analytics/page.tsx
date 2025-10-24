"use client"

import { Sidebar } from "@/components/sidebar"
import { Card } from "@/components/ui/card"
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
    <div className="flex h-screen bg-background">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-8">
          <div className="mb-8">
            <h1 className="text-3xl font-bold text-foreground mb-2">Phân Tích Chuyến Đi</h1>
            <p className="text-foreground/60">Xem chi tiết thống kê và phân tích về chuyến đi của bạn</p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-8">
            <Card className="bg-card border-border p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-foreground/60 mb-1">Quãng Đường</p>
                  <p className="text-2xl font-bold text-foreground">42.5 km</p>
                </div>
                <TrendingUp className="w-8 h-8 text-primary" />
              </div>
            </Card>

            <Card className="bg-card border-border p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-foreground/60 mb-1">Thời Gian Lái</p>
                  <p className="text-2xl font-bold text-foreground">2h 15m</p>
                </div>
                <Clock className="w-8 h-8 text-primary" />
              </div>
            </Card>

            <Card className="bg-card border-border p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-foreground/60 mb-1">Tốc Độ Trung Bình</p>
                  <p className="text-2xl font-bold text-foreground">54 km/h</p>
                </div>
                <Gauge className="w-8 h-8 text-primary" />
              </div>
            </Card>

            <Card className="bg-card border-border p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-foreground/60 mb-1">Điểm An Toàn</p>
                  <p className="text-2xl font-bold text-primary">85/100</p>
                </div>
                <AlertTriangle className="w-8 h-8 text-primary" />
              </div>
            </Card>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            <Card className="bg-card border-border p-6">
              <h3 className="text-lg font-semibold text-foreground mb-4">Tốc Độ Theo Thời Gian</h3>
              <ResponsiveContainer width="100%" height={300}>
                <LineChart data={speedData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="var(--color-border)" />
                  <XAxis dataKey="time" stroke="var(--color-foreground)" />
                  <YAxis stroke="var(--color-foreground)" />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: "var(--color-card)",
                      border: "1px solid var(--color-border)",
                      color: "var(--color-foreground)",
                    }}
                  />
                  <Legend />
                  <Line
                    type="monotone"
                    dataKey="speed"
                    stroke="var(--color-primary)"
                    dot={{ fill: "var(--color-primary)" }}
                    name="Tốc Độ (km/h)"
                  />
                </LineChart>
              </ResponsiveContainer>
            </Card>

            <Card className="bg-card border-border p-6">
              <h3 className="text-lg font-semibold text-foreground mb-4">Mức Mệt Mỏi Theo Thời Gian</h3>
              <ResponsiveContainer width="100%" height={300}>
                <LineChart data={fatigueData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="var(--color-border)" />
                  <XAxis dataKey="time" stroke="var(--color-foreground)" />
                  <YAxis stroke="var(--color-foreground)" />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: "var(--color-card)",
                      border: "1px solid var(--color-border)",
                      color: "var(--color-foreground)",
                    }}
                  />
                  <Legend />
                  <Line
                    type="monotone"
                    dataKey="fatigue"
                    stroke="var(--color-chart-2)"
                    dot={{ fill: "var(--color-chart-2)" }}
                    name="Mệt Mỏi (%)"
                  />
                </LineChart>
              </ResponsiveContainer>
            </Card>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="bg-card border-border p-6">
              <h3 className="text-lg font-semibold text-foreground mb-4">So Sánh Điểm An Toàn</h3>
              <ResponsiveContainer width="100%" height={300}>
                <BarChart data={tripComparisonData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="var(--color-border)" />
                  <XAxis dataKey="trip" stroke="var(--color-foreground)" />
                  <YAxis stroke="var(--color-foreground)" />
                  <Tooltip
                    contentStyle={{
                      backgroundColor: "var(--color-card)",
                      border: "1px solid var(--color-border)",
                      color: "var(--color-foreground)",
                    }}
                  />
                  <Bar dataKey="score" fill="var(--color-primary)" name="Điểm An Toàn" />
                </BarChart>
              </ResponsiveContainer>
            </Card>

            <Card className="bg-card border-border p-6">
              <h3 className="text-lg font-semibold text-foreground mb-4">Khuyến Nghị</h3>
              <div className="space-y-3">
                <div className="p-3 bg-primary/10 rounded-lg border border-primary/20">
                  <p className="text-sm font-semibold text-foreground mb-1">Tăng Khoảng Cách An Toàn</p>
                  <p className="text-xs text-foreground/70">
                    Bạn đã có 2 cảnh báo va chạm. Hãy tăng khoảng cách với xe phía trước.
                  </p>
                </div>
                <div className="p-3 bg-primary/10 rounded-lg border border-primary/20">
                  <p className="text-sm font-semibold text-foreground mb-1">Nghỉ Ngơi Thường Xuyên</p>
                  <p className="text-xs text-foreground/70">
                    Mức mệt mỏi tăng nhanh sau 1.5 giờ lái. Hãy nghỉ ngơi 15 phút.
                  </p>
                </div>
                <div className="p-3 bg-primary/10 rounded-lg border border-primary/20">
                  <p className="text-sm font-semibold text-foreground mb-1">Tuân Thủ Giới Hạn Tốc Độ</p>
                  <p className="text-xs text-foreground/70">
                    Bạn đã vượt tốc độ 3 lần. Hãy tuân thủ giới hạn tốc độ để an toàn hơn.
                  </p>
                </div>
              </div>
            </Card>
          </div>
        </div>
      </main>
    </div>
  )
}
