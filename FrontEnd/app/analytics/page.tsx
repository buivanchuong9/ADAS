"use client"

import { Sidebar } from "@/components/sidebar"
import { MobileNav } from "@/components/mobile-nav"
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
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8">
          <div className="mb-6 sm:mb-8">
            <h1 className="text-2xl sm:text-3xl font-bold text-gray-900 mb-2">Phân Tích Chuyến Đi</h1>
            <p className="text-sm sm:text-base text-gray-600">Xem chi tiết thống kê và phân tích về chuyến đi của bạn</p>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3 sm:gap-4 mb-6 sm:mb-8">
            <Card className="bg-white border-2 border-blue-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">Quãng Đường</p>
                    <p className="text-2xl font-bold text-blue-600">42.5 km</p>
                  </div>
                  <TrendingUp className="w-8 h-8 text-blue-500" />
                </div>
              </div>
            </Card>

            <Card className="bg-white border-2 border-green-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">Thời Gian Lái</p>
                    <p className="text-2xl font-bold text-green-600">2h 15m</p>
                  </div>
                  <Clock className="w-8 h-8 text-green-500" />
                </div>
              </div>
            </Card>

            <Card className="bg-white border-2 border-purple-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">Tốc Độ Trung Bình</p>
                    <p className="text-2xl font-bold text-purple-600">54 km/h</p>
                  </div>
                  <Gauge className="w-8 h-8 text-purple-500" />
                </div>
              </div>
            </Card>

            <Card className="bg-white border-2 border-orange-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-4">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm text-gray-600 mb-1 font-medium">Điểm An Toàn</p>
                    <p className="text-2xl font-bold text-orange-600">85/100</p>
                  </div>
                  <AlertTriangle className="w-8 h-8 text-orange-500" />
                </div>
              </div>
            </Card>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            <Card className="bg-white border-2 border-blue-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">Tốc Độ Theo Thời Gian</h3>
                <ResponsiveContainer width="100%" height={300}>
                  <LineChart data={speedData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                    <XAxis dataKey="time" stroke="#6b7280" />
                    <YAxis stroke="#6b7280" />
                    <Tooltip
                      contentStyle={{
                        backgroundColor: "#ffffff",
                        border: "1px solid #e5e7eb",
                        color: "#1f2937",
                        borderRadius: "8px"
                      }}
                    />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey="speed"
                      stroke="#3b82f6"
                      strokeWidth={2}
                      dot={{ fill: "#3b82f6", r: 4 }}
                      name="Tốc Độ (km/h)"
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </Card>

            <Card className="bg-white border-2 border-red-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">Mức Mệt Mỏi Theo Thời Gian</h3>
                <ResponsiveContainer width="100%" height={300}>
                  <LineChart data={fatigueData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                    <XAxis dataKey="time" stroke="#6b7280" />
                    <YAxis stroke="#6b7280" />
                    <Tooltip
                      contentStyle={{
                        backgroundColor: "#ffffff",
                        border: "1px solid #e5e7eb",
                        color: "#1f2937",
                        borderRadius: "8px"
                      }}
                    />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey="fatigue"
                      stroke="#ef4444"
                      strokeWidth={2}
                      dot={{ fill: "#ef4444", r: 4 }}
                      name="Mệt Mỏi (%)"
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </Card>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="bg-white border-2 border-indigo-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">So Sánh Điểm An Toàn</h3>
                <ResponsiveContainer width="100%" height={300}>
                  <BarChart data={tripComparisonData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                    <XAxis dataKey="trip" stroke="#6b7280" />
                    <YAxis stroke="#6b7280" />
                    <Tooltip
                      contentStyle={{
                        backgroundColor: "#ffffff",
                        border: "1px solid #e5e7eb",
                        color: "#1f2937",
                        borderRadius: "8px"
                      }}
                    />
                    <Bar dataKey="score" fill="#6366f1" name="Điểm An Toàn" radius={[8, 8, 0, 0]} />
                  </BarChart>
                </ResponsiveContainer>
              </div>
            </Card>

            <Card className="bg-white border-2 border-green-200 shadow-sm hover:shadow-md transition-shadow">
              <div className="p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4">Khuyến Nghị</h3>
                <div className="space-y-3">
                  <div className="p-3 bg-blue-50 rounded-lg border-l-4 border-blue-500">
                    <p className="text-sm font-semibold text-blue-900 mb-1">Tăng Khoảng Cách An Toàn</p>
                    <p className="text-xs text-blue-700">
                      Bạn đã có 2 cảnh báo va chạm. Hãy tăng khoảng cách với xe phía trước.
                    </p>
                  </div>
                  <div className="p-3 bg-green-50 rounded-lg border-l-4 border-green-500">
                    <p className="text-sm font-semibold text-green-900 mb-1">Nghỉ Ngơi Thường Xuyên</p>
                    <p className="text-xs text-green-700">
                      Mức mệt mỏi tăng nhanh sau 1.5 giờ lái. Hãy nghỉ ngơi 15 phút.
                    </p>
                  </div>
                  <div className="p-3 bg-orange-50 rounded-lg border-l-4 border-orange-500">
                    <p className="text-sm font-semibold text-orange-900 mb-1">Tuân Thủ Giới Hạn Tốc Độ</p>
                    <p className="text-xs text-orange-700">
                      Bạn đã vượt tốc độ 3 lần. Hãy tuân thủ giới hạn tốc độ để an toàn hơn.
                    </p>
                  </div>
                </div>
              </div>
            </Card>
          </div>
        </div>
      </main>
    </div>
  )
}
