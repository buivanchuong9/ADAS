"use client"

import { useState } from "react"
import { Sidebar } from "@/components/sidebar"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { AlertTriangle, AlertCircle, Eye, Zap, Trash2 } from "lucide-react"

interface Event {
  id: string
  type: "collision" | "lane-departure" | "fatigue" | "distraction" | "speed"
  title: string
  description: string
  timestamp: string
  severity: "critical" | "warning" | "info"
  location: string
}

const mockEvents: Event[] = [
  {
    id: "1",
    type: "collision",
    title: "Cảnh Báo Va Chạm",
    description: "Phát hiện xe phía trước quá gần (TTC: 1.2s)",
    timestamp: "2024-10-24 14:32:15",
    severity: "critical",
    location: "Đường Nguyễn Huệ, TP.HCM",
  },
  {
    id: "2",
    type: "lane-departure",
    title: "Cảnh Báo Lệch Làn",
    description: "Xe lệch khỏi làn đường mà không bật tín hiệu",
    timestamp: "2024-10-24 14:28:42",
    severity: "warning",
    location: "Đường Lê Lợi, TP.HCM",
  },
  {
    id: "3",
    type: "fatigue",
    title: "Phát Hiện Mệt Mỏi",
    description: "Mức mệt mỏi của tài xế vượt quá ngưỡng cho phép",
    timestamp: "2024-10-24 14:15:30",
    severity: "warning",
    location: "Đường Trần Hưng Đạo, TP.HCM",
  },
  {
    id: "4",
    type: "distraction",
    title: "Phát Hiện Phân Tán",
    description: "Tài xế không tập trung vào đường",
    timestamp: "2024-10-24 14:05:18",
    severity: "warning",
    location: "Đường Pasteur, TP.HCM",
  },
  {
    id: "5",
    type: "speed",
    title: "Vượt Tốc Độ",
    description: "Tốc độ vượt quá giới hạn cho phép (80km/h > 60km/h)",
    timestamp: "2024-10-24 13:52:05",
    severity: "info",
    location: "Đường Võ Văn Kiệt, TP.HCM",
  },
]

export default function EventsDashboard() {
  const [events, setEvents] = useState<Event[]>(mockEvents)
  const [filter, setFilter] = useState<string>("all")

  const filteredEvents = filter === "all" ? events : events.filter((e) => e.type === filter)

  const getEventIcon = (type: Event["type"]) => {
    switch (type) {
      case "collision":
        return <AlertTriangle className="w-5 h-5" />
      case "lane-departure":
        return <AlertCircle className="w-5 h-5" />
      case "fatigue":
        return <Zap className="w-5 h-5" />
      case "distraction":
        return <Eye className="w-5 h-5" />
      case "speed":
        return <AlertCircle className="w-5 h-5" />
    }
  }

  const getSeverityColor = (severity: Event["severity"]) => {
    switch (severity) {
      case "critical":
        return "border-destructive/50 bg-destructive/5"
      case "warning":
        return "border-yellow-500/50 bg-yellow-500/5"
      case "info":
        return "border-blue-500/50 bg-blue-500/5"
    }
  }

  const getSeverityBadgeColor = (severity: Event["severity"]) => {
    switch (severity) {
      case "critical":
        return "bg-destructive/20 text-destructive"
      case "warning":
        return "bg-yellow-500/20 text-yellow-600"
      case "info":
        return "bg-blue-500/20 text-blue-600"
    }
  }

  const deleteEvent = (id: string) => {
    setEvents(events.filter((e) => e.id !== id))
  }

  return (
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-8">
          <div className="mb-8">
            <h1 className="text-3xl font-bold text-gray-900 mb-2">Nhật Ký Sự Kiện</h1>
            <p className="text-gray-600">Xem lịch sử các sự kiện và cảnh báo trong quá trình lái xe</p>
          </div>

          <div className="mb-6">
            <div className="flex gap-2 flex-wrap">
              <Button
                onClick={() => setFilter("all")}
                variant={filter === "all" ? "default" : "outline"}
                className={filter === "all" ? "bg-primary text-primary-foreground" : ""}
              >
                Tất Cả
              </Button>
              <Button
                onClick={() => setFilter("collision")}
                variant={filter === "collision" ? "default" : "outline"}
                className={filter === "collision" ? "bg-primary text-primary-foreground" : ""}
              >
                Va Chạm
              </Button>
              <Button
                onClick={() => setFilter("lane-departure")}
                variant={filter === "lane-departure" ? "default" : "outline"}
                className={filter === "lane-departure" ? "bg-primary text-primary-foreground" : ""}
              >
                Lệch Làn
              </Button>
              <Button
                onClick={() => setFilter("fatigue")}
                variant={filter === "fatigue" ? "default" : "outline"}
                className={filter === "fatigue" ? "bg-primary text-primary-foreground" : ""}
              >
                Mệt Mỏi
              </Button>
              <Button
                onClick={() => setFilter("distraction")}
                variant={filter === "distraction" ? "default" : "outline"}
                className={filter === "distraction" ? "bg-primary text-primary-foreground" : ""}
              >
                Phân Tán
              </Button>
            </div>
          </div>

          <div className="space-y-3">
            {filteredEvents.length === 0 ? (
              <Card className="bg-card border-border p-8 text-center">
                <p className="text-foreground/60">Không có sự kiện nào</p>
              </Card>
            ) : (
              filteredEvents.map((event) => (
                <Card key={event.id} className={`bg-card border-border p-4 ${getSeverityColor(event.severity)}`}>
                  <div className="flex items-start justify-between gap-4">
                    <div className="flex items-start gap-4 flex-1">
                      <div className="mt-1 text-foreground">{getEventIcon(event.type)}</div>
                      <div className="flex-1">
                        <div className="flex items-center gap-2 mb-1">
                          <h3 className="font-semibold text-foreground">{event.title}</h3>
                          <span className={`text-xs px-2 py-1 rounded-full ${getSeverityBadgeColor(event.severity)}`}>
                            {event.severity === "critical"
                              ? "Nguy Hiểm"
                              : event.severity === "warning"
                                ? "Cảnh Báo"
                                : "Thông Tin"}
                          </span>
                        </div>
                        <p className="text-sm text-foreground/70 mb-2">{event.description}</p>
                        <div className="flex gap-4 text-xs text-foreground/60">
                          <span>📍 {event.location}</span>
                          <span>🕐 {event.timestamp}</span>
                        </div>
                      </div>
                    </div>
                    <Button
                      onClick={() => deleteEvent(event.id)}
                      variant="ghost"
                      size="sm"
                      className="text-foreground/60 hover:text-destructive"
                    >
                      <Trash2 className="w-4 h-4" />
                    </Button>
                  </div>
                </Card>
              ))
            )}
          </div>
        </div>
      </main>
    </div>
  )
}
