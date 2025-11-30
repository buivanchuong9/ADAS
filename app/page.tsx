"use client"

import { useState, useEffect } from "react"
import { Sidebar } from "@/components/sidebar"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import {
  Activity,
  AlertTriangle,
  Car,
  Eye,
  TrendingUp,
  Zap,
  Shield,
  ArrowRight,
  CheckCircle2,
  Clock,
} from "lucide-react"
import Link from "next/link"

export default function HomePage() {
  const [stats, setStats] = useState({
    systemStatus: "hoạt động",
    activeCameras: 0,
    totalDetections: 0,
    alertsToday: 0,
  })

  useEffect(() => {
    // Lấy trạng thái hệ thống từ API
    const fetchStats = async () => {
      try {
        // Fetch system status
        const statusRes = await fetch("http://localhost:8000/api/status")
        const statusData = await statusRes.json()

        // Fetch alert stats
        const alertsRes = await fetch("http://localhost:8000/api/alerts/stats")
        const alertsData = await alertsRes.json()

        setStats({
          systemStatus: statusData.data?.status === "ok" ? "hoạt động" : "ngoại tuyến",
          activeCameras: 1, // Sẽ update khi có WebSocket connection
          totalDetections: 0, // Sẽ update từ database
          alertsToday: alertsData.total_alerts || 0,
        })
      } catch (err) {
        console.error("Failed to fetch stats:", err)
      }
    }

    fetchStats()
    // Refresh every 5 seconds
    const interval = setInterval(fetchStats, 5000)
    return () => clearInterval(interval)
  }, [])

  return (
    <div className="flex h-screen bg-background">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-8 space-y-8">
          {/* Hero Section */}
          <div className="relative overflow-hidden rounded-2xl bg-gradient-to-br from-primary/20 via-accent/10 to-background border border-primary/20 p-8">
            <div className="relative z-10">
              <div className="flex items-center gap-2 mb-4">
                <Shield className="w-8 h-8 text-primary" />
                <Badge variant="outline" className="border-primary/50 text-primary">
                  v3.0 Professional
                </Badge>
              </div>
              <h1 className="text-4xl font-bold text-foreground mb-3">
                Advanced Driver Assistance System
              </h1>
              <p className="text-lg text-muted-foreground max-w-2xl mb-6">
                Real-time AI-powered safety monitoring with WebSocket streaming, automatic data collection, and intelligent alerts.
              </p>
              <div className="flex gap-3">
                <Link href="/adas">
                  <Button size="lg" className="bg-primary hover:bg-primary/90 text-primary-foreground shadow-lg shadow-primary/20">
                    <Zap className="w-4 h-4 mr-2" />
                    Start Detection
                  </Button>
                </Link>
                <Link href="/dashboard">
                  <Button size="lg" variant="outline" className="border-border hover:bg-muted">
                    View Dashboard
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>

            {/* Decorative gradient orbs */}
            <div className="absolute top-0 right-0 w-64 h-64 bg-primary/10 rounded-full blur-3xl" />
            <div className="absolute bottom-0 left-0 w-48 h-48 bg-accent/10 rounded-full blur-3xl" />
          </div>

          {/* System Status Cards */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
            <Card className="border-border bg-card hover:shadow-lg hover:shadow-primary/5 transition-all">
              <CardHeader className="pb-3">
                <div className="flex items-center justify-between">
                  <CardTitle className="text-sm font-medium text-muted-foreground">Trạng Thái Hệ Thống</CardTitle>
                  <Activity className="w-4 h-4 text-success" />
                </div>
              </CardHeader>
              <CardContent>
                <div className="flex items-center gap-2">
                  <div className="w-2 h-2 rounded-full bg-success animate-pulse" />
                  <span className="text-2xl font-bold text-foreground capitalize">{stats.systemStatus}</span>
                </div>
                <p className="text-xs text-muted-foreground mt-1">Tất cả hệ thống trực tuyến</p>
              </CardContent>
            </Card>

            <Card className="border-border bg-card hover:shadow-lg hover:shadow-primary/5 transition-all">
              <CardHeader className="pb-3">
                <div className="flex items-center justify-between">
                  <CardTitle className="text-sm font-medium text-muted-foreground">Camera Hoạt Động</CardTitle>
                  <Car className="w-4 h-4 text-primary" />
                </div>
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-foreground">{stats.activeCameras}</div>
                <p className="text-xs text-muted-foreground mt-1">Giám sát thời gian thực</p>
              </CardContent>
            </Card>

            <Card className="border-border bg-card hover:shadow-lg hover:shadow-primary/5 transition-all">
              <CardHeader className="pb-3">
                <div className="flex items-center justify-between">
                  <CardTitle className="text-sm font-medium text-muted-foreground">Total Detections</CardTitle>
                  <Eye className="w-4 h-4 text-info" />
                </div>
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-foreground">{stats.totalDetections.toLocaleString()}</div>
                <p className="text-xs text-success flex items-center gap-1 mt-1">
                  <TrendingUp className="w-3 h-3" />
                  +12% so với tuần trước
                </p>
              </CardContent>
            </Card>

            <Card className="border-border bg-card hover:shadow-lg hover:shadow-primary/5 transition-all">
              <CardHeader className="pb-3">
                <div className="flex items-center justify-between">
                  <CardTitle className="text-sm font-medium text-muted-foreground">Alerts Today</CardTitle>
                  <AlertTriangle className="w-4 h-4 text-warning" />
                </div>
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-foreground">{stats.alertsToday}</div>
                <p className="text-xs text-muted-foreground mt-1">Cảnh báo an toàn đã phát</p>
              </CardContent>
            </Card>
          </div>

          {/* Quick Actions */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <Card className="border-border bg-card">
              <CardHeader>
                <CardTitle className="text-lg">Thao Tác Nhanh</CardTitle>
                <CardDescription>Các tác vụ và phím tắt thường dùng</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3">
                <Link href="/adas" className="flex items-center justify-between p-4 rounded-lg bg-muted hover:bg-muted/80 transition-colors group">
                  <div className="flex items-center gap-3">
                    <div className="w-10 h-10 rounded-lg bg-primary/10 flex items-center justify-center">
                      <Zap className="w-5 h-5 text-primary" />
                    </div>
                    <div>
                      <div className="font-medium text-foreground">Bắt Đầu Phát Hiện Trực Tiếp</div>
                      <div className="text-sm text-muted-foreground">Giám sát ADAS thời gian thực</div>
                    </div>
                  </div>
                  <ArrowRight className="w-5 h-5 text-muted-foreground group-hover:text-foreground group-hover:translate-x-1 transition-all" />
                </Link>

                <Link href="/driver-monitor" className="flex items-center justify-between p-4 rounded-lg bg-muted hover:bg-muted/80 transition-colors group">
                  <div className="flex items-center gap-3">
                    <div className="w-10 h-10 rounded-lg bg-accent/10 flex items-center justify-center">
                      <Eye className="w-5 h-5 text-accent" />
                    </div>
                    <div>
                      <div className="font-medium text-foreground">Giám Sát Tài Xế</div>
                      <div className="text-sm text-muted-foreground">Theo dõi hành vi tài xế</div>
                    </div>
                  </div>
                  <ArrowRight className="w-5 h-5 text-muted-foreground group-hover:text-foreground group-hover:translate-x-1 transition-all" />
                </Link>

                <Link href="/analytics" className="flex items-center justify-between p-4 rounded-lg bg-muted hover:bg-muted/80 transition-colors group">
                  <div className="flex items-center gap-3">
                    <div className="w-10 h-10 rounded-lg bg-info/10 flex items-center justify-center">
                      <TrendingUp className="w-5 h-5 text-info" />
                    </div>
                    <div>
                      <div className="font-medium text-foreground">Xem Phân Tích</div>
                      <div className="text-sm text-muted-foreground">Thông tin chi tiết hiệu suất</div>
                    </div>
                  </div>
                  <ArrowRight className="w-5 h-5 text-muted-foreground group-hover:text-foreground group-hover:translate-x-1 transition-all" />
                </Link>
              </CardContent>
            </Card>

            <Card className="border-border bg-card">
              <CardHeader>
                <CardTitle className="text-lg">Tính Năng Hệ Thống</CardTitle>
                <CardDescription>Điều làm nên sức mạnh của ADAS</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3">
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-success mt-0.5" />
                  <div>
                    <div className="font-medium text-foreground">Phát Trực Tiếp WebSocket Thời Gian Thực</div>
                    <div className="text-sm text-muted-foreground">Xử lý video độ trễ thấp</div>
                  </div>
                </div>

                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-success mt-0.5" />
                  <div>
                    <div className="font-medium text-foreground">Phát Hiện AI YOLOv11</div>
                    <div className="text-sm text-muted-foreground">Nhận dạng đối tượng hiện đại nhất</div>
                  </div>
                </div>

                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-success mt-0.5" />
                  <div>
                    <div className="font-medium text-foreground">Thu Thập Dữ Liệu Tự Động</div>
                    <div className="text-sm text-muted-foreground">Cải thiện mô hình liên tục</div>
                  </div>
                </div>

                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-success mt-0.5" />
                  <div>
                    <div className="font-medium text-foreground">Cảnh Báo Thông Minh</div>
                    <div className="text-sm text-muted-foreground">Cảnh báo bằng giọng nói và hình ảnh</div>
                  </div>
                </div>

                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-success mt-0.5" />
                  <div>
                    <div className="font-medium text-foreground">Triển Khai Docker</div>
                    <div className="text-sm text-muted-foreground">Cài đặt một lệnh</div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>

          {/* Recent Activity */}
          <Card className="border-border bg-card">
            <CardHeader>
              <div className="flex items-center justify-between">
                <div>
                  <CardTitle className="text-lg">Hoạt Động Gần Đây</CardTitle>
                  <CardDescription>Sự kiện và phát hiện mới nhất của hệ thống</CardDescription>
                </div>
                <Link href="/events">
                  <Button variant="outline" size="sm">
                    Xem Tất Cả
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div className="flex items-center gap-4 p-3 rounded-lg bg-muted/50">
                  <Clock className="w-4 h-4 text-muted-foreground" />
                  <div className="flex-1">
                    <div className="text-sm font-medium text-foreground">Hệ thống khởi động thành công</div>
                    <div className="text-xs text-muted-foreground">Tất cả dịch vụ hoạt động</div>
                  </div>
                  <div className="text-xs text-muted-foreground">Vừa xong</div>
                </div>

                <div className="flex items-center gap-4 p-3 rounded-lg bg-muted/50">
                  <CheckCircle2 className="w-4 h-4 text-success" />
                  <div className="flex-1">
                    <div className="text-sm font-medium text-foreground">Kết nối cơ sở dữ liệu thành công</div>
                    <div className="text-xs text-muted-foreground">SQLite sẵn sàng</div>
                  </div>
                  <div className="text-xs text-muted-foreground">1 phút trước</div>
                </div>

                <div className="flex items-center gap-4 p-3 rounded-lg bg-muted/50">
                  <Activity className="w-4 h-4 text-primary" />
                  <div className="flex-1">
                    <div className="text-sm font-medium text-foreground">Backend API trực tuyến</div>
                    <div className="text-xs text-muted-foreground">Cổng 8000 đang lắng nghe</div>
                  </div>
                  <div className="text-xs text-muted-foreground">2 phút trước</div>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </main >
    </div >
  )
}
