"use client"

import { useState, useEffect } from "react"
import { motion } from "framer-motion"
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
import { HighchartsChart } from "@/components/charts/highcharts-chart"
import { API_CONFIG } from "@/lib/api-config"

const containerVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1,
      delayChildren: 0.2,
    },
  },
}

const itemVariants = {
  hidden: { opacity: 0, y: 20 },
  visible: {
    opacity: 1,
    y: 0,
    transition: {
      duration: 0.5,
      ease: [0.4, 0, 0.2, 1],
    },
  },
}

export default function HomePage() {
  const [stats, setStats] = useState({
    systemStatus: "hoạt động",
    activeCameras: 0,
    totalDetections: 0,
    alertsToday: 0,
  })

  useEffect(() => {
    const fetchStats = async () => {
      try {
        const statusRes = await fetch(`${API_CONFIG.BASE_URL}/api/status`)
        const statusData = await statusRes.json()

        const alertsRes = await fetch(`${API_CONFIG.BASE_URL}/api/alerts/stats`)
        const alertsData = await alertsRes.json()

        setStats({
          systemStatus: statusData.status === "success" ? "hoạt động" : "ngoại tuyến",
          activeCameras: 1,
          totalDetections: 0,
          alertsToday: alertsData.data?.total_alerts || alertsData.total_alerts || 0,
        })
      } catch (err) {
        console.error("Failed to fetch stats:", err)
      }
    }

    fetchStats()
    const interval = setInterval(fetchStats, 5000)
    return () => clearInterval(interval)
  }, [])

  // Sample chart data
  const detectionChartData = [
    { name: 'Cars', y: 45 },
    { name: 'Pedestrians', y: 25 },
    { name: 'Cycles', y: 20 },
    { name: 'Others', y: 10 },
  ]

  const performanceChartData = [{
    name: 'Performance',
    data: [65, 72, 68, 75, 80, 78, 85],
    color: '#667eea',
  }]

  return (
    <div className="flex h-screen bg-background overflow-hidden">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <motion.div
          className="p-8 space-y-8"
          variants={containerVariants}
          initial="hidden"
          animate="visible"
        >
          {/* Hero Section - Premium Glassmorphism */}
          <motion.div
            variants={itemVariants}
            className="relative overflow-hidden rounded-3xl bg-gradient-to-br from-primary/20 via-accent/10 to-background border border-primary/20 p-10 backdrop-blur-xl"
          >
            {/* Animated gradient orbs */}
            <motion.div
              className="absolute top-0 right-0 w-96 h-96 bg-primary/20 rounded-full blur-3xl"
              animate={{
                scale: [1, 1.2, 1],
                opacity: [0.3, 0.5, 0.3],
              }}
              transition={{
                duration: 8,
                repeat: Infinity,
                ease: "easeInOut",
              }}
            />
            <motion.div
              className="absolute bottom-0 left-0 w-80 h-80 bg-accent/20 rounded-full blur-3xl"
              animate={{
                scale: [1, 1.3, 1],
                opacity: [0.3, 0.5, 0.3],
              }}
              transition={{
                duration: 10,
                repeat: Infinity,
                ease: "easeInOut",
              }}
            />

            <div className="relative z-10">
              <motion.div
                className="flex items-center gap-3 mb-6"
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.3 }}
              >
                <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-primary to-accent flex items-center justify-center shadow-lg shadow-primary/30">
                  <Shield className="w-6 h-6 text-white" />
                </div>
                <Badge variant="outline" className="border-primary/50 text-primary bg-primary/10 backdrop-blur-sm">
                  v3.0 Professional
                </Badge>
              </motion.div>

              <motion.h1
                className="text-5xl font-bold mb-4 bg-gradient-to-r from-foreground via-primary to-accent bg-clip-text text-transparent"
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.4 }}
              >
                Advanced Driver Assistance System
              </motion.h1>

              <motion.p
                className="text-xl text-muted-foreground max-w-3xl mb-8 leading-relaxed"
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.5 }}
              >
                Real-time AI-powered safety monitoring with WebSocket streaming, automatic data collection, and intelligent alerts.
              </motion.p>

              <motion.div
                className="flex gap-4"
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.6 }}
              >
                <Link href="/adas">
                  <Button size="lg" variant="gradient" className="shadow-xl">
                    <Zap className="w-5 h-5 mr-2" />
                    Start Detection
                  </Button>
                </Link>
                <Link href="/dashboard">
                  <Button size="lg" variant="glass" className="shadow-lg">
                    View Dashboard
                    <ArrowRight className="w-5 h-5 ml-2" />
                  </Button>
                </Link>
              </motion.div>
            </div>
          </motion.div>

          {/* System Status Cards - Premium Grid */}
          <motion.div
            variants={itemVariants}
            className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6"
          >
            {[
              {
                title: "Trạng Thái Hệ Thống",
                value: stats.systemStatus,
                icon: Activity,
                color: "success",
                description: "Tất cả hệ thống trực tuyến",
              },
              {
                title: "Camera Hoạt Động",
                value: stats.activeCameras.toString(),
                icon: Car,
                color: "primary",
                description: "Giám sát thời gian thực",
              },
              {
                title: "Total Detections",
                value: stats.totalDetections.toLocaleString(),
                icon: Eye,
                color: "info",
                description: "+12% so với tuần trước",
                trend: true,
              },
              {
                title: "Alerts Today",
                value: stats.alertsToday.toString(),
                icon: AlertTriangle,
                color: "warning",
                description: "Cảnh báo an toàn đã phát",
              },
            ].map((stat, index) => (
              <motion.div
                key={stat.title}
                variants={itemVariants}
                whileHover={{ y: -8, scale: 1.02 }}
                transition={{ type: "spring", stiffness: 300 }}
              >
                <Card glass className="border-border/50 hover:border-primary/30 transition-all duration-300">
                  <CardHeader className="pb-3">
                    <div className="flex items-center justify-between">
                      <CardTitle className="text-sm font-medium text-muted-foreground">
                        {stat.title}
                      </CardTitle>
                      <stat.icon className={`w-5 h-5 text-${stat.color}`} />
                    </div>
                  </CardHeader>
                  <CardContent>
                    <div className="flex items-center gap-2 mb-2">
                      {stat.title === "Trạng Thái Hệ Thống" && (
                        <motion.div
                          className="w-2 h-2 rounded-full bg-success"
                          animate={{ scale: [1, 1.2, 1], opacity: [1, 0.7, 1] }}
                          transition={{ duration: 2, repeat: Infinity }}
                        />
                      )}
                      <span className="text-3xl font-bold text-foreground capitalize">
                        {stat.value}
                      </span>
                    </div>
                    <p className="text-xs text-muted-foreground flex items-center gap-1">
                      {stat.trend && <TrendingUp className="w-3 h-3 text-success" />}
                      {stat.description}
                    </p>
                  </CardContent>
                </Card>
              </motion.div>
            ))}
          </motion.div>

          {/* Charts Section */}
          <motion.div
            variants={itemVariants}
            className="grid grid-cols-1 lg:grid-cols-2 gap-6"
          >
            <HighchartsChart
              title="Detection Distribution"
              description="Current detection breakdown"
              type="pie"
              data={detectionChartData}
              height={350}
            />
            <HighchartsChart
              title="System Performance"
              description="Performance over time"
              type="line"
              data={performanceChartData}
              height={350}
            />
          </motion.div>

          {/* Quick Actions & Features */}
          <motion.div
            variants={itemVariants}
            className="grid grid-cols-1 lg:grid-cols-2 gap-6"
          >
            <Card glass>
              <CardHeader>
                <CardTitle className="text-xl">Thao Tác Nhanh</CardTitle>
                <CardDescription>Các tác vụ và phím tắt thường dùng</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3">
                {[
                  {
                    href: "/adas",
                    icon: Zap,
                    title: "Bắt Đầu Phát Hiện Trực Tiếp",
                    description: "Giám sát ADAS thời gian thực",
                    gradient: "from-primary to-primary/80",
                  },
                  {
                    href: "/driver-monitor",
                    icon: Eye,
                    title: "Giám Sát Tài Xế",
                    description: "Theo dõi hành vi tài xế",
                    gradient: "from-accent to-accent/80",
                  },
                  {
                    href: "/analytics",
                    icon: TrendingUp,
                    title: "Xem Phân Tích",
                    description: "Thông tin chi tiết hiệu suất",
                    gradient: "from-info to-info/80",
                  },
                ].map((action) => (
                  <motion.div
                    key={action.href}
                    whileHover={{ x: 4 }}
                    transition={{ type: "spring", stiffness: 400 }}
                  >
                    <Link
                      href={action.href}
                      className="flex items-center justify-between p-4 rounded-xl bg-gradient-to-r from-white/5 to-white/0 border border-white/10 hover:border-primary/50 hover:from-white/10 hover:to-white/5 transition-all duration-300 group"
                    >
                      <div className="flex items-center gap-4">
                        <div className={`w-12 h-12 rounded-xl bg-gradient-to-br ${action.gradient} flex items-center justify-center shadow-lg group-hover:scale-110 transition-transform`}>
                          <action.icon className="w-6 h-6 text-white" />
                        </div>
                        <div>
                          <div className="font-semibold text-foreground group-hover:text-primary transition-colors">
                            {action.title}
                          </div>
                          <div className="text-sm text-muted-foreground">{action.description}</div>
                        </div>
                      </div>
                      <ArrowRight className="w-5 h-5 text-muted-foreground group-hover:text-primary group-hover:translate-x-1 transition-all" />
                    </Link>
                  </motion.div>
                ))}
              </CardContent>
            </Card>

            <Card glass>
              <CardHeader>
                <CardTitle className="text-xl">Tính Năng Hệ Thống</CardTitle>
                <CardDescription>Điều làm nên sức mạnh của ADAS</CardDescription>
              </CardHeader>
              <CardContent className="space-y-4">
                {[
                  "Phát Trực Tiếp WebSocket Thời Gian Thực",
                  "Phát Hiện AI YOLOv11",
                  "Thu Thập Dữ Liệu Tự Động",
                  "Cảnh Báo Thông Minh",
                  "Triển Khai Docker",
                ].map((feature, index) => (
                  <motion.div
                    key={feature}
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 0.7 + index * 0.1 }}
                    className="flex items-start gap-3"
                  >
                    <CheckCircle2 className="w-5 h-5 text-success mt-0.5 flex-shrink-0" />
                    <div>
                      <div className="font-medium text-foreground">{feature}</div>
                      <div className="text-sm text-muted-foreground">
                        {index === 0 && "Xử lý video độ trễ thấp"}
                        {index === 1 && "Nhận dạng đối tượng hiện đại nhất"}
                        {index === 2 && "Cải thiện mô hình liên tục"}
                        {index === 3 && "Cảnh báo bằng giọng nói và hình ảnh"}
                        {index === 4 && "Cài đặt một lệnh"}
                      </div>
                    </div>
                  </motion.div>
                ))}
              </CardContent>
            </Card>
          </motion.div>

          {/* Recent Activity */}
          <motion.div variants={itemVariants}>
            <Card glass>
              <CardHeader>
                <div className="flex items-center justify-between">
                  <div>
                    <CardTitle className="text-xl">Hoạt Động Gần Đây</CardTitle>
                    <CardDescription>Sự kiện và phát hiện mới nhất của hệ thống</CardDescription>
                  </div>
                  <Link href="/events">
                    <Button variant="glass" size="sm">
                      Xem Tất Cả
                      <ArrowRight className="w-4 h-4 ml-2" />
                    </Button>
                  </Link>
                </div>
              </CardHeader>
              <CardContent>
                <div className="space-y-3">
                  {[
                    { icon: Clock, text: "Hệ thống khởi động thành công", subtext: "Tất cả dịch vụ hoạt động", time: "Vừa xong" },
                    { icon: CheckCircle2, text: "Kết nối cơ sở dữ liệu thành công", subtext: "SQLite sẵn sàng", time: "1 phút trước", color: "success" },
                    { icon: Activity, text: "Backend API trực tuyến", subtext: "Cổng 8000 đang lắng nghe", time: "2 phút trước", color: "primary" },
                  ].map((activity, index) => (
                    <motion.div
                      key={index}
                      initial={{ opacity: 0, x: -20 }}
                      animate={{ opacity: 1, x: 0 }}
                      transition={{ delay: 1 + index * 0.1 }}
                      className="flex items-center gap-4 p-4 rounded-xl bg-gradient-to-r from-white/5 to-transparent border border-white/10 hover:border-primary/30 hover:from-white/10 transition-all duration-300"
                    >
                      <activity.icon className={`w-5 h-5 text-${activity.color || 'muted-foreground'}`} />
                      <div className="flex-1">
                        <div className="text-sm font-medium text-foreground">{activity.text}</div>
                        <div className="text-xs text-muted-foreground">{activity.subtext}</div>
                      </div>
                      <div className="text-xs text-muted-foreground">{activity.time}</div>
                    </motion.div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </motion.div>
        </motion.div>
      </main>
    </div>
  )
}
