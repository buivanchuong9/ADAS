"use client"

import { useEffect, useState } from "react"
import { Sidebar } from "@/components/sidebar"
import { MobileNav } from "@/components/mobile-nav"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Activity, AlertTriangle, Camera, Database, TrendingUp } from "lucide-react"

import { API_BASE_URL, getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

interface Stats {
  totalDetections: number
  totalTrips: number
  totalEvents: number
  avgSafetyScore: number
}

interface DetectionClass {
  class_name: string
  count: number
  avg_confidence: number
}

export default function DashboardPage() {
  const [stats, setStats] = useState<Stats>({
    totalDetections: 0,
    totalTrips: 0,
    totalEvents: 0,
    avgSafetyScore: 0
  })
  const [classes, setClasses] = useState<DetectionClass[]>([])
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    fetchData()
    const interval = setInterval(fetchData, 5000)
    return () => clearInterval(interval)
  }, [])

  const fetchData = async () => {
    try {
      // Fetch analytics
      const analyticsRes = await fetch(getApiUrl(API_ENDPOINTS.ADMIN_OVERVIEW))
      if (analyticsRes.ok) {
        const data = await analyticsRes.json()
        // Handle both direct data and wrapped response
        const statsData = data.data || data
        setStats(statsData)
      }

      // Fetch detection stats
      const detectionRes = await fetch(getApiUrl(API_ENDPOINTS.DETECTIONS_STATS))
      if (detectionRes.ok) {
        const data = await detectionRes.json()
        // Handle both formats
        const classesData = data.data?.classes || data.classes || []
        setClasses(classesData)
      }
    } catch (err) {
      console.error('Error fetching dashboard data:', err)
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8 space-y-4 sm:space-y-6">
          <div>
            <h1 className="text-2xl sm:text-3xl font-bold">📊 Dashboard</h1>
            <p className="text-sm sm:text-base text-muted-foreground mt-2">
              Tổng quan hệ thống ADAS với dữ liệu thật từ database
            </p>
          </div>

          {loading ? (
            <div className="text-center py-12">
              <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto"></div>
              <p className="mt-4 text-muted-foreground">Đang tải dữ liệu...</p>
            </div>
          ) : (
            <>
              {/* Stats Cards */}
              <div className="grid gap-3 sm:gap-4 grid-cols-1 sm:grid-cols-2 lg:grid-cols-4">
                <Card>
                  <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                    <CardTitle className="text-sm font-medium">Tổng Detections</CardTitle>
                    <Database className="h-4 w-4 text-muted-foreground" />
                  </CardHeader>
                  <CardContent>
                    <div className="text-2xl font-bold">{(stats.totalDetections ?? 0).toLocaleString()}</div>
                    <p className="text-xs text-muted-foreground mt-1">
                      Từ database thật
                    </p>
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                    <CardTitle className="text-sm font-medium">Số lớp đối tượng</CardTitle>
                    <Activity className="h-4 w-4 text-muted-foreground" />
                  </CardHeader>
                  <CardContent>
                    <div className="text-2xl font-bold">{classes.length}</div>
                    <p className="text-xs text-muted-foreground mt-1">
                      Classes detected
                    </p>
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                    <CardTitle className="text-sm font-medium">Trips</CardTitle>
                    <Camera className="h-4 w-4 text-muted-foreground" />
                  </CardHeader>
                  <CardContent>
                    <div className="text-2xl font-bold">{stats.totalTrips ?? 0}</div>
                    <p className="text-xs text-muted-foreground mt-1">
                      Chuyến đi đã ghi
                    </p>
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                    <CardTitle className="text-sm font-medium">Events</CardTitle>
                    <AlertTriangle className="h-4 w-4 text-muted-foreground" />
                  </CardHeader>
                  <CardContent>
                    <div className="text-2xl font-bold">{stats.totalEvents ?? 0}</div>
                    <p className="text-xs text-muted-foreground mt-1">
                      Sự kiện cảnh báo
                    </p>
                  </CardContent>
                </Card>
              </div>

              {/* Detection Classes */}
              <Card>
                <CardHeader>
                  <CardTitle>Phân bố theo lớp đối tượng</CardTitle>
                  <p className="text-sm text-muted-foreground">
                    Dữ liệu thật từ {(stats.totalDetections ?? 0).toLocaleString()} detections
                  </p>
                </CardHeader>
                <CardContent>
                  <div className="space-y-4">
                    {classes.map((cls, idx) => {
                      const countVal = cls.count ?? 0
                      return (
                      <div key={idx} className="space-y-2">
                        <div className="flex items-center justify-between">
                          <div className="flex items-center gap-2">
                            <Badge variant="outline">{cls.class_name}</Badge>
                            <span className="text-sm text-muted-foreground">
                              {countVal.toLocaleString()} detections
                            </span>
                          </div>
                          <span className="text-sm font-medium">
                            {(cls.avg_confidence * 100).toFixed(1)}% avg
                          </span>
                        </div>
                        <div className="h-2 bg-muted rounded-full overflow-hidden">
                          {(() => {
                            const counts = classes.map(c => c.count ?? 0)
                            const maxCount = Math.max(...counts, 1)
                            const width = (countVal / maxCount) * 100
                            return (
                          <div 
                            className="h-full bg-primary rounded-full transition-all"
                            style={{ 
                              width: `${width}%` 
                            }}
                          />
                            )
                          })()}
                        </div>
                      </div>
                      )
                    })}
                  </div>
                </CardContent>
              </Card>

              {/* Class Confidence */}
              <div className="grid gap-4 grid-cols-1 lg:grid-cols-2">
                <Card>
                  <CardHeader>
                    <CardTitle>Top 3 Classes</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {classes
                        .sort((a, b) => (b.count ?? 0) - (a.count ?? 0))
                        .slice(0, 3)
                        .map((cls, idx) => (
                          <div key={idx} className="flex items-center justify-between p-3 bg-muted rounded-lg">
                            <div>
                              <div className="font-semibold">{cls.class_name}</div>
                              <div className="text-sm text-muted-foreground">
                                {(cls.count ?? 0).toLocaleString()} detections
                              </div>
                            </div>
                            <Badge variant="default" className="text-lg">
                              #{idx + 1}
                            </Badge>
                          </div>
                        ))}
                    </div>
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <CardTitle>Confidence cao nhất</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {classes
                        .sort((a, b) => (b.avg_confidence ?? 0) - (a.avg_confidence ?? 0))
                        .slice(0, 3)
                        .map((cls, idx) => (
                          <div key={idx} className="flex items-center justify-between p-3 bg-muted rounded-lg">
                            <div>
                              <div className="font-semibold">{cls.class_name}</div>
                              <div className="text-sm text-muted-foreground">
                                Avg: {(cls.avg_confidence * 100).toFixed(1)}%
                              </div>
                            </div>
                            <TrendingUp className="h-5 w-5 text-green-500" />
                          </div>
                        ))}
                    </div>
                  </CardContent>
                </Card>
              </div>

              {/* Database Info */}
              <Card>
                <CardHeader>
                  <CardTitle>Database Status</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="grid gap-3 sm:gap-4 grid-cols-1 sm:grid-cols-3">
                    <div className="p-4 bg-muted rounded-lg">
                      <div className="text-2xl font-bold">{(stats.totalDetections ?? 0).toLocaleString()}</div>
                      <div className="text-sm text-muted-foreground">Total Rows</div>
                    </div>
                    <div className="p-4 bg-muted rounded-lg">
                      <div className="text-2xl font-bold">{classes.length}</div>
                      <div className="text-sm text-muted-foreground">Unique Classes</div>
                    </div>
                    <div className="p-4 bg-muted rounded-lg">
                      <div className="text-2xl font-bold">
                        {classes.length > 0 
                          ? ((classes.reduce((sum, c) => sum + (c.avg_confidence ?? 0), 0) / classes.length) * 100).toFixed(1)
                          : 0}%
                      </div>
                      <div className="text-sm text-muted-foreground">Avg Confidence</div>
                    </div>
                  </div>
                </CardContent>
              </Card>
            </>
          )}
        </div>
      </main>
    </div>
  )
}
