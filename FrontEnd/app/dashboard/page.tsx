"use client"

import { useEffect, useState } from "react"
import { Sidebar } from "@/components/sidebar"
import { MobileNav } from "@/components/mobile-nav"
import { GlassCard } from "@/components/ui/glass-card"
import { CircularGauge } from "@/components/ui/circular-gauge"
import { Badge } from "@/components/ui/badge"
import { Activity, AlertTriangle, Camera, Database, TrendingUp } from "lucide-react"
import Highcharts from "highcharts"
import HighchartsReact from "highcharts-react-official"

import { getApiUrl } from "@/lib/api-config"
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
        const statsData = data.data || data
        setStats(statsData)
      }

      // Fetch detection stats
      const detectionRes = await fetch(getApiUrl(API_ENDPOINTS.DETECTIONS_STATS))
      if (detectionRes.ok) {
        const data = await detectionRes.json()
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
    <div className="flex h-screen bg-bg-primary">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-4 sm:p-6 lg:p-8 space-y-4 sm:space-y-6">
          {/* Header */}
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold text-neon-cyan tracking-wider">DASHBOARD</h1>
              <p className="text-sm text-fg-secondary mt-1">
                Real-time system overview with live data
              </p>
            </div>
            <Badge className="glass-card border-neon-green/50 text-neon-green px-4 py-2">
              <div className="flex items-center gap-2">
                <div className="w-2 h-2 rounded-full bg-neon-green animate-pulse" style={{ boxShadow: '0 0 10px var(--neon-green)' }} />
                ONLINE
              </div>
            </Badge>
          </div>

          {loading ? (
            <div className="text-center py-12">
              <div className="inline-block animate-spin rounded-full h-12 w-12 border-b-2 border-neon-cyan" />
              <p className="mt-4 text-fg-secondary">Loading system data...</p>
            </div>
          ) : (
            <>
              {/* Highcharts Visualizations */}
              <div className="grid gap-6 grid-cols-1 lg:grid-cols-2">
                <GlassCard className="p-6">
                  <HighchartsReact
                    highcharts={Highcharts}
                    options={{
                      chart: {
                        type: 'area',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Xu Hướng Phát Hiện Thời Gian Thực',
                        style: {
                          color: '#00E5FF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '16px',
                          fontWeight: '600'
                        }
                      },
                      xAxis: {
                        categories: ['10:00', '10:05', '10:10', '10:15', '10:20', '10:25', '10:30'],
                        labels: {
                          style: {
                            color: '#BAE6FD',
                            fontFamily: 'var(--font-inter)',
                            fontSize: '11px'
                          }
                        },
                        lineColor: 'rgba(255, 255, 255, 0.1)',
                        tickColor: 'rgba(255, 255, 255, 0.1)'
                      },
                      yAxis: {
                        title: {
                          text: 'Số lượng phát hiện',
                          style: {
                            color: '#BAE6FD',
                            fontFamily: 'var(--font-inter)',
                            fontSize: '12px'
                          }
                        },
                        labels: {
                          style: {
                            color: '#BAE6FD',
                            fontFamily: 'var(--font-inter)',
                            fontSize: '11px'
                          }
                        },
                        gridLineColor: 'rgba(255, 255, 255, 0.05)'
                      },
                      tooltip: {
                        shared: true,
                        backgroundColor: 'rgba(10, 22, 40, 0.95)',
                        borderColor: '#00E5FF',
                        borderRadius: 8,
                        style: {
                          color: '#FFFFFF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '12px'
                        }
                      },
                      plotOptions: {
                        area: {
                          fillOpacity: 0.3,
                          marker: {
                            radius: 4,
                            lineWidth: 2
                          }
                        }
                      },
                      series: [{
                        name: 'Xe cộ',
                        data: [45, 52, 48, 61, 58, 65, 72],
                        color: '#00E5FF',
                        fillColor: {
                          linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                          stops: [
                            [0, 'rgba(0, 229, 255, 0.3)'],
                            [1, 'rgba(0, 229, 255, 0.05)']
                          ]
                        }
                      }, {
                        name: 'Người đi bộ',
                        data: [28, 31, 35, 29, 42, 38, 45],
                        color: '#00FFA3',
                        fillColor: {
                          linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                          stops: [
                            [0, 'rgba(0, 255, 163, 0.3)'],
                            [1, 'rgba(0, 255, 163, 0.05)']
                          ]
                        }
                      }],
                      legend: {
                        itemStyle: {
                          color: '#BAE6FD',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '12px',
                          fontWeight: '500'
                        },
                        itemHoverStyle: {
                          color: '#FFFFFF'
                        }
                      },
                      credits: {
                        enabled: false
                      }
                    }}
                  />
                </GlassCard>

                <GlassCard className="p-6">
                  <HighchartsReact
                    highcharts={Highcharts}
                    options={{
                      chart: {
                        type: 'spline',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Độ Chính Xác Phát Hiện Theo Thời Gian',
                        style: {
                          color: '#00E5FF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '16px',
                          fontWeight: '600'
                        }
                      },
                      xAxis: {
                        categories: ['T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'CN'],
                        labels: {
                          style: {
                            color: '#BAE6FD',
                            fontFamily: 'var(--font-inter)',
                            fontSize: '11px'
                          }
                        },
                        lineColor: 'rgba(255, 255, 255, 0.1)',
                        tickColor: 'rgba(255, 255, 255, 0.1)'
                      },
                      yAxis: {
                        title: {
                          text: 'Độ chính xác (%)',
                          style: {
                            color: '#BAE6FD',
                            fontFamily: 'var(--font-inter)',
                            fontSize: '12px'
                          }
                        },
                        labels: {
                          style: {
                            color: '#BAE6FD',
                            fontFamily: 'var(--font-inter)',
                            fontSize: '11px'
                          }
                        },
                        gridLineColor: 'rgba(255, 255, 255, 0.05)',
                        min: 90,
                        max: 100
                      },
                      tooltip: {
                        backgroundColor: 'rgba(10, 22, 40, 0.95)',
                        borderColor: '#00E5FF',
                        borderRadius: 8,
                        style: {
                          color: '#FFFFFF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '12px'
                        },
                        valueSuffix: '%'
                      },
                      plotOptions: {
                        spline: {
                          marker: {
                            radius: 4,
                            lineColor: '#050B14',
                            lineWidth: 2
                          }
                        }
                      },
                      series: [{
                        name: 'Độ chính xác',
                        data: [96.5, 97.2, 96.8, 98.1, 97.9, 98.5, 98.3],
                        color: '#00FFA3',
                        marker: {
                          symbol: 'circle'
                        }
                      }],
                      legend: {
                        enabled: false
                      },
                      credits: {
                        enabled: false
                      }
                    }}
                  />
                </GlassCard>
               </div>
            </>
          )}
        </div>
      </main>
    </div>
  )
}