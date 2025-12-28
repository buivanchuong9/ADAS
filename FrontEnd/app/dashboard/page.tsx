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
              {/* HUD Stats Grid */}
              <div className="grid gap-4 grid-cols-1 sm:grid-cols-2 lg:grid-cols-4">
                <GlassCard glow="cyan" className="p-6">
                  <div className="flex items-center justify-between mb-4">
                    <span className="text-xs text-fg-secondary uppercase tracking-wider">Total Detections</span>
                    <Database className="h-5 w-5 text-neon-cyan" />
                  </div>
                  <div className="digital-number text-3xl font-bold text-neon-cyan">
                    {(stats.totalDetections ?? 0).toLocaleString()}
                  </div>
                  <p className="text-xs text-fg-muted mt-2">Database records</p>
                </GlassCard>

                <GlassCard glow="cyan" className="p-6">
                  <div className="flex items-center justify-between mb-4">
                    <span className="text-xs text-fg-secondary uppercase tracking-wider">Object Classes</span>
                    <Activity className="h-5 w-5 text-neon-cyan" />
                  </div>
                  <div className="digital-number text-3xl font-bold text-neon-cyan">
                    {classes.length}
                  </div>
                  <p className="text-xs text-fg-muted mt-2">Unique types</p>
                </GlassCard>

                <GlassCard glow="green" className="p-6">
                  <div className="flex items-center justify-between mb-4">
                    <span className="text-xs text-fg-secondary uppercase tracking-wider">Trips Recorded</span>
                    <Camera className="h-5 w-5 text-neon-green" />
                  </div>
                  <div className="digital-number text-3xl font-bold text-neon-green">
                    {stats.totalTrips ?? 0}
                  </div>
                  <p className="text-xs text-fg-muted mt-2">Journey logs</p>
                </GlassCard>

                <GlassCard glow={stats.totalEvents > 10 ? "red" : "green"} className="p-6">
                  <div className="flex items-center justify-between mb-4">
                    <span className="text-xs text-fg-secondary uppercase tracking-wider">Safety Events</span>
                    <AlertTriangle className={`h-5 w-5 ${stats.totalEvents > 10 ? 'text-neon-red' : 'text-neon-green'}`} />
                  </div>
                  <div className={`digital-number text-3xl font-bold ${stats.totalEvents > 10 ? 'text-neon-red' : 'text-neon-green'}`}>
                    {stats.totalEvents ?? 0}
                  </div>
                  <p className="text-xs text-fg-muted mt-2">Alert warnings</p>
                </GlassCard>
              </div>

              {/* Detection Classes with Circular Gauges */}
              <div className="grid gap-6 grid-cols-1 xl:grid-cols-2">
                <GlassCard scanLines className="p-6">
                  <h3 className="text-xl font-bold text-neon-cyan mb-6 tracking-wide">DETECTION DISTRIBUTION</h3>
                  <div className="space-y-4">
                    {classes.map((cls, idx) => {
                      const countVal = cls.count ?? 0
                      const counts = classes.map(c => c.count ?? 0)
                      const maxCount = Math.max(...counts, 1)
                      const width = (countVal / maxCount) * 100

                      return (
                        <div key={idx} className="space-y-2">
                          <div className="flex items-center justify-between">
                            <div className="flex items-center gap-3">
                              <Badge className="glass-card border-neon-cyan/30 text-neon-cyan">
                                {cls.class_name}
                              </Badge>
                              <span className="text-sm text-fg-secondary digital-number">
                                {countVal.toLocaleString()}
                              </span>
                            </div>
                            <span className="text-sm font-medium text-neon-green digital-number">
                              {(cls.avg_confidence * 100).toFixed(1)}%
                            </span>
                          </div>
                          <div className="h-2 bg-white/5 rounded-full overflow-hidden">
                            <div
                              className="h-full bg-gradient-to-r from-neon-cyan to-neon-green rounded-full transition-all"
                              style={{
                                width: `${width}%`,
                                boxShadow: '0 0 10px var(--neon-cyan)'
                              }}
                            />
                          </div>
                        </div>
                      )
                    })}
                  </div>
                </GlassCard>

                {/* Top Classes with Circular Gauges */}
                <GlassCard scanLines className="p-6">
                  <h3 className="text-xl font-bold text-neon-cyan mb-6 tracking-wide">TOP PERFORMERS</h3>
                  <div className="grid grid-cols-3 gap-4">
                    {classes
                      .sort((a, b) => (b.count ?? 0) - (a.count ?? 0))
                      .slice(0, 3)
                      .map((cls, idx) => {
                        const counts = classes.map(c => c.count ?? 0)
                        const maxCount = Math.max(...counts, 1)
                        const percentage = ((cls.count ?? 0) / maxCount) * 100

                        return (
                          <div key={idx} className="flex flex-col items-center">
                            <CircularGauge
                              value={percentage}
                              max={100}
                              size={100}
                              color={idx === 0 ? 'cyan' : idx === 1 ? 'green' : 'yellow'}
                              showValue={false}
                            />
                            <div className="mt-3 text-center">
                              <div className="text-sm font-bold text-fg-primary">{cls.class_name}</div>
                              <div className="text-xs text-fg-secondary digital-number mt-1">
                                {(cls.count ?? 0).toLocaleString()}
                              </div>
                            </div>
                          </div>
                        )
                      })}
                  </div>
                </GlassCard>
              </div>



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

              <div className="grid gap-6 grid-cols-1 lg:grid-cols-3">
                <GlassCard className="p-6">
                  <HighchartsReact
                    highcharts={Highcharts}
                    options={{
                      chart: {
                        type: 'pie',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Sử Dụng CPU',
                        style: {
                          color: '#00E5FF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '18px',
                          fontWeight: '700',
                          marginBottom: '15px'
                        }
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
                        pointFormat: '<b>{point.percentage:.1f}%</b>'
                      },
                      plotOptions: {
                        pie: {
                          innerSize: '60%',
                          dataLabels: {
                            enabled: true,
                            connectorWidth: 2,
                            connectorColor: 'rgba(186, 230, 253, 0.3)',
                            distance: 20,
                            format: '<b>{point.name}</b><br/>{point.percentage:.1f}%',
                            style: {
                              color: '#FFFFFF',
                              fontFamily: 'var(--font-inter)',
                              fontSize: '13px',
                              fontWeight: '600',
                              textOutline: 'none'
                            }
                          },
                          showInLegend: false,
                          borderWidth: 0
                        }
                      },
                      series: [{
                        name: 'CPU',
                        data: [
                          { name: 'Đang dùng', y: 68, color: '#00E5FF' },
                          { name: 'Còn trống', y: 32, color: 'rgba(255, 255, 255, 0.15)' }
                        ]
                      }],
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
                        type: 'pie',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Sử Dụng GPU',
                        style: {
                          color: '#00FFA3',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '18px',
                          fontWeight: '700',
                          marginBottom: '15px'
                        }
                      },
                      tooltip: {
                        backgroundColor: 'rgba(10, 22, 40, 0.95)',
                        borderColor: '#00FFA3',
                        borderRadius: 8,
                        style: {
                          color: '#FFFFFF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '12px'
                        },
                        pointFormat: '<b>{point.percentage:.1f}%</b>'
                      },
                      plotOptions: {
                        pie: {
                          innerSize: '60%',
                          dataLabels: {
                            enabled: true,
                            connectorWidth: 2,
                            connectorColor: 'rgba(186, 230, 253, 0.3)',
                            distance: 20,
                            format: '<b>{point.name}</b><br/>{point.percentage:.1f}%',
                            style: {
                              color: '#FFFFFF',
                              fontFamily: 'var(--font-inter)',
                              fontSize: '13px',
                              fontWeight: '600',
                              textOutline: 'none'
                            }
                          },
                          showInLegend: false,
                          borderWidth: 0
                        }
                      },
                      series: [{
                        name: 'GPU',
                        data: [
                          { name: 'Đang dùng', y: 82, color: '#00FFA3' },
                          { name: 'Còn trống', y: 18, color: 'rgba(255, 255, 255, 0.15)' }
                        ]
                      }],
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
                        type: 'pie',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Sử Dụng RAM',
                        style: {
                          color: '#FFD700',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '18px',
                          fontWeight: '700',
                          marginBottom: '15px'
                        }
                      },
                      tooltip: {
                        backgroundColor: 'rgba(10, 22, 40, 0.95)',
                        borderColor: '#FFD700',
                        borderRadius: 8,
                        style: {
                          color: '#FFFFFF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '12px'
                        },
                        pointFormat: '<b>{point.percentage:.1f}%</b>'
                      },
                      plotOptions: {
                        pie: {
                          innerSize: '60%',
                          dataLabels: {
                            enabled: true,
                            connectorWidth: 2,
                            connectorColor: 'rgba(186, 230, 253, 0.3)',
                            distance: 20,
                            format: '<b>{point.name}</b><br/>{point.percentage:.1f}%',
                            style: {
                              color: '#FFFFFF',
                              fontFamily: 'var(--font-inter)',
                              fontSize: '13px',
                              fontWeight: '600',
                              textOutline: 'none'
                            }
                          },
                          showInLegend: false,
                          borderWidth: 0
                        }
                      },
                      series: [{
                        name: 'RAM',
                        data: [
                          { name: 'Đang dùng', y: 54, color: '#FFD700' },
                          { name: 'Còn trống', y: 46, color: 'rgba(255, 255, 255, 0.15)' }
                        ]
                      }],
                      credits: {
                        enabled: false
                      }
                    }}
                  />
                </GlassCard>
              </div>

              <div className="grid gap-6 grid-cols-1 lg:grid-cols-2">
                <GlassCard className="p-6">
                  <HighchartsReact
                    highcharts={Highcharts}
                    options={{
                      chart: {
                        type: 'column',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Phân Bố Mức Độ Sự Kiện',
                        style: {
                          color: '#00E5FF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '16px',
                          fontWeight: '600'
                        }
                      },
                      xAxis: {
                        categories: ['Thông tin', 'Cảnh báo', 'Nghiêm trọng', 'Khẩn cấp'],
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
                          text: 'Số lượng',
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
                        column: {
                          borderRadius: 4,
                          borderWidth: 0,
                          dataLabels: {
                            enabled: true,
                            style: {
                              color: '#FFFFFF',
                              fontFamily: 'var(--font-inter)',
                              fontSize: '11px',
                              fontWeight: '600',
                              textOutline: 'none'
                            }
                          }
                        }
                      },
                      series: [{
                        name: 'Sự kiện',
                        data: [
                          { y: 145, color: '#00E5FF' },
                          { y: 89, color: '#FFD700' },
                          { y: 34, color: '#FF9500' },
                          { y: 12, color: '#FF3B3B' }
                        ]
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

                <GlassCard className="p-6">
                  <HighchartsReact
                    highcharts={Highcharts}
                    options={{
                      chart: {
                        type: 'line',
                        backgroundColor: 'transparent',
                        height: 300
                      },
                      title: {
                        text: 'Thời Gian Phản Hồi (ms)',
                        style: {
                          color: '#00E5FF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '16px',
                          fontWeight: '600'
                        }
                      },
                      xAxis: {
                        categories: ['00:00', '04:00', '08:00', '12:00', '16:00', '20:00', '24:00'],
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
                          text: 'Mili giây',
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
                        backgroundColor: 'rgba(10, 22, 40, 0.95)',
                        borderColor: '#00E5FF',
                        borderRadius: 8,
                        style: {
                          color: '#FFFFFF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '12px'
                        },
                        valueSuffix: ' ms'
                      },
                      plotOptions: {
                        line: {
                          marker: {
                            radius: 4,
                            lineColor: '#050B14',
                            lineWidth: 2
                          }
                        }
                      },
                      series: [{
                        name: 'Phản hồi API',
                        data: [45, 38, 42, 51, 48, 43, 39],
                        color: '#B794F6',
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
