"use client"

import { useEffect, useState } from "react"
import { useRouter } from "next/navigation"
import { useAuth } from "@/contexts/auth-context"
import { useLanguage } from "@/contexts/language-context"
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
import { authService } from "@/lib/auth/auth.service"

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
  const router = useRouter()
  const { isAuthenticated, loading: authLoading } = useAuth()
  const { t } = useLanguage()
  const [stats, setStats] = useState<Stats>({
    totalDetections: 0,
    totalTrips: 0,
    totalEvents: 0,
    avgSafetyScore: 0
  })
  const [classes, setClasses] = useState<DetectionClass[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  // Redirect to login if not authenticated
  useEffect(() => {
    if (!authLoading && !isAuthenticated) {
      router.push('/login')
    }
  }, [isAuthenticated, authLoading, router])

  useEffect(() => {
    // Only fetch data if authenticated
    if (!isAuthenticated || authLoading) {
      return
    }

    fetchData()
    const interval = setInterval(fetchData, 30000) // Increased to 30 seconds to reduce load
    return () => clearInterval(interval)
  }, [isAuthenticated, authLoading])

  const fetchData = async () => {
    try {
      setError(null)

      // Get access token
      const session = await authService.getSession()
      if (!session?.access_token) {
        console.warn('No access token available')
        setError(t('dashboard.sessionExpired'))
        router.push('/login')
        return
      }

      const headers = {
        'Authorization': `Bearer ${session.access_token}`,
        'Content-Type': 'application/json'
      }

      // Fetch analytics with 405 fallback
      try {
        const analyticsRes = await fetch(getApiUrl(API_ENDPOINTS.ADMIN_OVERVIEW), { headers })
        if (analyticsRes.ok) {
          const data = await analyticsRes.json()
          const statsData = data.data || data
          setStats(statsData)
        } else if (analyticsRes.status === 401) {
          console.warn('Unauthorized - redirecting to login')
          router.push('/login')
          return
        } else if (analyticsRes.status === 405) {
          console.warn('⚠️ Backend endpoint not available (405) - using mock data')
          setStats({
            totalDetections: 1247,
            totalTrips: 89,
            totalEvents: 156,
            avgSafetyScore: 94.5
          })
        } else {
          console.error('Analytics API error:', analyticsRes.status)
        }
      } catch (err) {
        console.warn('Analytics API failed:', err)
        setStats({
          totalDetections: 1247,
          totalTrips: 89,
          totalEvents: 156,
          avgSafetyScore: 94.5
        })
      }

      // Fetch detection stats with 405 fallback
      try {
        const detectionRes = await fetch(getApiUrl(API_ENDPOINTS.DETECTIONS_STATS), { headers })
        if (detectionRes.ok) {
          const data = await detectionRes.json()
          const classesData = data.data?.classes || data.classes || []
          setClasses(classesData)
        } else if (detectionRes.status === 401) {
          console.warn('Unauthorized - redirecting to login')
          router.push('/login')
          return
        } else if (detectionRes.status === 405) {
          console.warn('⚠️ Backend endpoint not available (405) - using mock data')
          setClasses([
            { class_name: 'car', count: 523, avg_confidence: 0.92 },
            { class_name: 'person', count: 341, avg_confidence: 0.88 },
            { class_name: 'truck', count: 187, avg_confidence: 0.85 },
            { class_name: 'motorcycle', count: 196, avg_confidence: 0.90 }
          ])
        } else {
          console.error('Detection stats API error:', detectionRes.status)
        }
      } catch (err) {
        console.warn('Detection stats API failed:', err)
        setClasses([
          { class_name: 'car', count: 523, avg_confidence: 0.92 },
          { class_name: 'person', count: 341, avg_confidence: 0.88 },
          { class_name: 'truck', count: 187, avg_confidence: 0.85 },
          { class_name: 'motorcycle', count: 196, avg_confidence: 0.90 }
        ])
      }
    } catch (err) {
      console.error('Error fetching dashboard data:', err)
      setError(t('dashboard.usingMockData'))
    } finally {
      setLoading(false)
    }
  }

  // Show loading while checking authentication
  if (authLoading) {
    return (
      <div className="flex h-screen items-center justify-center bg-bg-primary">
        <div className="text-center">
          <div className="inline-block animate-spin rounded-full h-12 w-12 border-b-2 border-neon-cyan" />
          <p className="mt-4 text-fg-secondary">{t('dashboard.checkingAuth')}</p>
        </div>
      </div>
    )
  }

  // Don't render dashboard if not authenticated
  if (!isAuthenticated) {
    return null
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
              <h1 className="text-3xl font-bold text-neon-cyan tracking-wider">{t('dashboard.title')}</h1>
              <p className="text-sm text-fg-secondary mt-1">
                {t('dashboard.subtitle')}
              </p>
            </div>
            <Badge className="glass-card border-neon-green/50 text-neon-green px-4 py-2">
              <div className="flex items-center gap-2">
                <div className="w-2 h-2 rounded-full bg-neon-green animate-pulse" style={{ boxShadow: '0 0 10px var(--neon-green)' }} />
                {t('common.online')}
              </div>
            </Badge>
          </div>

          {/* Error Message */}
          {error && (
            <div className="glass-card border-red-500/50 bg-red-500/10 p-4 rounded-xl">
              <div className="flex items-center gap-3">
                <AlertTriangle className="w-5 h-5 text-red-400" />
                <p className="text-red-400 text-sm">{error}</p>
              </div>
            </div>
          )}

          {loading ? (
            <div className="text-center py-12">
              <div className="inline-block animate-spin rounded-full h-12 w-12 border-b-2 border-neon-cyan" />
              <p className="mt-4 text-fg-secondary">{t('dashboard.loadingData')}</p>
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
                        text: t('dashboard.detectionTrendTitle'),
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
                          text: t('settings.detectionCount'),
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
                        text: t('dashboard.accuracyTrendTitle'),
                        style: {
                          color: '#00E5FF',
                          fontFamily: 'var(--font-inter)',
                          fontSize: '16px',
                          fontWeight: '600'
                        }
                      },
                      xAxis: {
                        categories: [t('settings.monday'), t('settings.tuesday'), t('settings.wednesday'), t('settings.thursday'), t('settings.friday'), t('settings.saturday'), t('settings.sunday')],
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
                        name: t('settings.accuracy'),
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