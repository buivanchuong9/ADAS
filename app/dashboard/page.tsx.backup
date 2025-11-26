"use client"

import { useEffect, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Activity, AlertTriangle, Car, Clock } from "lucide-react"

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'

interface DashboardData {
  total_trips: number
  total_events: number
  total_cameras: number
  active_drivers: number
  recent_events?: Array<{
    id: number
    type: string
    severity: string
    description: string
    timestamp: string
  }>
}

export default function Dashboard() {
  const [dashboardData, setDashboardData] = useState<DashboardData>({
    total_trips: 0,
    total_events: 0,
    total_cameras: 0,
    active_drivers: 0,
    recent_events: []
  })
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    const fetchDashboardData = async () => {
      try {
        const response = await fetch(`${API_BASE_URL}/api/analytics/dashboard`)
        if (response.ok) {
          const data = await response.json()
          setDashboardData(data)
        }
      } catch (error) {
        console.error('Failed to fetch dashboard data:', error)
      } finally {
        setLoading(false)
      }
    }

    fetchDashboardData()
    // Refresh every 30 seconds
    const interval = setInterval(fetchDashboardData, 30000)
    return () => clearInterval(interval)
  }, [])

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 p-6">
      <div className="max-w-7xl mx-auto">
        <h1 className="text-4xl font-bold text-white mb-8">Dashboard ADAS</h1>
        
        {loading ? (
          <div className="text-white text-center">Loading...</div>
        ) : (
          <>
            {/* Stats Cards */}
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
              <Card className="bg-slate-800 border-slate-700">
                <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                  <CardTitle className="text-sm font-medium text-slate-200">
                    Total Trips
                  </CardTitle>
                  <Car className="h-4 w-4 text-blue-400" />
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-white">{dashboardData.total_trips}</div>
                </CardContent>
              </Card>

              <Card className="bg-slate-800 border-slate-700">
                <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                  <CardTitle className="text-sm font-medium text-slate-200">
                    Total Events
                  </CardTitle>
                  <AlertTriangle className="h-4 w-4 text-yellow-400" />
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-white">{dashboardData.total_events}</div>
                </CardContent>
              </Card>

              <Card className="bg-slate-800 border-slate-700">
                <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                  <CardTitle className="text-sm font-medium text-slate-200">
                    Active Cameras
                  </CardTitle>
                  <Activity className="h-4 w-4 text-green-400" />
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-white">{dashboardData.total_cameras}</div>
                </CardContent>
              </Card>

              <Card className="bg-slate-800 border-slate-700">
                <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                  <CardTitle className="text-sm font-medium text-slate-200">
                    Active Drivers
                  </CardTitle>
                  <Clock className="h-4 w-4 text-purple-400" />
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-white">{dashboardData.active_drivers}</div>
                </CardContent>
              </Card>
            </div>

            {/* Recent Events */}
            <Card className="bg-slate-800 border-slate-700">
              <CardHeader>
                <CardTitle className="text-white">Recent Events</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  {dashboardData.recent_events && dashboardData.recent_events.length > 0 ? (
                    dashboardData.recent_events.map((event) => (
                      <div key={event.id} className="flex items-start space-x-4 border-b border-slate-700 pb-4">
                        <div className={`p-2 rounded-lg ${
                          event.severity === 'critical' ? 'bg-red-500/20' :
                          event.severity === 'warning' ? 'bg-yellow-500/20' :
                          'bg-blue-500/20'
                        }`}>
                          <AlertTriangle className={`h-5 w-5 ${
                            event.severity === 'critical' ? 'text-red-400' :
                            event.severity === 'warning' ? 'text-yellow-400' :
                            'text-blue-400'
                          }`} />
                        </div>
                        <div className="flex-1">
                          <div className="flex items-center justify-between">
                            <h4 className="text-white font-semibold">{event.type}</h4>
                            <span className="text-xs text-slate-400">
                              {new Date(event.timestamp).toLocaleString()}
                            </span>
                          </div>
                          <p className="text-slate-300 text-sm mt-1">{event.description}</p>
                        </div>
                      </div>
                    ))
                  ) : (
                    <div className="text-slate-400 text-center py-8">No recent events</div>
                  )}
                </div>
              </CardContent>
            </Card>
          </>
        )}
      </div>
    </div>
  )
}
