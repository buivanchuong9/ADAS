"use client";

import { useState, useEffect } from "react";
import {
  Activity,
  AlertTriangle,
  Camera,
  TrendingUp,
  Zap,
  Eye,
  Users,
  Clock,
} from "lucide-react";
import { StatCard } from "@/components/stat-card";
import { LiveChart } from "@/components/live-chart";
import { LiveIndicator } from "@/components/live-indicator";
import { LoadingCard } from "@/components/loading-spinner";
import { ErrorCard } from "@/components/error-message";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import {
  useDashboardStats,
  useDetections,
  useAlerts,
  useSystemMetrics,
} from "@/lib/use-api";
import { cn } from "@/lib/utils";

export default function DashboardPage() {
  const [timeRange, setTimeRange] = useState<"1h" | "24h" | "7d">("1h");
  
  // Fetch data with auto-refresh
  const {
    data: stats,
    loading: statsLoading,
    error: statsError,
    refetch: refetchStats,
  } = useDashboardStats({ refreshInterval: 3000 }); // 3s refresh

  const {
    data: recentDetections,
    loading: detectionsLoading,
  } = useDetections({ limit: 10, refreshInterval: 2000 }); // 2s refresh

  const {
    data: recentAlerts,
    loading: alertsLoading,
  } = useAlerts({ limit: 5, refreshInterval: 2000 });

  const {
    data: metrics,
    loading: metricsLoading,
  } = useSystemMetrics({ refreshInterval: 5000 }); // 5s refresh

  // Loading state
  if (statsLoading && !stats) {
    return (
      <div className="container mx-auto p-6">
        <LoadingCard text="Loading dashboard..." />
      </div>
    );
  }

  // Error state
  if (statsError) {
    return (
      <div className="container mx-auto p-6">
        <ErrorCard
          title="Failed to load dashboard"
          message={statsError.message}
          retry={refetchStats}
        />
      </div>
    );
  }

  return (
    <div className="container mx-auto p-6 space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold">ADAS Dashboard</h1>
          <p className="text-muted-foreground">
            Real-time monitoring and analytics
          </p>
        </div>
        <div className="flex items-center gap-3">
          <LiveIndicator isLive={true} />
          <Button variant="outline" size="sm" onClick={refetchStats}>
            Refresh
          </Button>
        </div>
      </div>

      {/* Key Metrics */}
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        <StatCard
          title="Total Detections"
          value={stats?.total_detections?.toLocaleString() || "0"}
          description="Last 24 hours"
          icon={Eye}
          trend={{
            value: stats?.detection_trend || 0,
            label: "from yesterday",
          }}
        />
        <StatCard
          title="Active Alerts"
          value={stats?.active_alerts || "0"}
          description="Requires attention"
          icon={AlertTriangle}
          variant={stats?.active_alerts > 0 ? "warning" : "default"}
        />
        <StatCard
          title="Active Cameras"
          value={stats?.active_cameras || "0"}
          description={`${stats?.total_cameras || 0} total`}
          icon={Camera}
        />
        <StatCard
          title="System Health"
          value={`${stats?.system_health || 100}%`}
          description="All systems operational"
          icon={Activity}
          variant={
            (stats?.system_health || 100) >= 90
              ? "success"
              : (stats?.system_health || 100) >= 70
              ? "warning"
              : "danger"
          }
        />
      </div>

      {/* Charts Section */}
      <Tabs defaultValue="detections" className="w-full">
        <TabsList>
          <TabsTrigger value="detections">Detection Timeline</TabsTrigger>
          <TabsTrigger value="alerts">Alert Distribution</TabsTrigger>
          <TabsTrigger value="performance">Performance</TabsTrigger>
        </TabsList>

        <TabsContent value="detections" className="space-y-4">
          <div className="grid gap-4 lg:grid-cols-2">
            <LiveChart
              title="Detections Over Time"
              description="Real-time detection rate"
              type="line"
              data={{
                labels: stats?.detection_timeline?.labels || [],
                datasets: [
                  {
                    label: "Detections",
                    data: stats?.detection_timeline?.data || [],
                    borderColor: "rgb(59, 130, 246)",
                    backgroundColor: "rgba(59, 130, 246, 0.1)",
                    fill: true,
                  },
                ],
              }}
              height={300}
            />
            <LiveChart
              title="Detection by Class"
              description="Object type distribution"
              type="bar"
              data={{
                labels: stats?.detection_by_class?.labels || ["Car", "Person", "Bicycle", "Motorcycle"],
                datasets: [
                  {
                    label: "Count",
                    data: stats?.detection_by_class?.data || [120, 45, 12, 8],
                    backgroundColor: [
                      "rgba(59, 130, 246, 0.8)",
                      "rgba(16, 185, 129, 0.8)",
                      "rgba(245, 158, 11, 0.8)",
                      "rgba(239, 68, 68, 0.8)",
                    ],
                  },
                ],
              }}
              height={300}
            />
          </div>
        </TabsContent>

        <TabsContent value="alerts" className="space-y-4">
          <div className="grid gap-4 lg:grid-cols-2">
            <LiveChart
              title="Alert Severity Distribution"
              description="Breakdown by severity level"
              type="pie"
              data={{
                labels: ["Critical", "High", "Medium", "Low"],
                datasets: [
                  {
                    data: stats?.alert_distribution || [5, 12, 28, 55],
                    backgroundColor: [
                      "rgba(239, 68, 68, 0.8)",
                      "rgba(245, 158, 11, 0.8)",
                      "rgba(59, 130, 246, 0.8)",
                      "rgba(16, 185, 129, 0.8)",
                    ],
                  },
                ],
              }}
              height={300}
            />
            <Card>
              <CardHeader>
                <CardTitle>Recent Alerts</CardTitle>
                <CardDescription>Latest system notifications</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-3">
                  {alertsLoading ? (
                    <p className="text-sm text-muted-foreground">Loading alerts...</p>
                  ) : recentAlerts && recentAlerts.length > 0 ? (
                    recentAlerts.slice(0, 5).map((alert: any, idx: number) => (
                      <div
                        key={idx}
                        className="flex items-start gap-3 p-3 rounded-lg border"
                      >
                        <AlertTriangle
                          className={cn(
                            "h-5 w-5 mt-0.5",
                            alert.severity === "critical" && "text-red-500",
                            alert.severity === "high" && "text-orange-500",
                            alert.severity === "medium" && "text-yellow-500",
                            alert.severity === "low" && "text-blue-500"
                          )}
                        />
                        <div className="flex-1 space-y-1">
                          <div className="flex items-center justify-between">
                            <p className="text-sm font-medium">{alert.message}</p>
                            <Badge variant="outline" className="text-xs">
                              {alert.severity}
                            </Badge>
                          </div>
                          <p className="text-xs text-muted-foreground">
                            {new Date(alert.timestamp).toLocaleTimeString()}
                          </p>
                        </div>
                      </div>
                    ))
                  ) : (
                    <p className="text-sm text-muted-foreground">No recent alerts</p>
                  )}
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

        <TabsContent value="performance" className="space-y-4">
          <div className="grid gap-4 lg:grid-cols-3">
            <StatCard
              title="Avg FPS"
              value={metrics?.fps || "30"}
              description="Frames per second"
              icon={Zap}
              variant="success"
            />
            <StatCard
              title="Processing Time"
              value={`${metrics?.avg_processing_time || "45"}ms`}
              description="Per frame"
              icon={Clock}
            />
            <StatCard
              title="Model Accuracy"
              value={`${metrics?.model_accuracy || "94.5"}%`}
              description="Detection precision"
              icon={TrendingUp}
              variant="success"
            />
          </div>
          <LiveChart
            title="System Performance"
            description="FPS and processing time over time"
            type="line"
            data={{
              labels: metrics?.performance_timeline?.labels || [],
              datasets: [
                {
                  label: "FPS",
                  data: metrics?.performance_timeline?.fps || [],
                  borderColor: "rgb(16, 185, 129)",
                  backgroundColor: "rgba(16, 185, 129, 0.1)",
                  fill: true,
                },
                {
                  label: "Processing Time (ms)",
                  data: metrics?.performance_timeline?.processing_time || [],
                  borderColor: "rgb(245, 158, 11)",
                  backgroundColor: "rgba(245, 158, 11, 0.1)",
                  fill: true,
                },
              ],
            }}
            height={300}
          />
        </TabsContent>
      </Tabs>

      {/* Recent Detections */}
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle>Recent Detections</CardTitle>
              <CardDescription>Latest object detections from all cameras</CardDescription>
            </div>
            <Button variant="outline" size="sm">
              View All
            </Button>
          </div>
        </CardHeader>
        <CardContent>
          <div className="space-y-2">
            {detectionsLoading ? (
              <p className="text-sm text-muted-foreground py-4">Loading detections...</p>
            ) : recentDetections && recentDetections.length > 0 ? (
              <div className="overflow-auto">
                <table className="w-full">
                  <thead className="border-b">
                    <tr className="text-sm text-muted-foreground">
                      <th className="text-left py-2 px-3">Time</th>
                      <th className="text-left py-2 px-3">Camera</th>
                      <th className="text-left py-2 px-3">Object</th>
                      <th className="text-left py-2 px-3">Confidence</th>
                      <th className="text-left py-2 px-3">Distance</th>
                    </tr>
                  </thead>
                  <tbody>
                    {recentDetections.slice(0, 10).map((det: any, idx: number) => (
                      <tr key={idx} className="border-b last:border-0">
                        <td className="py-2 px-3 text-sm">
                          {new Date(det.timestamp).toLocaleTimeString()}
                        </td>
                        <td className="py-2 px-3 text-sm">{det.camera_id}</td>
                        <td className="py-2 px-3">
                          <Badge variant="outline">{det.class_name}</Badge>
                        </td>
                        <td className="py-2 px-3 text-sm">
                          {(det.confidence * 100).toFixed(1)}%
                        </td>
                        <td className="py-2 px-3 text-sm">
                          {det.distance ? `${det.distance.toFixed(1)}m` : "N/A"}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            ) : (
              <p className="text-sm text-muted-foreground py-4">No recent detections</p>
            )}
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
