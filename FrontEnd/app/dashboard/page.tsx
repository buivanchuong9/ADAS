"use client";

import Highcharts from "highcharts";
import HighchartsReact from "highcharts-react-official";
import { GlassCard } from "@/components/ui/glass-card";
import { useMemo } from "react";
import { useState, useEffect } from "react";
import { useRouter } from "next/navigation";
import { motion, Variants } from "framer-motion";
import { Sidebar } from "@/components/sidebar";
import { MobileNav } from "@/components/mobile-nav";
import { Header } from "@/components/header";
import { Button } from "@/components/ui/button";
import { useLanguage } from "@/contexts/language-context";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
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
  Camera,
  Cctv,
  BookOpen,

} from "lucide-react"
import Link from "next/link"
import { HighchartsChart } from "@/components/charts/highcharts-chart"
import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"


const containerVariants: Variants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1,
      delayChildren: 0.2,
    },
  },
};

const itemVariants: Variants = {
  hidden: { opacity: 0, y: 20 },
  visible: {
    opacity: 1,
    y: 0,
    transition: {
      duration: 0.5,
      ease: [0.4, 0, 0.2, 1] as const,
    },
  },
};

export default function HomePage() {
  const router = useRouter();
  const { t } = useLanguage();
    const aiUsageOptions = useMemo(
    () => ({
      chart: { type: "pie", backgroundColor: "transparent", height: 280 },
      title: {
        text: t("settings.aiUsage"),
        style: { color: "#ff7a1a", fontSize: "16px", fontWeight: "600" },
      },
      tooltip: {
        pointFormat: "<b>{point.percentage:.1f}%</b><br/>Số lượng: {point.y}",
      },
      plotOptions: {
        pie: {
          innerSize: "60%",
          dataLabels: {
            enabled: true,
            format: "<b>{point.name}</b><br>{point.percentage:.1f}%",
          },
        },
      },
      series: [
        {
          name: "Queries",
          data: [
            { name: "Driving Support", y: 456 },
            { name: "Data Analysis", y: 289 },
            { name: "Alerts", y: 178 },
            { name: "Other", y: 123 },
          ],
        },
      ],
      credits: { enabled: false },
    }),
    [t],
  );

  const confidenceDistOptions = useMemo(
    () => ({
      chart: { type: "column", backgroundColor: "transparent", height: 280 },
      title: {
        text: t("settings.confidenceDistribution"),
        style: { color: "#ff7a1a", fontSize: "16px", fontWeight: "600" },
      },
      xAxis: {
        categories: ["50-60%", "60-70%", "70-80%", "80-90%", "90-100%"],
      },
      series: [
        {
          name: t("settings.detections"),
          data: [89, 234, 567, 892, 1245],
        },
      ],
      credits: { enabled: false },
    }),
    [t],
  );

  const notificationTimelineOptions = useMemo(
    () => ({
      chart: { type: "spline", backgroundColor: "transparent", height: 280 },
      title: {
        text: t("settings.notificationTimeline"),
        style: { color: "#ff7a1a", fontSize: "16px", fontWeight: "600" },
      },
      xAxis: {
        categories: ["0h", "3h", "6h", "9h", "12h", "15h", "18h", "21h", "24h"],
      },
      series: [
        { name: "Warnings", data: [12, 8, 15, 23, 34, 28, 19, 25, 18] },
        { name: "Critical", data: [3, 2, 5, 8, 12, 9, 6, 7, 4] },
      ],
      credits: { enabled: false },
    }),
    [t],
  );

  const [stats, setStats] = useState({
    systemStatus: t('common.online'),
    activeCameras: 0,
    totalDetections: 0,
    alertsToday: 0,
  });

  useEffect(() => {
    const fetchStats = async () => {
      try {
        // Fetch health status with timeout
        const statusRes = await fetch(getApiUrl(API_ENDPOINTS.HEALTH), {
          signal: AbortSignal.timeout(5000), // 5 second timeout
        }).catch(() => null);

        let systemStatus = t('common.offline');
        if (statusRes && statusRes.ok) {
          const statusData = await statusRes.json().catch(() => ({}));
          systemStatus = statusData.status === "success" ? t('common.online') : t('common.offline');
        }

        // Fetch alerts statistics with timeout
        const alertsRes = await fetch(getApiUrl(API_ENDPOINTS.ADMIN_STATISTICS), {
          signal: AbortSignal.timeout(5000), // 5 second timeout
        }).catch(() => null);

        let alertsToday = 0;
        if (alertsRes && alertsRes.ok) {
          const alertsData = await alertsRes.json().catch(() => ({}));
          alertsToday = alertsData.data?.total_alerts || alertsData.total_alerts || 0;
        }

        setStats({
          systemStatus,
          activeCameras: systemStatus === t('common.online') ? 1 : 0,
          totalDetections: 0,
          alertsToday,
        });
      } catch (err) {
        // Silently handle errors - backend may not be running
        // Set offline state without logging errors
        setStats({
          systemStatus: t('common.offline'),
          activeCameras: 0,
          totalDetections: 0,
          alertsToday: 0,
        });
      }
    };

    fetchStats();
    const interval = setInterval(fetchStats, 30000); // Check every 30 seconds instead of 5
    return () => clearInterval(interval);
  }, []);

  // Sample chart data
  const detectionChartData = [
    { name: t('settings.vehicles'), y: 45 },
    { name: t('settings.people'), y: 25 },
    { name: t('home.cycles'), y: 20 },
    { name: t('settings.other'), y: 10 },
  ];

  const performanceChartData = [
    {
      name: t('settings.performanceLabel'),
      data: [65, 72, 68, 75, 80, 78, 85],
      color: "#667eea",
    },
  ];

  return (
    <div className="flex flex-col h-screen bg-bg-primary overflow-hidden">
      <Header />
      <div className="flex flex-1 overflow-hidden">
        <MobileNav />
        <Sidebar />

        <main className="flex-1 overflow-auto">
          <motion.div
            className="p-4 sm:p-6 lg:p-8 space-y-6 sm:space-y-8"
            variants={containerVariants}
            initial="hidden"
            animate="visible"
          >
            {/* Hero Section - Dark Sci-Fi */}
            <motion.div
              variants={itemVariants}
              className="relative overflow-hidden rounded-3xl glass-card scan-lines p-8 lg:p-10"
            >
              {/* Animated gradient orbs */}
              <motion.div
                className="absolute top-0 right-0 w-96 h-96 bg-neon-cyan/10 rounded-full blur-3xl"
                animate={{
                  scale: [1, 1.2, 1],
                  opacity: [0.2, 0.4, 0.2],
                }}
                transition={{
                  duration: 8,
                  repeat: Infinity,
                  ease: "easeInOut",
                }}
              />
              <motion.div
                className="absolute bottom-0 left-0 w-80 h-80 bg-neon-purple/10 rounded-full blur-3xl"
                animate={{
                  scale: [1, 1.3, 1],
                  opacity: [0.2, 0.4, 0.2],
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
                  <Badge
                    variant="outline"
                    className="border-neon-cyan/50 text-neon-cyan glass-card"
                  >
                    v3.0 Professional
                  </Badge>
                </motion.div>

                <motion.h1
                  className="text-3xl sm:text-4xl md:text-5xl lg:text-6xl font-bold my-7 text-neon-cyan"
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.4 }}
                >
                  {t('home.title')}
                </motion.h1>

                <motion.div
                  className="flex flex-wrap gap-3 sm:gap-4"
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.6 }}
                >
                  <Link href="/adas">
                    <button
                      className="btn-neon w-full sm:w-auto"
                    >
                      <Zap className="w-5 h-5 mr-2 inline" />
                      {t('home.startDetection')}
                    </button>
                  </Link>
                  
                </motion.div>
              </div>
            </motion.div>

{/* === END DASHBOARD CHARTS === */}

            {/* System Status Cards - Premium Grid */}
            <motion.div
              variants={itemVariants}
              className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4 sm:gap-6"
            >
              {[
                {
                  title: t('home.systemStatus'),
                  value: stats.systemStatus,
                  icon: Activity,
                  color: "success",
                  description: t('home.allSystemsOnline'),
                },
                {
                  title: t('home.activeCameras'),
                  value: stats.activeCameras.toString(),
                  icon: Cctv,
                  color: "primary",
                  description: t('home.realTimeMonitoring'),
                },
                {
                  title: t('home.totalDetections'),
                  value: stats.totalDetections.toLocaleString(),
                  icon: BookOpen,
                  color: "info",
                  description: t('home.percentIncrease', { percent: 12 }),
                  trend: true,
                },
                {
                  title: t('home.alertsToday'),
                  value: stats.alertsToday.toString(),
                  icon: AlertTriangle,
                  color: "warning",
                  description: t('home.safetyAlertsIssued'),
                },
              ].map((stat, index) => (
                <motion.div
                  key={stat.title}
                  variants={itemVariants}
                  whileHover={{ y: -8, scale: 1.02 }}
                  transition={{ type: "spring", stiffness: 300 }}
                >
                  <Card
                    glass
                    className="border-border/50 hover:border-primary/30 transition-all duration-300"
                  >
                    <CardHeader className="pb-6">
                      <div className="flex items-center justify-between">
                        <CardTitle className="text-sm font-medium text-muted-foreground">
                          {stat.title}
                        </CardTitle>
                        <stat.icon className={`w-5 h-5 text-${stat.color}`} />
                      </div>
                    </CardHeader>
                    <CardContent className="pb-8">
                      <div className="flex items-center gap-2 mb-2">
                        {stat.title === t('home.systemStatus') && (
                          <motion.div
                            className="w-2 h-2 rounded-full bg-success"
                            animate={{ scale: [1, 1.2, 1], opacity: [1, 0.7, 1] }}
                            transition={{ duration: 2, repeat: Infinity }}
                          />
                        )}
                        <span className="text-xl font-bold text-foreground capitalize">
                          {stat.value}
                        </span>
                      </div>
                      <p className="text-xs text-muted-foreground flex items-center gap-1">
                        {stat.trend && (
                          <TrendingUp className="w-3 h-3 text-success" />
                        )}
                        {stat.description}
                      </p>
                    </CardContent>
                  </Card>
                </motion.div>
              ))}
            </motion.div>
            <div className="grid gap-6 grid-cols-1 lg:grid-cols-2">
  <GlassCard className="p-6">
    <HighchartsReact
      highcharts={Highcharts}
      options={aiUsageOptions}
    />
  </GlassCard>

  <GlassCard className="p-6">
    <HighchartsReact
      highcharts={Highcharts}
      options={confidenceDistOptions}
    />
  </GlassCard>
</div>

<GlassCard className="p-6">
  <HighchartsReact
    highcharts={Highcharts}
    options={notificationTimelineOptions}
  />
</GlassCard>

{/* === DASHBOARD CHARTS (GỘP TỪ DASHBOARD) === */}
<div className="grid gap-6 grid-cols-1 lg:grid-cols-2">
  <GlassCard className="p-6">
    <HighchartsReact
      highcharts={Highcharts}
      options={{
        chart: {
          type: "area",
          backgroundColor: "transparent",
          height: 300,
        },
        title: {
          text: t("dashboard.detectionTrendTitle"),
          style: {
            color: "#ff7a1a",
            fontFamily: "var(--font-inter)",
            fontSize: "16px",
            fontWeight: "600",
          },
        },
        xAxis: {
          categories: [
            "10:00",
            "10:05",
            "10:10",
            "10:15",
            "10:20",
            "10:25",
            "10:30",
          ],
          labels: {
            style: {
              color: "#111827",
              fontFamily: "var(--font-inter)",
              fontSize: "11px",
            },
          },
          lineColor: "rgba(255, 255, 255, 0.1)",
          tickColor: "rgba(255, 255, 255, 0.1)",
        },
        yAxis: {
          title: {
            text: t("settings.detectionCount"),
            style: {
              color: "#111827",
              fontFamily: "var(--font-inter)",
              fontSize: "12px",
            },
          },
          labels: {
            style: {
              color: "#111827",
              fontFamily: "var(--font-inter)",
              fontSize: "11px",
            },
          },
          gridLineColor: "rgba(255, 255, 255, 0.05)",
        },
        tooltip: {
          shared: true,
          backgroundColor: "rgba(10, 22, 40, 0.95)",
          borderColor: "#00E5FF",
          borderRadius: 8,
          style: {
            color: "#FFFFFF",
            fontFamily: "var(--font-inter)",
            fontSize: "12px",
          },
        },
        plotOptions: {
          area: {
            fillOpacity: 0.3,
            marker: {
              radius: 4,
              lineWidth: 2,
            },
          },
        },
        series: [
          {
            name: "Xe cộ",
            data: [45, 52, 48, 61, 58, 65, 72],
            color: "#00E5FF",
            fillColor: {
              linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
              stops: [
                [0, "rgba(0, 229, 255, 0.3)"],
                [1, "rgba(0, 229, 255, 0.05)"],
              ],
            },
          },
          {
            name: "Người đi bộ",
            data: [28, 31, 35, 29, 42, 38, 45],
            color: "#00FFA3",
            fillColor: {
              linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
              stops: [
                [0, "rgba(0, 255, 163, 0.3)"],
                [1, "rgba(0, 255, 163, 0.05)"],
              ],
            },
          },
        ],
        legend: {
          itemStyle: {
            color: "#111827",
            fontFamily: "var(--font-inter)",
            fontSize: "12px",
            fontWeight: "500",
          },
          itemHoverStyle: {
            color: "#000000",
          },
        },
        credits: {
          enabled: false,
        },
      }}
    />
  </GlassCard>

  <GlassCard className="p-6">
    <HighchartsReact
      highcharts={Highcharts}
      options={{
        chart: {
          type: "spline",
          backgroundColor: "transparent",
          height: 300,
        },
        title: {
          text: t("dashboard.accuracyTrendTitle"),
          style: {
            color: "#ff7a1a",
            fontFamily: "var(--font-inter)",
            fontSize: "16px",
            fontWeight: "600",
          },
        },
        xAxis: {
          categories: [
            t("settings.monday"),
            t("settings.tuesday"),
            t("settings.wednesday"),
            t("settings.thursday"),
            t("settings.friday"),
            t("settings.saturday"),
            t("settings.sunday"),
          ],
          labels: {
            style: {
              color: "#111827",
              fontFamily: "var(--font-inter)",
              fontSize: "11px",
            },
          },
          lineColor: "rgba(255, 255, 255, 0.1)",
          tickColor: "rgba(255, 255, 255, 0.1)",
        },
        yAxis: {
          title: {
            text: "Độ chính xác (%)",
            style: {
              color: "#111827",
              fontFamily: "var(--font-inter)",
              fontSize: "12px",
            },
          },
          labels: {
            style: {
              color: "#111827",
              fontFamily: "var(--font-inter)",
              fontSize: "11px",
            },
          },
          gridLineColor: "rgba(255, 255, 255, 0.05)",
          min: 90,
          max: 100,
        },
        tooltip: {
          backgroundColor: "rgba(10, 22, 40, 0.95)",
          borderColor: "#00E5FF",
          borderRadius: 8,
          style: {
            color: "#FFFFFF",
            fontFamily: "var(--font-inter)",
            fontSize: "12px",
          },
          valueSuffix: "%",
        },
        plotOptions: {
          spline: {
            marker: {
              radius: 4,
              lineColor: "#050B14",
              lineWidth: 2,
            },
          },
        },
        series: [
          {
            name: t("settings.accuracy"),
            data: [96.5, 97.2, 96.8, 98.1, 97.9, 98.5, 98.3],
            color: "#00FFA3",
            marker: {
              symbol: "circle",
            },
          },
        ],
        legend: {
          enabled: false,
        },
        credits: {
          enabled: false,
        },
      }}
    />
  </GlassCard>
</div>
            {/* Charts Section */}
            <motion.div
              variants={itemVariants}
              className="grid grid-cols-1 xl:grid-cols-2 gap-4 sm:gap-6"
            >
              <HighchartsChart
                title={t('home.detectionDistribution')}
                description={t('home.detectionDistributionDesc')}
                type="pie"
                data={detectionChartData}
                height={300}
                className="sm:pl-8"
              />
              <HighchartsChart
                title={t('home.systemPerformance')}
                description={t('home.systemPerformanceDesc')}
                type="line"
                data={performanceChartData}
                height={300}
              />
            </motion.div>

            {/* Quick Actions & Features */}
            <motion.div
              variants={itemVariants}
              className="grid grid-cols-1 xl:grid-cols-2 gap-4 sm:gap-6"
            >
              <Card glass className="xl:col-span-2 w-full">

                <CardHeader>
                  <CardTitle className="text-xl">{t('home.quickActions')}</CardTitle>
                  <CardDescription>
                    {t('home.quickActionsDesc')}
                  </CardDescription>
                </CardHeader>
                <CardContent className="pt-3 grid grid-cols-1 md:grid-cols-3 gap-6">
                  {[
                    {
                      href: "/adas",
                      icon: Zap,
                      title: t('home.startLiveDetection'),
                      description: t('home.startLiveDetectionDesc'),
                      gradient: "from-primary to-primary/80",
                    },
                    {
                      href: "/driver-monitor",
                      icon: Eye,
                      title: t('home.monitorDriver'),
                      description: t('home.monitorDriverDesc'),
                      gradient: "from-accent to-accent/80",
                    },
                    {
                      href: "/analytics",
                      icon: TrendingUp,
                      title: t('home.viewAnalytics'),
                      description: t('home.viewAnalyticsDesc'),
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
                          <div
                            className={`w-12 h-12 rounded-xl bg-gradient-to-br ${action.gradient} flex items-center justify-center shadow-lg group-hover:scale-110 transition-transform`}
                          >
                            <action.icon className="w-6 h-6 text-white" />
                          </div>
                          <div>
                            <div className="font-semibold text-foreground group-hover:text-primary transition-colors">
                              {action.title}
                            </div>
                            <div className="text-sm text-muted-foreground">
                              {action.description}
                            </div>
                          </div>
                        </div>
                        <ArrowRight className="w-5 h-5 text-muted-foreground group-hover:text-primary group-hover:translate-x-1 transition-all" />
                      </Link>
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
                      <CardTitle className="text-xl">{t('home.recentActivity')}</CardTitle>
                      <CardDescription>
                        {t('home.recentActivityDesc')}
                      </CardDescription>
                    </div>
                    <Link href="/events">
                      <Button variant="glass" size="sm">
                        {t('home.viewAll')}
                        <ArrowRight className="w-4 h-4 ml-2" />
                      </Button>
                    </Link>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="space-y-3">
                    {[
                      {
                        icon: Clock,
                        text: t('home.systemStarted'),
                        subtext: t('home.systemStartedDesc'),
                        time: t('home.justNow'),
                        color: "success",
                      },
                      
                    ].map((activity, index) => {
                      const iconColorClass =
                        activity.color === "success"
                          ? "text-success"
                          : activity.color === "primary"
                            ? "text-primary"
                            : "text-muted-foreground";

                      return (
                        <motion.div
                          key={index}
                          initial={{ opacity: 0, x: -20 }}
                          animate={{ opacity: 1, x: 0 }}
                          transition={{ delay: 1 + index * 0.1 }}
                          className="flex items-center gap-4 p-4 rounded-xl bg-gradient-to-r from-white/5 to-transparent border border-white/10 hover:border-primary/30 hover:from-white/10 transition-all duration-300"
                        >
                          <activity.icon
                            className={`w-5 h-5 ${iconColorClass}`}
                          />
                          <div className="flex-1">
                            <div className="text-sm font-medium text-foreground">
                              {activity.text}
                            </div>
                            <div className="text-xs text-muted-foreground">
                              {activity.subtext}
                            </div>
                          </div>
                          <div className="text-xs text-muted-foreground">
                            {activity.time}
                          </div>
                        </motion.div>
                      );
                    })}
                  </div>
                </CardContent>
              </Card>
            </motion.div>
          </motion.div>
        </main>
      </div>
    </div>
  );
}
