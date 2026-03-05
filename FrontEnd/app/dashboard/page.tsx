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
} from "lucide-react";
import Link from "next/link";
import { HighchartsChart } from "@/components/charts/highcharts-chart";
import { getApiUrl } from "@/lib/api-config";
import { API_ENDPOINTS } from "@/lib/api-endpoints";
import { RadarCanvas } from "@/components/radar-canvas";

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
  const [aiUsageData, setAiUsageData] = useState<{ name: string; y: number }[]>(
    [],
  );
  const aiUsageOptions = useMemo(
    () => ({
      chart: { type: "pie", backgroundColor: "transparent", height: 280 },
      title: {
        text: t("settings.aiUsage"),
        style: { color: "#ff7a1a", fontSize: "16px", fontWeight: "600" },
      },
      tooltip: {
        pointFormat: `<b>{point.percentage:.1f}%</b><br/>${t("home.tooltipQuantity")}: {point.y}`,
      },
      plotOptions: {
        pie: {
          innerSize: "60%",
          dataLabels: {
            enabled: true,
            format: "<b>{point.name}</b><br>{point.percentage:.1f}%",
            style: {
              color: "#111827",
              textOutline: "none",
            },
          },
        },
      },
      legend: {
        itemStyle: { color: "#374151", fontWeight: "500" },
        itemHoverStyle: { color: "#111827" },
      },
      series: [
        {
          name: "Queries",
          data:
            aiUsageData.length > 0
              ? aiUsageData
              : [
                  { name: "Driving Support", y: 0 },
                  { name: "Data Analysis", y: 0 },
                  { name: "Alerts", y: 0 },
                  { name: "Other", y: 0 },
                ],
        },
      ],
      credits: { enabled: false },
    }),
    [t, aiUsageData],
  );

  const [confidenceStatsCategories, setConfidenceStatsCategories] = useState<
    string[]
  >(["50-60%", "60-70%", "70-80%", "80-90%", "90-100%"]);
  const [confidenceStatsData, setConfidenceStatsData] = useState<number[]>([
    0, 0, 0, 0, 0,
  ]);

  const confidenceDistOptions = useMemo(
    () => ({
      chart: { type: "column", backgroundColor: "transparent", height: 280 },
      title: {
        text: t("settings.confidenceDistribution"),
        style: { color: "#ff7a1a", fontSize: "16px", fontWeight: "600" },
      },
      xAxis: {
        categories: confidenceStatsCategories,
        labels: { style: { color: "#374151" } },
      },
      yAxis: {
        title: {
          text: `${t("home.tooltipQuantity")} / ${t("settings.accuracy")}`,
          style: { color: "#111827" },
        },
        labels: { style: { color: "#374151" } },
      },
      legend: {
        itemStyle: { color: "#374151", fontWeight: "500" },
        itemHoverStyle: { color: "#111827" },
      },
      series: [
        {
          name: t("settings.detections"),
          data: confidenceStatsData,
        },
      ],
      credits: { enabled: false },
    }),
    [t, confidenceStatsCategories, confidenceStatsData],
  );

  const [stats, setStats] = useState({
    systemStatus: t("common.online"),
    activeCameras: 0,
    totalDetections: 0,
    alertsToday: 0,
  });

  // Additional Chart Data states
  const [detectionTrendCategories, setDetectionTrendCategories] = useState<
    string[]
  >([]);
  const [detectionTrendSeries, setDetectionTrendSeries] = useState<any[]>([]);

  const [accuracyCategories, setAccuracyCategories] = useState<string[]>([
    t("settings.monday"),
    t("settings.tuesday"),
    t("settings.wednesday"),
    t("settings.thursday"),
    t("settings.friday"),
    t("settings.saturday"),
    t("settings.sunday"),
  ]);
  const [accuracySeries, setAccuracySeries] = useState<any[]>([
    {
      name: t("settings.accuracy"),
      data: [0, 0, 0, 0, 0, 0, 0],
      color: "#00FFA3",
      marker: { symbol: "circle" },
    },
  ]);

  const [detectionChartData, setDetectionChartData] = useState<any[]>([
    { name: t("settings.vehicles"), y: 0 },
    { name: t("settings.people"), y: 0 },
    { name: t("home.cycles"), y: 0 },
    { name: t("settings.other"), y: 0 },
  ]);

  const [performanceCategories, setPerformanceCategories] = useState<string[]>(
    [],
  );
  const [performanceChartData, setPerformanceChartData] = useState<any[]>([
    {
      name: t("settings.performanceLabel"),
      data: [0, 0, 0, 0, 0, 0, 0],
      color: "#00FFA3",
      fillColor: {
        linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
        stops: [
          [0, "rgba(0, 255, 163, 0.3)"],
          [1, "rgba(0, 255, 163, 0.05)"],
        ],
      },
    },
  ]);

  useEffect(() => {
    const fetchStats = async () => {
      try {
        // Prefer dashboard cards endpoint for top KPIs
        let systemStatus = t("common.offline");
        let activeCameras = 0;
        let totalDetections = 0;
        let alertsToday = 0;

        const cardsRes = await fetch(
          getApiUrl(API_ENDPOINTS.ADMIN_DASHBOARD_CARDS),
          { signal: AbortSignal.timeout(5000) },
        ).catch(() => null);

        if (cardsRes && cardsRes.ok) {
          const cards = await cardsRes.json().catch(() => ({}));
          systemStatus = String(cards.system_status ?? t("common.offline"));
          activeCameras = Number(cards.active_cameras ?? 0);
          totalDetections = Number(cards.total_detections ?? 0);
          alertsToday = Number(cards.today_alerts ?? 0);
        } else {
          // Fallback: health + statistics endpoints
          const statusRes = await fetch(getApiUrl(API_ENDPOINTS.HEALTH), {
            signal: AbortSignal.timeout(5000),
          }).catch(() => null);

          systemStatus = t("common.offline");
          if (statusRes && statusRes.ok) {
            const statusData = await statusRes.json().catch(() => ({}));
            systemStatus =
              statusData.status === "success"
                ? t("common.online")
                : t("common.offline");
          }

          const alertsRes = await fetch(
            getApiUrl(API_ENDPOINTS.ADMIN_STATISTICS),
            {
              signal: AbortSignal.timeout(5000),
            },
          ).catch(() => null);

          if (alertsRes && alertsRes.ok) {
            const alertsData = await alertsRes.json().catch(() => ({}));
            alertsToday =
              alertsData.data?.total_alerts || alertsData.total_alerts || 0;
          }
        }

        // Fetch Detections Stats (for charts + optional totalDetections fallback)
        const detectionsRes = await fetch(
          getApiUrl(API_ENDPOINTS.DETECTIONS_STATS),
          {
            signal: AbortSignal.timeout(5000),
          },
        ).catch(() => null);

        if (detectionsRes && detectionsRes.ok) {
          const dStats = await detectionsRes.json().catch(() => ({}));
          if (dStats.success && dStats.classes) {
            const categories = dStats.classes.map((c: any) => c.class_name);
            const data = dStats.classes.map((c: any) => c.avg_confidence);
            setConfidenceStatsCategories(categories);
            setConfidenceStatsData(data);
          }
          if (!totalDetections && dStats.total_detections) {
            totalDetections = dStats.total_detections;
          }
        }

        setStats({
          systemStatus,
          activeCameras:
            activeCameras || systemStatus === t("common.online") ? 1 : 0,
          totalDetections,
          alertsToday,
        });
      } catch (err) {
        // Silently handle errors - backend may not be running
        setStats((prev) => ({ ...prev, systemStatus: t("common.offline") }));
      }
    };

    const fetchChartData = async () => {
      try {
        // AI Chat History
        const chatRes = await fetch(
          getApiUrl(API_ENDPOINTS.AI_CHAT_HISTORY) + "?limit=50",
          { signal: AbortSignal.timeout(5000) },
        ).catch(() => null);
        if (chatRes && chatRes.ok) {
          const chatData = await chatRes.json().catch(() => null);
          if (chatData?.messages) {
            // Assuming logic for messages distribution fallback
            const distribution = {
              "Driving Support": 0,
              "Data Analysis": 0,
              Alerts: 0,
              Other: 0,
            };
            // Insert logic to parse messages structure if details are available
            if (chatData.messages.length === 0) {
              // Empty distribution
            }
            const mappedChatData = Object.entries(distribution).map(
              ([k, v]) => ({ name: k, y: v as number }),
            );
            setAiUsageData(mappedChatData);
          }
        }

        // Detection Trend
        const trendRes = await fetch(
          getApiUrl(API_ENDPOINTS.DASHBOARD_CHART_DETECTION_TREND),
          { signal: AbortSignal.timeout(5000) },
        ).catch(() => null);
        if (trendRes && trendRes.ok) {
          const trendData = await trendRes.json().catch(() => null);
          if (trendData?.labels && trendData?.datasets) {
            setDetectionTrendCategories(trendData.labels);

            const mappedSeries = trendData.datasets.map((ds: any) => {
              let fillColor: any;
              let color: string;
              // check if API label refers to vehicles (supports both VI and EN labels)
              const isVehicle = /xe|vehicle|car/i.test(ds.label);
              if (isVehicle) {
                color = "#00E5FF";
                fillColor = {
                  linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                  stops: [
                    [0, "rgba(0, 229, 255, 0.3)"],
                    [1, "rgba(0, 229, 255, 0.05)"],
                  ],
                };
              } else {
                color = "#00FFA3";
                fillColor = {
                  linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                  stops: [
                    [0, "rgba(0, 255, 163, 0.3)"],
                    [1, "rgba(0, 255, 163, 0.05)"],
                  ],
                };
              }
              return {
                name: ds.label,
                data: ds.data,
                color: color,
                fillColor: fillColor,
              };
            });
            setDetectionTrendSeries(mappedSeries);
          }
        }

        // Detection Accuracy
        const accRes = await fetch(
          getApiUrl(API_ENDPOINTS.DASHBOARD_CHART_DETECTION_ACCURACY),
          { signal: AbortSignal.timeout(5000) },
        ).catch(() => null);
        if (accRes && accRes.ok) {
          const accData = await accRes.json().catch(() => null);
          if (accData?.labels && accData?.datasets) {
            setAccuracyCategories(accData.labels);
            setAccuracySeries(
              accData.datasets.map((ds: any) => ({
                name: ds.label || t("settings.accuracy"),
                data: ds.data,
                color: "#00FFA3",
                marker: { symbol: "circle" },
              })),
            );
          }
        }

        // Detection Distribution
        const distRes = await fetch(
          getApiUrl(API_ENDPOINTS.DASHBOARD_CHART_DETECTION_DISTRIBUTION),
          { signal: AbortSignal.timeout(5000) },
        ).catch(() => null);
        if (distRes && distRes.ok) {
          const distData = await distRes.json().catch(() => null);
          if (distData?.labels && distData?.datasets?.[0]?.data) {
            const colors = distData.datasets[0].backgroundColor || [];
            const mappedDist = distData.labels.map(
              (lbl: string, idx: number) => ({
                name: lbl,
                y: distData.datasets[0].data[idx],
                color: colors[idx] || undefined,
              }),
            );
            setDetectionChartData(mappedDist);
          }
        }

        // System Performance
        const perfRes = await fetch(
          getApiUrl(API_ENDPOINTS.DASHBOARD_CHART_SYSTEM_PERFORMANCE),
          { signal: AbortSignal.timeout(5000) },
        ).catch(() => null);
        if (perfRes && perfRes.ok) {
          const perfData = await perfRes.json().catch(() => null);
          if (perfData?.labels && perfData?.datasets?.[0]?.data) {
            setPerformanceCategories(perfData.labels);
            const mappedPerf = perfData.datasets.map((ds: any, idx: number) => {
              const isFirst = idx === 0;
              const color = isFirst ? "#00FFA3" : "#00E5FF";
              const rgba = isFirst ? "rgba(0, 255, 163" : "rgba(0, 229, 255";
              return {
                name: ds.label || t("settings.performanceLabel"),
                data: ds.data,
                color: color,
                fillColor: {
                  linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                  stops: [
                    [0, `${rgba}, 0.3)`],
                    [1, `${rgba}, 0.05)`],
                  ],
                },
              };
            });
            setPerformanceChartData(mappedPerf);
          }
        }
      } catch (err) {
        // Handle silently
      }
    };

    fetchStats();
    fetchChartData();
    const interval = setInterval(() => {
      fetchStats();
      fetchChartData();
    }, 30000); // Check every 30 seconds instead of 5
    return () => clearInterval(interval);
  }, [t]);

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
            {/* Hero Section - Light ADAS Banner */}
            <motion.div
              variants={itemVariants}
              className="relative overflow-hidden rounded-3xl border border-orange-100 bg-linear-to-r from-white via-orange-50/60 to-orange-100/70 p-6 sm:p-8 lg:p-10 shadow-[0_24px_60px_rgba(248,148,40,0.25)]"
            >
              <div className="absolute inset-0 pointer-events-none">
                <div className="absolute -right-20 -top-32 w-80 h-80 rounded-full bg-orange-200/50 blur-3xl" />
                <div className="absolute -left-10 -bottom-24 w-72 h-72 rounded-full bg-purple-200/40 blur-3xl" />
              </div>

              <div className="relative z-10 flex flex-col lg:flex-row items-stretch lg:items-center gap-8 lg:gap-12 lg:justify-between">
                {/* Left content */}
                <div className="flex-1 max-w-xl">
                  <motion.div
                    className="inline-flex items-center gap-2 rounded-full border border-orange-200 bg-white/80 px-3 py-1 text-xs sm:text-sm text-orange-500 shadow-sm mb-4"
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 0.2 }}
                  >
                    <Shield className="w-4 h-4" />
                    <span className="font-semibold tracking-wide">
                      V3.0 PROFESSIONAL
                    </span>
                  </motion.div>

                  <motion.h1
                    className="text-3xl sm:text-4xl md:text-5xl font-extrabold leading-tight text-slate-900 mb-4"
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: 0.3 }}
                  >
                    <span className="block">
                      Advanced <span className="text-orange-500">Driver</span>
                    </span>
                    <span className="block">
                      <span className="text-purple-500">Assistance</span>{" "}
                      <span>System</span>
                    </span>
                  </motion.h1>

                  <motion.p
                    className="text-sm sm:text-base text-slate-600 mb-6 max-w-md"
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: 0.4 }}
                  >
                    {t("home.heroTagline")}
                  </motion.p>

                  <motion.div
                    className="flex flex-wrap items-center gap-3 sm:gap-4 mb-6"
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: 0.5 }}
                  >
                    <span className="inline-flex items-center rounded-full bg-orange-50 px-3 py-1 text-xs sm:text-sm text-orange-600 border border-orange-100">
                      {t("home.heroBadgeFaceRecog")}
                    </span>
                    <span className="inline-flex items-center rounded-full bg-purple-50 px-3 py-1 text-xs sm:text-sm text-purple-600 border border-purple-100">
                      {t("home.heroBadgeEdgeAI")}
                    </span>
                    <span className="inline-flex items-center rounded-full bg-sky-50 px-3 py-1 text-xs sm:text-sm text-sky-600 border border-sky-100">
                      {t("home.heroBadgeVideoAnalysis")}
                    </span>
                  </motion.div>

                  <motion.div
                    className="flex flex-wrap items-center gap-4"
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: 0.6 }}
                  >
                    <Link href="/adas">
                      <button className="inline-flex items-center justify-center rounded-full bg-orange-500 px-6 py-3 text-sm sm:text-base font-semibold text-white shadow-lg shadow-orange-300/60 hover:bg-orange-600 transition-colors">
                        <Zap className="w-5 h-5 mr-2" />
                        {t("home.startDetection")}
                      </button>
                    </Link>
                    <div className="flex items-center gap-2 text-xs sm:text-sm text-slate-500">
                      <CheckCircle2 className="w-4 h-4 text-emerald-500" />
                      <span>{t("home.aiAccuracyStat")}</span>
                    </div>
                  </motion.div>
                </div>

                {/* Right radar card */}
                <motion.div
                  className="w-full max-w-xs mx-auto lg:mx-0 lg:ml-auto lg:w-80 self-center"
                  initial={{ opacity: 0, x: 30 }}
                  animate={{ opacity: 1, x: 0 }}
                  transition={{ delay: 0.4 }}
                >
                  <div className="relative rounded-3xl bg-white/90 border border-orange-100 shadow-[0_18px_45px_rgba(248,148,40,0.25)] p-4 flex flex-col items-center">
                    <div className="absolute -top-3 right-4">
                      <div className="inline-flex items-center rounded-full bg-white px-3 py-1 text-xs font-medium text-orange-500 border border-orange-100 shadow-sm">
                        <span className="w-1.5 h-1.5 rounded-full bg-orange-500 mr-2" />
                        {t("home.objectCount")}
                      </div>
                    </div>

                    <div className="mt-4 mb-4">
                      <RadarCanvas />
                    </div>

                    <div className="flex w-full justify-between items-center px-1">
                      <span className="text-[11px] uppercase tracking-wide text-slate-400">
                        {t("home.radarTracking")}
                      </span>
                      <div className="inline-flex items-center rounded-full bg-slate-900 text-sky-300 px-3 py-1 text-[11px] shadow">
                        <span className="w-1.5 h-1.5 rounded-full bg-sky-400 mr-2" />
                        {t("home.radarAiAcc")}
                      </div>
                    </div>
                  </div>
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
                  title: t("home.systemStatus"),
                  value: stats.systemStatus,
                  icon: Activity,
                  color: "success",
                  description: t("home.allSystemsOnline"),
                },
                {
                  title: t("home.activeCameras"),
                  value: stats.activeCameras.toString(),
                  icon: Cctv,
                  color: "primary",
                  description: t("home.realTimeMonitoring"),
                },
                {
                  title: t("home.totalDetections"),
                  value: stats.totalDetections.toLocaleString(),
                  icon: BookOpen,
                  color: "info",
                  description: t("home.percentIncrease", { percent: 12 }),
                  trend: true,
                },
                {
                  title: t("home.alertsToday"),
                  value: stats.alertsToday.toString(),
                  icon: AlertTriangle,
                  color: "warning",
                  description: t("home.safetyAlertsIssued"),
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
                        {stat.title === t("home.systemStatus") && (
                          <motion.div
                            className="w-2 h-2 rounded-full bg-success"
                            animate={{
                              scale: [1, 1.2, 1],
                              opacity: [1, 0.7, 1],
                            }}
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
                      categories:
                        detectionTrendCategories.length > 0
                          ? detectionTrendCategories
                          : [
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
                    series:
                      detectionTrendSeries.length > 0
                        ? detectionTrendSeries
                        : [
                            {
                              name: t("home.vehiclesFallback"),
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
                              name: t("home.pedestriansFallback"),
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
                      categories: accuracyCategories,
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
                    series: accuracySeries,
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
              <GlassCard className="p-6">
                <HighchartsReact
                  highcharts={Highcharts}
                  options={{
                    chart: {
                      type: "pie",
                      backgroundColor: "transparent",
                      height: 300,
                    },
                    title: {
                      text: t("home.detectionDistribution"),
                      style: {
                        color: "#ff7a1a",
                        fontFamily: "var(--font-inter)",
                        fontSize: "16px",
                        fontWeight: "600",
                      },
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
                      pointFormat:
                        "<b>{point.name}</b>: {point.percentage:.1f} %<br/>Số lượng: {point.y}",
                    },
                    plotOptions: {
                      pie: {
                        allowPointSelect: true,
                        cursor: "pointer",
                        dataLabels: {
                          enabled: true,
                          format:
                            "<b>{point.name}</b><br>{point.percentage:.1f}%",
                          style: {
                            color: "#111827",
                            textOutline: "none",
                            fontFamily: "var(--font-inter)",
                          },
                        },
                        showInLegend: true,
                        borderWidth: 2,
                        borderColor: "rgba(255, 255, 255, 0.5)",
                      },
                    },
                    series: [
                      {
                        type: "pie",
                        name: "Share",
                        data: detectionChartData,
                      },
                    ],
                    legend: {
                      itemStyle: {
                        color: "#111827",
                        fontFamily: "var(--font-inter)",
                        fontSize: "12px",
                        fontWeight: "500",
                      },
                      itemHoverStyle: { color: "#00E5FF" },
                    },
                    credits: { enabled: false },
                  }}
                />
              </GlassCard>

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
                      text: t("home.systemPerformance"),
                      style: {
                        color: "#ff7a1a",
                        fontFamily: "var(--font-inter)",
                        fontSize: "16px",
                        fontWeight: "600",
                      },
                    },
                    xAxis: {
                      categories:
                        performanceCategories.length > 0
                          ? performanceCategories
                          : [
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
                        marker: { radius: 4, lineWidth: 2, symbol: "diamond" },
                      },
                    },
                    series: performanceChartData,
                    legend: {
                      itemStyle: {
                        color: "#111827",
                        fontFamily: "var(--font-inter)",
                        fontSize: "12px",
                        fontWeight: "500",
                      },
                      itemHoverStyle: { color: "#000000" },
                    },
                    credits: { enabled: false },
                  }}
                />
              </GlassCard>
            </motion.div>

            {/* Quick Actions & Features */}
            <motion.div
              variants={itemVariants}
              className="grid grid-cols-1 xl:grid-cols-2 gap-4 sm:gap-6"
            >
              <Card glass className="xl:col-span-2 w-full">
                <CardHeader>
                  <CardTitle className="text-xl">
                    {t("home.quickActions")}
                  </CardTitle>
                  <CardDescription>
                    {t("home.quickActionsDesc")}
                  </CardDescription>
                </CardHeader>
                <CardContent className="pt-3 grid grid-cols-1 md:grid-cols-3 gap-6">
                  {[
                    {
                      href: "/adas",
                      icon: Zap,
                      title: t("home.monitorDriving"),
                      description: t("home.monitorDrivingDesc"),
                      gradient: "from-primary to-primary/80",
                    },
                    {
                      href: "/driver-monitor",
                      icon: Eye,
                      title: t("home.monitorDriver"),
                      description: t("home.monitorDriverDesc"),
                      gradient: "from-accent to-accent/80",
                    },
                    {
                      href: "/analytics",
                      icon: TrendingUp,
                      title: t("home.viewAnalytics"),
                      description: t("home.viewAnalyticsDesc"),
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
                        className="quick-action-link flex items-center justify-between p-4 rounded-xl bg-linear-to-r from-black/5 to-transparent border border-border/50 hover:border-primary/50 hover:from-primary/5 hover:to-transparent transition-all duration-300 group"
                      >
                        <div className="flex items-center gap-4">
                          <div
                            className={`w-12 h-12 rounded-xl bg-linear-to-br ${action.gradient} flex items-center justify-center shadow-lg group-hover:scale-110 transition-transform`}
                          >
                            <action.icon className="w-6 h-6 text-black" />
                          </div>
                          <div>
                            <div className="font-semibold text-primary transition-colors">
                              {action.title}
                            </div>
                            <div className="text-sm text-primary/80">
                              {action.description}
                            </div>
                          </div>
                        </div>
                        <ArrowRight className="w-5 h-5 text-primary group-hover:translate-x-1 transition-all" />
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
                      <CardTitle className="text-xl">
                        {t("home.recentActivity")}
                      </CardTitle>
                      <CardDescription>
                        {t("home.recentActivityDesc")}
                      </CardDescription>
                    </div>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="space-y-3">
                    {[
                      {
                        icon: Clock,
                        text: t("home.systemStarted"),
                        subtext: t("home.systemStartedDesc"),
                        time: t("home.justNow"),
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
                          className="flex items-center gap-4 p-4 rounded-xl bg-linear-to-r from-black/5 to-transparent border border-border/50 hover:border-primary/30 hover:from-primary/5 transition-all duration-300"
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
