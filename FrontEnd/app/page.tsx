"use client";

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
                  className="text-3xl sm:text-4xl md:text-5xl lg:text-6xl font-bold mb-4 text-neon-cyan"
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.4 }}
                >
                  {t('home.title')}
                </motion.h1>

                <motion.p
                  className="text-base sm:text-lg lg:text-xl text-fg-secondary max-w-3xl mb-8 leading-relaxed"
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.5 }}
                >
                  {t('home.subtitle')}
                </motion.p>

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
                  <Link href="/dashboard">
                    <Button
                      size="lg"
                      className="shadow-lg bg-white/20 backdrop-blur-md text-white hover:bg-white/30 w-full sm:w-auto inline-flex items-center justify-center gap-2 whitespace-nowrap"
                    >
                      <span className="inline-flex items-center gap-2 whitespace-nowrap">
                        {t('home.viewDashboard')}
                        <ArrowRight className="w-4 h-4 sm:w-5 sm:h-5 shrink-0" />
                      </span>
                    </Button>
                  </Link>
                </motion.div>
              </div>
            </motion.div>

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
              <Card glass>
                <CardHeader>
                  <CardTitle className="text-xl">{t('home.quickActions')}</CardTitle>
                  <CardDescription>
                    {t('home.quickActionsDesc')}
                  </CardDescription>
                </CardHeader>
                <CardContent className="space-y-3">
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

              <Card glass>
                <CardHeader>
                  <CardTitle className="text-xl">{t('home.systemFeatures')}</CardTitle>
                  <CardDescription>
                    {t('home.systemFeaturesDesc')}
                  </CardDescription>
                </CardHeader>
                <CardContent className="space-y-4">
                  {[
                    {
                      title: t('home.websocketStreaming'),
                      desc: t('home.websocketStreamingDesc'),
                    },
                    {
                      title: t('home.smartAlerts'),
                      desc: t('home.smartAlertsDesc'),
                    },
                  ].map((feature, index) => (
                    <motion.div
                      key={feature.title}
                      initial={{ opacity: 0, x: -20 }}
                      animate={{ opacity: 1, x: 0 }}
                      transition={{ delay: 0.7 + index * 0.1 }}
                      className="flex items-start gap-3"
                    >
                      <CheckCircle2 className="w-5 h-5 text-success mt-0.5 flex-shrink-0" />
                      <div>
                        <div className="font-medium text-foreground">
                          {feature.title}
                        </div>
                        <div className="text-sm text-muted-foreground">
                          {feature.desc}
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
                      },
                      {
                        icon: CheckCircle2,
                        text: t('home.dbConnected'),
                        subtext: t('home.dbConnectedDesc'),
                        time: t('home.minutesAgo', { count: 1 }),
                        color: "success",
                      },
                      {
                        icon: Activity,
                        text: t('home.apiOnline'),
                        subtext: t('home.apiOnlineDesc'),
                        time: t('home.minutesAgo', { count: 2 }),
                        color: "primary",
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
