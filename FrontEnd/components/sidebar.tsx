"use client";

import { useMemo } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { motion } from "framer-motion";
import { cn } from "@/lib/utils";
import { useLanguage } from "@/contexts/language-context";
import {
  LayoutDashboard,
  Video,
  Eye,
  Database,
  BarChart3,
  Settings,
  Car,
  Brain,
  AlertTriangle,
} from "lucide-react";

export function Sidebar() {
  const pathname = usePathname();
  const { t, language } = useLanguage();

  const navigation = useMemo(
    () => [
      {
        name: t("nav.dashboard"),
        href: "/dashboard",
        icon: LayoutDashboard,
        description: t("nav.dashboardDesc"),
      },
      {
        name: t("nav.adasMonitor"),
        href: "/adas",
        icon: Car,
        description: t("nav.adasMonitorDesc"),
      },
      {
        name: t("nav.driverMonitor"),
        href: "/driver-monitor",
        icon: Eye,
        description: t("nav.driverMonitorDesc"),
      },
      {
        name: t("nav.analytics"),
        href: "/analytics",
        icon: BarChart3,
        description: t("nav.analyticsDesc"),
      },
      {
        name: t("nav.aiAssistant"),
        href: "/ai-assistant",
        icon: Brain,
        description: t("nav.aiAssistantDesc"),
      },
    ],
    [language, t],
  );

  return (
    <motion.aside
      initial={{ x: -100, opacity: 0 }}
      animate={{ x: 0, opacity: 1 }}
      transition={{ duration: 0.5, ease: [0.4, 0, 0.2, 1] }}
      className="hidden lg:flex w-64 bg-surface border-r border-subtle flex-col relative overflow-hidden rounded-2xl"
      style={{
        backgroundColor: "var(--bg-surface)",
        borderColor: "var(--border-subtle)",
      }}
    >
      {/* Theme-aware overlay */}
      <div
        className="absolute inset-0 pointer-events-none opacity-50"
        style={{
          background:
            "linear-gradient(to bottom, var(--bg-subtle), transparent)",
        }}
      />

      {/* Logo Section */}
      <motion.div
        initial={{ opacity: 0, y: -20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.2 }}
        className="p-6 border-b relative z-10"
        style={{ borderColor: "var(--border-subtle)" }}
      >
        <Link
          href="/dashboard"
          className="sidebar-nav-link flex flex-col items-center gap-3 group"
        >
          <motion.div
            className="w-32 h-32 rounded-2xl overflow-hidden shadow-lg ring-2 ring-offset-2 ring-[var(--primary)]"
            style={{
              boxShadow: "var(--shadow-soft)",
            }}
            whileHover={{ scale: 1.05 }}
            transition={{ type: "spring", stiffness: 400 }}
          >
            <img
              src="/adas-logo.jpg"
              alt="ADAS Logo"
              className="w-full h-full object-cover"
            />
          </motion.div>
        </Link>
      </motion.div>

      {/* Navigation */}
      <nav className="flex-1 p-4 space-y-1 relative z-10">
        {navigation.map((item, index) => {
          const isActive = pathname === item.href;
          const Icon = item.icon;

          return (
            <motion.div
              key={item.name}
              initial={{ opacity: 0, x: -20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.3 + index * 0.05 }}
            >
              <Link
                href={item.href}
                className="sidebar-nav-link group flex items-center gap-3 px-4 py-3 rounded-xl relative overflow-hidden transition-all duration-300"
                style={{
                  background: isActive ? "#fff7ed" : "transparent",
                  color: isActive ? "#ea580c" : "#6b7280",
                  borderLeft: isActive
                    ? "3px solid #f97316"
                    : "3px solid transparent",
                  boxShadow: isActive
                    ? "0 0 20px rgba(249,115,22,0.15), inset 0 1px 0 rgba(255,255,255,0.5)"
                    : "none",
                }}
              >
                {/* Hover overlay gradient */}
                {!isActive && (
                  <div
                    className="absolute inset-0 rounded-xl opacity-0 group-hover:opacity-100 transition-opacity duration-300 pointer-events-none"
                    style={{
                      background:
                        "linear-gradient(135deg, rgba(255,247,237,0.6), rgba(255,237,213,0.8))",
                    }}
                  />
                )}

                {/* Active glow bar */}
                {isActive && (
                  <div
                    className="absolute -left-1 top-1/2 -translate-y-1/2 w-1 h-8 rounded-r-full blur-sm"
                    style={{ background: "#f97316" }}
                  />
                )}

                <Icon
                  className="w-5 h-5 shrink-0 relative z-10 transition-transform duration-300 group-hover:scale-110 group-hover:rotate-3"
                  style={{
                    color: isActive ? "#ea580c" : "#9ca3af",
                  }}
                />

                <div className="flex-1 relative z-10">
                  <div
                    style={{
                      fontWeight: 600,
                      lineHeight: 1.3,
                      fontSize: "0.875rem",
                      color: isActive ? "#ea580c" : "#374151",
                    }}
                  >
                    {item.name}
                  </div>
                  <div
                    className="text-xs"
                    style={{
                      color: "#9ca3af",
                      opacity: isActive ? 0.9 : 0.7,
                    }}
                  >
                    {item.description}
                  </div>
                </div>
              </Link>
            </motion.div>
          );
        })}
      </nav>

      {/* Footer */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.7 }}
        className="p-4 border-t relative z-10"
        style={{ borderColor: "var(--border-subtle)" }}
      >
        <Link
          href="/settings"
          className="sidebar-nav-link group flex items-center gap-3 px-4 py-3 rounded-xl relative overflow-hidden transition-all duration-300"
          style={{ color: "#374151" }}
        >
          {/* Hover overlay */}
          <div
            className="absolute inset-0 rounded-xl opacity-0 group-hover:opacity-100 transition-opacity duration-300"
            style={{
              background:
                "linear-gradient(135deg, rgba(249,115,22,0.05), rgba(251,146,60,0.08))",
            }}
          />
          <motion.div
            className="relative z-10"
            whileHover={{ rotate: 90, scale: 1.1 }}
            transition={{ duration: 0.4, ease: [0.34, 1.56, 0.64, 1] }}
          >
            <Settings className="w-5 h-5 transition-colors duration-300 group-hover:text-orange-500" />
          </motion.div>
          <span className="text-sm font-semibold relative z-10 transition-colors duration-300 group-hover:text-orange-600">
            {t("nav.settings")}
          </span>
        </Link>

        {/* Status indicator */}
        <motion.div
          initial={{ scale: 0.9, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
          transition={{ delay: 0.8 }}
          className="mt-4 px-4 py-3 rounded-xl border"
          style={{
            backgroundColor: "var(--bg-surface)",
            borderColor: "var(--border-subtle)",
            boxShadow: "var(--shadow-soft)",
          }}
        >
          <div className="flex items-center gap-2">
            <div className="relative flex items-center justify-center w-3 h-3">
              <div
                className="absolute w-2.5 h-2.5 rounded-full animate-ping"
                style={{ background: "rgba(74, 222, 128, 0.6)" }}
              />
              <motion.div
                className="w-2 h-2 rounded-full relative z-10"
                style={{
                  background:
                    "radial-gradient(circle at 30% 30%, #4ade80, #16a34a)",
                  boxShadow:
                    "0 0 5px rgba(34, 197, 94, 0.4), inset -1px -1px 2px rgba(0,0,0,0.3)",
                }}
                animate={{ scale: [1, 1.15, 1], opacity: [0.8, 0.3, 0.8] }}
                transition={{
                  duration: 2,
                  repeat: Infinity,
                  ease: "easeInOut",
                }}
              >
                <div
                  className="absolute inset-0 rounded-full"
                  style={{
                    boxShadow: "inset 1px 1px 1px rgba(255,255,255,0.4)",
                  }}
                ></div>
              </motion.div>
            </div>
            <span
              className="text-xs font-semibold"
              style={{ color: "var(--success)" }}
            >
              {t("nav.systemOnline")}
            </span>
          </div>
        </motion.div>
      </motion.div>
    </motion.aside>
  );
}
