"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { motion } from "framer-motion";
import { cn } from "@/lib/utils";
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

const navigation = [
  {
    name: "Bảng Điều Khiển",
    href: "/dashboard",
    icon: LayoutDashboard,
    description: "Tổng quan hệ thống",
  },
  {
    name: "Phát Hiện Trực Tiếp",
    href: "/",
    icon: Video,
    description: "ADAS thời gian thực",
  },
  {
    name: "Giám Sát ADAS",
    href: "/adas",
    icon: Car,
    description: "Giám sát nâng cao",
  },
  {
    name: "Giám Sát Tài Xế",
    href: "/driver-monitor",
    icon: Eye,
    description: "Hành vi tài xế",
  },
  {
    name: "Phân Tích",
    href: "/analytics",
    icon: BarChart3,
    description: "Số liệu hiệu suất",
  },
  {
    name: "Thu Thập Dữ Liệu",
    href: "/data-collection",
    icon: Database,
    description: "Quản lý dataset",
  },
  {
    name: "Trợ Lý AI",
    href: "/ai-assistant",
    icon: Brain,
    description: "Hỗ trợ AI",
  },
  {
    name: "Sự Kiện",
    href: "/events",
    icon: AlertTriangle,
    description: "Lịch sử cảnh báo",
  },
];

export function Sidebar() {
  const pathname = usePathname();

  return (
    <motion.aside
      initial={{ x: -100, opacity: 0 }}
      animate={{ x: 0, opacity: 1 }}
      transition={{ duration: 0.5, ease: [0.4, 0, 0.2, 1] }}
      className="hidden lg:flex w-64 glass-panel border-r border-white/10 flex-col relative overflow-hidden rounded-2xl scan-lines"
    >
      {/* Glassmorphism overlay */}
      <div className="absolute inset-0 bg-gradient-to-b from-white/5 via-transparent to-transparent pointer-events-none" />

      {/* Logo Section */}
      <motion.div
        initial={{ opacity: 0, y: -20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.2 }}
        className="p-6 border-b border-sidebar-border/50 relative z-10"
      >
        <Link href="/" className="flex flex-col items-center gap-3 group">
          <motion.div
            className="w-32 h-32 rounded-2xl overflow-hidden shadow-2xl ring-2 ring-neon-cyan/30 glow-pulse-cyan"
            whileHover={{ scale: 1.05 }}
            transition={{ type: "spring", stiffness: 400 }}
          >
            <img
              src="/adas-logo.jpg"
              alt="ADAS Logo"
              className="w-full h-full object-cover"
            />
          </motion.div>
          <div className="text-center">
            <p className="text-xs font-semibold text-neon-cyan tracking-wide">
              ADAS Platform <span className="text-fg-secondary">• v3.0</span>
            </p>
          </div>
        </Link>
      </motion.div>

      {/* Navigation */}
      <nav className="flex-1 p-4 space-y-2 overflow-y-auto relative z-10">
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
                className={cn(
                  "flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-300 group relative overflow-hidden",
                  isActive
                    ? "glass-card glow-cyan text-neon-cyan border-neon-cyan/50"
                    : "glass-card border-transparent hover:border-neon-cyan/30 hover:glow-cyan text-fg-primary hover:text-neon-cyan"
                )}
              >
                {/* Active indicator */}
                {isActive && (
                  <motion.div
                    className="absolute left-0 top-1/2 -translate-y-1/2 w-1 h-10 bg-gradient-to-b from-primary to-accent rounded-r-full"
                    layoutId="activeIndicator"
                    transition={{ type: "spring", stiffness: 300, damping: 30 }}
                  />
                )}

                {/* Hover shine effect */}
                <div className="absolute inset-0 bg-gradient-to-r from-transparent via-white/10 to-transparent -translate-x-full group-hover:translate-x-full transition-transform duration-1000" />

                <Icon
                  className={cn(
                    "w-5 h-5 transition-all duration-300 relative z-10",
                    isActive
                      ? "scale-110 text-primary"
                      : "group-hover:scale-110 group-hover:text-primary"
                  )}
                />

                <div className="flex-1 relative z-10">
                  <div className="font-semibold text-sm">{item.name}</div>
                  <div
                    className={cn(
                      "text-xs transition-opacity",
                      isActive
                        ? "opacity-90"
                        : "opacity-60 group-hover:opacity-80"
                    )}
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
        className="p-4 border-t border-sidebar-border/50 relative z-10"
      >
        <Link
          href="/settings"
          className="flex items-center gap-3 px-4 py-3 rounded-xl text-sidebar-foreground hover:bg-sidebar-accent/50 hover:text-sidebar-accent-foreground transition-all duration-300 group"
        >
          <Settings className="w-5 h-5 group-hover:rotate-90 transition-transform duration-500" />
          <span className="font-semibold text-sm">Cài Đặt</span>
        </Link>

        {/* Status indicator */}
        <motion.div
          initial={{ scale: 0.9, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
          transition={{ delay: 0.8 }}
          className="mt-4 px-4 py-3 rounded-xl glass-card border-neon-green/30 glow-green"
        >
          <div className="flex items-center gap-2">
            <motion.div
              className="w-2 h-2 rounded-full bg-neon-green"
              animate={{ scale: [1, 1.2, 1], opacity: [1, 0.7, 1] }}
              transition={{ duration: 2, repeat: Infinity }}
              style={{ boxShadow: '0 0 10px var(--neon-green)' }}
            />
            <span className="text-xs font-semibold text-neon-green">
              Hệ Thống Trực Tuyến
            </span>
          </div>
        </motion.div>
      </motion.div>
    </motion.aside>
  );
}
