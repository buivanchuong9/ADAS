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
    name: "Trợ Lý AI",
    href: "/ai-assistant",
    icon: Brain,
    description: "Hỗ trợ AI",
  },
];

export function Sidebar() {
  const pathname = usePathname();

  return (
    <motion.aside
      initial={{ x: -100, opacity: 0 }}
      animate={{ x: 0, opacity: 1 }}
      transition={{ duration: 0.5, ease: [0.4, 0, 0.2, 1] }}
      className="hidden lg:flex w-64 bg-surface border-r border-subtle flex-col relative overflow-hidden rounded-2xl"
      style={{
        backgroundColor: 'var(--bg-surface)',
        borderColor: 'var(--border-subtle)',
      }}
    >
      {/* Theme-aware overlay */}
      <div 
        className="absolute inset-0 pointer-events-none opacity-50"
        style={{
          background: 'linear-gradient(to bottom, var(--bg-subtle), transparent)',
        }}
      />

      {/* Logo Section */}
      <motion.div
        initial={{ opacity: 0, y: -20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.2 }}
        className="p-6 border-b relative z-10"
        style={{ borderColor: 'var(--border-subtle)' }}
      >
        <Link href="/" className="flex flex-col items-center gap-3 group">
          <motion.div
            className="w-32 h-32 rounded-2xl overflow-hidden shadow-lg ring-2 ring-offset-2 ring-[var(--primary)]"
            style={{
              boxShadow: 'var(--shadow-soft)',
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
                  "flex items-center gap-3 px-4 py-3 rounded-full transition-all duration-300 group relative overflow-hidden",
                  isActive
                    ? "bg-primary-soft"
                    : "hover:bg-subtle"
                )}
                style={{
                  backgroundColor: isActive ? 'var(--primary-soft)' : 'transparent',
                  color: isActive ? 'var(--primary)' : 'var(--text-main)',
                  border: isActive ? '1px solid var(--primary)' : '1px solid transparent',
                }}
              >
                {/* Active indicator */}
                {isActive && (
                  <motion.div
                    className="absolute left-0 top-1/2 -translate-y-1/2 w-1 h-8 rounded-r-full"
                    style={{ backgroundColor: 'var(--primary)' }}
                    layoutId="activeIndicator"
                    transition={{ type: "spring", stiffness: 300, damping: 30 }}
                  />
                )}

                {/* Hover effect */}
                <div 
                  className="absolute inset-0 -translate-x-full group-hover:translate-x-0 transition-transform duration-1000"
                  style={{
                    background: 'linear-gradient(to right, transparent, var(--bg-subtle), transparent)',
                  }}
                />

                <Icon
                  className={cn(
                    "w-5 h-5 transition-all duration-300 relative z-10",
                    isActive && "scale-110"
                  )}
                  style={{ color: isActive ? 'var(--primary)' : 'var(--text-muted)' }}
                />

                <div className="flex-1 relative z-10">
                  <div 
                    className="font-semibold text-sm"
                    style={{ color: isActive ? 'var(--primary)' : 'var(--text-main)' }}
                  >
                    {item.name}
                  </div>
                  <div
                    className="text-xs transition-opacity"
                    style={{ 
                      color: 'var(--text-muted)',
                      opacity: isActive ? 0.9 : 0.6,
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
        style={{ borderColor: 'var(--border-subtle)' }}
      >
        <Link
          href="/settings"
          className="flex items-center gap-3 px-4 py-3 rounded-full transition-all duration-300 group"
          style={{
            color: 'var(--text-main)',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.backgroundColor = 'var(--bg-subtle)';
            e.currentTarget.style.color = 'var(--primary)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.backgroundColor = 'transparent';
            e.currentTarget.style.color = 'var(--text-main)';
          }}
        >
          <Settings className="w-5 h-5 group-hover:rotate-90 transition-transform duration-500" />
          <span className="font-semibold text-sm">Cài Đặt</span>
        </Link>

        {/* Status indicator */}
        <motion.div
          initial={{ scale: 0.9, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
          transition={{ delay: 0.8 }}
          className="mt-4 px-4 py-3 rounded-xl border"
          style={{
            backgroundColor: 'var(--bg-surface)',
            borderColor: 'var(--border-subtle)',
            boxShadow: 'var(--shadow-soft)',
          }}
        >
          <div className="flex items-center gap-2">
            <motion.div
              className="w-2 h-2 rounded-full"
              style={{ backgroundColor: 'var(--success)' }}
              animate={{ scale: [1, 1.2, 1], opacity: [1, 0.7, 1] }}
              transition={{ duration: 2, repeat: Infinity }}
            />
            <span 
              className="text-xs font-semibold"
              style={{ color: 'var(--success)' }}
            >
              Hệ Thống Trực Tuyến
            </span>
          </div>
        </motion.div>
      </motion.div>
    </motion.aside>
  );
}
