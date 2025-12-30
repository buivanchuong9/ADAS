"use client";
import { useEffect } from "react";
import { useState } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { motion, AnimatePresence } from "framer-motion";
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
  Menu,
  X,
} from "lucide-react";
import { Button } from "@/components/ui/button";

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

export function MobileNav() {
  const [isOpen, setIsOpen] = useState(false);
  useEffect(() => {
  if (isOpen) {
    document.body.style.overflow = "hidden";
    document.body.style.touchAction = "none";
  } else {
    document.body.style.overflow = "";
    document.body.style.touchAction = "";
  }

  return () => {
    document.body.style.overflow = "";
    document.body.style.touchAction = "";
  };
}, [isOpen]);

  const pathname = usePathname();

  return (
    <>
      {/* Mobile Menu Button - Fixed at top */}
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        className="lg:hidden fixed top-4 left-4 z-50"
      >
        <Button
  variant="ghost"
  size="icon"
  onClick={() => setIsOpen(!isOpen)}
  className="
    bg-transparent
    border-none
    shadow-none
    hover:bg-white/5
    active:bg-white/10
    text-neon-cyan
  "
>
  {isOpen ? <X className="h-5 w-5" /> : <Menu className="h-5 w-5" />}
  
</Button>

      </motion.div>

      {/* Overlay */}
      <AnimatePresence>
        {isOpen && (
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            onClick={() => setIsOpen(false)}
            className="lg:hidden fixed inset-0 bg-black/50 backdrop-blur-sm z-40"
          />
        )}
      </AnimatePresence>

      {/* Mobile Drawer */}
      <AnimatePresence>
        {isOpen && (
          <motion.aside
  initial={{ x: "-100%" }}
  animate={{ x: 0 }}
  exit={{ x: "-100%" }}
  transition={{ type: "spring", stiffness: 300, damping: 30 }}
  className="
    lg:hidden
    fixed left-0 top-0
    h-dvh
    w-72
    glass-panel
    border-r border-white/10
    shadow-2xl
    z-50
    flex flex-col
    scan-lines
  "
>

            {/* Logo Section */}
            <div className="p-6 border-b border-white/10 shrink-0">
              <Link
                href="/"
                className="flex flex-col items-center gap-3"
                onClick={() => setIsOpen(false)}
              >
                <div className="w-24 h-24 rounded-xl overflow-hidden shadow-xl ring-2 ring-neon-cyan/30 glow-pulse-cyan">
                  <img
                    src="/adas-logo.jpg"
                    alt="ADAS Logo"
                    className="w-full h-full object-cover"
                  />
                </div>
                <div className="text-center">
                  <p className="text-xs font-semibold text-neon-cyan">
                    ADAS Platform <span className="text-fg-secondary">• v3.0</span>
                  </p>
                </div>
              </Link>
            </div>

            {/* Navigation (Scrollable) */}
<div
  className="
    flex-1
    overflow-y-auto
    overscroll-contain
    touch-pan-y
    scroll-smooth
  "
>
  <nav className="p-4 space-y-2">
    {navigation.map((item) => {
      const isActive = pathname === item.href;
      const Icon = item.icon;

      return (
        <Link
          key={item.name}
          href={item.href}
          onClick={() => setIsOpen(false)}
          className={cn(
            "flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200",
            isActive
              ? "glass-card glow-cyan text-neon-cyan border-neon-cyan/50"
              : "glass-card border-transparent hover:border-neon-cyan/30 hover:text-neon-cyan"
          )}
        >
          <Icon className={cn("w-5 h-5", isActive && "text-purple-600")} />
          <div className="flex-1">
            <div className="font-semibold text-sm">{item.name}</div>
            <div className="text-xs opacity-60">{item.description}</div>
          </div>
        </Link>
      );
    })}
  </nav>
</div>


            {/* Footer */}
            <div className="p-4 border-t border-gray-200 mt-auto shrink-0">
              <Link
                href="/settings"
                onClick={() => setIsOpen(false)}
                className="flex items-center gap-3 px-4 py-3 rounded-xl text-gray-700 hover:bg-gray-50 transition-all"
              >
                <Settings className="w-5 h-5" />
                <span className="font-semibold text-sm">Cài Đặt</span>
              </Link>

              <div className="mt-4 px-4 py-3 rounded-xl bg-green-50 border border-green-200">
                <div className="flex items-center gap-2">
                  <div className="w-2 h-2 rounded-full bg-green-500" />
                  <span className="text-sm font-medium text-green-700">
                    System Online
                  </span>
                </div>
              </div>
            </div>
          </motion.aside>
        )}
      </AnimatePresence>
    </>
  );
}
