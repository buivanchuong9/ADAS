"use client"

import Link from "next/link"
import { usePathname } from "next/navigation"
import { cn } from "@/lib/utils"
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
} from "lucide-react"

const navigation = [
  {
    name: "Bảng Điều Khiển",
    href: "/dashboard",
    icon: LayoutDashboard,
    description: "Tổng quan hệ thống"
  },
  {
    name: "Phát Hiện Trực Tiếp",
    href: "/",
    icon: Video,
    description: "ADAS thời gian thực"
  },
  {
    name: "Giám Sát ADAS",
    href: "/adas",
    icon: Car,
    description: "Giám sát nâng cao"
  },
  {
    name: "Giám Sát Tài Xế",
    href: "/driver-monitor",
    icon: Eye,
    description: "Hành vi tài xế"
  },
  {
    name: "Phân Tích",
    href: "/analytics",
    icon: BarChart3,
    description: "Số liệu hiệu suất"
  },
  {
    name: "Thu Thập Dữ Liệu",
    href: "/data-collection",
    icon: Database,
    description: "Quản lý dataset"
  },
  {
    name: "Trợ Lý AI",
    href: "/ai-assistant",
    icon: Brain,
    description: "Hỗ trợ AI"
  },
  {
    name: "Sự Kiện",
    href: "/events",
    icon: AlertTriangle,
    description: "Lịch sử cảnh báo"
  },
]

export function Sidebar() {
  const pathname = usePathname()

  return (
    <aside className="w-64 bg-sidebar border-r border-sidebar-border flex flex-col">
      {/* Logo Section */}
      <div className="p-6 border-b border-sidebar-border">
        <Link href="/" className="flex items-center gap-3 group">
          <div className="w-10 h-10 rounded-lg bg-gradient-to-br from-primary to-accent flex items-center justify-center shadow-lg shadow-primary/20 transition-transform group-hover:scale-105">
            <Car className="w-6 h-6 text-white" />
          </div>
          <div>
            <h1 className="text-lg font-bold text-sidebar-foreground">Nền Tảng ADAS</h1>
            <p className="text-xs text-muted-foreground">v3.0 Chuyên Nghiệp</p>
          </div>
        </Link>
      </div>

      {/* Navigation */}
      <nav className="flex-1 p-4 space-y-1 overflow-y-auto">
        {navigation.map((item) => {
          const isActive = pathname === item.href
          const Icon = item.icon

          return (
            <Link
              key={item.name}
              href={item.href}
              className={cn(
                "flex items-center gap-3 px-4 py-3 rounded-lg transition-all duration-200 group relative",
                isActive
                  ? "bg-sidebar-primary text-sidebar-primary-foreground shadow-lg shadow-primary/20"
                  : "text-sidebar-foreground hover:bg-sidebar-accent hover:text-sidebar-accent-foreground"
              )}
            >
              {/* Active indicator */}
              {isActive && (
                <div className="absolute left-0 top-1/2 -translate-y-1/2 w-1 h-8 bg-accent rounded-r-full" />
              )}

              <Icon className={cn(
                "w-5 h-5 transition-transform",
                isActive ? "scale-110" : "group-hover:scale-110"
              )} />

              <div className="flex-1">
                <div className="font-medium text-sm">{item.name}</div>
                <div className={cn(
                  "text-xs transition-opacity",
                  isActive ? "opacity-90" : "opacity-60 group-hover:opacity-80"
                )}>
                  {item.description}
                </div>
              </div>
            </Link>
          )
        })}
      </nav>

      {/* Footer */}
      <div className="p-4 border-t border-sidebar-border">
        <Link
          href="/settings"
          className="flex items-center gap-3 px-4 py-3 rounded-lg text-sidebar-foreground hover:bg-sidebar-accent hover:text-sidebar-accent-foreground transition-all duration-200"
        >
          <Settings className="w-5 h-5" />
          <span className="font-medium text-sm">Cài Đặt</span>
        </Link>

        {/* Status indicator */}
        <div className="mt-4 px-4 py-3 rounded-lg bg-success/10 border border-success/20">
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 rounded-full bg-success animate-pulse" />
            <span className="text-xs font-medium text-success">Hệ Thống Trực Tuyến</span>
          </div>
        </div>
      </div>
    </aside>
  )
}
