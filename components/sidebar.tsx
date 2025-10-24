"use client"

import Link from "next/link"
import { usePathname } from "next/navigation"
import { Activity, AlertTriangle, BarChart3, MessageCircle, Monitor, Video } from "lucide-react"

const navItems = [
  { href: "/", label: "Phát Hiện Trực Tiếp", icon: Video },
  { href: "/driver-monitor", label: "Giám Sát Tài Xế", icon: Monitor },
  { href: "/events", label: "Nhật Ký Sự Kiện", icon: AlertTriangle },
  { href: "/analytics", label: "Phân Tích Chuyến Đi", icon: BarChart3 },
  { href: "/ai-assistant", label: "Trợ Lý AI", icon: MessageCircle },
]

export function Sidebar() {
  const pathname = usePathname()

  return (
    <aside className="w-64 bg-sidebar border-r border-sidebar-border h-screen flex flex-col">
      <div className="p-6 border-b border-sidebar-border">
        <div className="flex items-center gap-2">
          <Activity className="w-8 h-8 text-primary" />
          <h1 className="text-xl font-bold text-sidebar-foreground">ADAS</h1>
        </div>
        <p className="text-xs text-sidebar-foreground/60 mt-1">Hệ Thống Hỗ Trợ Lái Xe</p>
      </div>

      <nav className="flex-1 p-4 space-y-2">
        {navItems.map((item) => {
          const Icon = item.icon
          const isActive = pathname === item.href
          return (
            <Link
              key={item.href}
              href={item.href}
              className={`flex items-center gap-3 px-4 py-3 rounded-lg transition-colors ${
                isActive
                  ? "bg-sidebar-primary text-sidebar-primary-foreground"
                  : "text-sidebar-foreground hover:bg-sidebar-accent/20"
              }`}
            >
              <Icon className="w-5 h-5" />
              <span className="text-sm font-medium">{item.label}</span>
            </Link>
          )
        })}
      </nav>

      <div className="p-4 border-t border-sidebar-border">
        <div className="bg-sidebar-accent/10 rounded-lg p-3">
          <p className="text-xs text-sidebar-foreground/70">
            Trạng Thái: <span className="text-green-400 font-semibold">Hoạt Động</span>
          </p>
        </div>
      </div>
    </aside>
  )
}
