"use client";
import { useEffect, useMemo } from "react";
import { useState } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { motion, AnimatePresence } from "framer-motion";
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
  Menu,
  X,
} from "lucide-react";
import { Button } from "@/components/ui/button";

export function MobileNav() {
  const [isOpen, setIsOpen] = useState(false);
  const { t, language } = useLanguage();
  const pathname = usePathname();

  const navigation = useMemo(() => [
    { name: t("nav.dashboard"), href: "/dashboard", icon: LayoutDashboard, description: t("nav.dashboardDesc") },
    { name: t("nav.liveDetection"), href: "/", icon: Video, description: t("nav.liveDetectionDesc") },
    { name: t("nav.adasMonitor"), href: "/adas", icon: Car, description: t("nav.adasMonitorDesc") },
    { name: t("nav.driverMonitor"), href: "/driver-monitor", icon: Eye, description: t("nav.driverMonitorDesc") },
    { name: t("nav.analytics"), href: "/analytics", icon: BarChart3, description: t("nav.analyticsDesc") },
    { name: t("nav.dataCollection"), href: "/data-collection", icon: Database, description: t("nav.dataCollectionDesc") },
    { name: t("nav.aiAssistant"), href: "/ai-assistant", icon: Brain, description: t("nav.aiAssistantDesc") },
    { name: t("nav.events"), href: "/events", icon: AlertTriangle, description: t("nav.eventsDesc") },
  ], [language, t]);
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
                    {t("header.platformName")} <span className="text-fg-secondary">• v3.0</span>
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
          key={item.href}
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
                <span className="font-semibold text-sm">{t("nav.settings")}</span>
              </Link>

              <div className="mt-4 px-4 py-3 rounded-xl bg-green-50 border border-green-200">
                <div className="flex items-center gap-2">
                  <div className="w-2 h-2 rounded-full bg-green-500" />
                  <span className="text-sm font-medium text-green-700">
                    {t("nav.systemOnline")}
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
