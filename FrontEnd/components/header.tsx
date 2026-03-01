"use client";

import { motion } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { useLanguage } from "@/contexts/language-context";
import { LogIn, LogOut, User, UserCircle } from "lucide-react";
import Link from "next/link";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar";

export function Header() {
  const { isAuthenticated, user, logout } = useAuth();
  const { t } = useLanguage();

  const handleLogout = async () => {
    await logout();
    // logout() already handles redirect to /login
  };

  // Safe fallback for display name
  const displayName =
    user?.username || user?.email?.split("@")[0] || t("common.user") || "User";
  const displayEmail = user?.email || "No email";
  const userInitials = displayName?.substring(0, 1)?.toUpperCase() || "U";

  return (
    <motion.header
      initial={{ y: -100, opacity: 0 }}
      animate={{ y: 0, opacity: 1 }}
      transition={{ duration: 0.5, ease: [0.4, 0, 0.2, 1] }}
      className="sticky top-0 z-50 w-full backdrop-blur-md"
      style={{
        backgroundColor: "var(--bg-surface)",
        borderBottom: "1px solid var(--border-subtle)",
      }}
    >
      <div className="flex items-center justify-between px-4 sm:px-6 lg:px-8 h-16">
        {/* Left side - Logo/Title */}
        <Link href="/" className="flex items-center gap-3 group">
          <div className="w-8 h-8 rounded-lg bg-linear-to-br from-primary to-accent flex items-center justify-center shadow-lg shadow-primary/30 group-hover:shadow-primary/50 transition-shadow">
            <User className="w-5 h-5 text-black" />
          </div>
          <div className="hidden sm:block">
            <h2
              className="text-sm font-semibold tracking-wide group-hover:text-primary transition-colors"
              style={{ color: "var(--primary)" }}
            >
              {t("header.platformName")}
            </h2>
          </div>
        </Link>

        {/* Right side - Auth buttons */}
        <div className="flex items-center gap-3">
          {isAuthenticated ? (
            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                <div className="cursor-pointer px-4 py-2 rounded-lg border-2 border-border bg-black/5 hover:bg-black/10 hover:border-primary/50 transition-all select-none">
                  <span className="text-sm font-medium text-foreground">
                    {t("header.greeting")} {displayName}
                  </span>
                </div>
              </DropdownMenuTrigger>
              <DropdownMenuContent
                align="end"
                className="w-64 bg-white border border-gray-200 p-2 shadow-xl animate-in fade-in slide-in-from-top-2 duration-200"
              >
                {/* Custom Header in Dropdown */}
                <div className="flex items-center gap-3 p-2 mb-2">
                  <Avatar className="h-10 w-10 border border-gray-200">
                    <AvatarFallback className="bg-primary/20 text-primary font-bold">
                      {userInitials}
                    </AvatarFallback>
                  </Avatar>
                  <div className="flex flex-col overflow-hidden">
                    <span className="font-semibold text-gray-900 truncate">
                      {displayName}
                    </span>
                    <span className="text-xs text-gray-500 truncate">
                      {displayEmail}
                    </span>
                  </div>
                </div>

                <DropdownMenuSeparator className="bg-gray-200" />

                <Link href="/profile">
                  <DropdownMenuItem className="cursor-pointer hover:bg-gray-100 focus:bg-gray-100 p-2.5 rounded-lg my-1 text-gray-700">
                    <UserCircle className="w-4 h-4 mr-3 text-blue-500" />
                    <span>{t("header.profile")}</span>
                  </DropdownMenuItem>
                </Link>

                <DropdownMenuSeparator className="bg-gray-200" />

                <DropdownMenuItem
                  onClick={handleLogout}
                  className="cursor-pointer text-red-600 hover:text-red-700 hover:bg-red-50 focus:bg-red-50 p-2.5 rounded-lg my-1 group/item"
                >
                  <LogOut className="w-4 h-4 mr-3 group-hover/item:text-red-700 transition-colors" />
                  <span>{t("header.logout")}</span>
                </DropdownMenuItem>
              </DropdownMenuContent>
            </DropdownMenu>
          ) : (
            <Link href="/login">
              <motion.button
                whileHover={{ scale: 1.05 }}
                whileTap={{ scale: 0.95 }}
                className="btn-neon flex items-center gap-2 px-4 py-2 text-sm"
              >
                <LogIn className="w-4 h-4" />
                <span>{t("header.login")}</span>
              </motion.button>
            </Link>
          )}
        </div>
      </div>

      {/* Accent line */}
      <div
        className="absolute bottom-0 left-0 right-0 h-px opacity-50"
        style={{
          background:
            "linear-gradient(to right, transparent, var(--primary), transparent)",
        }}
      />
    </motion.header>
  );
}
