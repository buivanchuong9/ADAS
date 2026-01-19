"use client";

import { motion } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { useLanguage } from "@/contexts/language-context";
import { LogIn, LogOut, User } from "lucide-react";
import { ThemeToggle } from "./theme-toggle";
import Link from "next/link";

export function Header() {
  const { isAuthenticated, user, logout } = useAuth();
  const { t } = useLanguage();

  const handleLogout = async () => {
    await logout();
    // logout() already handles redirect to /login
  };

  return (
    <motion.header
      initial={{ y: -100, opacity: 0 }}
      animate={{ y: 0, opacity: 1 }}
      transition={{ duration: 0.5, ease: [0.4, 0, 0.2, 1] }}
      className="sticky top-0 z-50 w-full backdrop-blur-md"
      style={{
        backgroundColor: 'var(--bg-surface)',
        borderBottom: '1px solid var(--border-subtle)',
      }}
    >
      <div className="flex items-center justify-between px-4 sm:px-6 lg:px-8 h-16">
        {/* Left side - Logo/Title */}
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-primary to-accent flex items-center justify-center shadow-lg shadow-primary/30">
            <User className="w-5 h-5 text-white" />
          </div>
          <div className="hidden sm:block">
            <h2
              className="text-sm font-semibold tracking-wide"
              style={{ color: 'var(--primary)' }}
            >
              ADAS Platform
            </h2>
          </div>
        </div>

        {/* Right side - Theme Toggle & Auth buttons */}
        <div className="flex items-center gap-3">
          <ThemeToggle size="sm" />
          {isAuthenticated ? (
            <>
              <motion.div
                initial={{ opacity: 0, x: 20 }}
                animate={{ opacity: 1, x: 0 }}
                className="hidden sm:flex items-center gap-3 px-4 py-2 rounded-xl border"
                style={{
                  backgroundColor: 'var(--bg-surface)',
                  borderColor: 'var(--border-subtle)',
                  boxShadow: 'var(--shadow-soft)',
                }}
              >
                <div
                  className="w-2 h-2 rounded-full animate-pulse"
                  style={{ backgroundColor: 'var(--success)' }}
                />
                <span
                  className="text-sm"
                  style={{ color: 'var(--text-muted)' }}
                >
                  {t('header.greeting')}{" "}
                  <span
                    className="font-semibold"
                    style={{ color: 'var(--primary)' }}
                  >
                    {user?.username}
                  </span>
                </span>
              </motion.div>
              <motion.button
                whileHover={{ scale: 1.05 }}
                whileTap={{ scale: 0.95 }}
                onClick={handleLogout}
                className="btn-neon btn-neon-red flex items-center gap-2 px-4 py-2 text-sm"
              >
                <LogOut className="w-4 h-4" />
                <span className="hidden sm:inline">{t('header.logout')}</span>
              </motion.button>
            </>
          ) : (
            <Link href="/login">
              <motion.button
                whileHover={{ scale: 1.05 }}
                whileTap={{ scale: 0.95 }}
                className="btn-neon flex items-center gap-2 px-4 py-2 text-sm"
              >
                <LogIn className="w-4 h-4" />
                <span>{t('header.login')}</span>
              </motion.button>
            </Link>
          )}
        </div>
      </div>

      {/* Accent line */}
      <div
        className="absolute bottom-0 left-0 right-0 h-px opacity-50"
        style={{
          background: 'linear-gradient(to right, transparent, var(--primary), transparent)',
        }}
      />
    </motion.header>
  );
}

