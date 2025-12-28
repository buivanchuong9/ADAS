"use client";

import { useState } from "react";
import { motion } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { LogIn, LogOut, User } from "lucide-react";
import { AuthModal } from "./auth-modal";

export function Header() {
  const { isAuthenticated, user, logout } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);

  // Prevent opening modal if already authenticated
  const handleOpenAuthModal = () => {
    if (isAuthenticated) {
      return; // Already authenticated, don't show login
    }
    setShowAuthModal(true);
  };

  const handleLogout = () => {
    logout();
    // Reload to reset state
    window.location.reload();
  };

  return (
    <>
      <motion.header
        initial={{ y: -100, opacity: 0 }}
        animate={{ y: 0, opacity: 1 }}
        transition={{ duration: 0.5, ease: [0.4, 0, 0.2, 1] }}
        className="sticky top-0 z-50 w-full glass-panel border-b border-white/10 backdrop-blur-md"
      >
        <div className="flex items-center justify-between px-4 sm:px-6 lg:px-8 h-16">
          {/* Left side - Logo/Title */}
          <div className="flex items-center gap-3">
            <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-primary to-accent flex items-center justify-center shadow-lg shadow-primary/30">
              <User className="w-5 h-5 text-white" />
            </div>
            <div className="hidden sm:block">
              <h2 className="text-sm font-semibold text-neon-cyan tracking-wide">
                ADAS Platform
              </h2>
            </div>
          </div>

          {/* Right side - Auth buttons */}
          <div className="flex items-center gap-3">
            {isAuthenticated ? (
              <>
                <motion.div
                  initial={{ opacity: 0, x: 20 }}
                  animate={{ opacity: 1, x: 0 }}
                  className="hidden sm:flex items-center gap-3 px-4 py-2 rounded-xl glass-card border-neon-cyan/30"
                >
                  <div className="w-2 h-2 rounded-full bg-neon-green animate-pulse" 
                    style={{ boxShadow: '0 0 10px var(--neon-green)' }}
                  />
                  <span className="text-sm text-fg-secondary">
                    Xin chào,{" "}
                    <span className="text-neon-cyan font-semibold">
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
                  <span className="hidden sm:inline">Đăng xuất</span>
                </motion.button>
              </>
            ) : (
              <motion.button
                whileHover={{ scale: 1.05 }}
                whileTap={{ scale: 0.95 }}
                onClick={handleOpenAuthModal}
                className="btn-neon flex items-center gap-2 px-4 py-2 text-sm"
              >
                <LogIn className="w-4 h-4" />
                <span>Đăng nhập</span>
              </motion.button>
            )}
          </div>
        </div>

        {/* Scan line effect */}
        <div className="absolute bottom-0 left-0 right-0 h-px bg-gradient-to-r from-transparent via-neon-cyan to-transparent opacity-50" />
      </motion.header>

      {/* Auth Modal */}
      {showAuthModal && (
        <AuthModal
          isOpen={showAuthModal}
          onClose={() => setShowAuthModal(false)}
        />
      )}
    </>
  );
}

