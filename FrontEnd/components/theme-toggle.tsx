'use client';

import { motion } from 'framer-motion';
import { Sun, Moon } from 'lucide-react';
import { useTheme } from '@/contexts/theme-context';
import { cn } from '@/lib/utils';

interface ThemeToggleProps {
  className?: string;
  size?: 'sm' | 'md' | 'lg';
}

export function ThemeToggle({ className, size = 'md' }: ThemeToggleProps) {
  const { theme, toggleTheme } = useTheme();
  const isDark = theme === 'dark';

  const sizeClasses = {
    sm: 'w-8 h-8',
    md: 'w-10 h-10',
    lg: 'w-12 h-12',
  };

  const iconSizes = {
    sm: 16,
    md: 20,
    lg: 24,
  };

  return (
    <motion.button
      onClick={toggleTheme}
      className={cn(
        'relative flex items-center justify-center rounded-full',
        'bg-white/10 dark:bg-white/5',
        'border border-white/20 dark:border-white/10',
        'backdrop-blur-md',
        'hover:bg-white/20 dark:hover:bg-white/10',
        'transition-all duration-300',
        'focus:outline-none focus:ring-2 focus:ring-primary/50',
        sizeClasses[size],
        className
      )}
      aria-label={isDark ? 'Chuyển sang chế độ sáng' : 'Chuyển sang chế độ tối'}
      title={isDark ? 'Chuyển sang chế độ sáng' : 'Chuyển sang chế độ tối'}
      whileHover={{ scale: 1.1 }}
      whileTap={{ scale: 0.9 }}
      transition={{ type: "spring", stiffness: 400, damping: 17 }}
    >
      <motion.div
        className="absolute inset-0 flex items-center justify-center"
        initial={false}
        animate={{ rotate: isDark ? 0 : 180 }}
        transition={{ duration: 0.5, ease: "easeInOut" }}
      >
        {isDark ? (
          <Sun 
            className="text-yellow-400" 
            size={iconSizes[size]} 
            strokeWidth={2.5} 
          />
        ) : (
          <Moon 
            className="text-blue-400" 
            size={iconSizes[size]} 
            strokeWidth={2.5} 
          />
        )}
      </motion.div>
    </motion.button>
  );
}
