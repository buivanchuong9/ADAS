'use client';

import { useEffect, useState } from 'react';
import { motion } from 'framer-motion';
import { Sun, Moon } from 'lucide-react';
import styles from './ThemeToggle.module.css';

export default function ThemeToggle() {
    const [isDark, setIsDark] = useState(true); // DEFAULT = DARK MODE
    const [mounted, setMounted] = useState(false);

    useEffect(() => {
        const savedTheme = localStorage.getItem('overview-theme');
        const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
        // Default to dark if no saved preference
        const initialDark = savedTheme === 'light' ? false : true;

        setIsDark(initialDark);
        if (initialDark) {
            document.documentElement.classList.add('dark-theme');
        } else {
            document.documentElement.classList.remove('dark-theme');
        }
        setMounted(true);
    }, []);

    const toggleTheme = () => {
        const newIsDark = !isDark;
        setIsDark(newIsDark);

        if (newIsDark) {
            document.documentElement.classList.add('dark-theme');
            localStorage.setItem('overview-theme', 'dark');
        } else {
            document.documentElement.classList.remove('dark-theme');
            localStorage.setItem('overview-theme', 'light');
        }
    };

    return (
        <motion.button
            onClick={toggleTheme}
            className={styles.toggle}
            aria-label="Toggle theme"
            title={isDark ? 'Chuyển sang chế độ sáng' : 'Chuyển sang chế độ tối'}
            whileHover={{ scale: 1.1 }}
            whileTap={{ scale: 0.9 }}
            transition={{ type: "spring", stiffness: 400, damping: 17 }}
        >
            <motion.div
                className={styles.iconWrapper}
                initial={false}
                animate={{ rotate: isDark ? 180 : 0 }}
                transition={{ duration: 0.5, ease: "easeInOut" }}
            >
                {isDark ? (
                    <Sun className={styles.icon} size={20} strokeWidth={2.5} />
                ) : (
                    <Moon className={styles.icon} size={20} strokeWidth={2.5} />
                )}
            </motion.div>
        </motion.button>
    );
}
