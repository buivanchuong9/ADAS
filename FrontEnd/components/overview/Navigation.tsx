'use client';

import { useState } from 'react';
import { AuthModal } from '@/components/auth-modal';
import ThemeToggle from './ThemeToggle';
import styles from './Navigation.module.css';

export default function Navigation() {
    const [showAuthModal, setShowAuthModal] = useState(false);

    return (
        <>
            <nav className={styles.nav}>
                <div className={styles.logo}>
                    <div className={styles.statusDot}></div>
                    ADAS SYSTEM
                </div>
                <div className={styles.navRight}>
                    <div className={styles.version}>v3.0-production</div>
                    <ThemeToggle />
                    <button
                        onClick={() => setShowAuthModal(true)}
                        className={styles.btnPrimary}
                    >
                        TRUY CẬP HỆ THỐNG
                    </button>
                </div>
            </nav>

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
