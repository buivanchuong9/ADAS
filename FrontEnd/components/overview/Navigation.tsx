'use client';

import { useState } from 'react';
import { useAuth } from '@/contexts/auth-context';
import { AuthModal } from '@/components/auth-modal';
import { User, LogOut } from 'lucide-react';
import ThemeToggle from './ThemeToggle';
import styles from './Navigation.module.css';

export default function Navigation() {
    const { isAuthenticated, user, logout } = useAuth();
    const [showAuthModal, setShowAuthModal] = useState(false);
    const [showAccountMenu, setShowAccountMenu] = useState(false);

    const handleLogout = () => {
        logout();
        setShowAccountMenu(false);
    };

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

                    {!isAuthenticated ? (
                        <button
                            onClick={() => setShowAuthModal(true)}
                            className={styles.btnPrimary}
                        >
                            TRUY CẬP HỆ THỐNG
                        </button>
                    ) : (
                        <div className={styles.accountWrapper}>
                            <button
                                onClick={() => setShowAccountMenu(!showAccountMenu)}
                                className={styles.btnAccount}
                            >
                                <User size={18} />
                                <span>TÀI KHOẢN</span>
                            </button>

                            {showAccountMenu && (
                                <div className={styles.accountMenu}>
                                    <div className={styles.menuHeader}>
                                        <div className={styles.menuUsername}>{user?.username}</div>
                                        <div className={styles.menuEmail}>{user?.email}</div>
                                    </div>
                                    <div className={styles.menuDivider}></div>
                                    <button
                                        onClick={() => window.location.href = '/dashboard'}
                                        className={styles.menuItem}
                                    >
                                        Dashboard
                                    </button>
                                    <button
                                        onClick={handleLogout}
                                        className={styles.menuItem}
                                    >
                                        <LogOut size={16} />
                                        <span>Đăng xuất</span>
                                    </button>
                                </div>
                            )}
                        </div>
                    )}
                </div>
            </nav>

            {showAuthModal && (
                <AuthModal
                    isOpen={showAuthModal}
                    onClose={() => setShowAuthModal(false)}
                />
            )}
        </>
    );
}
