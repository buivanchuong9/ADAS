'use client';

import styles from './Footer.module.css';

export default function Footer() {
    return (
        <footer className={styles.footer}>
            <div className={styles.footerContent}>
                <div className={styles.brand}>ADAS SYSTEM v3.0</div>
                <div className={styles.copyright}>
                    © 2026 Developed by ADAS-TEAM. All rights reserved.
                </div>
            </div>
        </footer>
    );
}
