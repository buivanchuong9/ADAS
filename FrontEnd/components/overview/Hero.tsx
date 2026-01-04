'use client';

import { motion } from 'framer-motion';
import SystemLog from './SystemLog';
import styles from './Hero.module.css';
import btnStyles from './ButtonStyles.module.css';

export default function Hero() {
    return (
        <section className={styles.hero}>
            <div className={styles.heroText}>
                <span className={styles.label}>SẢN PHẨM CỦA ADAS TEAM</span>
                <h1>
                    Hệ Thống Hỗ Trợ Lái Xe Bằng AI <br />
                    An Toàn Chủ Động
                </h1>
                <p>
                    Nền tảng tích hợp AI Vision nhận diện vật thể thời gian thực,
                    cảnh báo sớm va chạm và giám sát hành vi tài xế với độ chính
                    xác cao. Được xây dựng trên YOLOv11 và tối ưu cho edge computing.
                </p>
                <div className={styles.btnGroup}>
                    <motion.a
                        href="#demo"
                        className={btnStyles.btnPrimary}
                        whileHover={{ scale: 1.05, y: -2 }}
                        whileTap={{ scale: 0.95 }}
                        transition={{ type: "spring", stiffness: 400, damping: 17 }}
                    >
                        XEM DEMO THỰC TẾ
                    </motion.a>
                    <motion.a
                        href="#gallery"
                        className={btnStyles.btnSecondary}
                        whileHover={{ scale: 1.05, y: -2 }}
                        whileTap={{ scale: 0.95 }}
                        transition={{ type: "spring", stiffness: 400, damping: 17 }}
                    >
                        DỮ LIỆU HUẤN LUYỆN
                    </motion.a>
                </div>
            </div>

            <SystemLog />
        </section>
    );
}
