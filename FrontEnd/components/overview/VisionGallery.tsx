'use client';

import Image from 'next/image';
import styles from './VisionGallery.module.css';

const GALLERY_ITEMS = [
    {
        src: '/adas-night.png',
        alt: 'Night driving scenario - ADAS training data',
        label: 'SCENARIO: NIGHT_MODE // LOW LIGHT',
    },
    {
        src: '/adas-rain.png',
        alt: 'Rain driving scenario - ADAS training data',
        label: 'SCENARIO: RAIN / FOG // OBSTRUCTED',
    },
    {
        src: '/adas-driver.png',
        alt: 'Driver monitoring - DMS training data',
        label: 'TARGET: DRIVER_FACE // ATTENTION TRACK',
    },
    {
        src: '/adas-highway.png',
        alt: 'Highway scenario - ADAS training data',
        label: 'SCENARIO: HIGHWAY // HIGH SPEED',
    },
];

export default function VisionGallery() {
    return (
        <section className={styles.section} id="gallery">
            <div className={styles.header}>
                <span className={styles.label}>KHẢ NĂNG XỬ LÝ ĐA DẠNG</span>
                <h2>Dữ Liệu Kịch Bản Thực Tế</h2>
                <p>
                    Mô hình AI được huấn luyện trên 70,000+ hình ảnh để hoạt động ổn định trong mọi điều kiện môi trường.
                </p>
            </div>

            <div className={styles.galleryGrid}>
                {GALLERY_ITEMS.map((item, index) => (
                    <div key={index} className={styles.visionCard}>
                        <Image
                            src={item.src}
                            alt={item.alt}
                            fill
                            sizes="(max-width: 768px) 100vw, 50vw"
                            className={styles.cardImage}
                        />
                        <div className={styles.scanOverlay}></div>
                        <div className={styles.cardLabel}>{item.label}</div>
                    </div>
                ))}
            </div>
        </section>
    );
}
