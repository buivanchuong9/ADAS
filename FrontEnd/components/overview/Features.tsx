'use client';

import styles from './Features.module.css';

const FEATURES = [
    {
        icon: '👁️',
        title: 'Thị Giác Máy Tính',
        description:
            'Sử dụng Deep Learning để phân loại phương tiện và người đi bộ trong điều kiện giao thông hỗn loạn.',
    },
    {
        icon: '📡',
        title: 'Xử Lý Tại Biên (Edge)',
        description:
            'Thuật toán chạy trực tiếp trên phần cứng xe. Không phụ thuộc đường truyền mạng.',
    },
    {
        icon: '🧠',
        title: 'Phân Tích Hành Vi',
        description:
            'Hệ thống DMS theo dõi cử chỉ khuôn mặt tài xế để phát hiện buồn ngủ.',
    },
];

export default function Features() {
    return (
        <section className={styles.section}>
            <div className={styles.header}>
                <span className={styles.label}>CÔNG NGHỆ CỐT LÕI</span>
                <h2>Kiến Trúc Hệ Thống</h2>
            </div>

            <div className={styles.fGrid}>
                {FEATURES.map((feature, index) => (
                    <div key={index} className={styles.fCard}>
                        <div className={styles.iconBox}>{feature.icon}</div>
                        <h3>{feature.title}</h3>
                        <p>{feature.description}</p>
                    </div>
                ))}
            </div>
        </section>
    );
}
