/**
 * Vietnamese Translations for ADAS Platform
 * Centralized translation file for consistency
 */

export const translations = {
    // Navigation
    nav: {
        dashboard: 'Bảng Điều Khiển',
        liveDetection: 'Phát Hiện Trực Tiếp',
        adasMonitor: 'Giám Sát ADAS',
        driverMonitor: 'Giám Sát Tài Xế',
        analytics: 'Phân Tích',
        dataCollection: 'Thu Thập Dữ Liệu',
        aiAssistant: 'Trợ Lý AI',
        events: 'Sự Kiện',
        settings: 'Cài Đặt',
    },

    // Object Classes (YOLO Detection)
    objects: {
        person: 'Người',
        bicycle: 'Xe Đạp',
        car: 'Xe Ô Tô',
        motorcycle: 'Xe Máy',
        airplane: 'Máy Bay',
        bus: 'Xe Buýt',
        train: 'Tàu Hỏa',
        truck: 'Xe Tải',
        boat: 'Thuyền',
        'traffic light': 'Đèn Giao Thông',
        'fire hydrant': 'Vòi Cứu Hỏa',
        'stop sign': 'Biển Dừng',
        'parking meter': 'Đồng Hồ Đỗ Xe',
        bench: 'Ghế Dài',
        bird: 'Chim',
        cat: 'Mèo',
        dog: 'Chó',
        horse: 'Ngựa',
        sheep: 'Cừu',
        cow: 'Bò',
        elephant: 'Voi',
        bear: 'Gấu',
        zebra: 'Ngựa Vằn',
        giraffe: 'Hươu Cao Cổ',
    },

    // Alert Messages
    alerts: {
        collisionWarning: 'Cảnh Báo Va Chạm',
        laneDeparture: 'Cảnh Báo Lệch Làn',
        pedestrianDetected: 'Phát Hiện Người Đi Bộ',
        vehicleTooClose: 'Xe Phía Trước Quá Gần',
        speedWarning: 'Cảnh Báo Tốc Độ',
        drowsinessDetected: 'Phát Hiện Buồn Ngủ',
        distractionDetected: 'Phát Hiện Mất Tập Trung',
        phoneUse: 'Phát Hiện Sử Dụng Điện Thoại',
    },

    // UI Common
    common: {
        start: 'Bắt Đầu',
        stop: 'Dừng',
        pause: 'Tạm Dừng',
        resume: 'Tiếp Tục',
        save: 'Lưu',
        cancel: 'Hủy',
        delete: 'Xóa',
        edit: 'Sửa',
        view: 'Xem',
        download: 'Tải Xuống',
        upload: 'Tải Lên',
        search: 'Tìm Kiếm',
        filter: 'Lọc',
        export: 'Xuất',
        import: 'Nhập',
        refresh: 'Làm Mới',
        loading: 'Đang Tải...',
        error: 'Lỗi',
        success: 'Thành Công',
        warning: 'Cảnh Báo',
        info: 'Thông Tin',
    },

    // Stats & Metrics
    stats: {
        systemStatus: 'Trạng Thái Hệ Thống',
        activeCameras: 'Camera Hoạt Động',
        totalDetections: 'Tổng Phát Hiện',
        alertsToday: 'Cảnh Báo Hôm Nay',
        fps: 'Tốc Độ Khung Hình',
        inferenceTime: 'Thời Gian Xử Lý',
        confidence: 'Độ Tin Cậy',
        distance: 'Khoảng Cách',
        speed: 'Tốc Độ',
        ttc: 'Thời Gian Va Chạm',
    },

    // Homepage
    home: {
        title: 'Hệ Thống Hỗ Trợ Lái Xe Thông Minh',
        subtitle: 'Giám sát an toàn bằng AI thời gian thực với WebSocket streaming, thu thập dữ liệu tự động và cảnh báo thông minh',
        startDetection: 'Bắt Đầu Phát Hiện',
        viewDashboard: 'Xem Bảng Điều Khiển',
        quickActions: 'Thao Tác Nhanh',
        systemFeatures: 'Tính Năng Hệ Thống',
        recentActivity: 'Hoạt Động Gần Đây',
        allSystemsOnline: 'Tất cả hệ thống hoạt động',
        monitoringInRealtime: 'Giám sát thời gian thực',
    },

    // ADAS Page
    adas: {
        title: 'Giám Sát ADAS Nâng Cao',
        subtitle: 'Phát hiện đối tượng và cảnh báo nguy hiểm thời gian thực',
        startDetection: 'Bắt Đầu Phát Hiện',
        stopDetection: 'Dừng Phát Hiện',
        detectionStats: 'Thống Kê Phát Hiện',
        objectsDetected: 'Đối Tượng Phát Hiện',
        dangerAlerts: 'Cảnh Báo Nguy Hiểm',
        autoLearning: 'Học Tự Động',
        dataCollected: 'Dữ Liệu Thu Thập',
        clickToStart: 'Nhấn nút bên dưới để bắt đầu',
    },

    // Driver Monitor
    driver: {
        title: 'Giám Sát Tài Xế',
        subtitle: 'Theo dõi tình trạng tài xế và phát hiện mệt mỏi, phân tán',
        startMonitoring: 'Bắt Đầu Giám Sát',
        stopMonitoring: 'Dừng Giám Sát',
        driverMetrics: 'Chỉ Số Tài Xế',
        fatigue: 'Mệt Mỏi',
        distraction: 'Phân Tán',
        eyeStatus: 'Trạng Thái Mắt',
        open: 'Mở',
        closed: 'Đóng',
        status: 'Tình Trạng',
        normal: 'Bình Thường',
        tired: 'Mệt Mỏi',
        warning: 'Cảnh Báo',
        eyesClosed: 'Phát hiện mắt đóng',
        highFatigue: 'Mức mệt mỏi cao',
        distractionDetected: 'Phát hiện phân tán',
    },

    // System Features
    features: {
        realtimeWebSocket: 'WebSocket Thời Gian Thực',
        realtimeDesc: 'Xử lý video độ trễ thấp',
        yoloDetection: 'Phát Hiện AI YOLOv11',
        yoloDesc: 'Nhận dạng đối tượng tiên tiến',
        autoCollection: 'Thu Thập Dữ Liệu Tự Động',
        autoCollectionDesc: 'Cải thiện mô hình liên tục',
        intelligentAlerts: 'Cảnh Báo Thông Minh',
        intelligentAlertsDesc: 'Cảnh báo bằng giọng nói và hình ảnh',
        dockerDeployment: 'Triển Khai Docker',
        dockerDeploymentDesc: 'Cài đặt một lệnh',
    },

    // Status
    status: {
        operational: 'Hoạt Động',
        degraded: 'Suy Giảm',
        offline: 'Ngoại Tuyến',
        healthy: 'Khỏe Mạnh',
        unhealthy: 'Không Ổn Định',
        systemOnline: 'Hệ Thống Trực Tuyến',
    },
} as const

export type TranslationKey = typeof translations

// Helper function to get nested translation
export function t(path: string): string {
    const keys = path.split('.')
    let value: any = translations

    for (const key of keys) {
        value = value[key]
        if (value === undefined) {
            console.warn(`Translation not found: ${path}`)
            return path
        }
    }

    return value
}

// Helper to translate object class
export function translateObjectClass(className: string): string {
    const normalized = className.toLowerCase().trim()
    return translations.objects[normalized as keyof typeof translations.objects] || className
}
