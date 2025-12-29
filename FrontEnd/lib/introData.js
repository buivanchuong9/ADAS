import { Shield, Zap, Brain, Eye, AlertTriangle, Sparkles } from "lucide-react";

export const introSlides = [
    {
        id: 1,
        themeColor: "#00FF88",
        gradientFrom: "#00FF88",
        gradientTo: "#00CC6A",
        icon: Shield,
        title: "AN TOÀN TUYỆT ĐỐI",
        subtitle: "Bảo vệ mọi chuyến đi của bạn",
        description: "Hệ thống ADAS tiên tiến với công nghệ AI thế hệ mới, giám sát 24/7 để đảm bảo an toàn tuyệt đối cho bạn và gia đình.",
        features: [
            "Phát hiện va chạm trước 3-5 giây",
            "Cảnh báo điểm mù tự động",
            "Hỗ trợ giữ làn đường thông minh"
        ],
        buttonText: "Tiếp tục",
    },
    {
        id: 2,
        themeColor: "#FFB800",
        gradientFrom: "#FFB800",
        gradientTo: "#FF8C00",
        icon: Zap,
        title: "PHẢN ỨNG TỨC THÌ",
        subtitle: "Nhanh hơn con người 10 lần",
        description: "AI xử lý hàng triệu dữ liệu mỗi giây, phản ứng tức thì với mọi tình huống nguy hiểm trên đường.",
        features: [
            "Thời gian phản hồi < 100ms",
            "Xử lý 30 khung hình/giây",
            "Cảnh báo âm thanh + hình ảnh"
        ],
        buttonText: "Tiếp tục",
    },
    {
        id: 3,
        themeColor: "#A855F7",
        gradientFrom: "#A855F7",
        gradientTo: "#7C3AED",
        icon: Brain,
        title: "DỰ ĐOÁN TRƯỚC",
        subtitle: "AI biết nguy hiểm sắp xảy ra",
        description: "Công nghệ học máy phân tích hành vi giao thông, dự đoán rủi ro trước khi chúng xảy ra.",
        features: [
            "Phân tích hành vi người lái",
            "Dự đoán hành động xe khác",
            "Cảnh báo nguy cơ tiềm ẩn"
        ],
        buttonText: "Tiếp tục",
    },
    {
        id: 4,
        themeColor: "#00E5FF",
        gradientFrom: "#00E5FF",
        gradientTo: "#00B8D4",
        icon: Eye,
        title: "AI NHÌN THẤY TẤT CẢ",
        subtitle: "360° quan sát không ngừng nghỉ",
        description: "Camera AI giám sát toàn cảnh xung quanh xe, không bỏ sót bất kỳ chi tiết nào, ngay cả trong điều kiện thiếu sáng.",
        features: [
            "Nhận diện 80+ loại đối tượng",
            "Hoạt động cả ngày lẫn đêm",
            "Phạm vi quan sát 360 độ"
        ],
        buttonText: "Tiếp tục",
    },
    {
        id: 5,
        themeColor: "#FF3333",
        gradientFrom: "#FF3333",
        gradientTo: "#CC0000",
        icon: AlertTriangle,
        title: "CẢNH BÁO RỦI RO",
        subtitle: "Mỗi giây đều quan trọng",
        description: "Hệ thống cảnh báo đa cấp độ, từ nhắc nhở nhẹ nhàng đến cảnh báo khẩn cấp, giúp bạn luôn chủ động trên mọi cung đường.",
        features: [
            "Cảnh báo phân cấp thông minh",
            "Âm thanh + rung + hình ảnh",
            "Tùy chỉnh độ nhạy cảnh báo"
        ],
        buttonText: "Tiếp tục",
    },
    {
        id: 6,
        themeColor: "#00E5FF",
        gradientFrom: "#00E5FF",
        gradientTo: "#667EEA",
        icon: Sparkles,
        title: "AI BẢO VỆ AN TOÀN",
        subtitle: "Công nghệ ADAS thế hệ mới",
        description: "Trải nghiệm sự khác biệt của công nghệ AI tiên tiến nhất. Hệ thống ADAS không chỉ là công cụ hỗ trợ, mà là người bạn đồng hành đáng tin cậy trên mọi hành trình.",
        features: [
            "Cập nhật AI liên tục",
            "Tích hợp đám mây thông minh",
            "Hỗ trợ 24/7 mọi lúc mọi nơi"
        ],
        buttonText: "Trải nghiệm AI",
        isFinal: true,
    },
];
