import type { LucideIcon } from "lucide-react";
import { Shield, Zap, Brain, Eye, AlertTriangle, Sparkles } from "lucide-react";

export type IntroSlide = {
  id: number;
  bg: string;
  themeColor: string;
  gradientFrom?: string;
  gradientTo?: string;
  icon: LucideIcon;
  title: string;
  subtitle: string;
  description: string;
  features: string[];
  buttonText: string;
  isFinal?: boolean;
};

export const introSlides: IntroSlide[] = [
  {
    id: 1,
    bg: "https://xehay.vn/uploads/images/2020/4/3/Xehay-ANTD-220420-22.jpg",
    themeColor: "#FF3333",
    gradientFrom: "#FF3333",
    gradientTo: "#CC0000",
    icon: AlertTriangle,
    title: "BẢO VỆ THÔNG MINH",
    subtitle: "AI giám sát từng chuyển động, ngăn ngừa rủi do va chạm trong tích tắc.",
    description: "",
    features: [],
    buttonText: "Khám phá ngay",
  },
  {
    id: 2,
    bg: "https://images.unsplash.com/photo-1503376780353-7e6692767b70?q=80&w=1200",
    themeColor: "#00E5FF",
    gradientFrom: "#00E5FF",
    gradientTo: "#00B8D4",
    icon: Eye,
    title: "THỊ GIÁC SIÊU CẤP",
    subtitle: "Nhận diện vật thể chuẩn xác ngay cả trong điều kiện thiếu sáng.",
    description: "",
    features: [],
    buttonText: "Tìm hiểu thêm",
  },
  {
    id: 3,
    bg: "https://cdn-media.sforum.vn/storage/app/media/ctv_seo4/Van%20Pham/7/hinh-nen-sieu-xe-1.jpg",
    themeColor: "#A855F7",
    gradientFrom: "#A855F7",
    gradientTo: "#7C3AED",
    icon: Brain,
    title: "DỰ ĐOÁN HÀNH VI",
    subtitle: "Phân tích và dự báo quỹ đạo di chuyển của các phương tiện xung quanh.",
    description: "",
    features: [],
    buttonText: "Xem tiếp",
  },
  {
    id: 4,
    bg: "https://images.unsplash.com/photo-1511919884226-fd3cad34687c?q=80&w=1200",
    themeColor: "#FFB800",
    gradientFrom: "#FFB800",
    gradientTo: "#FF8C00",
    icon: Zap,
    title: "PHẢN ỨNG TỨC THÌ",
    subtitle: "Xử lý dữ liệu tốc độ cao, hỗ trợ tài lái đưa ra quyết định tối ưu.",
    description: "",
    features: [],
    buttonText: "Tiếp tục",
  },
  {
    id: 5,
    bg: "https://images.unsplash.com/photo-1552519507-da3b142c6e3d?q=80&w=1200",
    themeColor: "#00FF88",
    gradientFrom: "#00FF88",
    gradientTo: "#00CC6A",
    icon: Shield,
    title: "AN TÂM TRÊN MỌI DẶM ĐƯỜNG",
    subtitle: "Hệ thống ADAS tiên tiến - Người bạn đồng hành tin cậy của gia đình bạn.",
    description: "",
    features: [],
    buttonText: "Bắt đầu ngay",
  },
];