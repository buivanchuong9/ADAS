(globalThis.TURBOPACK || (globalThis.TURBOPACK = [])).push([typeof document === "object" ? document.currentScript : undefined,
"[project]/Desktop/AI/ADAS/FrontEnd/lib/utils.ts [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "cn",
    ()=>cn
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$clsx$2f$dist$2f$clsx$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/clsx/dist/clsx.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$tailwind$2d$merge$2f$dist$2f$bundle$2d$mjs$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/tailwind-merge/dist/bundle-mjs.mjs [app-client] (ecmascript)");
;
;
function cn(...inputs) {
    return (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$tailwind$2d$merge$2f$dist$2f$bundle$2d$mjs$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["twMerge"])((0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$clsx$2f$dist$2f$clsx$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["clsx"])(inputs));
}
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
"[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "Sidebar",
    ()=>Sidebar
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/jsx-dev-runtime.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/client/app-dir/link.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$navigation$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/navigation.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/framer-motion/dist/es/render/components/motion/proxy.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/lib/utils.ts [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$layout$2d$dashboard$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__LayoutDashboard$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/layout-dashboard.js [app-client] (ecmascript) <export default as LayoutDashboard>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$video$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Video$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/video.js [app-client] (ecmascript) <export default as Video>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$eye$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Eye$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/eye.js [app-client] (ecmascript) <export default as Eye>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$database$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Database$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/database.js [app-client] (ecmascript) <export default as Database>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$chart$2d$column$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__BarChart3$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/chart-column.js [app-client] (ecmascript) <export default as BarChart3>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$settings$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Settings$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/settings.js [app-client] (ecmascript) <export default as Settings>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$car$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Car$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/car.js [app-client] (ecmascript) <export default as Car>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$brain$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Brain$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/brain.js [app-client] (ecmascript) <export default as Brain>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$triangle$2d$alert$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__AlertTriangle$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/triangle-alert.js [app-client] (ecmascript) <export default as AlertTriangle>");
;
var _s = __turbopack_context__.k.signature();
"use client";
;
;
;
;
;
const navigation = [
    {
        name: "Bảng Điều Khiển",
        href: "/dashboard",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$layout$2d$dashboard$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__LayoutDashboard$3e$__["LayoutDashboard"],
        description: "Tổng quan hệ thống"
    },
    {
        name: "Phát Hiện Trực Tiếp",
        href: "/",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$video$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Video$3e$__["Video"],
        description: "ADAS thời gian thực"
    },
    {
        name: "Giám Sát ADAS",
        href: "/adas",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$car$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Car$3e$__["Car"],
        description: "Giám sát nâng cao"
    },
    {
        name: "Giám Sát Tài Xế",
        href: "/driver-monitor",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$eye$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Eye$3e$__["Eye"],
        description: "Hành vi tài xế"
    },
    {
        name: "Phân Tích",
        href: "/analytics",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$chart$2d$column$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__BarChart3$3e$__["BarChart3"],
        description: "Số liệu hiệu suất"
    },
    {
        name: "Thu Thập Dữ Liệu",
        href: "/data-collection",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$database$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Database$3e$__["Database"],
        description: "Quản lý dataset"
    },
    {
        name: "Trợ Lý AI",
        href: "/ai-assistant",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$brain$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Brain$3e$__["Brain"],
        description: "Hỗ trợ AI"
    },
    {
        name: "Sự Kiện",
        href: "/events",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$triangle$2d$alert$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__AlertTriangle$3e$__["AlertTriangle"],
        description: "Lịch sử cảnh báo"
    }
];
function Sidebar() {
    _s();
    const pathname = (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$navigation$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["usePathname"])();
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].aside, {
        initial: {
            x: -100,
            opacity: 0
        },
        animate: {
            x: 0,
            opacity: 1
        },
        transition: {
            duration: 0.5,
            ease: [
                0.4,
                0,
                0.2,
                1
            ]
        },
        className: "hidden lg:flex w-64 glass-panel border-r border-white/10 flex-col relative overflow-hidden rounded-2xl scan-lines",
        children: [
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                className: "absolute inset-0 bg-gradient-to-b from-white/5 via-transparent to-transparent pointer-events-none"
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                lineNumber: 81,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                initial: {
                    opacity: 0,
                    y: -20
                },
                animate: {
                    opacity: 1,
                    y: 0
                },
                transition: {
                    delay: 0.2
                },
                className: "p-6 border-b border-sidebar-border/50 relative z-10",
                children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["default"], {
                    href: "/",
                    className: "flex flex-col items-center gap-3 group",
                    children: [
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                            className: "w-32 h-32 rounded-2xl overflow-hidden shadow-2xl ring-2 ring-neon-cyan/30 glow-pulse-cyan",
                            whileHover: {
                                scale: 1.05
                            },
                            transition: {
                                type: "spring",
                                stiffness: 400
                            },
                            children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("img", {
                                src: "/adas-logo.jpg",
                                alt: "ADAS Logo",
                                className: "w-full h-full object-cover"
                            }, void 0, false, {
                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                lineNumber: 96,
                                columnNumber: 13
                            }, this)
                        }, void 0, false, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                            lineNumber: 91,
                            columnNumber: 11
                        }, this),
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                            className: "text-center",
                            children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("p", {
                                className: "text-xs font-semibold text-neon-cyan tracking-wide",
                                children: [
                                    "ADAS Platform ",
                                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                                        className: "text-fg-secondary",
                                        children: "• v3.0"
                                    }, void 0, false, {
                                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                        lineNumber: 104,
                                        columnNumber: 29
                                    }, this)
                                ]
                            }, void 0, true, {
                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                lineNumber: 103,
                                columnNumber: 13
                            }, this)
                        }, void 0, false, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                            lineNumber: 102,
                            columnNumber: 11
                        }, this)
                    ]
                }, void 0, true, {
                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                    lineNumber: 90,
                    columnNumber: 9
                }, this)
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                lineNumber: 84,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("nav", {
                className: "flex-1 p-4 space-y-2 overflow-y-auto relative z-10",
                children: navigation.map((item, index)=>{
                    const isActive = pathname === item.href;
                    const Icon = item.icon;
                    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                        initial: {
                            opacity: 0,
                            x: -20
                        },
                        animate: {
                            opacity: 1,
                            x: 0
                        },
                        transition: {
                            delay: 0.3 + index * 0.05
                        },
                        children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["default"], {
                            href: item.href,
                            className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-300 group relative overflow-hidden", isActive ? "glass-card glow-cyan text-neon-cyan border-neon-cyan/50" : "glass-card border-transparent hover:border-neon-cyan/30 hover:glow-cyan text-fg-primary hover:text-neon-cyan"),
                            children: [
                                isActive && /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                                    className: "absolute left-0 top-1/2 -translate-y-1/2 w-1 h-10 bg-gradient-to-b from-primary to-accent rounded-r-full",
                                    layoutId: "activeIndicator",
                                    transition: {
                                        type: "spring",
                                        stiffness: 300,
                                        damping: 30
                                    }
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                    lineNumber: 134,
                                    columnNumber: 19
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                    className: "absolute inset-0 bg-gradient-to-r from-transparent via-white/10 to-transparent -translate-x-full group-hover:translate-x-full transition-transform duration-1000"
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                    lineNumber: 142,
                                    columnNumber: 17
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(Icon, {
                                    className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("w-5 h-5 transition-all duration-300 relative z-10", isActive ? "scale-110 text-primary" : "group-hover:scale-110 group-hover:text-primary")
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                    lineNumber: 144,
                                    columnNumber: 17
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                    className: "flex-1 relative z-10",
                                    children: [
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                            className: "font-semibold text-sm",
                                            children: item.name
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                            lineNumber: 154,
                                            columnNumber: 19
                                        }, this),
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                            className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("text-xs transition-opacity", isActive ? "opacity-90" : "opacity-60 group-hover:opacity-80"),
                                            children: item.description
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                            lineNumber: 155,
                                            columnNumber: 19
                                        }, this)
                                    ]
                                }, void 0, true, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                    lineNumber: 153,
                                    columnNumber: 17
                                }, this)
                            ]
                        }, void 0, true, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                            lineNumber: 123,
                            columnNumber: 15
                        }, this)
                    }, item.name, false, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                        lineNumber: 117,
                        columnNumber: 13
                    }, this);
                })
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                lineNumber: 111,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                initial: {
                    opacity: 0,
                    y: 20
                },
                animate: {
                    opacity: 1,
                    y: 0
                },
                transition: {
                    delay: 0.7
                },
                className: "p-4 border-t border-sidebar-border/50 relative z-10",
                children: [
                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["default"], {
                        href: "/settings",
                        className: "flex items-center gap-3 px-4 py-3 rounded-xl text-sidebar-foreground hover:bg-sidebar-accent/50 hover:text-sidebar-accent-foreground transition-all duration-300 group",
                        children: [
                            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$settings$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Settings$3e$__["Settings"], {
                                className: "w-5 h-5 group-hover:rotate-90 transition-transform duration-500"
                            }, void 0, false, {
                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                lineNumber: 183,
                                columnNumber: 11
                            }, this),
                            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                                className: "font-semibold text-sm",
                                children: "Cài Đặt"
                            }, void 0, false, {
                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                lineNumber: 184,
                                columnNumber: 11
                            }, this)
                        ]
                    }, void 0, true, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                        lineNumber: 179,
                        columnNumber: 9
                    }, this),
                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                        initial: {
                            scale: 0.9,
                            opacity: 0
                        },
                        animate: {
                            scale: 1,
                            opacity: 1
                        },
                        transition: {
                            delay: 0.8
                        },
                        className: "mt-4 px-4 py-3 rounded-xl glass-card border-neon-green/30 glow-green",
                        children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                            className: "flex items-center gap-2",
                            children: [
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                                    className: "w-2 h-2 rounded-full bg-neon-green",
                                    animate: {
                                        scale: [
                                            1,
                                            1.2,
                                            1
                                        ],
                                        opacity: [
                                            1,
                                            0.7,
                                            1
                                        ]
                                    },
                                    transition: {
                                        duration: 2,
                                        repeat: Infinity
                                    },
                                    style: {
                                        boxShadow: '0 0 10px var(--neon-green)'
                                    }
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                    lineNumber: 195,
                                    columnNumber: 13
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                                    className: "text-xs font-semibold text-neon-green",
                                    children: "Hệ Thống Trực Tuyến"
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                                    lineNumber: 201,
                                    columnNumber: 13
                                }, this)
                            ]
                        }, void 0, true, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                            lineNumber: 194,
                            columnNumber: 11
                        }, this)
                    }, void 0, false, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                        lineNumber: 188,
                        columnNumber: 9
                    }, this)
                ]
            }, void 0, true, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
                lineNumber: 173,
                columnNumber: 7
            }, this)
        ]
    }, void 0, true, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx",
        lineNumber: 74,
        columnNumber: 5
    }, this);
}
_s(Sidebar, "xbyQPtUVMO7MNj7WjJlpdWqRcTo=", false, function() {
    return [
        __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$navigation$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["usePathname"]
    ];
});
_c = Sidebar;
var _c;
__turbopack_context__.k.register(_c, "Sidebar");
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
"[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "Button",
    ()=>Button,
    "buttonVariants",
    ()=>buttonVariants
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/jsx-dev-runtime.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/index.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f40$radix$2d$ui$2f$react$2d$slot$2f$dist$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/@radix-ui/react-slot/dist/index.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$class$2d$variance$2d$authority$2f$dist$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/class-variance-authority/dist/index.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/framer-motion/dist/es/render/components/motion/proxy.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/lib/utils.ts [app-client] (ecmascript)");
;
var _s = __turbopack_context__.k.signature();
"use client";
;
;
;
;
;
const buttonVariants = (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$class$2d$variance$2d$authority$2f$dist$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cva"])("inline-flex items-center justify-center gap-2 whitespace-nowrap rounded-xl text-sm font-semibold transition-all duration-300 disabled:pointer-events-none disabled:opacity-50 [&_svg]:pointer-events-none [&_svg:not([class*='size-'])]:size-4 shrink-0 [&_svg]:shrink-0 outline-none focus-visible:ring-2 focus-visible:ring-primary/50 focus-visible:ring-offset-2 focus-visible:ring-offset-background relative overflow-hidden group", {
    variants: {
        variant: {
            default: 'bg-linear-to-r from-primary via-primary/90 to-primary text-primary-foreground shadow-lg shadow-primary/25 hover:shadow-xl hover:shadow-primary/40 hover:scale-[1.02] active:scale-[0.98]',
            destructive: 'bg-linear-to-r from-destructive via-destructive/90 to-destructive text-white shadow-lg shadow-destructive/25 hover:shadow-xl hover:shadow-destructive/40 hover:scale-[1.02] active:scale-[0.98]',
            outline: 'border-2 border-border/50 bg-background/50 backdrop-blur-md shadow-sm hover:bg-accent/50 hover:border-primary/50 hover:shadow-md hover:scale-[1.02] active:scale-[0.98]',
            secondary: 'bg-linear-to-r from-secondary via-secondary/90 to-secondary text-secondary-foreground shadow-lg shadow-secondary/25 hover:shadow-xl hover:shadow-secondary/40 hover:scale-[1.02] active:scale-[0.98]',
            ghost: 'hover:bg-accent/50 hover:text-accent-foreground backdrop-blur-sm hover:shadow-md hover:scale-[1.02] active:scale-[0.98]',
            link: 'text-primary underline-offset-4 hover:underline',
            glass: 'bg-white/5 backdrop-blur-xl border border-white/10 text-foreground shadow-lg hover:bg-white/10 hover:border-white/20 hover:shadow-xl hover:scale-[1.02] active:scale-[0.98]',
            gradient: 'bg-linear-to-r from-primary via-accent to-primary bg-[length:200%_100%] text-white shadow-lg shadow-primary/30 hover:shadow-xl hover:shadow-primary/50 hover:scale-[1.02] active:scale-[0.98] hover:bg-[position:100%_0] transition-all duration-500'
        },
        size: {
            default: 'h-11 px-6 py-2.5 has-[>svg]:px-5',
            sm: 'h-9 rounded-lg gap-1.5 px-4 has-[>svg]:px-3.5 text-xs',
            lg: 'h-12 rounded-xl px-8 has-[>svg]:px-6 text-base',
            icon: 'size-11 rounded-xl',
            'icon-sm': 'size-9 rounded-lg',
            'icon-lg': 'size-12 rounded-xl'
        }
    },
    defaultVariants: {
        variant: 'default',
        size: 'default'
    }
});
// Ripple effect component
const Ripple = ({ className })=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].span, {
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("absolute inset-0 rounded-xl", className),
        initial: {
            scale: 0,
            opacity: 0.5
        },
        animate: {
            scale: 4,
            opacity: 0
        },
        transition: {
            duration: 0.6
        }
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx",
        lineNumber: 46,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0));
_c = Ripple;
function Button({ className, variant, size, asChild = false, children, ...props }) {
    _s();
    const Comp = asChild ? __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f40$radix$2d$ui$2f$react$2d$slot$2f$dist$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Slot"] : __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].button;
    const [ripples, setRipples] = __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["useState"]([]);
    const rippleIdRef = __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["useRef"](0);
    const { onDrag, ...restProps } = props;
    const handleClick = (e)=>{
        if (restProps.onClick) {
            restProps.onClick(e);
        }
        // Create ripple effect
        const rect = e.currentTarget.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const newRipple = {
            x,
            y,
            id: rippleIdRef.current++
        };
        setRipples((prev)=>[
                ...prev,
                newRipple
            ]);
        setTimeout(()=>{
            setRipples((prev)=>prev.filter((r)=>r.id !== newRipple.id));
        }, 600);
    };
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(Comp, {
        "data-slot": "button",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])(buttonVariants({
            variant,
            size,
            className
        })),
        onClick: handleClick,
        whileHover: {
            scale: 1.02
        },
        whileTap: {
            scale: 0.98
        },
        transition: {
            type: "spring",
            stiffness: 400,
            damping: 17
        },
        ...restProps,
        children: [
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                className: "absolute inset-0 -translate-x-full translate-y-full bg-linear-to-r from-transparent via-white/20 to-transparent opacity-0 group-hover:translate-x-full group-hover:translate-y-0 group-hover:opacity-100 transition-transform duration-1000 ease-in-out"
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx",
                lineNumber: 100,
                columnNumber: 7
            }, this),
            ripples.map((ripple)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].span, {
                    className: "absolute rounded-full bg-white/30",
                    style: {
                        left: ripple.x,
                        top: ripple.y,
                        width: 20,
                        height: 20,
                        x: -10,
                        y: -10
                    },
                    initial: {
                        scale: 0,
                        opacity: 1
                    },
                    animate: {
                        scale: 4,
                        opacity: 0
                    },
                    transition: {
                        duration: 0.6
                    }
                }, ripple.id, false, {
                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx",
                    lineNumber: 104,
                    columnNumber: 9
                }, this)),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                className: "relative z-10",
                children: children
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx",
                lineNumber: 121,
                columnNumber: 7
            }, this)
        ]
    }, void 0, true, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx",
        lineNumber: 90,
        columnNumber: 5
    }, this);
}
_s(Button, "mtLs6RgEzCVBXYjTwo5PLO87AYM=");
_c1 = Button;
;
var _c, _c1;
__turbopack_context__.k.register(_c, "Ripple");
__turbopack_context__.k.register(_c1, "Button");
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
"[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "MobileNav",
    ()=>MobileNav
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/jsx-dev-runtime.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/index.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/client/app-dir/link.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$navigation$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/navigation.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/framer-motion/dist/es/render/components/motion/proxy.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$components$2f$AnimatePresence$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/framer-motion/dist/es/components/AnimatePresence/index.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/lib/utils.ts [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$layout$2d$dashboard$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__LayoutDashboard$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/layout-dashboard.js [app-client] (ecmascript) <export default as LayoutDashboard>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$video$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Video$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/video.js [app-client] (ecmascript) <export default as Video>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$eye$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Eye$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/eye.js [app-client] (ecmascript) <export default as Eye>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$database$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Database$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/database.js [app-client] (ecmascript) <export default as Database>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$chart$2d$column$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__BarChart3$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/chart-column.js [app-client] (ecmascript) <export default as BarChart3>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$settings$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Settings$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/settings.js [app-client] (ecmascript) <export default as Settings>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$car$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Car$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/car.js [app-client] (ecmascript) <export default as Car>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$brain$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Brain$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/brain.js [app-client] (ecmascript) <export default as Brain>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$triangle$2d$alert$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__AlertTriangle$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/triangle-alert.js [app-client] (ecmascript) <export default as AlertTriangle>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$menu$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Menu$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/menu.js [app-client] (ecmascript) <export default as Menu>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$x$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__X$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/x.js [app-client] (ecmascript) <export default as X>");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$button$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/components/ui/button.tsx [app-client] (ecmascript)");
;
var _s = __turbopack_context__.k.signature();
"use client";
;
;
;
;
;
;
;
const navigation = [
    {
        name: "Bảng Điều Khiển",
        href: "/dashboard",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$layout$2d$dashboard$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__LayoutDashboard$3e$__["LayoutDashboard"],
        description: "Tổng quan hệ thống"
    },
    {
        name: "Phát Hiện Trực Tiếp",
        href: "/",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$video$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Video$3e$__["Video"],
        description: "ADAS thời gian thực"
    },
    {
        name: "Giám Sát ADAS",
        href: "/adas",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$car$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Car$3e$__["Car"],
        description: "Giám sát nâng cao"
    },
    {
        name: "Giám Sát Tài Xế",
        href: "/driver-monitor",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$eye$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Eye$3e$__["Eye"],
        description: "Hành vi tài xế"
    },
    {
        name: "Phân Tích",
        href: "/analytics",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$chart$2d$column$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__BarChart3$3e$__["BarChart3"],
        description: "Số liệu hiệu suất"
    },
    {
        name: "Thu Thập Dữ Liệu",
        href: "/data-collection",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$database$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Database$3e$__["Database"],
        description: "Quản lý dataset"
    },
    {
        name: "Trợ Lý AI",
        href: "/ai-assistant",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$brain$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Brain$3e$__["Brain"],
        description: "Hỗ trợ AI"
    },
    {
        name: "Sự Kiện",
        href: "/events",
        icon: __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$triangle$2d$alert$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__AlertTriangle$3e$__["AlertTriangle"],
        description: "Lịch sử cảnh báo"
    }
];
function MobileNav() {
    _s();
    const [isOpen, setIsOpen] = (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["useState"])(false);
    const pathname = (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$navigation$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["usePathname"])();
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Fragment"], {
        children: [
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                initial: {
                    opacity: 0
                },
                animate: {
                    opacity: 1
                },
                className: "lg:hidden fixed top-4 left-4 z-50",
                children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$button$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Button"], {
                    variant: "outline",
                    size: "icon",
                    onClick: ()=>setIsOpen(!isOpen),
                    className: "bg-white/90 backdrop-blur-md border-gray-200 shadow-lg",
                    children: isOpen ? /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$x$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__X$3e$__["X"], {
                        className: "h-5 w-5"
                    }, void 0, false, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                        lineNumber: 92,
                        columnNumber: 21
                    }, this) : /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$menu$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Menu$3e$__["Menu"], {
                        className: "h-5 w-5"
                    }, void 0, false, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                        lineNumber: 92,
                        columnNumber: 49
                    }, this)
                }, void 0, false, {
                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                    lineNumber: 86,
                    columnNumber: 9
                }, this)
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                lineNumber: 81,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$components$2f$AnimatePresence$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["AnimatePresence"], {
                children: isOpen && /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
                    initial: {
                        opacity: 0
                    },
                    animate: {
                        opacity: 1
                    },
                    exit: {
                        opacity: 0
                    },
                    onClick: ()=>setIsOpen(false),
                    className: "lg:hidden fixed inset-0 bg-black/50 backdrop-blur-sm z-40"
                }, void 0, false, {
                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                    lineNumber: 99,
                    columnNumber: 11
                }, this)
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                lineNumber: 97,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$components$2f$AnimatePresence$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["AnimatePresence"], {
                children: isOpen && /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].aside, {
                    initial: {
                        x: "-100%"
                    },
                    animate: {
                        x: 0
                    },
                    exit: {
                        x: "-100%"
                    },
                    transition: {
                        type: "spring",
                        stiffness: 300,
                        damping: 30
                    },
                    className: "lg:hidden fixed left-0 top-0 bottom-0 w-72 glass-panel border-r border-white/10 shadow-2xl z-50 overflow-y-auto scan-lines",
                    children: [
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                            className: "p-6 border-b border-white/10",
                            children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["default"], {
                                href: "/",
                                className: "flex flex-col items-center gap-3",
                                onClick: ()=>setIsOpen(false),
                                children: [
                                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                        className: "w-24 h-24 rounded-xl overflow-hidden shadow-xl ring-2 ring-neon-cyan/30 glow-pulse-cyan",
                                        children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("img", {
                                            src: "/adas-logo.jpg",
                                            alt: "ADAS Logo",
                                            className: "w-full h-full object-cover"
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                            lineNumber: 127,
                                            columnNumber: 19
                                        }, this)
                                    }, void 0, false, {
                                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                        lineNumber: 126,
                                        columnNumber: 17
                                    }, this),
                                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                        className: "text-center",
                                        children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("p", {
                                            className: "text-xs font-semibold text-neon-cyan",
                                            children: [
                                                "ADAS Platform ",
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                                                    className: "text-fg-secondary",
                                                    children: "• v3.0"
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                                    lineNumber: 135,
                                                    columnNumber: 35
                                                }, this)
                                            ]
                                        }, void 0, true, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                            lineNumber: 134,
                                            columnNumber: 19
                                        }, this)
                                    }, void 0, false, {
                                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                        lineNumber: 133,
                                        columnNumber: 17
                                    }, this)
                                ]
                            }, void 0, true, {
                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                lineNumber: 121,
                                columnNumber: 15
                            }, this)
                        }, void 0, false, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                            lineNumber: 120,
                            columnNumber: 13
                        }, this),
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("nav", {
                            className: "p-4 space-y-2",
                            children: navigation.map((item)=>{
                                const isActive = pathname === item.href;
                                const Icon = item.icon;
                                return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["default"], {
                                    href: item.href,
                                    onClick: ()=>setIsOpen(false),
                                    className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200", isActive ? "glass-card glow-cyan text-neon-cyan border-neon-cyan/50" : "glass-card border-transparent hover:border-neon-cyan/30 hover:text-neon-cyan"),
                                    children: [
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(Icon, {
                                            className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("w-5 h-5", isActive && "text-purple-600")
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                            lineNumber: 159,
                                            columnNumber: 21
                                        }, this),
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                            className: "flex-1",
                                            children: [
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                                    className: "font-semibold text-sm",
                                                    children: item.name
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                                    lineNumber: 161,
                                                    columnNumber: 23
                                                }, this),
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                                    className: "text-xs opacity-60",
                                                    children: item.description
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                                    lineNumber: 162,
                                                    columnNumber: 23
                                                }, this)
                                            ]
                                        }, void 0, true, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                            lineNumber: 160,
                                            columnNumber: 21
                                        }, this)
                                    ]
                                }, item.name, true, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                    lineNumber: 148,
                                    columnNumber: 19
                                }, this);
                            })
                        }, void 0, false, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                            lineNumber: 142,
                            columnNumber: 13
                        }, this),
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                            className: "p-4 border-t border-gray-200 mt-auto",
                            children: [
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$client$2f$app$2d$dir$2f$link$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["default"], {
                                    href: "/settings",
                                    onClick: ()=>setIsOpen(false),
                                    className: "flex items-center gap-3 px-4 py-3 rounded-xl text-gray-700 hover:bg-gray-50 transition-all",
                                    children: [
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$settings$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Settings$3e$__["Settings"], {
                                            className: "w-5 h-5"
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                            lineNumber: 176,
                                            columnNumber: 17
                                        }, this),
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                                            className: "font-semibold text-sm",
                                            children: "Cài Đặt"
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                            lineNumber: 177,
                                            columnNumber: 17
                                        }, this)
                                    ]
                                }, void 0, true, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                    lineNumber: 171,
                                    columnNumber: 15
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                    className: "mt-4 px-4 py-3 rounded-xl bg-green-50 border border-green-200",
                                    children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                        className: "flex items-center gap-2",
                                        children: [
                                            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                                                className: "w-2 h-2 rounded-full bg-green-500"
                                            }, void 0, false, {
                                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                                lineNumber: 182,
                                                columnNumber: 19
                                            }, this),
                                            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("span", {
                                                className: "text-sm font-medium text-green-700",
                                                children: "System Online"
                                            }, void 0, false, {
                                                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                                lineNumber: 183,
                                                columnNumber: 19
                                            }, this)
                                        ]
                                    }, void 0, true, {
                                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                        lineNumber: 181,
                                        columnNumber: 17
                                    }, this)
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                                    lineNumber: 180,
                                    columnNumber: 15
                                }, this)
                            ]
                        }, void 0, true, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                            lineNumber: 170,
                            columnNumber: 13
                        }, this)
                    ]
                }, void 0, true, {
                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                    lineNumber: 112,
                    columnNumber: 11
                }, this)
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx",
                lineNumber: 110,
                columnNumber: 7
            }, this)
        ]
    }, void 0, true);
}
_s(MobileNav, "LvAKcOhSuBLzTufKlhNwaSeG0Po=", false, function() {
    return [
        __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$navigation$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["usePathname"]
    ];
});
_c = MobileNav;
var _c;
__turbopack_context__.k.register(_c, "MobileNav");
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
"[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "Card",
    ()=>Card,
    "CardAction",
    ()=>CardAction,
    "CardContent",
    ()=>CardContent,
    "CardDescription",
    ()=>CardDescription,
    "CardFooter",
    ()=>CardFooter,
    "CardHeader",
    ()=>CardHeader,
    "CardTitle",
    ()=>CardTitle
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/jsx-dev-runtime.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/index.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/framer-motion/dist/es/render/components/motion/proxy.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/lib/utils.ts [app-client] (ecmascript)");
"use client";
;
;
;
;
const Card = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c = ({ className, hover = true, glass = false, children, ...props }, ref)=>{
    const { onDrag, ...restProps } = props;
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$framer$2d$motion$2f$dist$2f$es$2f$render$2f$components$2f$motion$2f$proxy$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["motion"].div, {
        ref: ref,
        "data-slot": "card",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("relative flex flex-col gap-6 rounded-2xl border py-6 overflow-hidden", glass ? "bg-white/5 backdrop-blur-xl border-white/10 shadow-xl" : "bg-card text-card-foreground border-border/50 shadow-lg", hover && "hover:shadow-2xl transition-all duration-300", className),
        initial: {
            opacity: 0,
            y: 20
        },
        animate: {
            opacity: 1,
            y: 0
        },
        transition: {
            duration: 0.3
        },
        whileHover: hover ? {
            y: -4,
            scale: 1.01
        } : undefined,
        ...restProps,
        children: [
            glass && /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Fragment"], {
                children: [
                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                        className: "absolute inset-0 bg-linear-to-br from-white/10 via-transparent to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500"
                    }, void 0, false, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
                        lineNumber: 37,
                        columnNumber: 11
                    }, ("TURBOPACK compile-time value", void 0)),
                    /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                        className: "absolute top-0 left-0 w-full h-px bg-linear-to-r from-transparent via-white/20 to-transparent"
                    }, void 0, false, {
                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
                        lineNumber: 38,
                        columnNumber: 11
                    }, ("TURBOPACK compile-time value", void 0))
                ]
            }, void 0, true),
            !glass && /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                className: "absolute inset-0 rounded-2xl bg-linear-to-br from-white/5 via-transparent to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500 pointer-events-none"
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
                lineNumber: 44,
                columnNumber: 9
            }, ("TURBOPACK compile-time value", void 0)),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                className: "relative z-10",
                children: children
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
                lineNumber: 47,
                columnNumber: 7
            }, ("TURBOPACK compile-time value", void 0))
        ]
    }, void 0, true, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 17,
        columnNumber: 5
    }, ("TURBOPACK compile-time value", void 0));
});
_c1 = Card;
Card.displayName = "Card";
const CardHeader = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c2 = ({ className, ...props }, ref)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        ref: ref,
        "data-slot": "card-header",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("@container/card-header grid auto-rows-min grid-rows-[auto_auto] items-start gap-2 px-6 has-data-[slot=card-action]:grid-cols-[1fr_auto] [.border-b]:pb-6 text-white", className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 57,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0)));
_c3 = CardHeader;
CardHeader.displayName = "CardHeader";
const CardTitle = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c4 = ({ className, ...props }, ref)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        ref: ref,
        "data-slot": "card-title",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("leading-none font-bold text-lg bg-linear-to-r from-foreground to-foreground/80 bg-clip-text text-transparent text-white", className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 73,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0)));
_c5 = CardTitle;
CardTitle.displayName = "CardTitle";
const CardDescription = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c6 = ({ className, ...props }, ref)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        ref: ref,
        "data-slot": "card-description",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("text-muted-foreground text-sm leading-relaxed", className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 89,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0)));
_c7 = CardDescription;
CardDescription.displayName = "CardDescription";
const CardAction = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c8 = ({ className, ...props }, ref)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        ref: ref,
        "data-slot": "card-action",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("col-start-2 row-span-2 row-start-1 self-start justify-self-end", className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 102,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0)));
_c9 = CardAction;
CardAction.displayName = "CardAction";
const CardContent = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c10 = ({ className, ...props }, ref)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        ref: ref,
        "data-slot": "card-content",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("px-6", className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 118,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0)));
_c11 = CardContent;
CardContent.displayName = "CardContent";
const CardFooter = /*#__PURE__*/ __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$index$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["forwardRef"](_c12 = ({ className, ...props }, ref)=>/*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        ref: ref,
        "data-slot": "card-footer",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])("flex items-center px-6 [.border-t]:pt-6", className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx",
        lineNumber: 131,
        columnNumber: 3
    }, ("TURBOPACK compile-time value", void 0)));
_c13 = CardFooter;
CardFooter.displayName = "CardFooter";
;
var _c, _c1, _c2, _c3, _c4, _c5, _c6, _c7, _c8, _c9, _c10, _c11, _c12, _c13;
__turbopack_context__.k.register(_c, "Card$React.forwardRef");
__turbopack_context__.k.register(_c1, "Card");
__turbopack_context__.k.register(_c2, "CardHeader$React.forwardRef");
__turbopack_context__.k.register(_c3, "CardHeader");
__turbopack_context__.k.register(_c4, "CardTitle$React.forwardRef");
__turbopack_context__.k.register(_c5, "CardTitle");
__turbopack_context__.k.register(_c6, "CardDescription$React.forwardRef");
__turbopack_context__.k.register(_c7, "CardDescription");
__turbopack_context__.k.register(_c8, "CardAction$React.forwardRef");
__turbopack_context__.k.register(_c9, "CardAction");
__turbopack_context__.k.register(_c10, "CardContent$React.forwardRef");
__turbopack_context__.k.register(_c11, "CardContent");
__turbopack_context__.k.register(_c12, "CardFooter$React.forwardRef");
__turbopack_context__.k.register(_c13, "CardFooter");
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
"[project]/Desktop/AI/ADAS/FrontEnd/components/ui/alert.tsx [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "Alert",
    ()=>Alert,
    "AlertDescription",
    ()=>AlertDescription,
    "AlertTitle",
    ()=>AlertTitle
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/jsx-dev-runtime.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$class$2d$variance$2d$authority$2f$dist$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/class-variance-authority/dist/index.mjs [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/lib/utils.ts [app-client] (ecmascript)");
;
;
;
const alertVariants = (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$class$2d$variance$2d$authority$2f$dist$2f$index$2e$mjs__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cva"])('relative w-full rounded-lg border px-4 py-3 text-sm grid has-[>svg]:grid-cols-[calc(var(--spacing)*4)_1fr] grid-cols-[0_1fr] has-[>svg]:gap-x-3 gap-y-0.5 items-start [&>svg]:size-4 [&>svg]:translate-y-0.5 [&>svg]:text-current', {
    variants: {
        variant: {
            default: 'bg-card text-card-foreground',
            destructive: 'text-destructive bg-card [&>svg]:text-current *:data-[slot=alert-description]:text-destructive/90'
        }
    },
    defaultVariants: {
        variant: 'default'
    }
});
function Alert({ className, variant, ...props }) {
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        "data-slot": "alert",
        role: "alert",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])(alertVariants({
            variant
        }), className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/alert.tsx",
        lineNumber: 28,
        columnNumber: 5
    }, this);
}
_c = Alert;
function AlertTitle({ className, ...props }) {
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        "data-slot": "alert-title",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])('col-start-2 line-clamp-1 min-h-4 font-medium tracking-tight', className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/alert.tsx",
        lineNumber: 39,
        columnNumber: 5
    }, this);
}
_c1 = AlertTitle;
function AlertDescription({ className, ...props }) {
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        "data-slot": "alert-description",
        className: (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$lib$2f$utils$2e$ts__$5b$app$2d$client$5d$__$28$ecmascript$29$__["cn"])('text-muted-foreground col-start-2 grid justify-items-start gap-1 text-sm [&_p]:leading-relaxed', className),
        ...props
    }, void 0, false, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/components/ui/alert.tsx",
        lineNumber: 55,
        columnNumber: 5
    }, this);
}
_c2 = AlertDescription;
;
var _c, _c1, _c2;
__turbopack_context__.k.register(_c, "Alert");
__turbopack_context__.k.register(_c1, "AlertTitle");
__turbopack_context__.k.register(_c2, "AlertDescription");
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
"[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx [app-client] (ecmascript)", ((__turbopack_context__) => {
"use strict";

__turbopack_context__.s([
    "default",
    ()=>AIAssistant
]);
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/next/dist/compiled/react/jsx-dev-runtime.js [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$sidebar$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/components/sidebar.tsx [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$mobile$2d$nav$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/components/mobile-nav.tsx [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$card$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/components/ui/card.tsx [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$alert$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/components/ui/alert.tsx [app-client] (ecmascript)");
var __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$info$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Info$3e$__ = __turbopack_context__.i("[project]/Desktop/AI/ADAS/FrontEnd/node_modules/lucide-react/dist/esm/icons/info.js [app-client] (ecmascript) <export default as Info>");
"use client";
;
;
;
;
;
;
function AIAssistant() {
    return /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
        className: "flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50",
        children: [
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$mobile$2d$nav$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["MobileNav"], {}, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                lineNumber: 18,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$sidebar$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Sidebar"], {}, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                lineNumber: 19,
                columnNumber: 7
            }, this),
            /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("main", {
                className: "flex-1 overflow-auto",
                children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                    className: "p-4 sm:p-6 lg:p-8",
                    children: [
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("div", {
                            className: "mb-6 sm:mb-8",
                            children: [
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("h1", {
                                    className: "text-2xl sm:text-3xl font-bold text-foreground mb-2",
                                    children: "Trợ Lý AI"
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 24,
                                    columnNumber: 13
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("p", {
                                    className: "text-sm sm:text-base text-foreground/60",
                                    children: "Nhận lời khuyên và hỗ trợ từ trợ lý AI thông minh"
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 25,
                                    columnNumber: 13
                                }, this)
                            ]
                        }, void 0, true, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                            lineNumber: 23,
                            columnNumber: 11
                        }, this),
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$alert$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Alert"], {
                            className: "mb-6 border-yellow-300 bg-yellow-50",
                            children: [
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$lucide$2d$react$2f$dist$2f$esm$2f$icons$2f$info$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__$3c$export__default__as__Info$3e$__["Info"], {
                                    className: "h-4 w-4 text-yellow-600"
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 30,
                                    columnNumber: 13
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$alert$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["AlertTitle"], {
                                    className: "text-yellow-800 font-semibold",
                                    children: "Cần tích hợp AI Backend"
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 31,
                                    columnNumber: 13
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$alert$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["AlertDescription"], {
                                    className: "text-yellow-700",
                                    children: [
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("p", {
                                            className: "mb-2",
                                            children: "Tính năng này cần tích hợp với AI model (GPT/Claude/Llama):"
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                            lineNumber: 33,
                                            columnNumber: 15
                                        }, this),
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("ul", {
                                            className: "list-disc list-inside text-sm space-y-1",
                                            children: [
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: [
                                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("code", {
                                                            className: "bg-yellow-100 px-1 rounded",
                                                            children: "POST /api/ai-chat"
                                                        }, void 0, false, {
                                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                            lineNumber: 35,
                                                            columnNumber: 21
                                                        }, this),
                                                        " - Gửi message và nhận response"
                                                    ]
                                                }, void 0, true, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 35,
                                                    columnNumber: 17
                                                }, this),
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: [
                                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("code", {
                                                            className: "bg-yellow-100 px-1 rounded",
                                                            children: "WebSocket /ws/ai-chat"
                                                        }, void 0, false, {
                                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                            lineNumber: 36,
                                                            columnNumber: 21
                                                        }, this),
                                                        " - Streaming responses"
                                                    ]
                                                }, void 0, true, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 36,
                                                    columnNumber: 17
                                                }, this),
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: [
                                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("code", {
                                                            className: "bg-yellow-100 px-1 rounded",
                                                            children: "GET /api/ai-chat/history"
                                                        }, void 0, false, {
                                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                            lineNumber: 37,
                                                            columnNumber: 21
                                                        }, this),
                                                        " - Lịch sử chat"
                                                    ]
                                                }, void 0, true, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 37,
                                                    columnNumber: 17
                                                }, this)
                                            ]
                                        }, void 0, true, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                            lineNumber: 34,
                                            columnNumber: 15
                                        }, this),
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("p", {
                                            className: "mt-3 text-xs",
                                            children: "Có thể dùng: OpenAI API, Anthropic Claude, hoặc local LLM (Llama, Mistral)"
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                            lineNumber: 39,
                                            columnNumber: 15
                                        }, this)
                                    ]
                                }, void 0, true, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 32,
                                    columnNumber: 13
                                }, this)
                            ]
                        }, void 0, true, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                            lineNumber: 29,
                            columnNumber: 11
                        }, this),
                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$card$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["Card"], {
                            className: "border-indigo-200 bg-indigo-50/30",
                            children: [
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$card$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["CardHeader"], {
                                    children: /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$card$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["CardTitle"], {
                                        className: "text-indigo-700",
                                        children: "Chờ AI Integration"
                                    }, void 0, false, {
                                        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                        lineNumber: 46,
                                        columnNumber: 15
                                    }, this)
                                }, void 0, false, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 45,
                                    columnNumber: 13
                                }, this),
                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])(__TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$components$2f$ui$2f$card$2e$tsx__$5b$app$2d$client$5d$__$28$ecmascript$29$__["CardContent"], {
                                    className: "space-y-3",
                                    children: [
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("p", {
                                            className: "text-sm text-indigo-600",
                                            children: "Tính năng AI Assistant sẽ cung cấp:"
                                        }, void 0, false, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                            lineNumber: 49,
                                            columnNumber: 15
                                        }, this),
                                        /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("ul", {
                                            className: "list-disc list-inside text-sm text-indigo-600 space-y-1",
                                            children: [
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: "Phân tích hành vi lái xe và đưa ra lời khuyên"
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 53,
                                                    columnNumber: 17
                                                }, this),
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: "Giải thích các cảnh báo an toàn"
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 54,
                                                    columnNumber: 17
                                                }, this),
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: "Đề xuất cải thiện kỹ năng lái xe"
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 55,
                                                    columnNumber: 17
                                                }, this),
                                                /*#__PURE__*/ (0, __TURBOPACK__imported__module__$5b$project$5d2f$Desktop$2f$AI$2f$ADAS$2f$FrontEnd$2f$node_modules$2f$next$2f$dist$2f$compiled$2f$react$2f$jsx$2d$dev$2d$runtime$2e$js__$5b$app$2d$client$5d$__$28$ecmascript$29$__["jsxDEV"])("li", {
                                                    children: "Trả lời câu hỏi về ADAS features"
                                                }, void 0, false, {
                                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                                    lineNumber: 56,
                                                    columnNumber: 17
                                                }, this)
                                            ]
                                        }, void 0, true, {
                                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                            lineNumber: 52,
                                            columnNumber: 15
                                        }, this)
                                    ]
                                }, void 0, true, {
                                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                                    lineNumber: 48,
                                    columnNumber: 13
                                }, this)
                            ]
                        }, void 0, true, {
                            fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                            lineNumber: 44,
                            columnNumber: 11
                        }, this)
                    ]
                }, void 0, true, {
                    fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                    lineNumber: 22,
                    columnNumber: 9
                }, this)
            }, void 0, false, {
                fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
                lineNumber: 21,
                columnNumber: 7
            }, this)
        ]
    }, void 0, true, {
        fileName: "[project]/Desktop/AI/ADAS/FrontEnd/app/ai-assistant/page.tsx",
        lineNumber: 17,
        columnNumber: 5
    }, this);
}
_c = AIAssistant;
var _c;
__turbopack_context__.k.register(_c, "AIAssistant");
if (typeof globalThis.$RefreshHelpers$ === 'object' && globalThis.$RefreshHelpers !== null) {
    __turbopack_context__.k.registerExports(__turbopack_context__.m, globalThis.$RefreshHelpers$);
}
}),
]);

//# sourceMappingURL=Desktop_AI_ADAS_FrontEnd_b30f7c1f._.js.map