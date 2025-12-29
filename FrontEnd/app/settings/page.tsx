"use client"

import { useEffect, useRef, useState } from "react"
import { Sidebar } from "@/components/sidebar"
import { MobileNav } from "@/components/mobile-nav"
import { GlassCard } from "@/components/ui/glass-card"
import { Badge } from "@/components/ui/badge"
import Highcharts from "highcharts"
import HighchartsReact from "highcharts-react-official"
import {
    Settings,
    Monitor,
    Bell,
    Brain,
    Camera,
    Sliders,
    Volume2,
    Mail,
    Smartphone,
    Gauge,
    Save,
    RotateCcw,
    ChevronDown,
    TrendingUp,
    Activity,
    Zap
} from "lucide-react"

export default function SettingsPage() {
    // System Settings
    const [language, setLanguage] = useState("vi")
    const [theme, setTheme] = useState("dark")
    const [timezone, setTimezone] = useState("Asia/Ho_Chi_Minh")
    const [autoSave, setAutoSave] = useState(true)

    // Display Settings
    const [videoQuality, setVideoQuality] = useState("1080p")
    const [showFPS, setShowFPS] = useState(true)
    const [overlayOpacity, setOverlayOpacity] = useState(80)
    const [showHUD, setShowHUD] = useState(true)

    // Notification Settings
    const [alertSound, setAlertSound] = useState(true)
    const [warningThreshold, setWarningThreshold] = useState(75)
    const [emailNotif, setEmailNotif] = useState(false)
    const [pushNotif, setPushNotif] = useState(true)

    // AI Assistant Settings
    const [aiEnabled, setAiEnabled] = useState(true)
    const [aiLanguage, setAiLanguage] = useState("vi")
    const [voiceFeedback, setVoiceFeedback] = useState(false)
    const [autoSuggestions, setAutoSuggestions] = useState(true)

    // Detection Settings
    const [confidenceThreshold, setConfidenceThreshold] = useState(70)
    const [frameSkip, setFrameSkip] = useState(0)
    const [recordingQuality, setRecordingQuality] = useState("high")

    // Advanced Settings
    const [debugMode, setDebugMode] = useState(false)
    const [perfMonitoring, setPerfMonitoring] = useState(true)
    const [dataRetention, setDataRetention] = useState(30)

    const [hasChanges, setHasChanges] = useState(false)
    const [showSaveConfirm, setShowSaveConfirm] = useState(false)

    const handleSave = () => {
        setShowSaveConfirm(true)
        setTimeout(() => {
            setShowSaveConfirm(false)
            setHasChanges(false)
        }, 2000)
    }

    const handleReset = () => {
        setLanguage("vi")
        setTheme("dark")
        setVideoQuality("1080p")
        setShowFPS(true)
        setOverlayOpacity(80)
        setAlertSound(true)
        setWarningThreshold(75)
        setAiEnabled(true)
        setConfidenceThreshold(70)
        setHasChanges(false)
    }

    // Highcharts configurations
    const systemPerformanceOptions: Highcharts.Options = {
        chart: {
          type: "pie",
          backgroundColor: "transparent",
          height: 280,
          spacingBottom: 50, // chừa chỗ để chữ "Hiệu suất" nằm dưới vòng tròn
          events: {
            render: function () {
              const chart = this as Highcharts.Chart;
      
              const series = chart.series?.[0] as Highcharts.Series | undefined;
              if (!series) return;
      
              // Pie series có center = [x, y, diameter] nhưng typings không expose -> cast any
              const center = (series as any).center as [number, number, number] | undefined;
              if (!center) return;
      
              const [cx, cy, diameter] = center;
      
              const centerX = chart.plotLeft + cx;
              const centerY = chart.plotTop + cy;
      
              const value =
                (series.points?.[0] as any)?.y ??
                ((series.options as any)?.data?.[0] as any)?.y ??
                0;
      
              const anyChart = chart as any;
              if (!anyChart.__perfTexts) anyChart.__perfTexts = {};
      
              // 85% ở giữa vòng tròn
              if (!anyChart.__perfTexts.percent) {
                anyChart.__perfTexts.percent = chart.renderer
                  .text("", 0, 0)
                  .css({
                    fontSize: "32px",
                    fontWeight: "700",
                    fontFamily: "var(--font-orbitron)",
                    color: "#00E5FF",
                    textOutline: "none",
                  })
                  .attr({
                    zIndex: 10,
                    "text-anchor": "middle",
                  })
                  .add();
              }
      
              anyChart.__perfTexts.percent.attr({
                text: `${value}%`,
                x: centerX,
                y: centerY + 12, // canh giữa theo mắt
              });
      
              // "Hiệu suất" nằm dưới vòng tròn (ngoài ring)
              const labelY = centerY + diameter / 2 + 26;
      
              if (!anyChart.__perfTexts.label) {
                anyChart.__perfTexts.label = chart.renderer
                  .text("Hiệu suất", 0, 0)
                  .css({
                    fontSize: "12px",
                    fontWeight: "500",
                    fontFamily: "var(--font-inter)",
                    color: "#BAE6FD",
                    textOutline: "none",
                  })
                  .attr({
                    zIndex: 10,
                    "text-anchor": "middle",
                  })
                  .add();
              }
      
              anyChart.__perfTexts.label.attr({
                x: centerX,
                y: labelY,
              });
            },
          },
        },
      
        title: {
          text: "Hiệu Suất Hệ Thống",
          style: {
            color: "#00E5FF",
            fontFamily: "var(--font-inter)",
            fontSize: "16px",
            fontWeight: "600",
          },
        },
      
        tooltip: {
          pointFormat: "<b>{point.percentage:.1f}%</b>",
          backgroundColor: "rgba(10, 22, 40, 0.95)",
          borderColor: "#00E5FF",
          borderRadius: 8,
          style: {
            color: "#FFFFFF",
            fontFamily: "var(--font-inter)",
            fontSize: "12px",
          },
        },
      
        plotOptions: {
          pie: {
            innerSize: "70%",
            borderWidth: 0,
            dataLabels: {
              enabled: false, // tắt để không vẽ chữ trong chart nữa
            },
            states: {
              hover: { enabled: false },
            },
            // Đẩy donut lên chút để có chỗ cho chữ ở dưới
            center: ["50%", "45%"],
          },
        },
      
        series: [
          {
            type: "pie",
            name: "Performance",
            data: [
              { y: 85, color: "#00FFA3" },
              { y: 15, color: "rgba(255, 255, 255, 0.05)" },
            ],
            enableMouseTracking: false,
          },
        ],
      
        credits: {
          enabled: false,
        },
    };
    const detectionStatsOptions = {
        chart: {
            type: 'area',
            backgroundColor: 'transparent',
            height: 280
        },
        title: {
            text: 'Thống Kê Phát Hiện 7 Ngày',
            style: {
                color: '#00E5FF',
                fontFamily: 'var(--font-inter)',
                fontSize: '16px',
                fontWeight: '600'
            }
        },
        xAxis: {
            categories: ['T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'CN'],
            labels: {
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '11px'
                }
            },
            lineColor: 'rgba(255, 255, 255, 0.1)',
            tickColor: 'rgba(255, 255, 255, 0.1)'
        },
        yAxis: {
            title: {
                text: 'Số lượng',
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '12px'
                }
            },
            labels: {
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '11px'
                }
            },
            gridLineColor: 'rgba(255, 255, 255, 0.05)'
        },
        tooltip: {
            shared: true,
            backgroundColor: 'rgba(10, 22, 40, 0.95)',
            borderColor: '#00E5FF',
            borderRadius: 8,
            style: {
                color: '#FFFFFF',
                fontFamily: 'var(--font-inter)',
                fontSize: '12px'
            }
        },
        plotOptions: {
            area: {
                fillOpacity: 0.3,
                marker: {
                    radius: 4,
                    lineWidth: 2
                }
            }
        },
        series: [{
            name: 'Xe',
            data: [245, 312, 289, 356, 401, 378, 423],
            color: '#00E5FF',
            fillColor: {
                linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                stops: [
                    [0, 'rgba(0, 229, 255, 0.3)'],
                    [1, 'rgba(0, 229, 255, 0.05)']
                ]
            }
        }, {
            name: 'Người',
            data: [156, 189, 201, 234, 267, 245, 289],
            color: '#00FFA3',
            fillColor: {
                linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                stops: [
                    [0, 'rgba(0, 255, 163, 0.3)'],
                    [1, 'rgba(0, 255, 163, 0.05)']
                ]
            }
        }],
        legend: {
            itemStyle: {
                color: '#BAE6FD',
                fontFamily: 'var(--font-inter)',
                fontSize: '12px',
                fontWeight: '500'
            },
            itemHoverStyle: {
                color: '#FFFFFF'
            }
        },
        credits: {
            enabled: false
        }
    }

    const aiUsageOptions = {
        chart: {
            type: 'pie',
            backgroundColor: 'transparent',
            height: 280
        },
        title: {
            text: 'Sử Dụng AI Assistant',
            style: {
                color: '#00E5FF',
                fontFamily: 'var(--font-inter)',
                fontSize: '16px',
                fontWeight: '600'
            }
        },
        tooltip: {
            backgroundColor: 'rgba(10, 22, 40, 0.95)',
            borderColor: '#00E5FF',
            borderRadius: 8,
            style: {
                color: '#FFFFFF',
                fontFamily: 'var(--font-inter)',
                fontSize: '12px'
            },
            pointFormat: '<b>{point.percentage:.1f}%</b><br/>Số lượng: {point.y}'
        },
        plotOptions: {
            pie: {
                innerSize: '60%',
                depth: 45,
                dataLabels: {
                    enabled: true,
                    format: '<b>{point.name}</b><br>{point.percentage:.1f}%',
                    style: {
                        color: '#FFFFFF',
                        fontFamily: 'var(--font-inter)',
                        fontSize: '11px',
                        fontWeight: '500',
                        textOutline: 'none'
                    },
                    distance: 10
                },
                borderWidth: 2,
                borderColor: '#050B14'
            }
        },
        series: [{
            name: 'Queries',
            data: [
                { name: 'Hỗ trợ lái xe', y: 456, color: '#00E5FF' },
                { name: 'Phân tích dữ liệu', y: 289, color: '#00FFA3' },
                { name: 'Cảnh báo', y: 178, color: '#FFD700' },
                { name: 'Khác', y: 123, color: '#B794F6' }
            ]
        }],
        credits: {
            enabled: false
        }
    }

    const confidenceDistOptions = {
        chart: {
            type: 'column',
            backgroundColor: 'transparent',
            height: 280
        },
        title: {
            text: 'Phân Bố Độ Tin Cậy',
            style: {
                color: '#00E5FF',
                fontFamily: 'var(--font-inter)',
                fontSize: '16px',
                fontWeight: '600'
            }
        },
        xAxis: {
            categories: ['50-60%', '60-70%', '70-80%', '80-90%', '90-100%'],
            labels: {
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '11px'
                }
            },
            lineColor: 'rgba(255, 255, 255, 0.1)',
            tickColor: 'rgba(255, 255, 255, 0.1)'
        },
        yAxis: {
            title: {
                text: 'Số lượng phát hiện',
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '12px'
                }
            },
            labels: {
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '11px'
                }
            },
            gridLineColor: 'rgba(255, 255, 255, 0.05)'
        },
        tooltip: {
            backgroundColor: 'rgba(10, 22, 40, 0.95)',
            borderColor: '#00E5FF',
            borderRadius: 8,
            style: {
                color: '#FFFFFF',
                fontFamily: 'var(--font-inter)',
                fontSize: '12px'
            }
        },
        plotOptions: {
            column: {
                borderRadius: 4,
                borderWidth: 0,
                dataLabels: {
                    enabled: true,
                    style: {
                        color: '#FFFFFF',
                        fontFamily: 'var(--font-inter)',
                        fontSize: '11px',
                        fontWeight: '600',
                        textOutline: 'none'
                    }
                }
            }
        },
        series: [{
            name: 'Phát hiện',
            data: [89, 234, 567, 892, 1245],
            color: {
                linearGradient: { x1: 0, y1: 0, x2: 0, y2: 1 },
                stops: [
                    [0, '#00E5FF'],
                    [1, '#00FFA3']
                ]
            }
        }],
        legend: {
            enabled: false
        },
        credits: {
            enabled: false
        }
    }

    const notificationTimelineOptions = {
        chart: {
            type: 'spline',
            backgroundColor: 'transparent',
            height: 280
        },
        title: {
            text: 'Lịch Sử Thông Báo 24h',
            style: {
                color: '#00E5FF',
                fontFamily: 'var(--font-inter)',
                fontSize: '16px',
                fontWeight: '600'
            }
        },
        xAxis: {
            categories: ['0h', '3h', '6h', '9h', '12h', '15h', '18h', '21h', '24h'],
            labels: {
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '11px'
                }
            },
            lineColor: 'rgba(255, 255, 255, 0.1)',
            tickColor: 'rgba(255, 255, 255, 0.1)'
        },
        yAxis: {
            title: {
                text: 'Số thông báo',
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '12px'
                }
            },
            labels: {
                style: {
                    color: '#BAE6FD',
                    fontFamily: 'var(--font-inter)',
                    fontSize: '11px'
                }
            },
            gridLineColor: 'rgba(255, 255, 255, 0.05)'
        },
        tooltip: {
            backgroundColor: 'rgba(10, 22, 40, 0.95)',
            borderColor: '#00E5FF',
            borderRadius: 8,
            style: {
                color: '#FFFFFF',
                fontFamily: 'var(--font-inter)',
                fontSize: '12px'
            },
            shared: true
        },
        plotOptions: {
            spline: {
                marker: {
                    radius: 4,
                    lineColor: '#050B14',
                    lineWidth: 2
                }
            }
        },
        series: [{
            name: 'Cảnh báo',
            data: [12, 8, 15, 23, 34, 28, 19, 25, 18],
            color: '#FFD700',
            marker: {
                symbol: 'circle'
            }
        }, {
            name: 'Nguy hiểm',
            data: [3, 2, 5, 8, 12, 9, 6, 7, 4],
            color: '#FF3B3B',
            marker: {
                symbol: 'circle'
            }
        }],
        legend: {
            itemStyle: {
                color: '#BAE6FD',
                fontFamily: 'var(--font-inter)',
                fontSize: '12px',
                fontWeight: '500'
            },
            itemHoverStyle: {
                color: '#FFFFFF'
            }
        },
        credits: {
            enabled: false
        }
    }

    return (
        <div className="flex h-screen bg-bg-primary">
            <MobileNav />
            <Sidebar />

            <main className="flex-1 overflow-auto">
                <div className="p-4 sm:p-6 lg:p-8 space-y-6" style={{ fontFamily: 'var(--font-inter)' }}>
                    {/* Header */}
                    <div className="flex items-center justify-between">
                        <div>
                            <h1 className="text-3xl font-bold text-neon-cyan tracking-wider" style={{ fontFamily: 'var(--font-orbitron)' }}>CÀI ĐẶT</h1>
                            <p className="text-sm text-fg-secondary mt-1 font-medium">
                                Tùy chỉnh hệ thống ADAS theo nhu cầu của bạn
                            </p>
                        </div>
                        <div className="flex gap-3">
                            <button
                                onClick={handleReset}
                                className="glass-card border-neon-yellow/50 text-neon-yellow px-4 py-2 rounded-lg hover:glow-yellow transition-all flex items-center gap-2 font-semibold"
                            >
                                <RotateCcw className="w-4 h-4" />
                                <span className="hidden sm:inline">Đặt Lại</span>
                            </button>
                            <button
                                onClick={handleSave}
                                disabled={!hasChanges}
                                className={`glass-card px-4 py-2 rounded-lg transition-all flex items-center gap-2 font-semibold ${hasChanges
                                    ? "border-neon-green/50 text-neon-green hover:glow-green"
                                    : "border-white/10 text-fg-muted cursor-not-allowed"
                                    }`}
                            >
                                <Save className="w-4 h-4" />
                                <span className="hidden sm:inline">Lưu</span>
                            </button>
                        </div>
                    </div>

                    {/* Save Confirmation */}
                    {showSaveConfirm && (
                        <div className="glass-card border-neon-green/50 glow-green p-4 flex items-center gap-3">
                            <div className="w-2 h-2 rounded-full bg-neon-green animate-pulse" />
                            <span className="text-neon-green font-semibold">Đã lưu cài đặt thành công!</span>
                        </div>
                    )}

                    {/* Analytics Dashboard */}
                    <div className="grid gap-6 grid-cols-1 lg:grid-cols-3">
                        <GlassCard className="p-6">
                            <HighchartsReact highcharts={Highcharts} options={systemPerformanceOptions} />
                        </GlassCard>
                        <GlassCard className="p-6 lg:col-span-2">
                            <HighchartsReact highcharts={Highcharts} options={detectionStatsOptions} />
                        </GlassCard>
                    </div>

                    <div className="grid gap-6 grid-cols-1 lg:grid-cols-2">
                        <GlassCard className="p-6">
                            <HighchartsReact highcharts={Highcharts} options={aiUsageOptions} />
                        </GlassCard>
                        <GlassCard className="p-6">
                            <HighchartsReact highcharts={Highcharts} options={confidenceDistOptions} />
                        </GlassCard>
                    </div>

                    <GlassCard className="p-6">
                        <HighchartsReact highcharts={Highcharts} options={notificationTimelineOptions} />
                    </GlassCard>

                    {/* Quick Stats */}
                    <div className="grid gap-4 grid-cols-1 sm:grid-cols-2 lg:grid-cols-4">
                        <GlassCard glow="cyan" className="p-6">
                            <div className="flex items-center justify-between mb-3">
                                <TrendingUp className="w-5 h-5 text-neon-cyan" />
                                <Badge className="glass-card border-neon-cyan/30 text-neon-cyan text-xs font-semibold">+12%</Badge>
                            </div>
                            <div className="digital-number text-2xl font-bold text-neon-cyan mb-1">3,247</div>
                            <p className="text-xs text-fg-secondary font-medium">Phát hiện hôm nay</p>
                        </GlassCard>

                        <GlassCard glow="green" className="p-6">
                            <div className="flex items-center justify-between mb-3">
                                <Activity className="w-5 h-5 text-neon-green" />
                                <Badge className="glass-card border-neon-green/30 text-neon-green text-xs font-semibold">Tốt</Badge>
                            </div>
                            <div className="digital-number text-2xl font-bold text-neon-green mb-1">98.5%</div>
                            <p className="text-xs text-fg-secondary font-medium">Độ chính xác</p>
                        </GlassCard>

                        <GlassCard glow="yellow" className="p-6">
                            <div className="flex items-center justify-between mb-3">
                                <Brain className="w-5 h-5 text-neon-yellow" />
                                <Badge className="glass-card border-neon-yellow/30 text-neon-yellow text-xs font-semibold">Active</Badge>
                            </div>
                            <div className="digital-number text-2xl font-bold text-neon-yellow mb-1">1,046</div>
                            <p className="text-xs text-fg-secondary font-medium">AI Queries</p>
                        </GlassCard>

                        <GlassCard className="p-6 border-neon-purple/30">
                            <div className="flex items-center justify-between mb-3">
                                <Zap className="w-5 h-5 text-neon-purple" />
                                <Badge className="glass-card border-neon-purple/30 text-neon-purple text-xs font-semibold">85%</Badge>
                            </div>
                            <div className="digital-number text-2xl font-bold text-neon-purple mb-1">24 FPS</div>
                            <p className="text-xs text-fg-secondary font-medium">Hiệu suất</p>
                        </GlassCard>
                    </div>

                    {/* System Settings */}
                    <GlassCard scanLines className="p-6">
                        <div className="flex items-center gap-3 mb-6">
                            <Settings className="w-6 h-6 text-neon-cyan" />
                            <h2 className="text-xl font-bold text-neon-cyan tracking-wide" style={{ fontFamily: 'var(--font-orbitron)' }}>HỆ THỐNG</h2>
                        </div>
                        <div className="grid gap-6 md:grid-cols-2">
                            <div className="space-y-2">
                                <label className="text-sm font-semibold text-fg-primary">Ngôn Ngữ</label>
                                <div className="relative">
                                    <select
                                        value={language}
                                        onChange={(e) => { setLanguage(e.target.value); setHasChanges(true) }}
                                        className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                                    >
                                        <option value="vi">Tiếng Việt</option>
                                        <option value="en">English</option>
                                    </select>
                                    <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                                </div>
                                <p className="text-xs text-fg-muted font-medium">Ngôn ngữ hiển thị giao diện</p>
                            </div>

                            <div className="space-y-2">
                                <label className="text-sm font-semibold text-fg-primary">Chế Độ Giao Diện</label>
                                <div className="relative">
                                    <select
                                        value={theme}
                                        onChange={(e) => { setTheme(e.target.value); setHasChanges(true) }}
                                        className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                                    >
                                        <option value="dark">Tối</option>
                                        <option value="auto">Tự Động</option>
                                    </select>
                                    <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                                </div>
                                <p className="text-xs text-fg-muted font-medium">Chế độ hiển thị màn hình</p>
                            </div>

                            <div className="space-y-2">
                                <label className="text-sm font-semibold text-fg-primary">Múi Giờ</label>
                                <div className="relative">
                                    <select
                                        value={timezone}
                                        onChange={(e) => { setTimezone(e.target.value); setHasChanges(true) }}
                                        className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                                    >
                                        <option value="Asia/Ho_Chi_Minh">GMT+7 (Việt Nam)</option>
                                        <option value="Asia/Tokyo">GMT+9 (Tokyo)</option>
                                        <option value="America/New_York">GMT-5 (New York)</option>
                                    </select>
                                    <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                                </div>
                                <p className="text-xs text-fg-muted font-medium">Múi giờ hiển thị</p>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Tự Động Lưu</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">Lưu cài đặt tự động</p>
                                    </div>
                                    <button
                                        onClick={() => { setAutoSave(!autoSave); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${autoSave ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${autoSave ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={autoSave ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>
                        </div>
                    </GlassCard>

                    {/* Display Settings */}
                    <GlassCard scanLines className="p-6">
                        <div className="flex items-center gap-3 mb-6">
                            <Monitor className="w-6 h-6 text-neon-cyan" />
                            <h2 className="text-xl font-bold text-neon-cyan tracking-wide" style={{ fontFamily: 'var(--font-orbitron)' }}>HIỂN THỊ</h2>
                        </div>
                        <div className="grid gap-6 md:grid-cols-2">
                            <div className="space-y-2">
                                <label className="text-sm font-semibold text-fg-primary">Chất Lượng Video</label>
                                <div className="relative">
                                    <select
                                        value={videoQuality}
                                        onChange={(e) => { setVideoQuality(e.target.value); setHasChanges(true) }}
                                        className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                                    >
                                        <option value="720p">720p (HD)</option>
                                        <option value="1080p">1080p (Full HD)</option>
                                        <option value="4k">4K (Ultra HD)</option>
                                    </select>
                                    <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                                </div>
                                <p className="text-xs text-fg-muted font-medium">Độ phân giải video stream</p>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Hiển Thị FPS</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">Hiện số khung hình/giây</p>
                                    </div>
                                    <button
                                        onClick={() => { setShowFPS(!showFPS); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${showFPS ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${showFPS ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={showFPS ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-3 md:col-span-2">
                                <div className="flex items-center justify-between">
                                    <label className="text-sm font-semibold text-fg-primary">Độ Mờ Overlay</label>
                                    <span className="text-sm font-bold text-neon-cyan digital-number">{overlayOpacity}%</span>
                                </div>
                                <input
                                    type="range"
                                    min="0"
                                    max="100"
                                    step="5"
                                    value={overlayOpacity}
                                    onChange={(e) => { setOverlayOpacity(Number(e.target.value)); setHasChanges(true) }}
                                    className="w-full h-2 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-neon-cyan [&::-webkit-slider-thumb]:cursor-pointer"
                                    style={{
                                        background: `linear-gradient(to right, var(--neon-cyan) 0%, var(--neon-cyan) ${overlayOpacity}%, rgba(255,255,255,0.1) ${overlayOpacity}%, rgba(255,255,255,0.1) 100%)`
                                    }}
                                />
                                <p className="text-xs text-fg-muted font-medium">Độ trong suốt của lớp phát hiện</p>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Hiển Thị HUD</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">Hiện thông tin trên màn hình</p>
                                    </div>
                                    <button
                                        onClick={() => { setShowHUD(!showHUD); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${showHUD ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${showHUD ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={showHUD ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>
                        </div>
                    </GlassCard>

                    {/* Notification Settings */}
                    <GlassCard scanLines className="p-6">
                        <div className="flex items-center gap-3 mb-6">
                            <Bell className="w-6 h-6 text-neon-cyan" />
                            <h2 className="text-xl font-bold text-neon-cyan tracking-wide" style={{ fontFamily: 'var(--font-orbitron)' }}>THÔNG BÁO</h2>
                        </div>
                        <div className="grid gap-6 md:grid-cols-2">
                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div className="flex items-center gap-2">
                                        <Volume2 className="w-4 h-4 text-fg-secondary" />
                                        <div>
                                            <label className="text-sm font-semibold text-fg-primary">Âm Thanh Cảnh Báo</label>
                                            <p className="text-xs text-fg-muted mt-1 font-medium">Phát âm thanh khi có cảnh báo</p>
                                        </div>
                                    </div>
                                    <button
                                        onClick={() => { setAlertSound(!alertSound); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${alertSound ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${alertSound ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={alertSound ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-3">
                                <div className="flex items-center justify-between">
                                    <label className="text-sm font-semibold text-fg-primary">Ngưỡng Cảnh Báo</label>
                                    <span className="text-sm font-bold text-neon-yellow digital-number">{warningThreshold}%</span>
                                </div>
                                <input
                                    type="range"
                                    min="50"
                                    max="100"
                                    step="5"
                                    value={warningThreshold}
                                    onChange={(e) => { setWarningThreshold(Number(e.target.value)); setHasChanges(true) }}
                                    className="w-full h-2 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-neon-yellow [&::-webkit-slider-thumb]:cursor-pointer"
                                    style={{
                                        background: `linear-gradient(to right, var(--neon-yellow) 0%, var(--neon-yellow) ${(warningThreshold - 50) * 2}%, rgba(255,255,255,0.1) ${(warningThreshold - 50) * 2}%, rgba(255,255,255,0.1) 100%)`
                                    }}
                                />
                                <p className="text-xs text-fg-muted font-medium">Mức độ tin cậy tối thiểu để cảnh báo</p>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div className="flex items-center gap-2">
                                        <Mail className="w-4 h-4 text-fg-secondary" />
                                        <div>
                                            <label className="text-sm font-semibold text-fg-primary">Email</label>
                                            <p className="text-xs text-fg-muted mt-1 font-medium">Gửi thông báo qua email</p>
                                        </div>
                                    </div>
                                    <button
                                        onClick={() => { setEmailNotif(!emailNotif); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${emailNotif ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${emailNotif ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={emailNotif ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div className="flex items-center gap-2">
                                        <Smartphone className="w-4 h-4 text-fg-secondary" />
                                        <div>
                                            <label className="text-sm font-semibold text-fg-primary">Push Notification</label>
                                            <p className="text-xs text-fg-muted mt-1 font-medium">Thông báo đẩy trên thiết bị</p>
                                        </div>
                                    </div>
                                    <button
                                        onClick={() => { setPushNotif(!pushNotif); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${pushNotif ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${pushNotif ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={pushNotif ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>
                        </div>
                    </GlassCard>

                    {/* AI Assistant Settings */}
                    <GlassCard scanLines className="p-6">
                        <div className="flex items-center gap-3 mb-6">
                            <Brain className="w-6 h-6 text-neon-cyan" />
                            <h2 className="text-xl font-bold text-neon-cyan tracking-wide" style={{ fontFamily: 'var(--font-orbitron)' }}>TRỢ LÝ AI</h2>
                        </div>
                        <div className="grid gap-6 md:grid-cols-2">
                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Kích Hoạt AI</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">Bật/tắt trợ lý AI</p>
                                    </div>
                                    <button
                                        onClick={() => { setAiEnabled(!aiEnabled); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${aiEnabled ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${aiEnabled ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={aiEnabled ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-2">
                                <label className="text-sm font-semibold text-fg-primary">Ngôn Ngữ Phản Hồi</label>
                                <div className="relative">
                                    <select
                                        value={aiLanguage}
                                        onChange={(e) => { setAiLanguage(e.target.value); setHasChanges(true) }}
                                        className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                                        disabled={!aiEnabled}
                                    >
                                        <option value="vi">Tiếng Việt</option>
                                        <option value="en">English</option>
                                        <option value="auto">Tự Động</option>
                                    </select>
                                    <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                                </div>
                                <p className="text-xs text-fg-muted font-medium">Ngôn ngữ trả lời của AI</p>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Phản Hồi Giọng Nói</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">AI trả lời bằng giọng nói</p>
                                    </div>
                                    <button
                                        onClick={() => { setVoiceFeedback(!voiceFeedback); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${voiceFeedback ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                        disabled={!aiEnabled}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${voiceFeedback ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={voiceFeedback ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Gợi Ý Tự Động</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">AI đề xuất hành động</p>
                                    </div>
                                    <button
                                        onClick={() => { setAutoSuggestions(!autoSuggestions); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${autoSuggestions ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                        disabled={!aiEnabled}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${autoSuggestions ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={autoSuggestions ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>
                        </div>
                    </GlassCard>

                    {/* Detection Settings */}
                    <GlassCard scanLines className="p-6">
                        <div className="flex items-center gap-3 mb-6">
                            <Camera className="w-6 h-6 text-neon-cyan" />
                            <h2 className="text-xl font-bold text-neon-cyan tracking-wide" style={{ fontFamily: 'var(--font-orbitron)' }}>PHÁT HIỆN</h2>
                        </div>
                        <div className="grid gap-6 md:grid-cols-2">
                            <div className="space-y-3 md:col-span-2">
                                <div className="flex items-center justify-between">
                                    <label className="text-sm font-semibold text-fg-primary">Ngưỡng Tin Cậy</label>
                                    <span className="text-sm font-bold text-neon-green digital-number">{confidenceThreshold}%</span>
                                </div>
                                <input
                                    type="range"
                                    min="50"
                                    max="95"
                                    step="5"
                                    value={confidenceThreshold}
                                    onChange={(e) => { setConfidenceThreshold(Number(e.target.value)); setHasChanges(true) }}
                                    className="w-full h-2 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-neon-green [&::-webkit-slider-thumb]:cursor-pointer"
                                    style={{
                                        background: `linear-gradient(to right, var(--neon-green) 0%, var(--neon-green) ${(confidenceThreshold - 50) * (100 / 45)}%, rgba(255,255,255,0.1) ${(confidenceThreshold - 50) * (100 / 45)}%, rgba(255,255,255,0.1) 100%)`
                                    }}
                                />
                                <p className="text-xs text-fg-muted font-medium">Độ tin cậy tối thiểu để hiển thị phát hiện</p>
                            </div>

                            <div className="space-y-3">
                                <div className="flex items-center justify-between">
                                    <label className="text-sm font-semibold text-fg-primary">Bỏ Qua Khung Hình</label>
                                    <span className="text-sm font-bold text-neon-cyan digital-number">{frameSkip}</span>
                                </div>
                                <input
                                    type="range"
                                    min="0"
                                    max="5"
                                    step="1"
                                    value={frameSkip}
                                    onChange={(e) => { setFrameSkip(Number(e.target.value)); setHasChanges(true) }}
                                    className="w-full h-2 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-neon-cyan [&::-webkit-slider-thumb]:cursor-pointer"
                                    style={{
                                        background: `linear-gradient(to right, var(--neon-cyan) 0%, var(--neon-cyan) ${frameSkip * 20}%, rgba(255,255,255,0.1) ${frameSkip * 20}%, rgba(255,255,255,0.1) 100%)`
                                    }}
                                />
                                <p className="text-xs text-fg-muted font-medium">Số khung hình bỏ qua để tăng hiệu suất</p>
                            </div>

                            <div className="space-y-2">
                                <label className="text-sm font-semibold text-fg-primary">Chất Lượng Ghi Hình</label>
                                <div className="relative">
                                    <select
                                        value={recordingQuality}
                                        onChange={(e) => { setRecordingQuality(e.target.value); setHasChanges(true) }}
                                        className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                                    >
                                        <option value="low">Thấp (tiết kiệm)</option>
                                        <option value="medium">Trung Bình</option>
                                        <option value="high">Cao (khuyến nghị)</option>
                                    </select>
                                    <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                                </div>
                                <p className="text-xs text-fg-muted font-medium">Chất lượng video được lưu</p>
                            </div>
                        </div>
                    </GlassCard>

                    {/* Advanced Settings */}
                    <GlassCard scanLines className="p-6">
                        <div className="flex items-center gap-3 mb-6">
                            <Sliders className="w-6 h-6 text-neon-cyan" />
                            <h2 className="text-xl font-bold text-neon-cyan tracking-wide" style={{ fontFamily: 'var(--font-orbitron)' }}>NÂNG CAO</h2>
                            <Badge className="glass-card border-neon-yellow/50 text-neon-yellow text-xs font-semibold">
                                Chuyên Gia
                            </Badge>
                        </div>
                        <div className="grid gap-6 md:grid-cols-2">
                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Chế Độ Debug</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">Hiển thị thông tin chi tiết</p>
                                    </div>
                                    <button
                                        onClick={() => { setDebugMode(!debugMode); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${debugMode ? "bg-neon-red/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${debugMode ? "translate-x-7 bg-neon-red" : "bg-fg-muted"
                                                }`}
                                            style={debugMode ? { boxShadow: "0 0 10px var(--neon-red)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-2">
                                <div className="flex items-center justify-between">
                                    <div>
                                        <label className="text-sm font-semibold text-fg-primary">Giám Sát Hiệu Suất</label>
                                        <p className="text-xs text-fg-muted mt-1 font-medium">Theo dõi CPU/GPU/RAM</p>
                                    </div>
                                    <button
                                        onClick={() => { setPerfMonitoring(!perfMonitoring); setHasChanges(true) }}
                                        className={`relative w-14 h-7 rounded-full transition-all ${perfMonitoring ? "bg-neon-green/30" : "bg-white/10"
                                            }`}
                                    >
                                        <div
                                            className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${perfMonitoring ? "translate-x-7 bg-neon-green" : "bg-fg-muted"
                                                }`}
                                            style={perfMonitoring ? { boxShadow: "0 0 10px var(--neon-green)" } : {}}
                                        />
                                    </button>
                                </div>
                            </div>

                            <div className="space-y-3 md:col-span-2">
                                <div className="flex items-center justify-between">
                                    <label className="text-sm font-semibold text-fg-primary">Thời Gian Lưu Dữ Liệu</label>
                                    <span className="text-sm font-bold text-neon-cyan digital-number">{dataRetention} ngày</span>
                                </div>
                                <input
                                    type="range"
                                    min="7"
                                    max="90"
                                    step="7"
                                    value={dataRetention}
                                    onChange={(e) => { setDataRetention(Number(e.target.value)); setHasChanges(true) }}
                                    className="w-full h-2 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-neon-cyan [&::-webkit-slider-thumb]:cursor-pointer"
                                    style={{
                                        background: `linear-gradient(to right, var(--neon-cyan) 0%, var(--neon-cyan) ${((dataRetention - 7) / 83) * 100}%, rgba(255,255,255,0.1) ${((dataRetention - 7) / 83) * 100}%, rgba(255,255,255,0.1) 100%)`
                                    }}
                                />
                                <p className="text-xs text-fg-muted font-medium">Số ngày lưu trữ dữ liệu phát hiện</p>
                            </div>
                        </div>
                    </GlassCard>

                    {/* Info Card */}
                    <GlassCard className="p-6 border-neon-cyan/30">
                        <div className="flex items-start gap-4">
                            <div className="w-10 h-10 rounded-full glass-card border-neon-cyan/50 flex items-center justify-center flex-shrink-0">
                                <Gauge className="w-5 h-5 text-neon-cyan" />
                            </div>
                            <div className="flex-1">
                                <h3 className="text-sm font-bold text-neon-cyan mb-2" style={{ fontFamily: 'var(--font-orbitron)' }}>LƯU Ý</h3>
                                <p className="text-xs text-fg-secondary leading-relaxed font-medium">
                                    Một số cài đặt có thể ảnh hưởng đến hiệu suất hệ thống.
                                    Nếu bạn gặp vấn đề về độ trễ hoặc hiệu suất, hãy thử giảm chất lượng video,
                                    tăng số khung hình bỏ qua, hoặc tắt một số tính năng không cần thiết.
                                </p>
                            </div>
                        </div>
                    </GlassCard>
                </div>
            </main>
        </div>
    )
}
