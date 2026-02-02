"use client";

import { useEffect, useRef, useState } from "react";
import Link from "next/link";
import { useLanguage } from "@/contexts/language-context";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { GlassCard } from "@/components/ui/glass-card";
import { Input } from "@/components/ui/input";
import { useToast } from "@/components/ui/use-toast";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { ScrollArea } from "@/components/ui/scroll-area";
import { getApiUrl } from "@/lib/api-config";
import { API_ENDPOINTS } from "@/lib/api-endpoints";
import {
  ArrowLeft,
  Upload,
  PlayCircle,
  Eye,
  Loader2,
  AlertTriangle,
  Sparkles,
  ShieldCheck,
  RefreshCw,
  Clock,
  FileVideo,
  CheckCircle2,
  Database,
} from "lucide-react";

type VideoItem = {
  id: number;
  job_id: string;
  video_filename: string;
  video_path: string;
  status: string;
  progress_percent?: number;
  created_at: string;
  duration_seconds?: number | null;
  video_size_mb?: number | null;
};

export default function DriverMonitorPage() {
  const { toast } = useToast();
  const { t } = useLanguage();
  const videoRef = useRef<HTMLVideoElement>(null);

  const [isMonitoring, setIsMonitoring] = useState(false);
  const [fatigueLevel, setFatigueLevel] = useState(0);
  const [distractionLevel, setDistractionLevel] = useState(0);
  const [eyesClosed, setEyesClosed] = useState(false);
  const [blinkRate, setBlinkRate] = useState(0);
  const [progress, setProgress] = useState(0);

  const [jobId, setJobId] = useState<string | null>(null);
  const [isUploading, setIsUploading] = useState(false);
  const [processingMsg, setProcessingMsg] = useState("");

  const consecutive404Ref = useRef(0);
  const startedAtRef = useRef(0);

  // Video selection state (tái sử dụng từ tab ADAS)
  const [file, setFile] = useState<File | null>(null);
  const [videoUrl, setVideoUrl] = useState<string | null>(null);
  const [selectedVideo, setSelectedVideo] = useState<VideoItem | null>(null);
  const [showVideoDialog, setShowVideoDialog] = useState(false);
  const [availableVideos, setAvailableVideos] = useState<VideoItem[]>([]);
  const [loadingVideos, setLoadingVideos] = useState(false);

  // Cleanup video URL on unmount
  useEffect(() => {
    return () => {
      if (videoUrl?.startsWith("blob:")) {
        URL.revokeObjectURL(videoUrl);
      }
    };
  }, [videoUrl]);

  // Load video list when dialog opens
  const loadVideoList = async () => {
    try {
      setLoadingVideos(true);
      const res = await fetch(
        getApiUrl(`${API_ENDPOINTS.VIDEOS_LIST}?limit=20`),
      );
      const data = await res.json();

      let videos: VideoItem[] = [];
      if (data?.videos && Array.isArray(data.videos)) {
        videos = data.videos;
      } else if (Array.isArray(data)) {
        videos = data;
      }

      setAvailableVideos(videos);

      if (videos.length === 0) {
        toast({
          title: t("adas.noVideos"),
          description: t("adas.noVideosDesc"),
        });
      }
    } catch (err) {
      console.error("❌ [VideoList] Error:", err);
      toast({
        title: t("adas.videoListError"),
        description: t("adas.videoListErrorDesc"),
        variant: "destructive",
      });
    } finally {
      setLoadingVideos(false);
    }
  };

  // Open video selection dialog
  const useSampleVideo = async () => {
    setShowVideoDialog(true);
    await loadVideoList();
  };

  // Handle file upload
  const handleFile = (f: File | null) => {
    setFile(f);
    if (videoUrl?.startsWith("blob:")) URL.revokeObjectURL(videoUrl);
    setVideoUrl(f ? URL.createObjectURL(f) : null);
    setSelectedVideo(null);
    setIsMonitoring(false);
  };

  // Select video from list
  const selectVideo = (video: VideoItem) => {
    let playUrl = "";

    if (video.status === "completed") {
      const resultFilename = video.video_filename.replace(
        ".mp4",
        "_result.mp4",
      );
      playUrl = getApiUrl(
        API_ENDPOINTS.VIDEO_DOWNLOAD(video.job_id, resultFilename),
      );
    } else {
      playUrl = getApiUrl(
        API_ENDPOINTS.VIDEO_SAMPLE(video.job_id, video.video_filename),
      );
    }

    setVideoUrl(playUrl);
    setSelectedVideo(video);
    setFile(null);
    setShowVideoDialog(false);
    setIsMonitoring(false);

    toast({
      title: t("driverMonitor.videoSelected"),
      description: t("driverMonitor.videoSelectedDesc", {
        filename: video.video_filename,
      }),
    });
  };

  // Start monitoring: POST /api/driver-monitor/analyze (chỉ khi có file upload)
  const startMonitoring = async () => {
    if (!file) {
      toast({
        title: t("driverMonitor.uploadRequiredTitle"),
        description: t("driverMonitor.uploadRequiredDesc"),
        variant: "destructive",
      });
      return;
    }
    if (!videoUrl) {
      toast({
        title: t("driverMonitor.noVideoSelected"),
        description: t("driverMonitor.noVideoSelectedDesc"),
        variant: "destructive",
      });
      return;
    }

    const fileSizeMB = (file.size / (1024 * 1024)).toFixed(2);
    try {
      setIsUploading(true);
      setProcessingMsg(
        t("adas.uploadingVideoToServerWait", { size: fileSizeMB }),
      );
      consecutive404Ref.current = 0;

      const formData = new FormData();
      formData.append("file", file);
      formData.append("camera_id", "in_cabin_camera");
      formData.append("device", "cuda");

      const res = await fetch(getApiUrl(API_ENDPOINTS.DRIVER_MONITOR_ANALYZE), {
        method: "POST",
        body: formData,
      });

      let data: { job_id?: string; id?: string; progress_percent?: number };
      try {
        data = await res.json();
      } catch {
        throw new Error("Invalid server response");
      }
      if (!res.ok) {
        const msg =
          (data as any)?.detail ??
          (data as any)?.message ??
          `HTTP ${res.status}`;
        throw new Error(String(msg));
      }

      const newJobId = (data.job_id ?? (data as any).id) as string | undefined;
      if (!newJobId) throw new Error("Missing job_id in response");

      setJobId(newJobId);
      const pct =
        typeof data.progress_percent === "number"
          ? data.progress_percent
          : typeof (data as any).progress === "number"
            ? (data as any).progress
            : 0;
      setProgress(pct);
      setFatigueLevel(0);
      setDistractionLevel(0);
      setEyesClosed(false);
      setBlinkRate(0);
      setProcessingMsg(t("adas.analyzingProgressNoTime", { progress: pct }));
      startedAtRef.current = Date.now();

      setIsMonitoring(true);
      toast({
        title: t("driverMonitor.monitoringStarted"),
        description: t("driverMonitor.analyzingVideo"),
      });
      // Video không hiển thị trong lúc phân tích (overlay như ADAS), không cần play ở đây
    } catch (err: any) {
      console.error("startMonitoring error:", err);
      setProcessingMsg("");
      toast({
        title: t("driverMonitor.apiError"),
        description: err?.message ?? String(err),
        variant: "destructive",
      });
      setIsMonitoring(false);
    } finally {
      setIsUploading(false);
    }
  };

  // Poll 1: /api/video/result/{job_id} — progress và completed/failed
  useEffect(() => {
    if (!isMonitoring || !jobId) return;
    let cancelled = false;
    startedAtRef.current = Date.now();

    const pollResult = async () => {
      try {
        const res = await fetch(getApiUrl(API_ENDPOINTS.VIDEO_RESULT(jobId)));
        if (cancelled) return;

        if (!res.ok) {
          const errBody = await res.json().catch(() => ({}));
          console.warn(
            "[DriverResult] GET /api/video/result/...",
            res.status,
            errBody,
          );
          if (res.status === 404) {
            consecutive404Ref.current += 1;
            if (consecutive404Ref.current >= 5) {
              toast({
                title: t("driverMonitor.jobNotFoundTitle"),
                description: t("driverMonitor.jobNotFoundDesc"),
                variant: "destructive",
              });
              setProcessingMsg("");
              setIsMonitoring(false);
            }
          }
          return;
        }

        consecutive404Ref.current = 0;
        const data = await res.json();
        if (cancelled) return;

        const pct =
          typeof data.progress_percent === "number"
            ? data.progress_percent
            : typeof data.progress === "number"
              ? data.progress
              : undefined;
        if (typeof pct === "number") setProgress(pct);

        const elapsed = Math.floor((Date.now() - startedAtRef.current) / 1000);
        const timeStr =
          elapsed >= 60
            ? `${Math.floor(elapsed / 60)}:${(elapsed % 60).toString().padStart(2, "0")}`
            : `${elapsed}s`;
        setProcessingMsg(
          t("adas.analyzingProgress", {
            progress: typeof pct === "number" ? pct : 0,
            time: timeStr,
          }),
        );

        const st = (data.status || "").toLowerCase();
        if (st === "completed" || st === "failed" || st === "error") {
          setProcessingMsg("");
          setIsMonitoring(false);
          if (st === "completed") {
            if (data.video_filename) {
              const resultFilename = String(data.video_filename).replace(
                /\.mp4$/i,
                "_result.mp4",
              );
              setVideoUrl(
                getApiUrl(API_ENDPOINTS.VIDEO_DOWNLOAD(jobId, resultFilename)),
              );
            } else if (data.result_path || data.result_video_url) {
              const p = data.result_video_url || data.result_path;
              setVideoUrl(String(p).startsWith("http") ? p : getApiUrl(p));
            }
          }
          toast({
            title:
              st === "completed"
                ? t("driverMonitor.jobCompleted")
                : t("driverMonitor.jobFailed"),
            description:
              data.error_message ||
              (Array.isArray(data.recommendations)
                ? data.recommendations.join(". ")
                : data.recommendations) ||
              undefined,
          });
        }
      } catch (err) {
        if (!cancelled) console.error("[DriverResult] poll error:", err);
      }
    };

    pollResult();
    const id = setInterval(pollResult, 1000);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [isMonitoring, jobId, t, toast]);

  // Poll 2: /api/driver-status — fatigue, distraction, eyes_closed, blink_rate
  useEffect(() => {
    if (!isMonitoring) return;
    let cancelled = false;

    const pollDriverStatus = async () => {
      try {
        const res = await fetch(getApiUrl(API_ENDPOINTS.DRIVER_STATUS_CURRENT));
        if (!res.ok) return;
        const data = await res.json();
        if (cancelled) return;
        const st = data?.status ?? data;
        if (!st) return;
        if (typeof st.fatigue_level === "number")
          setFatigueLevel(Math.round(st.fatigue_level));
        if (typeof st.distraction_level === "number")
          setDistractionLevel(Math.round(st.distraction_level));
        if (typeof st.eyes_closed === "boolean") setEyesClosed(st.eyes_closed);
        if (typeof st.blink_rate === "number")
          setBlinkRate(Math.round(st.blink_rate));
      } catch (err) {
        if (!cancelled) console.error("[DriverStatus] poll error:", err);
      }
    };

    pollDriverStatus();
    const id = setInterval(pollDriverStatus, 1000);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [isMonitoring]);

  // Stop monitoring
  const stopMonitoring = () => {
    if (videoRef.current) {
      videoRef.current.pause();
      videoRef.current.currentTime = 0;
    }
    setProcessingMsg("");
    setIsMonitoring(false);
  };

  return (
    <div className="flex flex-col min-h-screen bg-bg-primary text-fg-primary">
      <header className="flex items-center justify-between p-3 sm:p-5 border-b border-white/10 glass-card backdrop-blur-xl">
        <div className="flex items-center gap-2 sm:gap-3">
          <Link href="/">
            <Button
              variant="ghost"
              size="icon"
              className="text-fg-secondary hover:text-neon-cyan"
            >
              <ArrowLeft className="w-4 h-4 sm:w-5 sm:h-5" />
            </Button>
          </Link>
          <div>
            <div className="flex items-center gap-1 sm:gap-2 flex-wrap">
              <Badge className="gap-1 text-xs bg-neon-cyan/20 text-neon-cyan border-neon-cyan/50">
                <Sparkles className="w-3 h-3" />
                <span className="hidden sm:inline">
                  {t("driverMonitor.realtimeAI")}
                </span>
                <span className="sm:hidden">{t("driverMonitor.ai")}</span>
              </Badge>
              <Badge className="gap-1 text-xs bg-neon-green/20 text-neon-green border-neon-green/50">
                <ShieldCheck className="w-3 h-3" />
                <span className="hidden sm:inline">
                  {t("driverMonitor.badge")}
                </span>
                <span className="sm:hidden">
                  {t("driverMonitor.badgeShort")}
                </span>
              </Badge>
            </div>
            <h1 className="text-3xl font-bold flex items-center gap-2 mt-1 sm:mt-2 text-neon-cyan tracking-wider uppercase">
              <Eye className="w-8 h-8 text-neon-cyan" />
              <span className="hidden sm:inline text-neon-cyan">
                {t("driverMonitor.title")}
              </span>
              <span className="sm:hidden text-neon-cyan">
                {t("driverMonitor.titleShort")}
              </span>
            </h1>
            <p className="text-xs sm:text-sm text-fg-secondary">
              {t("driverMonitor.subtitle")}
            </p>
          </div>
        </div>
        <div className="hidden lg:flex items-center gap-2"></div>
      </header>

      <main className="flex-1 p-3 sm:p-4 lg:p-6">
        <div className="grid gap-4 sm:gap-6 xl:grid-cols-3">
          {/* Left Panel - Controls */}
          <div className="space-y-4 xl:col-span-1">
            <GlassCard glow="cyan" className="p-6">
              <div className="mb-4">
                <h3 className="text-lg font-bold text-neon-cyan flex items-center gap-2 tracking-wide">
                  <Upload className="w-4 h-4" />
                  {t("adas.step1Title")}
                </h3>
                <p className="text-xs text-fg-secondary mt-1">
                  {t("adas.step1Desc")}
                </p>
              </div>
              <div className="space-y-4">
                <Input
                  type="file"
                  accept="video/*"
                  onChange={(e) => handleFile(e.target.files?.[0] || null)}
                  disabled={isMonitoring}
                  className="
                    cursor-pointer glass-card border-neon-cyan/30
                    text-fg-primary file:text-neon-cyan
                    video-file-input
                  "
                />
                <div className="flex flex-col sm:flex-row gap-2">
                  <Button
                    onClick={useSampleVideo}
                    disabled={loadingVideos || isMonitoring}
                    className="flex-1 glass-card border-2 border-neon-cyan/50 bg-neon-cyan/10 text-neon-cyan hover:bg-neon-cyan/20 font-semibold"
                  >
                    <span className="flex items-center justify-center gap-2">
                      {loadingVideos ? (
                        <Loader2 className="h-4 w-4 animate-spin" />
                      ) : (
                        <PlayCircle className="h-4 w-4" />
                      )}
                      <span className="hidden sm:inline">
                        {t("adas.sampleVideo")}
                      </span>
                      <span className="sm:hidden">
                        {t("adas.sampleVideoShort")}
                      </span>
                    </span>
                  </Button>
                </div>

                <div className="grid grid-cols-2 gap-3 text-sm">
                  <div
                    className={`rounded-lg glass-card border-2 p-3 ${
                      file || videoUrl
                        ? "border-neon-green/50"
                        : "border-neon-red/50"
                    }`}
                  >
                    <div className="text-xs text-fg-secondary font-medium tracking-wide">
                      {t("adas.status")}
                    </div>
                    <div
                      className={`flex items-center gap-2 text-sm font-medium
                          antialiased
                          transition-colors duration-300
                          ${
                            isMonitoring
                              ? "text-neon-yellow drop-shadow-[0_0_6px_rgba(250,204,21,0.45)]"
                              : file || videoUrl
                                ? "text-neon-green drop-shadow-[0_0_6px_rgba(34,197,94,0.45)]"
                                : "text-neon-red drop-shadow-[0_0_6px_rgba(239,68,68,0.45)]"
                          }
                        `}
                    >
                      <Loader2
                        className={`h-3.5 w-3.5
                            ${
                              isMonitoring
                                ? "animate-spin opacity-90"
                                : "opacity-70"
                            }
                          `}
                      />
                      <span className="leading-none mt-[5px]">
                        {isMonitoring
                          ? t("driverMonitor.monitoring")
                          : file || videoUrl
                            ? t("adas.ready")
                            : t("adas.notReady")}
                      </span>
                    </div>
                  </div>
                  <div className="rounded-lg glass-card border-2 border-neon-green/30 p-3">
                    <div className="text-xs text-fg-secondary font-medium">
                      {t("adas.videoSource")}
                    </div>
                    <div className="font-semibold text-neon-green">
                      {file
                        ? t("adas.newUpload")
                        : videoUrl
                          ? t("adas.sampleVideo")
                          : t("adas.notSelected")}
                    </div>
                  </div>
                </div>

                {/* Start/Stop Monitoring Button */}
                {processingMsg && (
                  <div className="text-sm text-fg-primary flex items-center gap-2 rounded-md glass-card border-2 border-neon-yellow/50 px-3 py-2">
                    <Loader2 className="h-4 w-4 animate-spin text-neon-yellow" />
                    {processingMsg}
                  </div>
                )}

                {!isMonitoring ? (
                  <Button
                    onClick={startMonitoring}
                    disabled={!file || isUploading}
                    className="w-full glass-card border-2 border-neon-green/50 bg-neon-green/10 text-neon-green hover:bg-neon-green/20 font-bold disabled:opacity-50 disabled:cursor-not-allowed"
                  >
                    {isUploading ? (
                      <>
                        <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                        {t("driverMonitor.uploading")}
                      </>
                    ) : (
                      <>
                        <PlayCircle className="h-4 w-4 mr-2" />
                        {t("driverMonitor.startMonitoring")}
                      </>
                    )}
                  </Button>
                ) : (
                  <Button
                    onClick={stopMonitoring}
                    className="w-full glass-card border-2 border-neon-red/50 bg-neon-red/10 text-neon-red hover:bg-neon-red/20 font-bold"
                  >
                    <AlertTriangle className="h-4 w-4 mr-2" />
                    {t("driverMonitor.stopMonitoring")}
                  </Button>
                )}
              </div>
            </GlassCard>

            <GlassCard className="p-6">
              <div className="mb-4">
                <h3 className="text-lg font-bold text-neon-green flex items-center gap-2 tracking-wide">
                  <ShieldCheck className="w-4 h-4" />
                  {t("driverMonitor.monitoringInfoTitle")}
                </h3>
                <p className="text-xs text-fg-secondary mt-1">
                  {t("driverMonitor.monitoringInfoDesc")}
                </p>
              </div>
              <div className="text-sm text-fg-secondary space-y-2">
                <div className="flex items-center gap-2">
                  <Badge className="gap-1 bg-neon-cyan/20 text-neon-cyan border-neon-cyan/50">
                    <Eye className="w-3 h-3" />
                    Fatigue
                  </Badge>
                  <span>{t("driverMonitor.fatigueDetection")}</span>
                </div>
                <div className="flex items-center gap-2">
                  <Badge className="gap-1 bg-neon-yellow/20 text-neon-yellow border-neon-yellow/50">
                    <AlertTriangle className="w-3 h-3" />
                    Distraction
                  </Badge>
                  <span>{t("driverMonitor.distractionDetection")}</span>
                </div>
                <div className="flex items-center gap-2">
                  <Badge className="gap-1 bg-neon-green/20 text-neon-green border-neon-green/50">
                    <ShieldCheck className="w-3 h-3" />
                    Eyes
                  </Badge>
                  <span>{t("driverMonitor.eyeTracking")}</span>
                </div>
              </div>
            </GlassCard>
          </div>

          {/* Right Panel - Video Display */}
          <GlassCard glow="green" className="xl:col-span-2 h-full p-6">
            <div className="mb-4">
              <div className="flex items-center justify-between">
                <h3 className="text-xl font-bold text-neon-green tracking-wide">
                  {t("driverMonitor.step2Title")}
                </h3>
                {isMonitoring && (
                  <Badge
                    className="
                    gap-1
                    bg-red-500/10
                    text-red-400
                    border border-red-500/40
                    animate-pulse
                    [animation-duration:1s]
                    shadow-[0_0_12px_rgba(255,0,0,0.6)]
                  "
                  >
                    <AlertTriangle className="h-3 w-3" />
                    {t("driverMonitor.monitoring")}
                  </Badge>
                )}
              </div>
              <p className="text-xs text-fg-secondary mt-1">
                {t("driverMonitor.step2Desc")}
              </p>
            </div>
            <div className="relative aspect-video bg-black/30 rounded-lg overflow-hidden border-2 border-neon-green/50 shadow-lg">
              {isUploading || isMonitoring ? (
                <div className="absolute inset-0 glass-card flex flex-col items-center justify-center text-neon-cyan gap-4">
                  <Loader2 className="h-12 w-12 animate-spin text-neon-cyan" />
                  <div className="text-center space-y-2">
                    {isUploading ? (
                      <>
                        <p className="text-lg font-semibold">
                          {t("adas.uploadingVideo")}
                        </p>
                        <p className="text-sm text-fg-secondary">
                          {t("adas.uploadingVideoDesc")}
                        </p>
                        <Badge className="gap-1 bg-neon-cyan/20 text-neon-cyan border-neon-cyan/50">
                          <Upload className="w-3 h-3 animate-pulse" />
                          Đang upload...
                        </Badge>
                      </>
                    ) : (
                      <>
                        <p className="text-lg font-semibold">
                          {t("adas.analyzingVideo")}
                        </p>
                        <p className="text-sm text-fg-secondary">
                          {t("adas.analyzingVideoDesc")}
                        </p>
                      </>
                    )}
                  </div>
                  <div className="w-full max-w-md px-8 space-y-2">
                    <div className="h-3 bg-black/50 rounded-full overflow-hidden border border-neon-cyan/50">
                      <div
                        className="h-full bg-gradient-to-r from-neon-cyan to-neon-green transition-all duration-500 ease-out"
                        style={{
                          width: `${Math.min(100, Math.max(0, progress))}%`,
                        }}
                      />
                    </div>
                    <div className="flex justify-between text-xs text-fg-secondary">
                      <span>{progress}%</span>
                      <span>
                        {isUploading ? "Đang upload..." : "Đang xử lý..."}
                      </span>
                    </div>
                  </div>
                  {processingMsg && (
                    <p className="text-sm text-neon-yellow max-w-md text-center px-4">
                      {processingMsg}
                    </p>
                  )}
                  {!isUploading && isMonitoring && (
                    <p className="text-xs text-fg-secondary">
                      {t("driverMonitor.analyzingStatus", {
                        fatigue: fatigueLevel,
                        distraction: distractionLevel,
                        eyes: eyesClosed
                          ? t("driverMonitor.eyesClosed")
                          : t("driverMonitor.eyesOpen"),
                        blink: blinkRate,
                      })}
                    </p>
                  )}
                </div>
              ) : videoUrl ? (
                <video
                  ref={videoRef}
                  key={videoUrl}
                  controls
                  autoPlay
                  muted
                  loop
                  playsInline
                  className="w-full h-full object-contain"
                  style={{ maxHeight: "600px" }}
                  onError={(e) => {
                    const err = e.currentTarget.error;
                    console.log("VIDEO ERROR CODE:", err?.code);
                    console.log("VIDEO ERROR MSG:", err?.message);
                    console.log("VIDEO URL:", videoUrl);
                  }}
                >
                  <source src={videoUrl} type="video/mp4" />
                </video>
              ) : (
                <div className="absolute inset-0 flex flex-col items-center justify-center text-fg-secondary gap-2">
                  <Upload className="w-8 h-8 text-neon-cyan" />
                  <p>{t("adas.noVideoMessage")}</p>
                </div>
              )}
            </div>
            {isUploading || isMonitoring ? (
              <div className="mt-4 text-sm text-fg-secondary flex items-center gap-2">
                <AlertTriangle className="h-10 w-10 text-red-500 animate-pulse [animation-duration:0.8s] drop-shadow-[0_0_8px_rgba(255,0,0,0.8)]" />
                Dữ liệu sau phân tích sẽ được lưu vào hệ thống và có thể truy
                xuất ở bước &quot;Video mẫu&quot;.
              </div>
            ) : videoUrl && !isMonitoring ? (
              <div className="mt-4 flex flex-wrap gap-3">
                <Button
                  onClick={startMonitoring}
                  disabled={!file}
                  className="gap-2 bg-gradient-to-r from-neon-cyan to-neon-green text-black font-bold hover:from-neon-cyan/80 hover:to-neon-green/80 disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  <PlayCircle className="w-4 h-4" />
                  {t("driverMonitor.startMonitoring")}
                </Button>
                <Button
                  variant="outline"
                  onClick={() => {
                    setVideoUrl(null);
                    setFile(null);
                    setSelectedVideo(null);
                  }}
                  className="gap-2 glass-card border-neon-cyan/50 text-neon-cyan hover:bg-neon-cyan/10"
                >
                  <RefreshCw className="w-4 h-4" />
                  {t("driverMonitor.selectAnotherVideo")}
                </Button>
              </div>
            ) : null}
          </GlassCard>
        </div>
      </main>

      {/* Video Selection Dialog - Tái sử dụng từ tab ADAS */}
      <Dialog open={showVideoDialog} onOpenChange={setShowVideoDialog}>
        <DialogContent className="glass-card border-2 border-neon-cyan/50">
          <DialogHeader className="flex-shrink-0">
            <DialogTitle className="text-2xl font-bold text-neon-cyan flex items-center gap-2">
              <FileVideo className="w-6 h-6" />
              {t("adas.selectSampleVideo")}
            </DialogTitle>
            <DialogDescription className="text-fg-secondary">
              {t("driverMonitor.selectSampleVideoDesc", {
                count: availableVideos.length,
              })}
            </DialogDescription>
          </DialogHeader>

          <ScrollArea className="flex-1 min-h-0 pr-4 overflow-x-hidden">
            {loadingVideos ? (
              <div className="flex items-center justify-center h-40">
                <Loader2 className="h-8 w-8 animate-spin text-neon-cyan" />
                <span className="ml-3 text-fg-secondary">
                  {t("adas.loadingVideoList")}
                </span>
              </div>
            ) : availableVideos.length === 0 ? (
              <div className="flex flex-col items-center justify-center h-40 text-fg-secondary">
                <FileVideo className="w-12 h-12 mb-3 text-neon-cyan/50" />
                <p>Chưa có video nào trong database.</p>
                <p className="text-sm mt-1">Hãy upload video mới để bắt đầu.</p>
              </div>
            ) : (
              <div className="grid gap-3">
                {availableVideos.map((video) => (
                  <div
                    key={video.id}
                    onClick={() => selectVideo(video)}
                    className="w-full text-left glass-card border-2 border-neon-cyan/30 hover:border-neon-cyan hover:bg-neon-cyan/5 transition-all p-4 rounded-lg group cursor-pointer"
                  >
                    <div className="flex items-start justify-between gap-4">
                      <div className="flex-1 min-w-0">
                        <div className="flex items-center gap-2 mb-2">
                          <FileVideo className="w-5 h-5 text-neon-cyan shrink-0" />
                          <h4 className="font-semibold text-fg-primary truncate group-hover:text-neon-cyan transition-colors">
                            {video.video_filename || `Video #${video.id}`}
                          </h4>
                        </div>

                        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 text-xs text-fg-secondary">
                          <div className="flex items-center gap-1">
                            <Clock className="w-3 h-3" />
                            <span>
                              {video.duration_seconds
                                ? `${Math.floor(
                                    video.duration_seconds / 60,
                                  )}:${(video.duration_seconds % 60)
                                    .toString()
                                    .padStart(2, "0")}`
                                : "N/A"}
                            </span>
                          </div>

                          <div className="flex items-center gap-1">
                            <Database className="w-3 h-3" />
                            <span>
                              {video.video_size_mb
                                ? `${video.video_size_mb.toFixed(1)} MB`
                                : "N/A"}
                            </span>
                          </div>

                          <div className="flex items-center gap-1">
                            {video.status === "completed" ? (
                              <Badge
                                variant="outline"
                                className="text-[10px] h-5 border-neon-green text-neon-green bg-neon-green/10"
                              >
                                <CheckCircle2 className="w-3 h-3 mr-1" />
                                {t("adas.completed")}
                              </Badge>
                            ) : video.status === "processing" ? (
                              <Badge
                                variant="outline"
                                className="text-[10px] h-5 border-neon-yellow text-neon-yellow bg-neon-yellow/10"
                              >
                                <Loader2 className="w-3 h-3 mr-1 animate-spin" />
                                {t("adas.processing")}
                              </Badge>
                            ) : (
                              <Badge
                                variant="outline"
                                className="text-[10px] h-5 border-neon-cyan text-neon-cyan bg-neon-cyan/10"
                              >
                                {t("adas.notStarted")}
                              </Badge>
                            )}
                          </div>

                          {video.created_at && (
                            <div className="text-xs opacity-70">
                              {new Date(video.created_at).toLocaleDateString(
                                "vi-VN",
                              )}
                            </div>
                          )}
                        </div>
                      </div>

                      <Button
                        size="sm"
                        className="glass-card bg-neon-cyan/20 text-neon-cyan border border-neon-cyan/50 hover:bg-neon-cyan/30 shrink-0"
                      >
                        <PlayCircle className="w-4 h-4" />
                        {t("common.select")}
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </ScrollArea>
        </DialogContent>
      </Dialog>
    </div>
  );
}
