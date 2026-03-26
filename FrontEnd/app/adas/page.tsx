"use client";

import { useEffect, useRef, useState } from "react";
import Link from "next/link";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { GlassCard } from "@/components/ui/glass-card";
import { Input } from "@/components/ui/input";
import { useToast } from "@/components/ui/use-toast";
import { getApiUrl } from "@/lib/api-config";
import { API_ENDPOINTS } from "@/lib/api-endpoints";
import { useLanguage } from "@/contexts/language-context";
import {
  ArrowLeft,
  Upload,
  PlayCircle,
  Film,
  CheckCircle2,
  Loader2,
  AlertTriangle,
  Sparkles,
  Database,
  ShieldCheck,
  RefreshCw,
  Clock,
  FileVideo,
  XCircle,
} from "lucide-react";
import { useVideoProgress } from "@/hooks/use-video-progress";

type VisionResponse = {
  message?: string;
  data?: any;
};

export default function ADASPage() {
  const { toast } = useToast();
  const { t } = useLanguage();
  const [file, setFile] = useState<File | null>(null);
  const [previewUrl, setPreviewUrl] = useState<string | null>(null);
  const [uploading, setUploading] = useState(false);
  const [processingMsg, setProcessingMsg] = useState<string>("");
  const [result, setResult] = useState<VisionResponse | null>(null);

  // Video processing state
  const [currentJobId, setCurrentJobId] = useState<string | null>(null);
  const [processingProgress, setProcessingProgress] = useState(0);
  const [isProcessing, setIsProcessing] = useState(false);
  const [processedVideoUrl, setProcessedVideoUrl] = useState<string | null>(
    null,
  );

  const normalizeProgress = (value: number | undefined | null): number => {
    if (typeof value !== "number" || Number.isNaN(value)) return 0;
    if (value < 0) return 0;
    if (value > 100) return 100;
    return Math.round(value);
  };

  // Fallback polling flag
  const pollingRef = useRef<boolean>(false);

  // ref lưu thời điểm bắt đầu phân tích (không bị reset bởi re-render)
  const analysisStartedAtRef = useRef<number>(0);
  // ref để timer interval luôn đọc được progress mới nhất (tránh stale closure)
  const processingProgressRef = useRef<number>(0);
  useEffect(() => {
    processingProgressRef.current = processingProgress;
  }, [processingProgress]);

  useEffect(() => {
    if (!isProcessing || uploading) return;
    analysisStartedAtRef.current = Date.now();

    const interval = setInterval(() => {
      const elapsed = Math.floor(
        (Date.now() - analysisStartedAtRef.current) / 1000,
      );
      const minutes = Math.floor(elapsed / 60);
      const seconds = elapsed % 60;
      const timeString =
        minutes > 0
          ? `${minutes}:${seconds.toString().padStart(2, "0")}`
          : `${elapsed}s`;
      // Dùng ref thay vì state để tránh stale closure
      setProcessingMsg(
        t("adas.analyzingProgress", {
          progress: processingProgressRef.current,
          time: timeString,
        }),
      );
    }, 1000);

    return () => clearInterval(interval);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isProcessing, uploading]);

  // WebSocket progress monitoring
  const {
    progress: wsProgress,
    status: wsStatus,
    isFinished: wsIsFinished,
    error: wsError,
    processingTime: wsProcessingTime,
    isConnected: wsIsConnected,
  } = useVideoProgress(currentJobId, isProcessing && !pollingRef.current);

  // Update local state when WebSocket data changes
  useEffect(() => {
    if (currentJobId && isProcessing) {
      if (!pollingRef.current) {
        setProcessingProgress(normalizeProgress(wsProgress));

        // Handle completion
        if (wsIsFinished && wsStatus === "completed") {
          console.log("✅ [WebSocket] Processing completed!");
          setIsProcessing(false);
          fetchProcessedVideo(currentJobId);
        }

        // Handle errors
        if (wsError) {
          console.error("❌ [WebSocket] Error:", wsError);
          toast({
            title: t("adas.analysisError") || "WebSocket Timeout/Error",
            description: "Chuyển sang chế độ lấy dữ liệu dự phòng...",
          });
          // Fallback to polling if WebSocket fails
          console.log("🔄 Falling back to polling...");
          pollingRef.current = true;
          pollForResult(currentJobId);
        }
      }
    } else if (!isProcessing) {
      pollingRef.current = false;
    }
  }, [
    wsProgress,
    wsStatus,
    wsIsFinished,
    wsError,
    wsProcessingTime,
    wsIsConnected,
    currentJobId,
    isProcessing,
  ]);

  useEffect(() => {
    return () => {
      if (previewUrl?.startsWith("blob:")) {
        URL.revokeObjectURL(previewUrl);
      }
    };
  }, [previewUrl]);

  useEffect(() => {
    console.log("[PlayerState]", {
      isProcessing,
      previewUrl,
    });
  }, [isProcessing, previewUrl]);

  const handleFile = (f: File | null) => {
    setResult(null);
    setProcessingMsg("");
    setFile(f);
    if (previewUrl?.startsWith("blob:")) URL.revokeObjectURL(previewUrl);
    setPreviewUrl(f ? URL.createObjectURL(f) : null);
  };

  const uploadAndAnalyze = async () => {
    if (!file) {
      toast({
        title: t("adas.noVideoSelected"),
        description: t("adas.noVideoSelectedDesc"),
        variant: "destructive",
      });
      return;
    }

    // Show file size info
    const fileSizeMB = (file.size / (1024 * 1024)).toFixed(2);
    console.log(`📤 Uploading video: ${file.name} (${fileSizeMB} MB)`);

    try {
      setUploading(true);
      setIsProcessing(true);
      setProcessingProgress(0);
      setProcessingMsg(`Đang tải video lên server... (${fileSizeMB} MB)`);

      // Step 1: Upload video with timeout
      const formData = new FormData();
      formData.append("file", file);

      // Create upload promise with timeout (5 minutes for large files)
      const uploadTimeout = 5 * 60 * 1000; // 5 minutes
      const uploadPromise = fetch(getApiUrl("/api/video/upload"), {
        method: "POST",
        body: formData,
      });

      const timeoutPromise = new Promise((_, reject) =>
        setTimeout(
          () =>
            reject(
              new Error(
                "Upload timeout - Video quá lớn hoặc mạng chậm. Vui lòng thử lại với video nhỏ hơn.",
              ),
            ),
          uploadTimeout,
        ),
      );

      // Show upload progress message
      const progressInterval = setInterval(() => {
        setProcessingMsg((prev) => {
          if (prev.includes("...")) {
            return t("adas.uploadingVideoToServerWait", { size: fileSizeMB });
          }
          return prev + ".";
        });
      }, 1000);

      const uploadRes = (await Promise.race([
        uploadPromise,
        timeoutPromise,
      ])) as Response;
      clearInterval(progressInterval);

      // Parse response
      let uploadData: any;
      let errorMessage = "";

      try {
        uploadData = await uploadRes.json();
      } catch (parseErr) {
        console.error("❌ Failed to parse response:", parseErr);
        throw new Error(t("adas.invalidServerResponse"));
      }

      // Check for errors
      if (!uploadRes.ok) {
        // Extract error message from backend
        errorMessage =
          uploadData?.detail ||
          uploadData?.message ||
          `Upload failed with status ${uploadRes.status}`;

        if (uploadRes.status === 400) {
          errorMessage = `${t("adas.videoFormatError")}: ${errorMessage}`;
        } else if (uploadRes.status === 413) {
          errorMessage = t("adas.videoTooLarge");
        } else if (uploadRes.status === 500) {
          errorMessage = t("adas.serverError");
        }

        throw new Error(errorMessage);
      }

      const jobId = uploadData.job_id || uploadData.id;

      if (!jobId) {
        throw new Error(t("adas.noJobId"));
      }

      console.log("✅ Upload OK - Job:", jobId.substring(0, 8));
      setCurrentJobId(jobId);
      setUploading(false);

      toast({
        title: t("adas.uploadSuccess"),
        description: t("adas.uploadSuccessDesc", { size: fileSizeMB }),
      });

      // Step 2: WebSocket will automatically start monitoring via useVideoProgress hook
      setProcessingMsg(t("adas.connectingWebSocket"));
    } catch (err: any) {
      console.error("❌ [Upload] Error:", err);

      // Determine error type and show appropriate message
      let errorTitle = t("adas.uploadError");
      let errorDescription = err.message || t("adas.uploadErrorDesc");

      if (err.message.includes("timeout")) {
        errorTitle = t("adas.uploadTimeoutTitle");
        errorDescription = t("adas.uploadTimeoutDesc", { size: fileSizeMB });
      } else if (err.message.includes("Failed to fetch")) {
        errorTitle = t("adas.connectionError");
        errorDescription = t("adas.connectionErrorDesc");
      }

      toast({
        title: errorTitle,
        description: errorDescription,
        variant: "destructive",
        duration: 8000, // Show longer for errors
      });

      setUploading(false);
      setIsProcessing(false);
      setProcessingMsg("");
    }
  };

  // Fallback polling if WebSocket doesn't work (kept as backup)
  const pollForResult = async (jobId: string) => {
    const maxAttempts = 450; // 15 minutes max (450 attempts × 2 seconds = 900 seconds = 15 minutes)
    let attempts = 0;
    const startTime = Date.now();

    const poll = async () => {
      try {
        const res = await fetch(getApiUrl(API_ENDPOINTS.VIDEO_RESULT(jobId)));
        if (!res.ok) {
          throw new Error(`HTTP Error ${res.status}`);
        }
        const data = await res.json();

        // Only log if progress changed or status changed
        const newProgress = normalizeProgress(
          data.progress_percent ?? data.progress ?? 0,
        );
        if (attempts === 0 || data.status === "completed") {
          console.log(
            `[Job ${jobId.substring(0, 8)}] Status: ${data.status}, Progress: ${newProgress}%`,
          );
        }
        const elapsed = Math.floor(
          (Date.now() - (analysisStartedAtRef.current || Date.now())) / 1000,
        );
        const elapsedMinutes = Math.floor(elapsed / 60);
        const remainingSeconds = elapsed % 60;
        const timeString =
          elapsedMinutes > 0
            ? `${elapsedMinutes}:${remainingSeconds.toString().padStart(2, "0")}`
            : `${elapsed}s`;

        setProcessingProgress(newProgress);
        setProcessingMsg(
          t("adas.analyzingProgress", {
            progress: newProgress,
            time: timeString,
          }),
        );

        if (data.status === "completed") {
          setIsProcessing(false);
          fetchProcessedVideo(jobId);
          return;
        }

        if (data.status === "error") {
          throw new Error(data.error_message || t("adas.processingFailed"));
        }

        attempts++;
        if (attempts < maxAttempts && data.status !== "completed") {
          setTimeout(poll, 2000); // Poll every 2 seconds for faster updates
        } else if (attempts >= maxAttempts) {
          const elapsedMinutes = Math.floor((attempts * 2) / 60);
          throw new Error(
            `Processing timeout after ${elapsedMinutes} minutes. Video might be too long or server is overloaded. Please try a shorter video or contact support.`,
          );
        }
      } catch (err: any) {
        console.error("❌ [Poll] Error:", err);
        setIsProcessing(false);
        toast({
          title: t("adas.analysisError"),
          description: err.message,
          variant: "destructive",
        });
      }
    };

    poll();
  };

  // Fetch processed video URL
  const fetchProcessedVideo = async (jobId: string) => {
    try {
      const res = await fetch(getApiUrl(API_ENDPOINTS.VIDEO_RESULT(jobId)));
      if (!res.ok) {
        throw new Error(`HTTP Error ${res.status}`);
      }
      const data = await res.json();

      console.log(
        "✅ Completed! Processing time:",
        data.processing_time_seconds,
        "s",
        "| Data:",
        data,
      );

      if (data.status === "completed") {
        let downloadUrl: string;

        // Ưu tiên pattern đang hoạt động ở driver-monitor:
        // 1. video_filename + _result.mp4
        // 2. full_result_video_url / video_url từ API
        // 3. result_path
        // 4. fallback result.mp4
        if (data.video_filename) {
          const resultFilename = String(data.video_filename).replace(
            /\.mp4$/i,
            "_result.mp4",
          );
          downloadUrl = getApiUrl(
            API_ENDPOINTS.VIDEO_DOWNLOAD(jobId, resultFilename),
          );
          console.log("✅ Using video_filename pattern:", downloadUrl);
        } else if (data.full_result_video_url || data.video_url) {
          const p = String(data.full_result_video_url || data.video_url);
          downloadUrl = p.startsWith("http") ? p : getApiUrl(p);
          console.log("✅ Using API video URL:", downloadUrl);
        } else if (data.result_path) {
          const p = String(data.result_path);
          downloadUrl = p.startsWith("http") ? p : getApiUrl(p);
          console.log("✅ Using result_path:", downloadUrl);
        } else {
          downloadUrl = getApiUrl(
            API_ENDPOINTS.VIDEO_DOWNLOAD(jobId, "result.mp4"),
          );
          console.log("⚠️ Fallback result.mp4:", downloadUrl);
        }

        setProcessedVideoUrl(downloadUrl);
        setPreviewUrl(downloadUrl);

        const doneTime = Math.floor(data.processing_time_seconds || 0);
        setProcessingMsg(
          doneTime > 0
            ? `Phân tích thành công (${doneTime}s)`
            : "Phân tích thành công",
        );

        toast({
          title: "Phân tích hoàn tất!",
          description:
            doneTime > 0
              ? `Thời gian xử lý: ${doneTime}s`
              : undefined,
        });
      } else {
        console.error("❌ [Result] Job not completed:", data);
        throw new Error("Job not completed");
      }

      setResult(data);
    } catch (err: any) {
      console.error("❌ [FetchResult] Error:", err);
      toast({
        title: t("adas.fetchResultError"),
        description: err.message,
        variant: "destructive",
      });
    }
  };

  const resetAnalysis = () => {
    setFile(null);
    setPreviewUrl(null);
    setProcessedVideoUrl(null);
    setResult(null);
    setCurrentJobId(null);
    setProcessingMsg("");
    setProcessingProgress(0);
    setIsProcessing(false);
    setUploading(false);
  };

  return (
    <div className="flex flex-col min-h-screen bg-bg-primary text-fg-primary">
      <header className="flex items-center justify-between p-3 sm:p-5 border-b border-white/10 glass-card backdrop-blur-xl">
        <div className="flex items-center gap-2 sm:gap-3">
          <Link href="/dashboard">
            <Button
              variant="ghost"
              size="icon"
              className="text-fg-secondary hover:text-[rgb(var(--primary))]"
            >
              <ArrowLeft className="w-4 h-4 sm:w-5 sm:h-5" />
            </Button>
          </Link>
          <div>
            <h1 className="text-3xl font-bold flex items-center gap-2 mt-1 sm:mt-2 text-[rgb(var(--primary))] tracking-wider uppercase">
              <Film className="w-8 h-8 text-[rgb(var(--primary))]" />
              <span className="hidden sm:inline text-[rgb(var(--primary))]">
                {t("adas.title")}
              </span>
              <span className="sm:hidden text-[rgb(var(--primary))]">
                {t("adas.titleShort")}
              </span>
            </h1>
            <p className="text-xs sm:text-sm text-fg-secondary">
              {t("adas.subtitle")}
            </p>
          </div>
        </div>
        <div className="hidden lg:flex items-center gap-2"></div>
      </header>

      <main className="flex-1 p-3 sm:p-4 lg:p-6">
        <div className="grid gap-4 sm:gap-6 xl:grid-cols-3">
          <div className="space-y-4 xl:col-span-1">
              <GlassCard glow="cyan" className="p-6">
                <div className="mb-4">
                  <h3 className="text-lg font-bold text-[rgb(var(--primary))] flex items-center gap-2 tracking-wide">
                    <Upload className="w-4 h-4" />
                    1) CHỌN VIDEO
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
                    disabled={uploading}
                    className="
                    cursor-pointer glass-card border-[color:rgb(var(--primary))]/30
                    text-fg-primary file:text-fg-primary
                    video-file-input
                  "
                  />

                  <div className="grid grid-cols-2 gap-3 text-sm">
                    <div
                      className={`rounded-lg glass-card border-2 p-3 ${
                        file || previewUrl
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
                              uploading || isProcessing
                                ? "text-neon-yellow drop-shadow-[0_0_6px_rgba(250,204,21,0.45)]"
                                : file || previewUrl
                                  ? "text-neon-green drop-shadow-[0_0_6px_rgba(34,197,94,0.45)]"
                                  : "text-neon-red drop-shadow-[0_0_6px_rgba(239,68,68,0.45)]"
                            }
                          `}
                      >
                        {uploading || isProcessing ? (
                          <Loader2 className="h-3.5 w-3.5 animate-spin opacity-90" />
                        ) : file || previewUrl ? (
                          <CheckCircle2 className="h-3.5 w-3.5 text-neon-green" />
                        ) : (
                          <XCircle className="h-3.5 w-3.5 text-neon-red" />
                        )}
                        <span className="leading-none mt-[5px]">
                          {uploading || isProcessing
                            ? t("adas.analyzing")
                            : file || previewUrl
                              ? processedVideoUrl
                                ? "Phân tích xong"
                                : "Sẵn sàng"
                              : t("adas.notReady")}
                        </span>
                      </div>
                    </div>
                    <div className="rounded-lg glass-card border-2 border-[color:rgb(var(--primary))]/25 p-3">
                      <div className="text-xs text-fg-secondary font-medium">
                        {t("adas.videoSource")}
                      </div>
                      <div
                        className={`font-semibold ${
                          file
                            ? "text-[rgb(var(--primary))]"
                            : "text-fg-primary"
                        }`}
                      >
                        {file ? t("adas.newUpload") : t("adas.notSelected")}
                      </div>
                    </div>
                  </div>

                  {processingMsg && (
                    <div className="text-sm text-fg-primary flex items-center gap-2 rounded-md glass-card border-2 border-neon-yellow/50 px-3 py-2">
                      {uploading || isProcessing ? (
                        <Loader2 className="h-4 w-4 animate-spin text-neon-yellow" />
                      ) : (
                        <CheckCircle2 className="h-4 w-4 text-neon-green" />
                      )}
                      {processingMsg}
                    </div>
                  )}

                  {/* Upload Button */}
                  <Button
                    onClick={uploadAndAnalyze}
                    disabled={!file || uploading || isProcessing}
                    className="w-full glass-card border-2 border-[color:rgb(var(--primary))]/40 bg-[color:rgb(var(--primary))]/10 text-[var(--primary-strong)] hover:bg-[color:rgb(var(--primary))]/20 font-bold disabled:opacity-50 disabled:cursor-not-allowed"
                  >
                    {uploading || isProcessing ? (
                      <>
                        <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                        {uploading ? t("adas.uploading") : t("adas.analyzing")}
                      </>
                    ) : (
                      <>
                        <Upload className="h-4 w-4 mr-2" />
                        {t("adas.sendVideoAnalysis")}
                      </>
                    )}
                  </Button>
                </div>
              </GlassCard>

              <GlassCard className="p-6">
                <div className="mb-4">
                  <h3 className="text-lg font-bold text-[rgb(var(--primary))] flex items-center gap-2 tracking-wide">
                    <ShieldCheck className="w-4 h-4" />
                    {t("adas.storageProcessTitle")}
                  </h3>
                  <p className="text-xs text-fg-secondary mt-1">
                    {t("adas.storageProcessDesc")}
                  </p>
                </div>
                <div className="text-sm text-fg-secondary space-y-2">
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-[var(--primary-soft)] text-[var(--primary-strong)] border-[color:rgb(var(--primary))]/40">
                      <Upload className="w-3 h-3" />
                      Upload
                    </Badge>
                    <span>{t("adas.step1Process")}</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-[var(--primary-soft)] text-[var(--primary-strong)] border-[color:rgb(var(--primary))]/40">
                      <Sparkles className="w-3 h-3" />
                      AI
                    </Badge>
                    <span>{t("adas.step2Process")}</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-[var(--primary-soft)] text-[var(--primary-strong)] border-[color:rgb(var(--primary))]/40">
                      <Database className="w-3 h-3" />
                      System
                    </Badge>
                    <span>{t("adas.step3Process")}</span>
                  </div>
                </div>
              </GlassCard>
            </div>

          <GlassCard glow="green" className="xl:col-span-2 h-full p-6">
            <div className="mb-4">
              <h3 className="text-xl font-bold text-[rgb(var(--primary))] tracking-wide">
                {t("adas.step2Title")}
              </h3>
              <p className="text-xs text-fg-secondary mt-1">
                {t("adas.step2Desc")}
              </p>
            </div>
            <div className="relative aspect-video bg-black/30 rounded-lg overflow-hidden border-2 border-[color:rgb(var(--primary))]/40 shadow-lg">
              {isProcessing ? (
                <div className="absolute inset-0 glass-card flex flex-col items-center justify-center text-[rgb(var(--primary))] gap-4">
                  <Loader2 className="h-12 w-12 animate-spin text-[rgb(var(--primary))]" />
                  <div className="text-center space-y-2">
                    {/* Show different message based on upload vs processing state */}
                    {uploading ? (
                      <>
                        <p className="text-lg font-semibold">
                          {t("adas.uploadingVideo")}
                        </p>
                        <p className="text-sm text-fg-secondary">
                          {t("adas.uploadingVideoDesc")}
                        </p>
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

                    {/* WebSocket Connection Status - only show when not uploading */}
                    {!uploading && wsIsConnected && (
                      <Badge className="gap-1 bg-[var(--primary-soft)] text-[var(--primary-strong)] border-[color:rgb(var(--primary))]/40">
                        <div className="w-2 h-2 bg-[rgb(var(--primary))] rounded-full animate-pulse" />
                        WebSocket Connected
                      </Badge>
                    )}

                    {/* Upload status badge */}
                    {uploading && (
                      <Badge className="gap-1 bg-[var(--primary-soft)] text-[var(--primary-strong)] border-[color:rgb(var(--primary))]/40">
                        <Upload className="w-3 h-3 animate-pulse" />
                        Đang upload...
                      </Badge>
                    )}
                  </div>

                  {/* Progress Bar */}
                  <div className="w-full max-w-md px-8 space-y-2">
                    <div className="h-3 bg-black/50 rounded-full overflow-hidden border border-[color:rgb(var(--primary))]/40">
                      <div
                        className="h-full bg-linear-to-r from-[rgb(var(--primary))] to-[var(--primary-strong)] transition-all duration-500 ease-out"
                        style={{ width: `${processingProgress}%` }}
                      />
                    </div>
                    <div className="flex justify-between text-xs text-fg-secondary">
                      <span>{processingProgress}%</span>
                      <span>
                        {uploading ? "Đang upload..." : "Đang xử lý..."}
                      </span>
                    </div>
                  </div>

                  {processingMsg && (
                    <p className="text-sm text-neon-yellow max-w-md text-center px-4">
                      {processingMsg}
                    </p>
                  )}
                </div>
              ) : previewUrl ? (
                <video
                  key={previewUrl}
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
                    console.log("VIDEO URL:", previewUrl);
                  }}
                >
                  <source src={previewUrl} type="video/mp4" />
                </video>
              ) : (
                <img
                  src="/images/preview/driving-monitor-preview.webp"
                  alt="Driving monitor preview"
                  className="w-full h-full object-cover"
                  style={{ maxHeight: "600px" }}
                  loading="lazy"
                />
              )}
            </div>
            {processedVideoUrl && !isProcessing ? (
              <div className="mt-4 flex flex-wrap gap-3">
                <Button
                  onClick={resetAnalysis}
                  className="gap-2 bg-linear-to-r from-[rgb(var(--primary))] to-[var(--primary-strong)] text-white font-bold hover:opacity-95"
                >
                  <RefreshCw className="w-4 h-4" />
                  {t("adas.analyzeAnotherVideo")}
                </Button>
                <Button
                  variant="outline"
                  onClick={() => window.history.back()}
                  className="gap-2 glass-card border-[color:rgb(var(--primary))]/40 text-[var(--primary-strong)] hover:bg-[var(--primary-soft)]"
                >
                  <ArrowLeft className="w-4 h-4" />
                  {t("common.back")}
                </Button>
              </div>
            ) : (
              <div className="mt-4" />
            )}
          </GlassCard>
        </div>
      </main>

    </div>
  );
}
