"use client"

import { useEffect, useState } from "react"
import Link from "next/link"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { GlassCard } from "@/components/ui/glass-card"
import { Input } from "@/components/ui/input"
import { useToast } from "@/components/ui/use-toast"
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog"
import { ScrollArea } from "@/components/ui/scroll-area"
import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"
import { ArrowLeft, Upload, PlayCircle, Film, CheckCircle2, Loader2, AlertTriangle, Sparkles, Database, ShieldCheck, RefreshCw, Clock, FileVideo } from "lucide-react"

type VisionResponse = {
  message?: string
  data?: any
}

type VideoItem = {
  id: number
  job_id: string
  video_filename: string
  video_path: string
  status: string
  progress_percent?: number
  created_at: string
  duration_seconds?: number | null
  video_size_mb?: number | null
}

export default function ADASPage() {
  const { toast } = useToast()
  const [file, setFile] = useState<File | null>(null)
  const [previewUrl, setPreviewUrl] = useState<string | null>(null)
  const [uploading, setUploading] = useState(false)
  const [processingMsg, setProcessingMsg] = useState<string>("")
  const [result, setResult] = useState<VisionResponse | null>(null)
  const [stage, setStage] = useState<"input" | "processing" | "done">("input")

  // Video processing state
  const [currentJobId, setCurrentJobId] = useState<string | null>(null)
  const [processingProgress, setProcessingProgress] = useState(0)
  const [isProcessing, setIsProcessing] = useState(false)
  const [processedVideoUrl, setProcessedVideoUrl] = useState<string | null>(null)
  const [showCompletionDialog, setShowCompletionDialog] = useState(false)

  // Video selection modal state
  const [showVideoDialog, setShowVideoDialog] = useState(false)
  const [availableVideos, setAvailableVideos] = useState<VideoItem[]>([])
  const [loadingVideos, setLoadingVideos] = useState(false)

  useEffect(() => {
    return () => {
      if (previewUrl?.startsWith("blob:")) {
        URL.revokeObjectURL(previewUrl)
      }
    }
  }, [previewUrl])

  useEffect(() => {
    console.log("[PlayerState]", {
      isProcessing,
      stage,
      previewUrl,
    })
  }, [isProcessing, stage, previewUrl])

  const handleFile = (f: File | null) => {
    setResult(null)
    setProcessingMsg("")
    setFile(f)
    if (previewUrl?.startsWith("blob:")) URL.revokeObjectURL(previewUrl)
    setPreviewUrl(f ? URL.createObjectURL(f) : null)
  }

  const uploadAndAnalyze = async () => {
    if (!file) {
      toast({ title: "Chưa chọn video", description: "Vui lòng chọn một file video để phân tích", variant: "destructive" })
      return
    }

    try {
      setUploading(true)
      setIsProcessing(true)
      setStage("processing")
      setProcessingProgress(0)
      setProcessingMsg("Đang tải video lên server...")

      // Step 1: Upload video
      const formData = new FormData()
      formData.append("file", file)

      const uploadRes = await fetch(getApiUrl('/api/video/upload'), {
        method: "POST",
        body: formData,
      })

      if (!uploadRes.ok) {
        throw new Error(`Upload failed: ${uploadRes.status}`)
      }

      const uploadData = await uploadRes.json()
      const jobId = uploadData.job_id || uploadData.id

      if (!jobId) {
        throw new Error('No job_id returned from upload')
      }

      console.log('✅ Upload OK - Job:', jobId.substring(0, 8))
      setCurrentJobId(jobId)
      setUploading(false)

      toast({
        title: "Upload thành công!",
        description: "Video đang được AI phân tích..."
      })

      // Step 2: Monitor progress via SSE
      setProcessingMsg("Đang phân tích video, vui lòng chờ...")
      pollForResult(jobId)

    } catch (err: any) {
      console.error('❌ [Upload] Error:', err)
      toast({
        title: "Lỗi upload",
        description: err.message || "Không thể tải video lên server.",
        variant: "destructive"
      })
      setUploading(false)
      setIsProcessing(false)
      setStage("input")
    }
  }

  // Monitor processing progress via SSE
  const startProgressMonitoring = (jobId: string) => {
    const sseUrl = getApiUrl(`/api/video/stream/${jobId}`)
    console.log('🔄 [SSE] Connecting to:', sseUrl)

    const eventSource = new EventSource(sseUrl)

    eventSource.addEventListener('progress', (event) => {
      try {
        const data = JSON.parse(event.data)
        console.log('📊 [Progress]', data)

        setProcessingProgress(data.progress || 0)
        setProcessingMsg(`Đang phân tích video... ${data.progress || 0}%`)

        if (data.event_count) {
          setProcessingMsg(`Đang phân tích... ${data.progress}% (Phát hiện ${data.event_count} sự kiện)`)
        }
      } catch (err) {
        console.error('❌ [SSE Parse] Error:', err)
      }
    })

    eventSource.addEventListener('complete', (event) => {
      try {
        const data = JSON.parse(event.data)
        console.log('✅ [Complete]', data)

        setProcessingProgress(100)
        setIsProcessing(false)
        eventSource.close()

        // Get result video URL
        fetchProcessedVideo(jobId)

      } catch (err) {
        console.error('❌ [SSE Complete] Error:', err)
      }
    })

    eventSource.addEventListener('error', (event: any) => {
      console.error('❌ [SSE] Error:', event)
      eventSource.close()

      // Fallback to polling if SSE fails
      pollForResult(jobId)
    })
  }

  // Fallback polling if SSE doesn't work
  const pollForResult = async (jobId: string) => {
    const maxAttempts = 60 // 5 minutes max
    let attempts = 0

    const poll = async () => {
      try {
        const res = await fetch(getApiUrl(`/api/video/result/${jobId}`))
        const data = await res.json()

        // Only log if progress changed or status changed
        const newProgress = data.progress_percent || 0
        if (attempts === 0 || newProgress !== processingProgress || data.status === 'completed') {
          console.log(`[Job ${jobId.substring(0, 8)}] Status: ${data.status}, Progress: ${newProgress}%`)
        }

        setProcessingProgress(newProgress)
        setProcessingMsg(`Đang phân tích... ${newProgress}%`)

        if (data.status === 'completed') {
          setIsProcessing(false)
          fetchProcessedVideo(jobId)
          return
        }

        if (data.status === 'error') {
          throw new Error(data.error_message || 'Processing failed')
        }

        attempts++
        if (attempts < maxAttempts && data.status !== 'completed') {
          setTimeout(poll, 2000) // Poll every 2 seconds for faster updates
        } else if (attempts >= maxAttempts) {
          throw new Error('Processing timeout')
        }

      } catch (err: any) {
        console.error('❌ [Poll] Error:', err)
        setIsProcessing(false)
        toast({
          title: "Lỗi phân tích",
          description: err.message,
          variant: "destructive"
        })
        setStage("input")
      }
    }

    poll()
  }

  // Fetch processed video URL
  const fetchProcessedVideo = async (jobId: string) => {
    try {
      const res = await fetch(getApiUrl(`/api/video/result/${jobId}`))
      const data = await res.json()

      console.log('✅ Completed! Processing time:', data.processing_time_seconds, 's')

      if (data.status === 'completed' && data.video_filename) {
        // Backend spec: GET /api/video/download/{job_id}/{filename}
        // Filename format: original_name_result.mp4
        const resultFilename = data.video_filename.replace('.mp4', '_result.mp4')
        const downloadUrl = getApiUrl(`/api/video/download/${jobId}/${resultFilename}`)


        setProcessedVideoUrl(downloadUrl)
        setPreviewUrl(downloadUrl)  // Auto-set video immediately
        setShowCompletionDialog(true)
        setStage("done")

        toast({
          title: "Phân tích hoàn tất!",
          description: `Thời gian xử lý: ${data.processing_time_seconds || 0}s`
        })
      } else {
        console.error('❌ [Result] Job not completed:', data)
        throw new Error('Job not completed or missing filename')
      }

      setResult(data)

    } catch (err: any) {
      console.error('❌ [FetchResult] Error:', err)
      toast({
        title: "Lỗi lấy kết quả",
        description: err.message,
        variant: "destructive"
      })
    }
  }

  // User confirms to view processed video
  const viewProcessedVideo = () => {
    setShowCompletionDialog(false)
    if (processedVideoUrl) {
      setPreviewUrl(processedVideoUrl)
    }
  }

  // Open video selection dialog
  const useSampleVideo = async () => {
    try {
      setLoadingVideos(true)
      setShowVideoDialog(true)

      // Backend: GET /api/video/list
      const res = await fetch(getApiUrl(`${API_ENDPOINTS.VIDEOS_LIST}?limit=20`))
      const data = await res.json()

      console.log('📹 [VideoList] Response:', data)

      // Parse response: { videos: [...], total: ... }
      let videos: VideoItem[] = []
      if (data?.videos && Array.isArray(data.videos)) {
        videos = data.videos
      } else if (Array.isArray(data)) {
        videos = data
      }

      setAvailableVideos(videos)

      if (videos.length === 0) {
        toast({
          title: "Chưa có video",
          description: "Hệ thống chưa có video nào. Hãy upload video mới.",
        })
      }
    } catch (err) {
      console.error('❌ [VideoList] Error:', err)
      toast({
        title: "Lỗi lấy danh sách video",
        description: "Không thể kết nối tới backend.",
        variant: "destructive"
      })
    } finally {
      setLoadingVideos(false)
    }
  }

  // Select a video from the list
  const selectVideo = (video: VideoItem) => {
    // If completed, check for result URL. Otherwise fallback to raw sample URL.
    let playUrl = ""

    // Construct Raw URL: /api/video/sample/{job_id}/{filename}
    // Construct Result URL: /api/video/download/{job_id}/{filename_result.mp4}

    if (video.status === 'completed') {
      const resultFilename = video.video_filename.replace('.mp4', '_result.mp4')
      playUrl = getApiUrl(API_ENDPOINTS.VIDEO_DOWNLOAD(video.job_id, resultFilename))

      toast({
        title: "Đã chọn video kết quả",
        description: `Đang phát kết quả phân tích của: ${video.video_filename}`,
      })

      // Set stages to done so it shows up
      setStage("done")
      setIsProcessing(false)
      setProcessedVideoUrl(playUrl)

    } else {
      playUrl = getApiUrl(API_ENDPOINTS.VIDEO_SAMPLE(video.job_id, video.video_filename))

      toast({
        title: "Đã chọn video gốc",
        description: `Đang phát video gốc: ${video.video_filename}`,
      })

      // Reset stages
      setStage("input")
      setIsProcessing(false)
      setResult(null)
    }

    console.log('✅ [VideoSelect] Playing:', playUrl)
    setPreviewUrl(playUrl)
    setShowVideoDialog(false)
    setProcessingMsg("")
  }

  return (
    <div className="flex flex-col min-h-screen bg-bg-primary text-fg-primary">
      <header className="flex items-center justify-between p-3 sm:p-5 border-b border-white/10 glass-card backdrop-blur-xl">
        <div className="flex items-center gap-2 sm:gap-3">
          <Link href="/">
            <Button variant="ghost" size="icon" className="text-fg-secondary hover:text-neon-cyan">
              <ArrowLeft className="w-4 h-4 sm:w-5 sm:h-5" />
            </Button>
          </Link>
          <div>
            <div className="flex items-center gap-1 sm:gap-2 flex-wrap">
              <Badge className="gap-1 text-xs bg-neon-cyan/20 text-neon-cyan border-neon-cyan/50">
                <Sparkles className="w-3 h-3" />
                <span className="hidden sm:inline">Realtime AI</span>
                <span className="sm:hidden">AI</span>
              </Badge>
              <Badge className="gap-1 text-xs bg-neon-green/20 text-neon-green border-neon-green/50">
                <ShieldCheck className="w-3 h-3" />
                <span className="hidden sm:inline">Saved to system</span>
                <span className="sm:hidden">Saved</span>
              </Badge>
            </div>
            <h1 className="text-lg sm:text-2xl font-bold flex items-center gap-2 mt-1 sm:mt-2 text-neon-cyan tracking-wider">
              <Film className="w-4 h-4 sm:w-5 sm:h-5" />
              <span className="hidden sm:inline">ADAS VIDEO ANALYSIS</span>
              <span className="sm:hidden">ADAS ANALYSIS</span>
            </h1>
            <p className="text-xs sm:text-sm text-fg-secondary">
              Upload hoặc dùng video mẫu, AI phân tích và lưu vào hệ thống.
            </p>
          </div>
        </div>
        <div className="hidden lg:flex items-center gap-2">
        </div>
      </header>

      <main className="flex-1 p-3 sm:p-4 lg:p-6">
        <div className="grid gap-4 sm:gap-6 xl:grid-cols-3">
          {stage === "input" ? (
            <div className="space-y-4 xl:col-span-1">
              <GlassCard glow="cyan" className="p-6">
                <div className="mb-4">
                  <h3 className="text-lg font-bold text-neon-cyan flex items-center gap-2 tracking-wide">
                    <Upload className="w-4 h-4" />
                    1) CHỌN VIDEO
                  </h3>
                  <p className="text-xs text-fg-secondary mt-1">Upload video hoặc dùng video mẫu từ hệ thống</p>
                </div>
                <div className="space-y-4">
                  <Input
                    type="file"
                    accept="video/*"
                    onChange={(e) => handleFile(e.target.files?.[0] || null)}
                    disabled={uploading}
                    className="
                    cursor-pointer glass-card border-neon-cyan/30
                    text-fg-primary file:text-neon-cyan
                    video-file-input
                  "
                  />
                  <div className="flex flex-col sm:flex-row gap-2">
                    <Button
                      onClick={useSampleVideo}
                      disabled={loadingVideos}
                      className="flex-1 glass-card border-2 border-neon-cyan/50 bg-neon-cyan/10 text-neon-cyan hover:bg-neon-cyan/20 font-semibold"
                    >
                      {/* Bọc nội dung trong 1 khối flex hàng ngang */}
                      <span className="flex items-center justify-center gap-2">
                        {loadingVideos ? (
                          <Loader2 className="h-4 w-4 animate-spin" />
                        ) : (
                          <PlayCircle className="h-4 w-4" />
                        )}

                        <span className="hidden sm:inline">Video mẫu</span>
                        <span className="sm:hidden">Mẫu</span>
                      </span>
                    </Button>
                  </div>

                  <div className="grid grid-cols-2 gap-3 text-sm">
                    <div className={`rounded-lg glass-card border-2 p-3 ${file || previewUrl
                      ? "border-neon-green/50"
                      : "border-neon-red/50"
                      }`}>
                      <div className="text-xs text-fg-secondary font-medium tracking-wide">
                        Trạng thái
                      </div>

                      <div
                        className={`flex items-center gap-2 text-sm font-medium
                            antialiased
                            transition-colors duration-300
                            ${uploading
                            ? "text-neon-yellow drop-shadow-[0_0_6px_rgba(250,204,21,0.45)]"
                            : (file || previewUrl)
                              ? "text-neon-green drop-shadow-[0_0_6px_rgba(34,197,94,0.45)]"
                              : "text-neon-red drop-shadow-[0_0_6px_rgba(239,68,68,0.45)]"
                          }
                          `}
                      >
                        <Loader2
                          className={`h-3.5 w-3.5
                              ${uploading ? "animate-spin opacity-90" : "opacity-70"}
                            `}
                        />
                        <span className="leading-none mt-[5px]">
                          {uploading
                            ? "Đang phân tích"
                            : (file || previewUrl)
                              ? "Sẵn sàng"
                              : "Chưa sẵn sàng"}
                        </span>
                      </div>

                    </div>
                    <div className="rounded-lg glass-card border-2 border-neon-green/30 p-3">
                      <div className="text-xs text-fg-secondary font-medium">Nguồn video</div>
                      <div className="font-semibold text-neon-green">{file ? "Upload mới" : previewUrl ? "Video mẫu" : "Chưa chọn"}</div>
                    </div>
                  </div>

                  {processingMsg && (
                    <div className="text-sm text-fg-primary flex items-center gap-2 rounded-md glass-card border-2 border-neon-yellow/50 px-3 py-2">
                      <Loader2 className="h-4 w-4 animate-spin text-neon-yellow" />
                      {processingMsg}
                    </div>
                  )}

                  {result && (
                    <div className="text-sm space-y-1 glass-card border-2 border-neon-green/50 p-3 rounded">
                      <div className="flex items-center gap-2 text-neon-green">
                        <CheckCircle2 className="h-4 w-4" />
                        Kết quả
                      </div>
                      <pre className="text-xs whitespace-pre-wrap break-all text-fg-secondary">
                        {JSON.stringify(result, null, 2)}
                      </pre>
                    </div>
                  )}

                  {/* Upload Button */}
                  <Button
                    onClick={uploadAndAnalyze}
                    disabled={!file || uploading || isProcessing}
                    className="w-full glass-card border-2 border-neon-green/50 bg-neon-green/10 text-neon-green hover:bg-neon-green/20 font-bold disabled:opacity-50 disabled:cursor-not-allowed"
                  >
                    {uploading || isProcessing ? (
                      <>
                        <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                        {uploading ? "Đang tải lên..." : "Đang phân tích..."}
                      </>
                    ) : (
                      <>
                        <Upload className="h-4 w-4 mr-2" />
                        Gửi Video Phân Tích
                      </>
                    )}
                  </Button>
                </div>
              </GlassCard>

              <GlassCard className="p-6">
                <div className="mb-4">
                  <h3 className="text-lg font-bold text-neon-green flex items-center gap-2 tracking-wide">
                    <ShieldCheck className="w-4 h-4" />
                    QUY TRÌNH LƯU TRỮ
                  </h3>
                  <p className="text-xs text-fg-secondary mt-1">Video đã phân tích sẽ vào hệ thống và sẵn sàng cho bước kế tiếp.</p>
                </div>
                <div className="text-sm text-fg-secondary space-y-2">
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-neon-cyan/20 text-neon-cyan border-neon-cyan/50"><Upload className="w-3 h-3" />Upload</Badge>
                    <span>Gửi video tới hệ thống</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-neon-yellow/20 text-neon-yellow border-neon-yellow/50"><Sparkles className="w-3 h-3" />AI</Badge>
                    <span>AI phân tích nội dung video</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-neon-green/20 text-neon-green border-neon-green/50"><Database className="w-3 h-3" />System</Badge>
                    <span>Lưu kết quả vào hệ thống và có thể lấy lại bằng "Video mẫu"</span>
                  </div>
                </div>
              </GlassCard>
            </div>
          ) : null}

          <GlassCard glow="green" className="xl:col-span-2 h-full p-6">
            <div className="mb-4">
              <div className="flex items-center justify-between">
                <h3 className="text-xl font-bold text-neon-green tracking-wide">2) XEM VIDEO ĐANG ĐƯỢC PHÂN TÍCH</h3>
                <Badge className="
                  gap-1
                  bg-red-500/10
                  text-red-400
                  border border-red-500/40
                  animate-pulse
                  [animation-duration:1s]
                  shadow-[0_0_12px_rgba(255,0,0,0.6)]
                ">
                  <AlertTriangle className="h-3 w-3" />
                  Dữ liệu đã được lưu mẫu
                </Badge>
              </div>
              <p className="text-xs text-fg-secondary mt-1">
                Video sẽ được gửi tới AI và lưu vào hệ thống. Bạn có thể dùng video mẫu để tránh upload lớn.
              </p>
            </div>
            <div className="relative aspect-video bg-black/30 rounded-lg overflow-hidden border-2 border-neon-green/50 shadow-lg">
              {isProcessing ? (
                <div className="absolute inset-0 glass-card flex flex-col items-center justify-center text-neon-cyan gap-4">
                  <Loader2 className="h-12 w-12 animate-spin text-neon-cyan" />
                  <div className="text-center space-y-2">
                    <p className="text-lg font-semibold">Đang phân tích video...</p>
                    <p className="text-sm text-fg-secondary">Vui lòng chờ trong giây lát</p>
                  </div>

                  {/* Progress Bar */}
                  <div className="w-full max-w-md px-8 space-y-2">
                    <div className="h-3 bg-black/50 rounded-full overflow-hidden border border-neon-cyan/50">
                      <div
                        className="h-full bg-gradient-to-r from-neon-cyan to-neon-green transition-all duration-500 ease-out"
                        style={{ width: `${processingProgress}%` }}
                      />
                    </div>
                    <div className="flex justify-between text-xs text-fg-secondary">
                      <span>{processingProgress}%</span>
                      <span>Đang xử lý...</span>
                    </div>
                  </div>

                  {processingMsg && (
                    <p className="text-sm text-neon-yellow animate-pulse">{processingMsg}</p>
                  )}
                </div>
              ) : previewUrl && stage === "done" ? (
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
                  <source
                    src="{previewUrl}"
                    type="video/mp4"
                  />
                </video>
              ) : (
                <div className="absolute inset-0 flex flex-col items-center justify-center text-fg-secondary gap-2">
                  <Upload className="w-8 h-8 text-neon-cyan" />
                  <p>Chưa có video. Upload hoặc dùng video mẫu.</p>
                </div>
              )}

            </div>
            {stage === "done" ? (
              <div className="mt-4 flex flex-wrap gap-3">
                <Button
                  onClick={() => setStage("input")}
                  className="gap-2 bg-gradient-to-r from-neon-cyan to-neon-green text-black font-bold hover:from-neon-cyan/80 hover:to-neon-green/80"
                >
                  <RefreshCw className="w-4 h-4" />
                  Phân tích video khác
                </Button>
                <Button
                  variant="outline"
                  onClick={() => window.history.back()}
                  className="gap-2 glass-card border-neon-cyan/50 text-neon-cyan hover:bg-neon-cyan/10"
                >
                  <ArrowLeft className="w-4 h-4" />
                  Quay lại
                </Button>
              </div>
            ) : (
              <div className="mt-4 text-sm text-fg-secondary flex items-center gap-2">
                <AlertTriangle
                  className="
                    h-10 w-10
                    text-red-500
                    animate-pulse
                    [animation-duration:0.8s]
                    drop-shadow-[0_0_8px_rgba(255,0,0,0.8)]
                    drop-shadow-[0_0_16px_rgba(255,0,0,1)]
                  "

                />

                Dữ liệu sau phân tích sẽ được lưu vào hệ thống và có thể truy xuất ở bước "Video mẫu".
              </div>

            )}
          </GlassCard>
        </div>
      </main>

      {/* Video Selection Dialog */}
      <Dialog open={showVideoDialog} onOpenChange={setShowVideoDialog}>
        <DialogContent className="glass-card border-2 border-neon-cyan/50 max-w-4xl max-h-[80vh]">
          <DialogHeader>
            <DialogTitle className="text-2xl font-bold text-neon-cyan flex items-center gap-2">
              <FileVideo className="w-6 h-6" />
              Chọn Video Mẫu
            </DialogTitle>
            <DialogDescription className="text-fg-secondary">
              Chọn một video từ database để phân tích. Tổng cộng có {availableVideos.length} video.
            </DialogDescription>
          </DialogHeader>

          <ScrollArea className="h-[500px] pr-4">
            {loadingVideos ? (
              <div className="flex items-center justify-center h-40">
                <Loader2 className="h-8 w-8 animate-spin text-neon-cyan" />
                <span className="ml-3 text-fg-secondary">Đang tải danh sách video...</span>
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
                          <FileVideo className="w-5 h-5 text-neon-cyan flex-shrink-0" />
                          <h4 className="font-semibold text-fg-primary truncate group-hover:text-neon-cyan transition-colors">
                            {video.video_filename || `Video #${video.id}`}
                          </h4>
                        </div>

                        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 text-xs text-fg-secondary">
                          <div className="flex items-center gap-1">
                            <Clock className="w-3 h-3" />
                            <span>
                              {video.duration_seconds
                                ? `${Math.floor(video.duration_seconds / 60)}:${(video.duration_seconds % 60).toString().padStart(2, '0')}`
                                : 'N/A'}
                            </span>
                          </div>

                          <div className="flex items-center gap-1">
                            <Database className="w-3 h-3" />
                            <span>
                              {video.video_size_mb
                                ? `${video.video_size_mb.toFixed(1)} MB`
                                : 'N/A'}
                            </span>
                          </div>

                          <div className="flex items-center gap-1">
                            {video.status === 'completed' ? (
                              <Badge variant="outline" className="text-[10px] h-5 border-neon-green text-neon-green bg-neon-green/10">
                                <CheckCircle2 className="w-3 h-3 mr-1" />
                                Đã xong
                              </Badge>
                            ) : video.status === 'processing' ? (
                              <Badge variant="outline" className="text-[10px] h-5 border-neon-yellow text-neon-yellow bg-neon-yellow/10">
                                <Loader2 className="w-3 h-3 mr-1 animate-spin" />
                                Đang chạy
                              </Badge>
                            ) : (
                              <Badge variant="outline" className="text-[10px] h-5 border-neon-cyan text-neon-cyan bg-neon-cyan/10">
                                Chưa chạy
                              </Badge>
                            )}
                          </div>

                          {video.created_at && (
                            <div className="text-xs opacity-70">
                              {new Date(video.created_at).toLocaleDateString('vi-VN')}
                            </div>
                          )}
                        </div>
                      </div>

                      <Button
                        size="sm"
                        className="glass-card bg-neon-cyan/20 text-neon-cyan border border-neon-cyan/50 hover:bg-neon-cyan/30 flex-shrink-0"
                      >
                        <PlayCircle className="w-4 h-4 mr-1" />
                        Chọn
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </ScrollArea>
        </DialogContent>
      </Dialog>

      {/* Processing Completion Dialog */}
      <Dialog open={showCompletionDialog} onOpenChange={setShowCompletionDialog}>
        <DialogContent className="glass-card border-2 border-neon-green/50">
          <DialogHeader>
            <DialogTitle className="text-2xl font-bold text-neon-green flex items-center gap-2">
              <CheckCircle2 className="w-6 h-6" />
              Phân Tích Hoàn Tất!
            </DialogTitle>
            <DialogDescription className="text-fg-secondary text-base mt-2">
              Video của bạn đã được AI phân tích thành công. Bạn có muốn xem video đã qua xử lý không?
            </DialogDescription>
          </DialogHeader>

          <div className="space-y-4 mt-4">
            {result && (
              <div className="glass-card border border-neon-cyan/30 p-4 rounded-lg">
                <h4 className="text-sm font-semibold text-neon-cyan mb-2">Thông tin phân tích:</h4>
                <div className="text-xs text-fg-secondary space-y-1">
                  <div className="flex justify-between">
                    <span>Job ID:</span>
                    <span className="font-mono text-neon-green">{currentJobId}</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Trạng thái:</span>
                    <Badge className="bg-neon-green/20 text-neon-green border-neon-green/50">
                      Hoàn thành
                    </Badge>
                  </div>
                </div>
              </div>
            )}

            <div className="flex gap-3">
              <Button
                onClick={viewProcessedVideo}
                className="flex-1 bg-gradient-to-r from-neon-cyan to-neon-green text-black font-bold hover:from-neon-cyan/80 hover:to-neon-green/80"
              >
                <PlayCircle className="w-5 h-5 mr-2" />
                Xem Video Ngay
              </Button>
              <Button
                variant="outline"
                onClick={() => {
                  setShowCompletionDialog(false)
                  setStage("input")
                }}
                className="glass-card border-neon-cyan/50 text-neon-cyan hover:bg-neon-cyan/10"
              >
                Đóng
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div >
  )
}
