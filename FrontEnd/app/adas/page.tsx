"use client"

import { useEffect, useState } from "react"
import Link from "next/link"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { GlassCard } from "@/components/ui/glass-card"
import { Input } from "@/components/ui/input"
import { useToast } from "@/components/ui/use-toast"
import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"
import { ArrowLeft, Upload, PlayCircle, Film, CheckCircle2, Loader2, AlertTriangle, Sparkles, Database, ShieldCheck, RefreshCw } from "lucide-react"

type VisionResponse = {
  message?: string
  data?: any
}

export default function ADASPage() {
  const { toast } = useToast()
  const [file, setFile] = useState<File | null>(null)
  const [previewUrl, setPreviewUrl] = useState<string | null>(null)
  const [uploading, setUploading] = useState(false)
  const [processingMsg, setProcessingMsg] = useState<string>("")
  const [result, setResult] = useState<VisionResponse | null>(null)
  const [sampleLoading, setSampleLoading] = useState(false)
  const [stage, setStage] = useState<"input" | "done">("input")

  useEffect(() => {
    return () => {
      if (previewUrl?.startsWith("blob:")) {
        URL.revokeObjectURL(previewUrl)
      }
    }
  }, [previewUrl])

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
      setProcessingMsg("Đang tải và phân tích video...")
      const formData = new FormData()
      formData.append("file", file)
      const res = await fetch(getApiUrl(API_ENDPOINTS.VISION_VIDEO), {
        method: "POST",
        body: formData,
      })
      const data = await res.json()
      setResult(data)
      setProcessingMsg("")
      toast({ title: "Đã gửi video", description: "Video đang được AI phân tích và lưu vào dataset." })
      setStage("done")
    } catch (err) {
      console.error(err)
      toast({ title: "Lỗi phân tích", description: "Không thể gửi video lên BE.", variant: "destructive" })
    } finally {
      setUploading(false)
    }
  }

  const useSampleVideo = async () => {
    try {
      setSampleLoading(true)
      setResult(null)
      setProcessingMsg("Đang lấy video mẫu...")
      const res = await fetch(getApiUrl(API_ENDPOINTS.DATASET))
      const data = await res.json()
      const first = (data?.data && Array.isArray(data.data) ? data.data[0] : Array.isArray(data) ? data[0] : null) || null
      const candidate = first?.video_url || first?.file_url || first?.file_path || null
      if (candidate) {
        setPreviewUrl(candidate)
        setProcessingMsg("Video mẫu sẵn sàng, hãy phân tích hoặc xem trước.")
      } else {
        setProcessingMsg("")
        toast({ title: "Không tìm thấy video mẫu", description: "Dataset chưa có video mẫu.", variant: "destructive" })
      }
    } catch (err) {
      console.error(err)
      setProcessingMsg("")
      toast({ title: "Lỗi lấy video mẫu", description: "Không thể lấy video mẫu từ dataset.", variant: "destructive" })
    } finally {
      setSampleLoading(false)
    }
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
                <span className="hidden sm:inline">Saved to dataset</span>
                <span className="sm:hidden">Saved</span>
              </Badge>
            </div>
            <h1 className="text-lg sm:text-2xl font-bold flex items-center gap-2 mt-1 sm:mt-2 text-neon-cyan tracking-wider">
              <Film className="w-4 h-4 sm:w-5 sm:h-5" />
              <span className="hidden sm:inline">ADAS VIDEO ANALYSIS</span>
              <span className="sm:hidden">ADAS ANALYSIS</span>
            </h1>
            <p className="text-xs sm:text-sm text-fg-secondary">
              Upload hoặc dùng video mẫu, AI phân tích và lưu vào dataset.
            </p>
          </div>
        </div>
        <div className="hidden lg:flex items-center gap-2">
          <Badge className="text-xs glass-card border-neon-cyan/50 text-neon-cyan">Backend: /vision/video</Badge>
          <Badge className="gap-1 text-xs bg-neon-green/20 text-neon-green border-neon-green/50">
            <Database className="w-3 h-3" />
            Dataset ready
          </Badge>
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
                  <p className="text-xs text-fg-secondary mt-1">Upload video hoặc dùng video mẫu từ dataset</p>
                </div>
                <div className="space-y-4">
                  <Input
                    type="file"
                    accept="video/*"
                    onChange={(e) => handleFile(e.target.files?.[0] || null)}
                    disabled={uploading}
                    className="cursor-pointer glass-card border-neon-cyan/30 text-fg-primary file:text-neon-cyan"
                  />
                  <div className="flex flex-col sm:flex-row gap-2">
                    <Button
                      className="flex-1 bg-gradient-to-r from-neon-cyan to-neon-green text-black font-bold hover:from-neon-cyan/80 hover:to-neon-green/80"
                      onClick={uploadAndAnalyze}
                      disabled={uploading || (!file && !previewUrl)}
                    >
                      {uploading ? <Loader2 className="h-4 w-4 mr-2 animate-spin" /> : <Upload className="h-4 w-4 mr-2" />}
                      <span className="hidden sm:inline">Phân tích video</span>
                      <span className="sm:hidden">Phân tích</span>
                    </Button>
                    <Button
                      onClick={useSampleVideo}
                      disabled={sampleLoading}
                      className="flex-1 glass-card border-2 border-neon-cyan/50 bg-neon-cyan/10 text-neon-cyan hover:bg-neon-cyan/20 font-semibold"
                    >
                      {sampleLoading ? <Loader2 className="h-4 w-4 mr-2 animate-spin" /> : <PlayCircle className="h-4 w-4 mr-2" />}
                      <span className="hidden sm:inline">Video mẫu</span>
                      <span className="sm:hidden">Mẫu</span>
                    </Button>
                  </div>

                  <div className="grid grid-cols-2 gap-3 text-sm">
                    <div className={`rounded-lg glass-card border-2 p-3 ${file || previewUrl
                        ? "border-neon-green/50"
                        : "border-neon-red/50"
                      }`}>
                      <div className="text-xs text-fg-secondary font-medium">Trạng thái</div>
                      <div className={`font-semibold flex items-center gap-2 digital-number ${uploading
                          ? "text-neon-yellow"
                          : (file || previewUrl)
                            ? "text-neon-green"
                            : "text-neon-red"
                        }`}>
                        <Loader2 className={`h-3.5 w-3.5 ${uploading ? "animate-spin" : "text-fg-muted"}`} />
                        {uploading ? "Đang phân tích" : (file || previewUrl) ? "Sẵn sàng" : "Chưa sẵn sàng"}
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
                </div>
              </GlassCard>

              <GlassCard className="p-6">
                <div className="mb-4">
                  <h3 className="text-lg font-bold text-neon-green flex items-center gap-2 tracking-wide">
                    <ShieldCheck className="w-4 h-4" />
                    QUY TRÌNH LƯU TRỮ
                  </h3>
                  <p className="text-xs text-fg-secondary mt-1">Video đã phân tích sẽ vào dataset và sẵn sàng cho bước kế tiếp.</p>
                </div>
                <div className="text-sm text-fg-secondary space-y-2">
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-neon-cyan/20 text-neon-cyan border-neon-cyan/50"><Upload className="w-3 h-3" />Upload</Badge>
                    <span>Gửi video tới /vision/video</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-neon-yellow/20 text-neon-yellow border-neon-yellow/50"><Sparkles className="w-3 h-3" />AI</Badge>
                    <span>AI phân tích nội dung video</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className="gap-1 bg-neon-green/20 text-neon-green border-neon-green/50"><Database className="w-3 h-3" />Dataset</Badge>
                    <span>Lưu kết quả vào dataset và có thể lấy lại bằng "Video mẫu"</span>
                  </div>
                </div>
              </GlassCard>
            </div>
          ) : null}

          <GlassCard glow="green" className="xl:col-span-2 h-full p-6">
            <div className="mb-4">
              <div className="flex items-center justify-between">
                <h3 className="text-xl font-bold text-neon-green tracking-wide">2) XEM VIDEO ĐANG ĐƯỢC PHÂN TÍCH</h3>
                <Badge className="gap-1 bg-neon-yellow/20 text-neon-yellow border-neon-yellow/50">
                  <AlertTriangle className="h-3 w-3" />
                  Lưu vào dataset tự động
                </Badge>
              </div>
              <p className="text-xs text-fg-secondary mt-1">
                Video sẽ được gửi tới AI và lưu vào dataset. Bạn có thể dùng video mẫu để tránh upload lớn.
              </p>
            </div>
            <div className="relative aspect-video bg-black/30 rounded-lg overflow-hidden border-2 border-neon-green/50 shadow-lg">
              {previewUrl ? (
                <video
                  key={previewUrl}
                  src={previewUrl}
                  controls
                  autoPlay
                  muted
                  loop
                  className="w-full h-full object-contain"
                  style={{ maxHeight: "600px" }}
                />
              ) : (
                <div className="absolute inset-0 flex flex-col items-center justify-center text-fg-secondary gap-2">
                  <Upload className="w-8 h-8 text-neon-cyan" />
                  <p>Chưa có video. Upload hoặc dùng video mẫu.</p>
                </div>
              )}
              {uploading && (
                <div className="absolute inset-0 glass-card flex flex-col items-center justify-center text-neon-cyan gap-2">
                  <Loader2 className="h-6 w-6 animate-spin" />
                  <p>Đang phân tích...</p>
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
                <AlertTriangle className="h-4 w-4 text-neon-yellow" />
                Dữ liệu sau phân tích sẽ được lưu vào dataset và có thể truy xuất ở bước "Video mẫu".
              </div>
            )}
          </GlassCard>
        </div>
      </main>
    </div>
  )
}
