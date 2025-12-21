"use client"

import { useEffect, useState } from "react"
import Link from "next/link"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
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
    <div className="flex flex-col min-h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50 text-gray-900">
      <header className="flex items-center justify-between p-3 sm:p-5 border-b border-white/5 bg-card/70 backdrop-blur-xl">
        <div className="flex items-center gap-2 sm:gap-3">
          <Link href="/">
            <Button variant="ghost" size="icon" className="text-muted-foreground">
              <ArrowLeft className="w-4 h-4 sm:w-5 sm:h-5" />
            </Button>
          </Link>
          <div>
            <div className="flex items-center gap-1 sm:gap-2 flex-wrap">
              <Badge variant="outline" className="gap-1 text-xs">
                <Sparkles className="w-3 h-3 text-primary" />
                <span className="hidden sm:inline">Realtime AI</span>
                <span className="sm:hidden">AI</span>
              </Badge>
              <Badge variant="outline" className="gap-1 text-xs">
                <ShieldCheck className="w-3 h-3 text-success" />
                <span className="hidden sm:inline">Saved to dataset</span>
                <span className="sm:hidden">Saved</span>
              </Badge>
            </div>
            <h1 className="text-lg sm:text-2xl font-bold flex items-center gap-2 mt-1 sm:mt-2">
              <Film className="w-4 h-4 sm:w-5 sm:h-5 text-primary" />
              <span className="hidden sm:inline">ADAS Video Analysis</span>
              <span className="sm:hidden">ADAS Analysis</span>
            </h1>
            <p className="text-xs sm:text-sm text-muted-foreground">
              Upload hoặc dùng video mẫu, AI phân tích và lưu vào dataset.
            </p>
          </div>
        </div>
        <div className="hidden lg:flex items-center gap-2">
          <Badge variant="outline" className="text-xs">Backend: /vision/video</Badge>
          <Badge variant="secondary" className="gap-1 text-xs">
            <Database className="w-3 h-3" />
            Dataset ready
          </Badge>
        </div>
      </header>

      <main className="flex-1 p-3 sm:p-4 lg:p-6">
        <div className="grid gap-4 sm:gap-6 xl:grid-cols-3">
          {stage === "input" ? (
            <div className="space-y-4 xl:col-span-1">
              <Card className="border-blue-300 shadow-lg bg-white">
                <CardHeader>
                  <CardTitle className="flex items-center gap-2">
                    <Upload className="w-4 h-4 text-primary" />
                    1) Chọn video
                  </CardTitle>
                  <CardDescription>Upload video hoặc dùng video mẫu từ dataset</CardDescription>
                </CardHeader>
                <CardContent className="space-y-4">
                  <Input
                    type="file"
                    accept="video/*"
                    onChange={(e) => handleFile(e.target.files?.[0] || null)}
                    disabled={uploading}
                    className="cursor-pointer border-blue-200 bg-blue-50/50 hover:bg-blue-50"
                  />
                  <div className="flex flex-col sm:flex-row gap-2">
                    <Button className="w-full" onClick={uploadAndAnalyze} disabled={uploading || (!file && !previewUrl)}>
                      {uploading ? <Loader2 className="h-4 w-4 mr-2 animate-spin" /> : <Upload className="h-4 w-4 mr-2" />}
                      <span className="hidden sm:inline">Phân tích video</span>
                      <span className="sm:hidden">Phân tích</span>
                    </Button>
                    <Button variant="secondary" onClick={useSampleVideo} disabled={sampleLoading} className="w-full sm:w-auto">
                      {sampleLoading ? <Loader2 className="h-4 w-4 mr-2 animate-spin" /> : <PlayCircle className="h-4 w-4 mr-2" />}
                      <span className="hidden sm:inline">Video mẫu</span>
                      <span className="sm:hidden">Mẫu</span>
                    </Button>
                  </div>

                  <div className="grid grid-cols-2 gap-3 text-sm">
                    <div className="rounded-lg border-2 border-blue-200 bg-blue-50 p-3 hover:bg-blue-100 transition-colors">
                      <div className="text-xs text-blue-700 font-medium">Trạng thái</div>
                      <div className="font-semibold flex items-center gap-2 text-blue-900">
                        <Loader2 className={`h-3.5 w-3.5 ${uploading ? "animate-spin text-blue-600" : "text-gray-400"}`} />
                        {uploading ? "Đang phân tích" : "Sẵn sàng"}
                      </div>
                    </div>
                    <div className="rounded-lg border-2 border-green-200 bg-green-50 p-3 hover:bg-green-100 transition-colors">
                      <div className="text-xs text-green-700 font-medium">Nguồn video</div>
                      <div className="font-semibold text-green-900">{file ? "Upload mới" : previewUrl ? "Video mẫu" : "Chưa chọn"}</div>
                    </div>
                  </div>

                  {processingMsg && (
                    <div className="text-sm text-gray-700 flex items-center gap-2 rounded-md border-2 border-purple-200 bg-purple-50 px-3 py-2">
                      <Loader2 className="h-4 w-4 animate-spin" />
                      {processingMsg}
                    </div>
                  )}

                  {result && (
                    <div className="text-sm space-y-1 bg-indigo-50 p-3 rounded border-2 border-indigo-200">
                      <div className="flex items-center gap-2 text-success">
                        <CheckCircle2 className="h-4 w-4" />
                        Kết quả
                      </div>
                      <pre className="text-xs whitespace-pre-wrap break-all">
                        {JSON.stringify(result, null, 2)}
                      </pre>
                    </div>
                  )}
                </CardContent>
              </Card>

              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center gap-2">
                    <ShieldCheck className="w-4 h-4 text-primary" />
                    Quy trình lưu trữ
                  </CardTitle>
                  <CardDescription className="text-sm">Video đã phân tích sẽ vào dataset và sẵn sàng cho bước kế tiếp.</CardDescription>
                </CardHeader>
                <CardContent className="text-sm text-muted-foreground space-y-2">
                  <div className="flex items-center gap-2">
                    <Badge variant="secondary" className="gap-1"><Upload className="w-3 h-3" />Upload</Badge>
                    <span>Gửi video tới /vision/video</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge variant="secondary" className="gap-1"><Sparkles className="w-3 h-3" />AI</Badge>
                    <span>AI phân tích nội dung video</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge variant="secondary" className="gap-1"><Database className="w-3 h-3" />Dataset</Badge>
                    <span>Lưu kết quả vào dataset và có thể lấy lại bằng “Video mẫu”</span>
                  </div>
                </CardContent>
              </Card>
            </div>
          ) : null}

          <Card className="xl:col-span-2 h-full shadow-lg bg-white border-2 border-blue-200">
            <CardHeader className="space-y-2 bg-gradient-to-r from-blue-50 to-purple-50">
              <div className="flex items-center justify-between">
                <CardTitle className="text-blue-900">2) Xem video đang được phân tích</CardTitle>
                <Badge variant="outline" className="gap-1">
                  <AlertTriangle className="h-3 w-3 text-warning" />
                  Lưu vào dataset tự động
                </Badge>
              </div>
              <CardDescription>
                Video sẽ được gửi tới AI và lưu vào dataset. Bạn có thể dùng video mẫu để tránh upload lớn.
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="relative aspect-video bg-gray-100 rounded-lg overflow-hidden border-2 border-blue-200 shadow-sm">
                {previewUrl ? (
                  <video
                    key={previewUrl}
                    src={previewUrl}
                    controls
                    autoPlay
                    muted
                    loop
                    className="w-full h-full object-contain"
                  />
                ) : (
                  <div className="absolute inset-0 flex flex-col items-center justify-center text-muted-foreground gap-2">
                    <Upload className="w-8 h-8" />
                    <p>Chưa có video. Upload hoặc dùng video mẫu.</p>
                  </div>
                )}
                {uploading && (
                  <div className="absolute inset-0 bg-gradient-to-br from-blue-100 to-purple-100 flex flex-col items-center justify-center text-gray-800 gap-2">
                    <Loader2 className="h-6 w-6 animate-spin" />
                    <p>Đang phân tích...</p>
                  </div>
                )}
              </div>
              {stage === "done" ? (
                <div className="mt-4 flex flex-wrap gap-3">
                  <Button variant="default" onClick={() => setStage("input")} className="gap-2">
                    <RefreshCw className="w-4 h-4" />
                    Phân tích video khác
                  </Button>
                  <Button variant="secondary" onClick={() => window.history.back()} className="gap-2">
                    <ArrowLeft className="w-4 h-4" />
                    Quay lại
                  </Button>
                </div>
              ) : (
                <div className="mt-4 text-sm text-muted-foreground flex items-center gap-2">
                  <AlertTriangle className="h-4 w-4 text-warning" />
                  Dữ liệu sau phân tích sẽ được lưu vào dataset và có thể truy xuất ở bước “Video mẫu”.
                </div>
              )}
            </CardContent>
          </Card>
        </div>
      </main>
    </div>
  )
}
