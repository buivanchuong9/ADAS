"use client";

import { useState } from "react";
import { Upload, CheckCircle, XCircle, Loader2, FileVideo } from "lucide-react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Textarea } from "@/components/ui/textarea";
import { Progress } from "@/components/ui/progress";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { Badge } from "@/components/ui/badge";

interface UploadResult {
  id: number;
  filename: string;
  file_size_mb: number;
  uploaded_at: string;
  status: string;
  ready_for_training: boolean;
}

export function VideoUploadCard() {
  const [file, setFile] = useState<File | null>(null);
  const [description, setDescription] = useState("");
  const [uploading, setUploading] = useState(false);
  const [progress, setProgress] = useState(0);
  const [result, setResult] = useState<UploadResult | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handleFileSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (e.target.files && e.target.files[0]) {
      setFile(e.target.files[0]);
      setResult(null);
      setError(null);
    }
  };

  const handleUpload = async () => {
    if (!file) {
      setError("Vui lòng chọn file video");
      return;
    }

    setUploading(true);
    setProgress(0);
    setError(null);
    setResult(null);

    try {
      const formData = new FormData();
      formData.append("file", file);
      if (description) {
        formData.append("description", description);
      }

      const apiUrl = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";

      // Upload file
      const response = await fetch(`${apiUrl}/api/videos/upload`, {
        method: "POST",
        body: formData,
      });

      if (!response.ok) {
        throw new Error(`Upload failed: ${response.statusText}`);
      }

      const data = await response.json();

      if (data.success) {
        setResult(data.data);
        setProgress(100);
        
        // Auto process for training
        setTimeout(async () => {
          try {
            const processResponse = await fetch(
              `${apiUrl}/api/videos/${data.data.id}/process`,
              { method: "POST" }
            );
            const processData = await processResponse.json();
            console.log("✅ Video processed for training:", processData);
          } catch (err) {
            console.error("Process error:", err);
          }
        }, 1000);
      } else {
        throw new Error(data.message || "Upload failed");
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : "Upload failed");
    } finally {
      setUploading(false);
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          <FileVideo className="h-5 w-5" />
          Upload Video cho Training
        </CardTitle>
        <CardDescription>
          Upload video để hệ thống tự động trích xuất frames và cải thiện model YOLO
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        {/* File Input */}
        <div className="space-y-2">
          <Label htmlFor="video-file">Chọn Video File</Label>
          <Input
            id="video-file"
            type="file"
            accept="video/*,.mp4,.avi,.mov,.mkv,.webm"
            onChange={handleFileSelect}
            disabled={uploading}
          />
          {file && (
            <div className="flex items-center gap-2 text-sm text-muted-foreground">
              <FileVideo className="h-4 w-4" />
              <span>{file.name}</span>
              <Badge variant="outline">
                {(file.size / 1024 / 1024).toFixed(2)} MB
              </Badge>
            </div>
          )}
        </div>

        {/* Description */}
        <div className="space-y-2">
          <Label htmlFor="description">Mô tả (tùy chọn)</Label>
          <Textarea
            id="description"
            placeholder="VD: Video giao thông ban ngày, nhiều xe..."
            value={description}
            onChange={(e) => setDescription(e.target.value)}
            disabled={uploading}
            rows={2}
          />
        </div>

        {/* Upload Button */}
        <Button
          onClick={handleUpload}
          disabled={!file || uploading}
          className="w-full"
        >
          {uploading ? (
            <>
              <Loader2 className="mr-2 h-4 w-4 animate-spin" />
              Đang upload...
            </>
          ) : (
            <>
              <Upload className="mr-2 h-4 w-4" />
              Upload & Training
            </>
          )}
        </Button>

        {/* Progress */}
        {uploading && (
          <div className="space-y-2">
            <Progress value={progress} className="h-2" />
            <p className="text-sm text-center text-muted-foreground">
              {progress}% hoàn thành
            </p>
          </div>
        )}

        {/* Success Result */}
        {result && (
          <Alert className="border-green-500 bg-green-50">
            <CheckCircle className="h-4 w-4 text-green-600" />
            <AlertDescription className="text-green-800">
              <div className="space-y-2">
                <p className="font-semibold">✅ Upload thành công!</p>
                <div className="text-sm space-y-1">
                  <p>📁 File: {result.filename}</p>
                  <p>💾 Size: {result.file_size_mb} MB</p>
                  <p>🆔 ID: {result.id}</p>
                  <p>⏰ Thời gian: {new Date(result.uploaded_at).toLocaleString()}</p>
                  <div className="flex items-center gap-2 mt-2">
                    <Badge variant="default" className="bg-green-600">
                      {result.status}
                    </Badge>
                    {result.ready_for_training && (
                      <Badge variant="outline" className="border-green-600 text-green-700">
                        ✓ Sẵn sàng training
                      </Badge>
                    )}
                  </div>
                  <p className="text-xs mt-2 text-muted-foreground">
                    💡 Video đang được xử lý tự động để trích xuất frames và labels cho training
                  </p>
                </div>
              </div>
            </AlertDescription>
          </Alert>
        )}

        {/* Error */}
        {error && (
          <Alert variant="destructive">
            <XCircle className="h-4 w-4" />
            <AlertDescription>
              <p className="font-semibold">❌ Lỗi upload</p>
              <p className="text-sm">{error}</p>
            </AlertDescription>
          </Alert>
        )}

        {/* Info */}
        <Alert>
          <AlertDescription className="text-sm">
            <p className="font-semibold mb-2">ℹ️ Hệ thống sẽ tự động:</p>
            <ul className="list-disc list-inside space-y-1 text-muted-foreground">
              <li>Trích xuất frames từ video (1 frame/giây)</li>
              <li>Chạy YOLO detector để tạo pseudo-labels</li>
              <li>Lưu frames + labels vào dataset</li>
              <li>Sẵn sàng cho incremental training</li>
              <li>Cải thiện độ chính xác model theo thời gian</li>
            </ul>
          </AlertDescription>
        </Alert>
      </CardContent>
    </Card>
  );
}
