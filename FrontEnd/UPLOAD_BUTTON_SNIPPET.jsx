// Thêm nút Upload này vào file /Users/chuong/Desktop/AI/ADAS/FrontEnd/app/adas/page.tsx
// Sau dòng 474 (sau result  display)

{/* Upload Button */ }
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
            Upload và Phân Tích
        </>
    )}
</Button>
