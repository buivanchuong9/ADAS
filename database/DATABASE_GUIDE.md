# 📊 ADAS Database System - Complete Guide

## Overview

Complete SQL Server database system for ADAS với khả năng:
- ✅ Lưu trữ real-time detections (1000+ detections/second)
- ✅ Quản lý training data cho ML models
- ✅ Track model performance và training history
- ✅ Analytics và reporting
- ✅ Auto-labeling từ high-confidence detections

## 📁 Files trong thư mục này

```
database/
├── README.md                          # Tài liệu này
├── sql-server-schema.sql              # Database schema đầy đủ (15 tables)
├── training-queries.sql               # Queries cho training data management
├── migrate_to_sql_server.py          # Migration script từ SQLite
└── auto_collect_training_data.py     # Auto thu thập training data
```

## 🚀 Quick Start

### Bước 1: Cài đặt SQL Server

**Option A: Windows Server (Production)**
```bash
# Download SQL Server 2019+
# https://www.microsoft.com/en-us/sql-server/sql-server-downloads

# Hoặc dùng SQL Server Express (miễn phí)
choco install sql-server-express
```

**Option B: Docker (Development - Mac/Linux)**
```bash
# Start SQL Server container
docker run -e "ACCEPT_EULA=Y" -e "SA_PASSWORD=YourPassword123!" \
  -p 1433:1433 --name adas-mssql \
  -d mcr.microsoft.com/mssql/server:2019-latest

# Đợi 30s để SQL Server khởi động
sleep 30
```

### Bước 2: Tạo Database Schema

```bash
# Trên Windows (SSMS)
# Mở SQL Server Management Studio
# File > Open > sql-server-schema.sql
# Execute (F5)

# Hoặc dùng command line
sqlcmd -S localhost -U sa -P "YourPassword123!" -i database/sql-server-schema.sql

# Trên Docker
docker cp database/sql-server-schema.sql adas-mssql:/tmp/
docker exec adas-mssql /opt/mssql-tools/bin/sqlcmd \
  -S localhost -U sa -P "YourPassword123!" \
  -i /tmp/sql-server-schema.sql
```

### Bước 3: Cấu hình Backend

Sửa file `backend-python/.env`:

```bash
# SQL Server connection
DATABASE_URL=mssql+pyodbc://sa:YourPassword123!@localhost:1433/ADAS_DB?driver=ODBC+Driver+18+for+SQL+Server&TrustServerCertificate=yes

# Hoặc với Windows Authentication
DATABASE_URL=mssql+pyodbc://localhost/ADAS_DB?driver=ODBC+Driver+18+for+SQL+Server&trusted_connection=yes
```

### Bước 4: Migrate Data (Optional)

Nếu đã có data trong SQLite:

```bash
cd database
pip install pyodbc

# Sửa connection string trong migrate_to_sql_server.py
python migrate_to_sql_server.py
```

### Bước 5: Khởi động Backend

```bash
cd backend-python
source venv/bin/activate  # Mac/Linux
# hoặc venv\Scripts\activate  # Windows

python main.py
```

Kiểm tra: http://localhost:8000/docs

## 📊 Database Tables

### Core Tables (8 tables)

1. **Drivers** - Thông tin tài xế
   - Name, Email, License Number
   - Safety Score (0-100)
   - Total Trips, Incidents

2. **Cameras** - Camera configuration
   - Name, Type (webcam, IP camera, smartphone)
   - Resolution, FPS, Stream URL
   - Status, Online status

3. **AIModels** - AI model versions
   - Model name, type (YOLOv8, etc.)
   - Accuracy, file size
   - Download status

4. **Trips** - Trip records
   - Start/end time, duration
   - Distance, average speed
   - Route (GPS coordinates)

5. **Events** - Alerts & incidents
   - Event type (collision warning, lane departure)
   - Severity (low/medium/high/critical)
   - Location, timestamp

6. **Detections** - Real-time object detection
   - Class name (car, person, truck...)
   - Confidence, bounding box
   - Distance from vehicle

7. **DriverStatuses** - Driver monitoring
   - Drowsiness, attention score
   - Eye closure, yawn detection
   - Head pose

8. **Analytics** - System metrics
   - FPS, latency, accuracy
   - Detection counts
   - Performance metrics

### Training & ML Tables (3 tables)

9. **ModelVersions** - Training history
   - Training start/end time
   - mAP, accuracy metrics
   - Training parameters
   - Model file paths

10. **TrainingImages** - Image dataset
    - Image path, resolution
    - Quality metrics (blur, brightness)
    - Dataset split (train/val/test)
    - Labeling status

11. **TrainingAnnotations** - Bounding boxes
    - Class name
    - Bounding box coordinates (YOLO format)
    - Confidence, difficulty flags
    - Annotation source (manual/auto)

## 🎯 Common Tasks

### 1. Xem Dashboard Statistics

```sql
EXEC GetDashboardStats;
```

Kết quả:
- Active trips
- Total events (critical)
- Online cameras
- Average safety score
- Recent detections

### 2. Lấy Recent Detections

```sql
SELECT TOP 100 * FROM vw_RecentDetections;
```

### 3. Phân tích Safety Score

```sql
SELECT 
    Name,
    SafetyScore,
    TotalTrips,
    TotalIncidents,
    CASE 
        WHEN SafetyScore >= 90 THEN 'Excellent'
        WHEN SafetyScore >= 75 THEN 'Good'
        WHEN SafetyScore >= 60 THEN 'Fair'
        ELSE 'Poor'
    END AS Rating
FROM Drivers
ORDER BY SafetyScore DESC;
```

### 4. Kiểm tra Training Dataset

```sql
EXEC GetTrainingDatasetStats;
```

Trả về:
- Total images (train/val/test)
- Labeled vs unlabeled
- Class distribution
- Average quality score

### 5. Auto-label High Confidence Detections

```sql
-- Tự động label 100 detections có confidence > 95%
EXEC AutoLabelDetections @MinConfidence = 0.95, @MaxImages = 100;
```

### 6. Tìm Classes cần thêm data

```sql
SELECT 
    ClassName,
    COUNT(*) AS SampleCount,
    500 - COUNT(*) AS SamplesNeeded
FROM TrainingAnnotations ta
JOIN TrainingImages ti ON ta.ImageId = ti.Id
GROUP BY ClassName
HAVING COUNT(*) < 500
ORDER BY SampleCount ASC;
```

## 🤖 Auto Training Data Collection

Script `auto_collect_training_data.py` tự động:
1. Lấy detections có confidence cao (>90%)
2. Kiểm tra chất lượng ảnh (blur, brightness)
3. Lưu vào YOLO format (train/val/test split)
4. Ghi vào database với metadata
5. Generate `data.yaml` cho training

**Chạy script:**

```bash
cd database
python auto_collect_training_data.py
```

Kết quả:
```
training_data/
├── data.yaml              # YOLO config
├── train/
│   ├── images/           # Training images
│   └── labels/           # YOLO annotations (.txt)
├── val/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/
```

## 🎓 Training Custom Model

Sau khi collect data:

```bash
# Install YOLOv8
pip install ultralytics

# Train model
yolo train data=training_data/data.yaml \
     model=yolov8n.pt \
     epochs=100 \
     imgsz=640 \
     batch=16 \
     device=0

# Evaluate
yolo val model=runs/detect/train/weights/best.pt \
     data=training_data/data.yaml

# Export to ONNX (for production)
yolo export model=runs/detect/train/weights/best.pt format=onnx
```

## 📈 Performance Tuning

### For High-Volume Detections (1000+/sec)

```sql
-- Enable table partitioning
CREATE PARTITION FUNCTION PF_DetectionDate (DATETIME2)
AS RANGE RIGHT FOR VALUES 
('2025-01-01', '2025-02-01', '2025-03-01', '2025-04-01');

-- Columnstore index for analytics
CREATE NONCLUSTERED COLUMNSTORE INDEX IX_Detections_Analytics
ON Detections (ClassName, Confidence, Timestamp);

-- Batch insert stored procedure
EXEC SaveBatchDetections @Detections = '[{"cls":"car","conf":0.95,...}]';
```

### Database Maintenance

```sql
-- Backup
BACKUP DATABASE ADAS_DB TO DISK = 'C:\Backups\ADAS_DB.bak';

-- Clean old detections (keep 30 days)
DELETE FROM Detections WHERE Timestamp < DATEADD(DAY, -30, GETUTCDATE());

-- Update statistics
UPDATE STATISTICS Detections;
REBUILD INDEX ALL ON Detections;
```

## 🔧 Troubleshooting

### Connection Issues

```bash
# Test SQL Server
sqlcmd -S localhost -U sa -P "YourPassword123!" -Q "SELECT @@VERSION"

# Check if running
docker ps | grep adas-mssql

# View logs
docker logs adas-mssql
```

### ODBC Driver (Mac/Linux)

```bash
# Mac
brew install unixodbc
brew tap microsoft/mssql-release
brew install msodbcsql18 mssql-tools18

# Ubuntu
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-get update
sudo ACCEPT_EULA=Y apt-get install -y msodbcsql18
```

### Migration Errors

```python
# Edit migrate_to_sql_server.py
SQL_SERVER_CONFIG = {
    'server': 'YOUR_SERVER',
    'database': 'ADAS_DB',
    'username': 'sa',
    'password': 'YOUR_PASSWORD',
}
```

## 📚 Advanced Queries

### Find Under-represented Classes

```sql
SELECT 
    d.ClassName,
    COUNT(*) AS LiveDetections,
    COALESCE(t.TrainingCount, 0) AS TrainingImages,
    500 - COALESCE(t.TrainingCount, 0) AS StillNeeded
FROM (
    SELECT DISTINCT ClassName FROM Detections
) d
LEFT JOIN (
    SELECT ClassName, COUNT(DISTINCT ImageId) AS TrainingCount
    FROM TrainingAnnotations
    GROUP BY ClassName
) t ON d.ClassName = t.ClassName
WHERE COALESCE(t.TrainingCount, 0) < 500
ORDER BY LiveDetections DESC;
```

### Model Performance Over Time

```sql
SELECT 
    CONVERT(DATE, TrainingStartTime) AS Date,
    ModelName,
    mAP50,
    ValidationAccuracy,
    InferenceTimeMs
FROM ModelVersions
WHERE Status = 'completed'
ORDER BY TrainingStartTime DESC;
```

### Driver Safety Trends

```sql
SELECT 
    d.Name,
    AVG(ds.AttentionScore) AS AvgAttention,
    SUM(CASE WHEN ds.Status = 'drowsy' THEN 1 ELSE 0 END) AS DrowsyIncidents,
    COUNT(DISTINCT t.Id) AS TotalTrips
FROM Drivers d
JOIN Trips t ON d.Id = t.DriverId
JOIN DriverStatuses ds ON d.Id = ds.DriverId
WHERE t.StartTime > DATEADD(MONTH, -1, GETUTCDATE())
GROUP BY d.Name
ORDER BY AvgAttention ASC;
```

## ✅ Production Checklist

- [ ] SQL Server 2019+ installed
- [ ] Database schema created (`sql-server-schema.sql`)
- [ ] Backend `.env` configured
- [ ] Initial seed data loaded
- [ ] Backup strategy configured
- [ ] Monitoring alerts set up
- [ ] Training data collection automated
- [ ] Model retraining scheduled (monthly)

## 🎯 Next Steps

1. **Deploy to Windows Server**
   - Install SQL Server
   - Run schema script
   - Configure backend connection
   - Set up automatic backups

2. **Start Collecting Training Data**
   - Run `auto_collect_training_data.py` daily
   - Review and clean data weekly
   - Retrain model monthly

3. **Monitor Performance**
   - Check `GetDashboardStats` daily
   - Review detection accuracy
   - Update models as needed

4. **Scale Up**
   - Add table partitioning for big data
   - Enable replication for high availability
   - Set up load balancing

## 📞 Support

Issues? Check:
- Database logs: `SELECT * FROM sys.dm_exec_query_stats`
- Backend logs: `backend-python/logs/`
- Connection test: `python -c "from database import engine; print(engine)"`

---

**Created for ADAS System v2.0**  
**Last updated: 2025-11-22**
