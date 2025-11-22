# ADAS Database Setup Guide

## 📊 SQL Server Database Schema

This directory contains the complete SQL Server database schema for the ADAS system, including:

### Core Tables (12 tables)
1. **Drivers** - Driver information and safety scores
2. **Cameras** - Camera configurations and status
3. **AIModels** - AI model versions and metadata
4. **Trips** - Trip records and statistics
5. **Events** - Alerts, warnings, and incidents
6. **Detections** - Real-time object detection results
7. **DriverStatuses** - Driver attention and drowsiness monitoring
8. **Analytics** - System metrics and performance data

### Training & ML Tables (3 tables)
9. **ModelVersions** - Training history and model performance
10. **TrainingImages** - Image dataset for model training
11. **TrainingAnnotations** - Bounding box annotations

## 🚀 Quick Start

### Option 1: SQL Server (Production - Windows Server)

#### Step 1: Install SQL Server
```bash
# On Windows Server
# Download and install SQL Server 2019 or later
# https://www.microsoft.com/en-us/sql-server/sql-server-downloads
```

#### Step 2: Create Database
```bash
# Run the schema script in SQL Server Management Studio (SSMS)
# Or use sqlcmd
sqlcmd -S localhost -U sa -P YourPassword123! -i sql-server-schema.sql
```

#### Step 3: Update Backend Configuration
```bash
# Edit backend-python/.env
DATABASE_URL=mssql+pyodbc://sa:YourPassword123!@localhost:1433/ADAS_DB?driver=ODBC+Driver+18+for+SQL+Server&TrustServerCertificate=yes
```

#### Step 4: Migrate Existing Data (Optional)
```bash
cd database
pip install pyodbc
python migrate_to_sql_server.py
```

### Option 2: Docker SQL Server (Development - Mac/Linux)

```bash
# Start SQL Server in Docker
docker run -e "ACCEPT_EULA=Y" -e "SA_PASSWORD=YourPassword123!" \
  -p 1433:1433 --name mssql \
  -d mcr.microsoft.com/mssql/server:2019-latest

# Wait 30 seconds for SQL Server to start
sleep 30

# Create database schema
docker exec -it mssql /opt/mssql-tools/bin/sqlcmd \
  -S localhost -U sa -P "YourPassword123!" \
  -i /sql-server-schema.sql

# Update .env file
DATABASE_URL=mssql+pyodbc://sa:YourPassword123!@localhost:1433/ADAS_DB?driver=ODBC+Driver+18+for+SQL+Server&TrustServerCertificate=yes
```

### Option 3: SQLite (Quick Testing)

```bash
# Already configured by default
DATABASE_URL=sqlite:///./adas_test.db

# No additional setup needed
# Data is stored in: backend-python/adas_test.db
```

## 📋 Database Features

### 1. High Performance Indexes
- Optimized queries for real-time detection
- Fast event lookup by severity and timestamp
- Efficient trip and driver analytics

### 2. Stored Procedures
- `GetDashboardStats` - Get system overview
- `GetTrainingDatasetStats` - Training data metrics
- `StartTrip` - Begin new trip with automatic driver updates
- `EndTrip` - Complete trip and calculate duration
- `SaveBatchDetections` - High-performance batch insert (1000+ detections/sec)

### 3. Views for Quick Access
- `vw_RecentDetections` - Latest 100 detections with camera info
- `vw_CriticalEvents` - High/critical severity events
- `vw_TrainingDatasetStatus` - Training data labeling progress

### 4. Training Data Management
The schema includes dedicated tables for:
- **Image Storage**: Track training images with quality metrics
- **Annotations**: Store bounding boxes with YOLO format support
- **Model Versions**: Track training history and performance
- **Data Augmentation**: Mark augmented images for better training

Example training workflow:
```sql
-- 1. Add training images
INSERT INTO TrainingImages (ImagePath, Width, Height, SourceType, DatasetSplit)
VALUES ('train/img001.jpg', 640, 480, 'webcam', 'train');

-- 2. Add annotations (YOLO format - normalized coordinates)
INSERT INTO TrainingAnnotations (ImageId, ClassName, BoundingBox, AnnotatedBy)
VALUES (1, 'car', '[0.5, 0.5, 0.3, 0.4]', 'auto-label');

-- 3. Get dataset statistics
EXEC GetTrainingDatasetStats;

-- 4. Start training and track version
INSERT INTO ModelVersions (ModelName, Version, TotalEpochs, Status)
VALUES ('yolov8n-custom', '1.0', 100, 'training');

-- 5. Update training results
UPDATE ModelVersions
SET mAP50 = 0.92, ValidationAccuracy = 0.89, Status = 'completed'
WHERE Id = 1;
```

## 🔧 Database Maintenance

### Backup Database
```sql
BACKUP DATABASE ADAS_DB 
TO DISK = 'C:\Backups\ADAS_DB.bak'
WITH FORMAT, COMPRESSION;
```

### Clean Old Detections (Keep last 30 days)
```sql
DELETE FROM Detections
WHERE Timestamp < DATEADD(DAY, -30, GETUTCDATE());
```

### Update Statistics
```sql
UPDATE STATISTICS Detections;
UPDATE STATISTICS Events;
UPDATE STATISTICS Trips;
```

### Check Database Size
```sql
SELECT 
    name AS TableName,
    SUM(reserved_page_count) * 8.0 / 1024 AS SizeMB
FROM sys.dm_db_partition_stats
GROUP BY name
ORDER BY SizeMB DESC;
```

## 📊 Sample Queries

### Get Detection Statistics
```sql
SELECT 
    ClassName,
    COUNT(*) AS TotalDetections,
    AVG(Confidence) AS AvgConfidence,
    AVG(DistanceMeters) AS AvgDistance
FROM Detections
WHERE Timestamp > DATEADD(HOUR, -24, GETUTCDATE())
GROUP BY ClassName
ORDER BY TotalDetections DESC;
```

### Get Driver Safety Report
```sql
SELECT 
    d.Name,
    d.SafetyScore,
    d.TotalTrips,
    d.TotalIncidents,
    COUNT(e.Id) AS CriticalEvents
FROM Drivers d
LEFT JOIN Events e ON d.Id = e.DriverId AND e.Severity = 'critical'
GROUP BY d.Name, d.SafetyScore, d.TotalTrips, d.TotalIncidents
ORDER BY d.SafetyScore DESC;
```

### Get Training Dataset Progress
```sql
SELECT 
    DatasetSplit,
    COUNT(*) AS TotalImages,
    SUM(CASE WHEN IsLabeled = 1 THEN 1 ELSE 0 END) AS LabeledImages,
    AVG(BlurScore) AS AvgQuality
FROM TrainingImages
GROUP BY DatasetSplit;
```

## 🚨 Troubleshooting

### Connection Issues
```bash
# Test SQL Server connection
sqlcmd -S localhost -U sa -P "YourPassword123!" -Q "SELECT @@VERSION"

# Check if SQL Server is running
docker ps | grep mssql
# or on Windows:
net start | find "SQL"
```

### ODBC Driver Issues (Mac/Linux)
```bash
# Install ODBC Driver
# On Mac:
brew install unixodbc
brew tap microsoft/mssql-release https://github.com/Microsoft/homebrew-mssql-release
brew install msodbcsql18 mssql-tools18

# On Ubuntu:
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
curl https://packages.microsoft.com/config/ubuntu/20.04/prod.list | sudo tee /etc/apt/sources.list.d/mssql-release.list
sudo apt-get update
sudo ACCEPT_EULA=Y apt-get install -y msodbcsql18
```

### Migration Errors
```python
# Edit migrate_to_sql_server.py and update connection settings:
SQL_SERVER_CONFIG = {
    'server': 'YOUR_SERVER',
    'database': 'ADAS_DB',
    'username': 'sa',
    'password': 'YOUR_PASSWORD',
}
```

## 📈 Performance Optimization

### For High-Volume Detection Storage (1000+ detections/second)
```sql
-- Use table partitioning by timestamp
CREATE PARTITION FUNCTION PF_Timestamp (DATETIME2)
AS RANGE RIGHT FOR VALUES 
('2025-01-01', '2025-02-01', '2025-03-01', ...);

-- Create columnstore index for analytics
CREATE NONCLUSTERED COLUMNSTORE INDEX IX_Detections_Analytics
ON Detections (ClassName, Confidence, Timestamp);
```

### For Training Data
```sql
-- Add full-text search for image paths
CREATE FULLTEXT INDEX ON TrainingImages(ImagePath)
KEY INDEX PK_TrainingImages;
```

## 📚 Resources

- [SQL Server Documentation](https://docs.microsoft.com/en-us/sql/)
- [ODBC Driver Download](https://docs.microsoft.com/en-us/sql/connect/odbc/)
- [SQLAlchemy SQL Server Dialect](https://docs.sqlalchemy.org/en/14/dialects/mssql.html)

## ✅ Next Steps

1. ✅ Create database using `sql-server-schema.sql`
2. ✅ Update `.env` with connection string
3. ✅ Test connection: `python -c "from database import engine; print(engine)"`
4. ✅ Migrate existing data (optional): `python database/migrate_to_sql_server.py`
5. ✅ Start backend: `python main.py`
6. ✅ Verify in dashboard: http://localhost:8000/docs
