-- =============================================
-- ADAS System - SQL Server Database Schema
-- Version: 2.0
-- Description: Complete database schema for ADAS system
--              Includes tables for real-time detection, 
--              training data, analytics, and system logs
-- =============================================

USE master;
GO

-- Create database if not exists
IF NOT EXISTS (SELECT * FROM sys.databases WHERE name = 'ADAS_DB')
BEGIN
    CREATE DATABASE ADAS_DB;
END
GO

USE ADAS_DB;
GO

-- =============================================
-- Drop existing tables (for clean install)
-- =============================================
IF OBJECT_ID('dbo.TrainingAnnotations', 'U') IS NOT NULL DROP TABLE dbo.TrainingAnnotations;
IF OBJECT_ID('dbo.TrainingImages', 'U') IS NOT NULL DROP TABLE dbo.TrainingImages;
IF OBJECT_ID('dbo.ModelVersions', 'U') IS NOT NULL DROP TABLE dbo.ModelVersions;
IF OBJECT_ID('dbo.Analytics', 'U') IS NOT NULL DROP TABLE dbo.Analytics;
IF OBJECT_ID('dbo.DriverStatuses', 'U') IS NOT NULL DROP TABLE dbo.DriverStatuses;
IF OBJECT_ID('dbo.Detections', 'U') IS NOT NULL DROP TABLE dbo.Detections;
IF OBJECT_ID('dbo.Events', 'U') IS NOT NULL DROP TABLE dbo.Events;
IF OBJECT_ID('dbo.Trips', 'U') IS NOT NULL DROP TABLE dbo.Trips;
IF OBJECT_ID('dbo.AIModels', 'U') IS NOT NULL DROP TABLE dbo.AIModels;
IF OBJECT_ID('dbo.Cameras', 'U') IS NOT NULL DROP TABLE dbo.Cameras;
IF OBJECT_ID('dbo.Drivers', 'U') IS NOT NULL DROP TABLE dbo.Drivers;
GO

-- =============================================
-- Core Tables
-- =============================================

-- Drivers Table
CREATE TABLE dbo.Drivers (
    Id INT PRIMARY KEY IDENTITY(1,1),
    Name NVARCHAR(200) NOT NULL,
    Email NVARCHAR(200) UNIQUE,
    PhoneNumber NVARCHAR(20),
    LicenseNumber NVARCHAR(50) UNIQUE,
    SafetyScore FLOAT DEFAULT 100.0,
    TotalTrips INT DEFAULT 0,
    TotalIncidents INT DEFAULT 0,
    Status NVARCHAR(50) DEFAULT 'active', -- active, inactive, suspended
    CreatedAt DATETIME2 DEFAULT GETUTCDATE(),
    UpdatedAt DATETIME2 DEFAULT GETUTCDATE(),
    
    INDEX IX_Drivers_Email (Email),
    INDEX IX_Drivers_Status (Status),
    INDEX IX_Drivers_SafetyScore (SafetyScore DESC)
);

-- Cameras Table
CREATE TABLE dbo.Cameras (
    Id INT PRIMARY KEY IDENTITY(1,1),
    Name NVARCHAR(200) NOT NULL,
    Location NVARCHAR(300),
    CameraType NVARCHAR(50), -- webcam, ip-camera, smartphone, usb
    Resolution NVARCHAR(20), -- 1920x1080, 1280x720, etc.
    FPS INT DEFAULT 30,
    StreamUrl NVARCHAR(500),
    Status NVARCHAR(50) DEFAULT 'active', -- active, inactive, maintenance
    IsOnline BIT DEFAULT 0,
    LastPingAt DATETIME2,
    CreatedAt DATETIME2 DEFAULT GETUTCDATE(),
    UpdatedAt DATETIME2 DEFAULT GETUTCDATE(),
    
    INDEX IX_Cameras_Status (Status),
    INDEX IX_Cameras_IsOnline (IsOnline)
);

-- AI Models Table
CREATE TABLE dbo.AIModels (
    Id INT PRIMARY KEY IDENTITY(1,1),
    Name NVARCHAR(200) NOT NULL,
    ModelType NVARCHAR(100), -- yolov8n, yolov8s, faster-rcnn, etc.
    Version NVARCHAR(50),
    FilePath NVARCHAR(500),
    FileSize BIGINT, -- bytes
    Accuracy FLOAT,
    IsActive BIT DEFAULT 0,
    IsDownloaded BIT DEFAULT 0,
    Description NVARCHAR(MAX),
    DownloadedAt DATETIME2,
    CreatedAt DATETIME2 DEFAULT GETUTCDATE(),
    
    INDEX IX_AIModels_IsActive (IsActive),
    INDEX IX_AIModels_ModelType (ModelType)
);

-- Trips Table
CREATE TABLE dbo.Trips (
    Id INT PRIMARY KEY IDENTITY(1,1),
    StartTime DATETIME2 NOT NULL DEFAULT GETUTCDATE(),
    EndTime DATETIME2,
    Duration INT, -- seconds
    Distance FLOAT, -- kilometers
    AverageSpeed FLOAT, -- km/h
    MaxSpeed FLOAT,
    Route NVARCHAR(MAX), -- JSON array of GPS coordinates
    Status NVARCHAR(50) DEFAULT 'active', -- active, completed, interrupted
    
    DriverId INT FOREIGN KEY REFERENCES dbo.Drivers(Id) ON DELETE SET NULL,
    CameraId INT FOREIGN KEY REFERENCES dbo.Cameras(Id) ON DELETE SET NULL,
    
    INDEX IX_Trips_StartTime (StartTime DESC),
    INDEX IX_Trips_Status (Status),
    INDEX IX_Trips_DriverId (DriverId),
    INDEX IX_Trips_CameraId (CameraId)
);

-- Events Table (Alerts, Warnings, Incidents)
CREATE TABLE dbo.Events (
    Id INT PRIMARY KEY IDENTITY(1,1),
    EventType NVARCHAR(100) NOT NULL, -- collision_warning, lane_departure, drowsiness, etc.
    Description NVARCHAR(MAX),
    Timestamp DATETIME2 NOT NULL DEFAULT GETUTCDATE(),
    Severity NVARCHAR(50), -- low, medium, high, critical
    Location NVARCHAR(300), -- GPS coordinates or description
    EventMetadata NVARCHAR(MAX), -- JSON with additional data
    
    TripId INT FOREIGN KEY REFERENCES dbo.Trips(Id) ON DELETE CASCADE,
    CameraId INT FOREIGN KEY REFERENCES dbo.Cameras(Id) ON DELETE SET NULL,
    DriverId INT FOREIGN KEY REFERENCES dbo.Drivers(Id) ON DELETE SET NULL,
    
    INDEX IX_Events_Timestamp (Timestamp DESC),
    INDEX IX_Events_EventType (EventType),
    INDEX IX_Events_Severity (Severity),
    INDEX IX_Events_TripId (TripId),
    INDEX IX_Events_DriverId (DriverId)
);

-- Detections Table (Real-time object detection results)
CREATE TABLE dbo.Detections (
    Id INT PRIMARY KEY IDENTITY(1,1),
    ClassName NVARCHAR(100) NOT NULL, -- car, person, truck, motorcycle, etc.
    Confidence FLOAT NOT NULL,
    BoundingBox NVARCHAR(200) NOT NULL, -- JSON [x, y, width, height]
    DistanceMeters FLOAT,
    RelativeSpeed FLOAT, -- km/h relative to vehicle
    Timestamp DATETIME2 NOT NULL DEFAULT GETUTCDATE(),
    FrameNumber INT,
    
    TripId INT FOREIGN KEY REFERENCES dbo.Trips(Id) ON DELETE CASCADE,
    CameraId INT FOREIGN KEY REFERENCES dbo.Cameras(Id) ON DELETE SET NULL,
    
    INDEX IX_Detections_Timestamp (Timestamp DESC),
    INDEX IX_Detections_ClassName (ClassName),
    INDEX IX_Detections_TripId (TripId),
    INDEX IX_Detections_CameraId (CameraId)
);

-- Driver Statuses Table (Drowsiness, attention, etc.)
CREATE TABLE dbo.DriverStatuses (
    Id INT PRIMARY KEY IDENTITY(1,1),
    Timestamp DATETIME2 NOT NULL DEFAULT GETUTCDATE(),
    Status NVARCHAR(100), -- alert, drowsy, distracted, normal
    EyesClosed BIT DEFAULT 0,
    YawnDetected BIT DEFAULT 0,
    HeadPose NVARCHAR(100), -- JSON {pitch, yaw, roll}
    AttentionScore FLOAT, -- 0-100
    
    DriverId INT FOREIGN KEY REFERENCES dbo.Drivers(Id) ON DELETE CASCADE,
    TripId INT FOREIGN KEY REFERENCES dbo.Trips(Id) ON DELETE CASCADE,
    
    INDEX IX_DriverStatuses_Timestamp (Timestamp DESC),
    INDEX IX_DriverStatuses_Status (Status),
    INDEX IX_DriverStatuses_DriverId (DriverId)
);

-- Analytics Table
CREATE TABLE dbo.Analytics (
    Id INT PRIMARY KEY IDENTITY(1,1),
    Timestamp DATETIME2 NOT NULL DEFAULT GETUTCDATE(),
    MetricType NVARCHAR(100) NOT NULL, -- total_detections, avg_confidence, fps, etc.
    Value FLOAT NOT NULL,
    Unit NVARCHAR(50),
    Category NVARCHAR(100),
    AnalyticsMetadata NVARCHAR(MAX), -- JSON
    
    TripId INT FOREIGN KEY REFERENCES dbo.Trips(Id) ON DELETE CASCADE,
    DriverId INT FOREIGN KEY REFERENCES dbo.Drivers(Id) ON DELETE SET NULL,
    
    INDEX IX_Analytics_Timestamp (Timestamp DESC),
    INDEX IX_Analytics_MetricType (MetricType),
    INDEX IX_Analytics_Category (Category)
);

-- =============================================
-- Training & Model Improvement Tables
-- =============================================

-- Model Versions Table (Track model training history)
CREATE TABLE dbo.ModelVersions (
    Id INT PRIMARY KEY IDENTITY(1,1),
    ModelName NVARCHAR(200) NOT NULL,
    Version NVARCHAR(50) NOT NULL,
    TrainingStartTime DATETIME2 NOT NULL,
    TrainingEndTime DATETIME2,
    TrainingDuration INT, -- seconds
    
    -- Training Metrics
    TotalEpochs INT,
    FinalLoss FLOAT,
    ValidationAccuracy FLOAT,
    mAP50 FLOAT, -- mean Average Precision at IoU=0.50
    mAP5095 FLOAT, -- mAP at IoU=0.50:0.95
    
    -- Training Data Stats
    TotalTrainingImages INT,
    TotalValidationImages INT,
    TotalAnnotations INT,
    ClassDistribution NVARCHAR(MAX), -- JSON {class: count}
    
    -- Model Files
    ModelFilePath NVARCHAR(500),
    ConfigFilePath NVARCHAR(500),
    WeightsFilePath NVARCHAR(500),
    
    -- Performance
    InferenceTimeMs FLOAT, -- average inference time
    ModelSizeMB FLOAT,
    
    Status NVARCHAR(50) DEFAULT 'training', -- training, completed, failed, deployed
    Notes NVARCHAR(MAX),
    CreatedBy NVARCHAR(200),
    CreatedAt DATETIME2 DEFAULT GETUTCDATE(),
    
    INDEX IX_ModelVersions_ModelName (ModelName),
    INDEX IX_ModelVersions_Version (Version),
    INDEX IX_ModelVersions_Status (Status),
    INDEX IX_ModelVersions_TrainingStartTime (TrainingStartTime DESC)
);

-- Training Images Table (Store images for model training)
CREATE TABLE dbo.TrainingImages (
    Id INT PRIMARY KEY IDENTITY(1,1),
    ImagePath NVARCHAR(500) NOT NULL UNIQUE,
    ImageHash NVARCHAR(64), -- SHA256 hash to avoid duplicates
    Width INT NOT NULL,
    Height INT NOT NULL,
    FileSize BIGINT, -- bytes
    Format NVARCHAR(10), -- jpg, png, etc.
    
    -- Source Information
    SourceType NVARCHAR(50), -- webcam, upload, augmented, synthetic
    SourceCameraId INT FOREIGN KEY REFERENCES dbo.Cameras(Id) ON DELETE SET NULL,
    SourceTripId INT FOREIGN KEY REFERENCES dbo.Trips(Id) ON DELETE SET NULL,
    
    -- Quality Metrics
    BlurScore FLOAT, -- 0-100, higher is sharper
    BrightnessScore FLOAT,
    IsAugmented BIT DEFAULT 0,
    AugmentationType NVARCHAR(100), -- flip, rotate, brightness, etc.
    
    -- Training Split
    DatasetSplit NVARCHAR(20), -- train, validation, test
    
    -- Labeling Status
    IsLabeled BIT DEFAULT 0,
    LabeledBy NVARCHAR(200),
    LabeledAt DATETIME2,
    ReviewStatus NVARCHAR(50), -- pending, approved, rejected
    
    CreatedAt DATETIME2 DEFAULT GETUTCDATE(),
    
    INDEX IX_TrainingImages_ImageHash (ImageHash),
    INDEX IX_TrainingImages_SourceType (SourceType),
    INDEX IX_TrainingImages_DatasetSplit (DatasetSplit),
    INDEX IX_TrainingImages_IsLabeled (IsLabeled),
    INDEX IX_TrainingImages_ReviewStatus (ReviewStatus)
);

-- Training Annotations Table (Bounding boxes for training)
CREATE TABLE dbo.TrainingAnnotations (
    Id INT PRIMARY KEY IDENTITY(1,1),
    ImageId INT NOT NULL FOREIGN KEY REFERENCES dbo.TrainingImages(Id) ON DELETE CASCADE,
    
    -- Annotation Data
    ClassName NVARCHAR(100) NOT NULL,
    BoundingBox NVARCHAR(200) NOT NULL, -- JSON [x, y, width, height] normalized 0-1
    Confidence FLOAT DEFAULT 1.0, -- for weak/uncertain labels
    
    -- Additional Attributes
    IsTruncated BIT DEFAULT 0,
    IsOccluded BIT DEFAULT 0,
    IsDifficult BIT DEFAULT 0,
    Angle FLOAT, -- rotation angle for oriented bounding boxes
    
    -- Quality Control
    AnnotatedBy NVARCHAR(200),
    VerifiedBy NVARCHAR(200),
    AnnotationSource NVARCHAR(50), -- manual, auto, corrected
    
    CreatedAt DATETIME2 DEFAULT GETUTCDATE(),
    UpdatedAt DATETIME2 DEFAULT GETUTCDATE(),
    
    INDEX IX_TrainingAnnotations_ImageId (ImageId),
    INDEX IX_TrainingAnnotations_ClassName (ClassName),
    INDEX IX_TrainingAnnotations_AnnotationSource (AnnotationSource)
);

GO

-- =============================================
-- Stored Procedures
-- =============================================

-- Get Dashboard Statistics
CREATE OR ALTER PROCEDURE dbo.GetDashboardStats
AS
BEGIN
    SET NOCOUNT ON;
    
    SELECT 
        (SELECT COUNT(*) FROM dbo.Trips WHERE Status = 'active') AS ActiveTrips,
        (SELECT COUNT(*) FROM dbo.Trips WHERE Status = 'completed') AS TotalTrips,
        (SELECT COUNT(*) FROM dbo.Events WHERE Severity = 'critical') AS CriticalEvents,
        (SELECT COUNT(*) FROM dbo.Cameras WHERE IsOnline = 1) AS OnlineCameras,
        (SELECT COUNT(*) FROM dbo.Drivers WHERE Status = 'active') AS ActiveDrivers,
        (SELECT AVG(SafetyScore) FROM dbo.Drivers WHERE Status = 'active') AS AvgSafetyScore,
        (SELECT COUNT(*) FROM dbo.Detections WHERE Timestamp > DATEADD(HOUR, -1, GETUTCDATE())) AS RecentDetections;
END
GO

-- Get Training Dataset Statistics
CREATE OR ALTER PROCEDURE dbo.GetTrainingDatasetStats
AS
BEGIN
    SET NOCOUNT ON;
    
    -- Overall stats
    SELECT 
        COUNT(*) AS TotalImages,
        SUM(CASE WHEN IsLabeled = 1 THEN 1 ELSE 0 END) AS LabeledImages,
        SUM(CASE WHEN DatasetSplit = 'train' THEN 1 ELSE 0 END) AS TrainingImages,
        SUM(CASE WHEN DatasetSplit = 'validation' THEN 1 ELSE 0 END) AS ValidationImages,
        SUM(CASE WHEN DatasetSplit = 'test' THEN 1 ELSE 0 END) AS TestImages,
        AVG(BlurScore) AS AvgBlurScore,
        SUM(FileSize) / (1024.0 * 1024.0) AS TotalSizeMB
    FROM dbo.TrainingImages;
    
    -- Annotations per class
    SELECT 
        ClassName,
        COUNT(*) AS AnnotationCount,
        AVG(Confidence) AS AvgConfidence
    FROM dbo.TrainingAnnotations
    GROUP BY ClassName
    ORDER BY AnnotationCount DESC;
END
GO

-- Start New Trip
CREATE OR ALTER PROCEDURE dbo.StartTrip
    @DriverId INT,
    @CameraId INT
AS
BEGIN
    SET NOCOUNT ON;
    
    DECLARE @TripId INT;
    
    INSERT INTO dbo.Trips (DriverId, CameraId, Status)
    VALUES (@DriverId, @CameraId, 'active');
    
    SET @TripId = SCOPE_IDENTITY();
    
    -- Update driver stats
    UPDATE dbo.Drivers
    SET TotalTrips = TotalTrips + 1
    WHERE Id = @DriverId;
    
    SELECT @TripId AS TripId;
END
GO

-- End Trip
CREATE OR ALTER PROCEDURE dbo.EndTrip
    @TripId INT
AS
BEGIN
    SET NOCOUNT ON;
    
    DECLARE @StartTime DATETIME2, @EndTime DATETIME2, @Duration INT;
    
    SELECT @StartTime = StartTime FROM dbo.Trips WHERE Id = @TripId;
    SET @EndTime = GETUTCDATE();
    SET @Duration = DATEDIFF(SECOND, @StartTime, @EndTime);
    
    UPDATE dbo.Trips
    SET EndTime = @EndTime,
        Duration = @Duration,
        Status = 'completed'
    WHERE Id = @TripId;
    
    SELECT @TripId AS TripId, @Duration AS Duration;
END
GO

-- Save Batch Detections (High Performance)
CREATE OR ALTER PROCEDURE dbo.SaveBatchDetections
    @Detections NVARCHAR(MAX) -- JSON array of detections
AS
BEGIN
    SET NOCOUNT ON;
    
    INSERT INTO dbo.Detections (ClassName, Confidence, BoundingBox, DistanceMeters, TripId, CameraId, Timestamp)
    SELECT 
        JSON_VALUE(value, '$.cls') AS ClassName,
        CAST(JSON_VALUE(value, '$.conf') AS FLOAT) AS Confidence,
        JSON_QUERY(value, '$.bbox') AS BoundingBox,
        CAST(JSON_VALUE(value, '$.distance_m') AS FLOAT) AS DistanceMeters,
        CAST(JSON_VALUE(value, '$.trip_id') AS INT) AS TripId,
        CAST(JSON_VALUE(value, '$.camera_id') AS INT) AS CameraId,
        GETUTCDATE() AS Timestamp
    FROM OPENJSON(@Detections);
    
    SELECT @@ROWCOUNT AS SavedCount;
END
GO

-- =============================================
-- Views for Quick Access
-- =============================================

-- Recent Detections View
CREATE OR ALTER VIEW dbo.vw_RecentDetections
AS
SELECT TOP 100
    d.Id,
    d.ClassName,
    d.Confidence,
    d.BoundingBox,
    d.DistanceMeters,
    d.Timestamp,
    c.Name AS CameraName,
    t.Id AS TripId
FROM dbo.Detections d
LEFT JOIN dbo.Cameras c ON d.CameraId = c.Id
LEFT JOIN dbo.Trips t ON d.TripId = t.Id
ORDER BY d.Timestamp DESC;
GO

-- Critical Events View
CREATE OR ALTER VIEW dbo.vw_CriticalEvents
AS
SELECT 
    e.Id,
    e.EventType,
    e.Description,
    e.Timestamp,
    e.Severity,
    e.Location,
    d.Name AS DriverName,
    c.Name AS CameraName
FROM dbo.Events e
LEFT JOIN dbo.Drivers d ON e.DriverId = d.Id
LEFT JOIN dbo.Cameras c ON e.CameraId = c.Id
WHERE e.Severity IN ('high', 'critical')
ORDER BY e.Timestamp DESC;
GO

-- Training Dataset Status View
CREATE OR ALTER VIEW dbo.vw_TrainingDatasetStatus
AS
SELECT 
    ti.Id,
    ti.ImagePath,
    ti.Width,
    ti.Height,
    ti.SourceType,
    ti.DatasetSplit,
    ti.IsLabeled,
    ti.ReviewStatus,
    COUNT(ta.Id) AS AnnotationCount,
    STRING_AGG(ta.ClassName, ', ') AS Classes
FROM dbo.TrainingImages ti
LEFT JOIN dbo.TrainingAnnotations ta ON ti.Id = ta.ImageId
GROUP BY ti.Id, ti.ImagePath, ti.Width, ti.Height, ti.SourceType, 
         ti.DatasetSplit, ti.IsLabeled, ti.ReviewStatus;
GO

-- =============================================
-- Initial Seed Data
-- =============================================

-- Seed AI Models
INSERT INTO dbo.AIModels (Name, ModelType, Version, Accuracy, IsActive, IsDownloaded, Description)
VALUES 
    ('YOLOv8 Nano', 'yolov8n', '8.0', 80.4, 1, 1, 'Fast and lightweight model for real-time detection'),
    ('YOLOv8 Small', 'yolov8s', '8.0', 86.6, 0, 0, 'Balance between speed and accuracy'),
    ('YOLOv8 Medium', 'yolov8m', '8.0', 88.3, 0, 0, 'Higher accuracy for critical applications'),
    ('YOLOv8 Large', 'yolov8l', '8.0', 90.1, 0, 0, 'Maximum accuracy for offline processing');

-- Seed Default Camera
INSERT INTO dbo.Cameras (Name, Location, CameraType, Resolution, FPS, Status, IsOnline)
VALUES 
    ('Default Webcam', 'Driver Dashboard', 'webcam', '1280x720', 30, 'active', 1),
    ('Front Camera', 'Windshield', 'ip-camera', '1920x1080', 60, 'active', 0);

-- Seed Default Driver
INSERT INTO dbo.Drivers (Name, Email, PhoneNumber, SafetyScore, Status)
VALUES 
    ('Test Driver', 'driver@adas.com', '+84123456789', 100.0, 'active');

GO

PRINT '✅ Database schema created successfully!';
PRINT '✅ Tables created: 12 core + 3 training tables';
PRINT '✅ Stored procedures created: 5';
PRINT '✅ Views created: 3';
PRINT '✅ Seed data inserted';
PRINT '';
PRINT '📊 Next steps:';
PRINT '1. Update .env file with SQL Server connection string';
PRINT '2. Run Python migration script to copy existing data';
PRINT '3. Configure backend to use SQL Server';
GO
