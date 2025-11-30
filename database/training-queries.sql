-- =============================================
-- Training Data Queries for ADAS System
-- Quick queries for managing training data and improving model
-- =============================================

USE ADAS_DB;
GO

-- =============================================
-- 1. DATASET STATISTICS
-- =============================================

-- Get overall dataset statistics
SELECT 
    'Total Images' AS Metric,
    COUNT(*) AS Value
FROM TrainingImages
UNION ALL
SELECT 
    'Labeled Images',
    SUM(CASE WHEN IsLabeled = 1 THEN 1 ELSE 0 END)
FROM TrainingImages
UNION ALL
SELECT 
    'Total Annotations',
    COUNT(*)
FROM TrainingAnnotations
UNION ALL
SELECT 
    'Unique Classes',
    COUNT(DISTINCT ClassName)
FROM TrainingAnnotations;

-- Class distribution (for balancing dataset)
SELECT 
    ta.ClassName,
    COUNT(*) AS AnnotationCount,
    COUNT(DISTINCT ta.ImageId) AS ImageCount,
    AVG(ta.Confidence) AS AvgConfidence,
    SUM(CASE WHEN ta.IsDifficult = 1 THEN 1 ELSE 0 END) AS DifficultSamples
FROM TrainingAnnotations ta
GROUP BY ta.ClassName
ORDER BY AnnotationCount DESC;

-- Dataset split distribution
SELECT 
    DatasetSplit,
    COUNT(*) AS ImageCount,
    SUM(CASE WHEN IsLabeled = 1 THEN 1 ELSE 0 END) AS LabeledCount,
    AVG(BlurScore) AS AvgQuality,
    SUM(FileSize) / (1024.0 * 1024.0) AS TotalSizeMB
FROM TrainingImages
GROUP BY DatasetSplit;

-- =============================================
-- 2. QUALITY CONTROL
-- =============================================

-- Find images needing review
SELECT TOP 50
    ti.Id,
    ti.ImagePath,
    ti.BlurScore,
    ti.BrightnessScore,
    COUNT(ta.Id) AS AnnotationCount,
    ti.ReviewStatus
FROM TrainingImages ti
LEFT JOIN TrainingAnnotations ta ON ti.Id = ta.ImageId
WHERE ti.ReviewStatus = 'pending' 
   OR ti.BlurScore < 40  -- Low quality images
GROUP BY ti.Id, ti.ImagePath, ti.BlurScore, ti.BrightnessScore, ti.ReviewStatus
ORDER BY ti.BlurScore ASC;

-- Find images with no annotations
SELECT 
    Id,
    ImagePath,
    SourceType,
    CreatedAt
FROM TrainingImages
WHERE IsLabeled = 0
ORDER BY CreatedAt DESC;

-- Find images with suspicious annotations (very low confidence)
SELECT 
    ti.ImagePath,
    ta.ClassName,
    ta.Confidence,
    ta.AnnotationSource
FROM TrainingAnnotations ta
JOIN TrainingImages ti ON ta.ImageId = ti.Id
WHERE ta.Confidence < 0.3
ORDER BY ta.Confidence ASC;

-- =============================================
-- 3. DATA COLLECTION FROM LIVE SYSTEM
-- =============================================

-- Export recent detections as training candidates
-- (High confidence detections that can be auto-labeled)
SELECT TOP 1000
    d.ClassName,
    d.Confidence,
    d.BoundingBox,
    c.Name AS CameraName,
    d.Timestamp
FROM Detections d
JOIN Cameras c ON d.CameraId = c.Id
WHERE d.Confidence > 0.9  -- High confidence only
  AND d.Timestamp > DATEADD(DAY, -7, GETUTCDATE())
ORDER BY d.Timestamp DESC;

-- Find rare classes that need more training data
SELECT 
    ClassName,
    COUNT(*) AS DetectionCount,
    AVG(Confidence) AS AvgConfidence
FROM Detections
WHERE Timestamp > DATEADD(DAY, -30, GETUTCDATE())
GROUP BY ClassName
HAVING COUNT(*) < 100  -- Rare classes
ORDER BY DetectionCount ASC;

-- =============================================
-- 4. AUTO-LABELING HELPERS
-- =============================================

-- Create procedure to auto-label high-confidence detections
CREATE OR ALTER PROCEDURE dbo.AutoLabelDetections
    @MinConfidence FLOAT = 0.95,
    @MaxImages INT = 100
AS
BEGIN
    SET NOCOUNT ON;
    
    DECLARE @NewImages TABLE (
        ImagePath NVARCHAR(500),
        ClassName NVARCHAR(100),
        BoundingBox NVARCHAR(200),
        Confidence FLOAT
    );
    
    -- Get high-confidence detections without existing training data
    INSERT INTO @NewImages
    SELECT TOP (@MaxImages)
        CONCAT('auto_label/', FORMAT(d.Timestamp, 'yyyyMMdd'), '/', d.Id, '.jpg') AS ImagePath,
        d.ClassName,
        d.BoundingBox,
        d.Confidence
    FROM Detections d
    WHERE d.Confidence >= @MinConfidence
      AND d.Timestamp > DATEADD(DAY, -7, GETUTCDATE())
      AND NOT EXISTS (
          SELECT 1 FROM TrainingImages ti
          WHERE ti.ImagePath LIKE '%' + CAST(d.Id AS NVARCHAR(20)) + '%'
      )
    ORDER BY d.Confidence DESC;
    
    -- Insert into training images
    INSERT INTO TrainingImages (ImagePath, Width, Height, SourceType, IsLabeled, DatasetSplit)
    SELECT 
        ImagePath,
        640,  -- Default resolution
        480,
        'auto-labeled',
        1,
        CASE 
            WHEN ROW_NUMBER() OVER (ORDER BY NEWID()) % 10 = 0 THEN 'validation'
            ELSE 'train'
        END
    FROM @NewImages;
    
    -- Add annotations
    INSERT INTO TrainingAnnotations (ImageId, ClassName, BoundingBox, Confidence, AnnotationSource)
    SELECT 
        ti.Id,
        n.ClassName,
        n.BoundingBox,
        n.Confidence,
        'auto'
    FROM TrainingImages ti
    JOIN @NewImages n ON ti.ImagePath = n.ImagePath;
    
    SELECT COUNT(*) AS ImagesAdded FROM @NewImages;
END
GO

-- =============================================
-- 5. MODEL PERFORMANCE TRACKING
-- =============================================

-- Compare model versions
SELECT 
    ModelName,
    Version,
    TrainingEndTime,
    mAP50,
    mAP5095,
    ValidationAccuracy,
    InferenceTimeMs,
    TotalTrainingImages,
    Status
FROM ModelVersions
ORDER BY TrainingEndTime DESC;

-- Find best performing model per class
SELECT 
    mv.ModelName,
    mv.Version,
    mv.mAP50,
    mv.ValidationAccuracy,
    mv.ClassDistribution
FROM ModelVersions mv
WHERE mv.Status = 'completed'
ORDER BY mv.mAP50 DESC;

-- Training progress over time
SELECT 
    CONVERT(DATE, TrainingStartTime) AS TrainingDate,
    COUNT(*) AS ModelsTrained,
    AVG(mAP50) AS AvgMAP,
    AVG(ValidationAccuracy) AS AvgAccuracy
FROM ModelVersions
WHERE Status = 'completed'
GROUP BY CONVERT(DATE, TrainingStartTime)
ORDER BY TrainingDate DESC;

-- =============================================
-- 6. DATA AUGMENTATION
-- =============================================

-- Mark images for augmentation (low quantity classes)
CREATE OR ALTER PROCEDURE dbo.MarkImagesForAugmentation
    @TargetClass NVARCHAR(100),
    @MinSamples INT = 500
AS
BEGIN
    SET NOCOUNT ON;
    
    DECLARE @CurrentCount INT;
    
    -- Count current samples
    SELECT @CurrentCount = COUNT(DISTINCT ti.Id)
    FROM TrainingImages ti
    JOIN TrainingAnnotations ta ON ti.Id = ta.ImageId
    WHERE ta.ClassName = @TargetClass;
    
    IF @CurrentCount < @MinSamples
    BEGIN
        -- Mark high-quality images of this class for augmentation
        UPDATE ti
        SET ti.SourceType = 'pending-augmentation'
        FROM TrainingImages ti
        JOIN TrainingAnnotations ta ON ti.Id = ta.ImageId
        WHERE ta.ClassName = @TargetClass
          AND ti.BlurScore > 60
          AND ti.IsAugmented = 0;
        
        SELECT 
            @TargetClass AS ClassName,
            @CurrentCount AS CurrentSamples,
            @MinSamples - @CurrentCount AS SamplesNeeded,
            @@ROWCOUNT AS ImagesMarkedForAugmentation;
    END
    ELSE
    BEGIN
        SELECT 
            @TargetClass AS ClassName,
            @CurrentCount AS CurrentSamples,
            'Sufficient samples' AS Status;
    END
END
GO

-- =============================================
-- 7. EXPORT QUERIES (For YOLO Training)
-- =============================================

-- Export training data in YOLO format
-- This query helps create the data.yaml file
SELECT 
    'train: ./train/images' AS Config
UNION ALL SELECT 'val: ./val/images'
UNION ALL SELECT 'test: ./test/images'
UNION ALL SELECT ''
UNION ALL SELECT 'nc: ' + CAST(COUNT(DISTINCT ClassName) AS VARCHAR(10))
FROM TrainingAnnotations
UNION ALL SELECT 'names:'
UNION ALL 
SELECT '  - ' + ClassName
FROM (
    SELECT DISTINCT ClassName 
    FROM TrainingAnnotations
) AS Classes;

-- Get annotation statistics for README
SELECT 
    ClassName,
    COUNT(*) AS Count,
    CAST(COUNT(*) * 100.0 / SUM(COUNT(*)) OVER () AS DECIMAL(5,2)) AS Percentage
FROM TrainingAnnotations
GROUP BY ClassName
ORDER BY Count DESC;

-- =============================================
-- 8. CLEANUP AND MAINTENANCE
-- =============================================

-- Remove duplicate images (same hash)
WITH Duplicates AS (
    SELECT 
        ImageHash,
        MIN(Id) AS KeepId
    FROM TrainingImages
    WHERE ImageHash IS NOT NULL
    GROUP BY ImageHash
    HAVING COUNT(*) > 1
)
DELETE FROM TrainingImages
WHERE ImageHash IN (SELECT ImageHash FROM Duplicates)
  AND Id NOT IN (SELECT KeepId FROM Duplicates);

-- Remove low-quality images (too blurry)
DELETE FROM TrainingImages
WHERE BlurScore < 20 AND IsLabeled = 0;

-- Archive old training versions
UPDATE ModelVersions
SET Status = 'archived'
WHERE TrainingEndTime < DATEADD(MONTH, -3, GETUTCDATE())
  AND Status = 'completed';

-- =============================================
-- 9. USEFUL STORED PROCEDURES
-- =============================================

-- Get training recommendations
CREATE OR ALTER PROCEDURE dbo.GetTrainingRecommendations
AS
BEGIN
    SET NOCOUNT ON;
    
    -- Classes that need more data
    SELECT 
        'Need More Data' AS Issue,
        ta.ClassName,
        COUNT(DISTINCT ti.Id) AS CurrentImages,
        500 - COUNT(DISTINCT ti.Id) AS ImagesNeeded
    FROM TrainingAnnotations ta
    JOIN TrainingImages ti ON ta.ImageId = ti.Id
    GROUP BY ta.ClassName
    HAVING COUNT(DISTINCT ti.Id) < 500
    
    UNION ALL
    
    -- Low quality images
    SELECT 
        'Low Quality Images' AS Issue,
        NULL AS ClassName,
        COUNT(*) AS CurrentImages,
        NULL AS ImagesNeeded
    FROM TrainingImages
    WHERE BlurScore < 40
    
    UNION ALL
    
    -- Unlabeled images
    SELECT 
        'Unlabeled Images' AS Issue,
        NULL AS ClassName,
        COUNT(*) AS CurrentImages,
        NULL AS ImagesNeeded
    FROM TrainingImages
    WHERE IsLabeled = 0;
END
GO

-- Execute recommendations
EXEC dbo.GetTrainingRecommendations;

PRINT '✅ Training queries loaded successfully!';
PRINT '📚 Use these queries to manage and improve your ML model';
GO
