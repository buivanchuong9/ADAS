"""
Migration Script: SQLite to SQL Server
Migrates existing ADAS data from SQLite to SQL Server
"""

import pyodbc
import sqlite3
import json
from datetime import datetime
from typing import Dict, List, Any 

# SQL Server Configuration
SQL_SERVER_CONFIG = {
    'server': 'localhost',  # Change to your server
    'database': 'ADAS_DB',
    'username': 'sa',
    'password': 'YourPassword123!',
    'driver': '{ODBC Driver 18 for SQL Server}',
    'trust_server_certificate': 'yes'
}

# SQLite Configuration
SQLITE_DB_PATH = './adas_test.db'


class DatabaseMigration:
    def __init__(self):
        self.sqlite_conn = None
        self.mssql_conn = None
        self.stats = {
            'drivers': 0,
            'cameras': 0,
            'trips': 0,
            'events': 0,
            'detections': 0,
            'ai_models': 0,
            'errors': []
        }
    
    def connect_sqlite(self):
        """Connect to SQLite database"""
        try:
            self.sqlite_conn = sqlite3.connect(SQLITE_DB_PATH)
            self.sqlite_conn.row_factory = sqlite3.Row
            print("✅ Connected to SQLite database")
            return True
        except Exception as e:
            print(f"❌ SQLite connection error: {e}")
            return False
    
    def connect_mssql(self):
        """Connect to SQL Server"""
        try:
            conn_str = (
                f"DRIVER={SQL_SERVER_CONFIG['driver']};"
                f"SERVER={SQL_SERVER_CONFIG['server']};"
                f"DATABASE={SQL_SERVER_CONFIG['database']};"
                f"UID={SQL_SERVER_CONFIG['username']};"
                f"PWD={SQL_SERVER_CONFIG['password']};"
                f"TrustServerCertificate={SQL_SERVER_CONFIG['trust_server_certificate']}"
            )
            self.mssql_conn = pyodbc.connect(conn_str)
            print("✅ Connected to SQL Server")
            return True
        except Exception as e:
            print(f"❌ SQL Server connection error: {e}")
            return False
    
    def migrate_drivers(self):
        """Migrate drivers table"""
        try:
            cursor_sqlite = self.sqlite_conn.cursor()
            cursor_mssql = self.mssql_conn.cursor()
            
            # Get data from SQLite
            cursor_sqlite.execute("SELECT * FROM drivers")
            rows = cursor_sqlite.fetchall()
            
            print(f"📦 Migrating {len(rows)} drivers...")
            
            for row in rows:
                cursor_mssql.execute("""
                    INSERT INTO dbo.Drivers 
                    (Name, Email, PhoneNumber, LicenseNumber, SafetyScore, TotalTrips, TotalIncidents, Status, CreatedAt)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    row['name'],
                    row['email'],
                    row['phone_number'],
                    row['license_number'],
                    row['safety_score'] or 100.0,
                    row['total_trips'] or 0,
                    row['total_incidents'] or 0,
                    row['status'] or 'active',
                    row['created_at'] or datetime.utcnow()
                ))
                self.stats['drivers'] += 1
            
            self.mssql_conn.commit()
            print(f"✅ Migrated {self.stats['drivers']} drivers")
        except Exception as e:
            print(f"❌ Error migrating drivers: {e}")
            self.stats['errors'].append(f"Drivers: {e}")
    
    def migrate_cameras(self):
        """Migrate cameras table"""
        try:
            cursor_sqlite = self.sqlite_conn.cursor()
            cursor_mssql = self.mssql_conn.cursor()
            
            cursor_sqlite.execute("SELECT * FROM cameras")
            rows = cursor_sqlite.fetchall()
            
            print(f"📦 Migrating {len(rows)} cameras...")
            
            for row in rows:
                cursor_mssql.execute("""
                    INSERT INTO dbo.Cameras 
                    (Name, Location, CameraType, Resolution, FPS, StreamUrl, Status, IsOnline, CreatedAt)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    row['name'],
                    row['location'],
                    row['camera_type'],
                    row['resolution'],
                    row['fps'] or 30,
                    row['stream_url'],
                    row['status'] or 'active',
                    row['is_online'] or 0,
                    row['created_at'] or datetime.utcnow()
                ))
                self.stats['cameras'] += 1
            
            self.mssql_conn.commit()
            print(f"✅ Migrated {self.stats['cameras']} cameras")
        except Exception as e:
            print(f"❌ Error migrating cameras: {e}")
            self.stats['errors'].append(f"Cameras: {e}")
    
    def migrate_trips(self):
        """Migrate trips table"""
        try:
            cursor_sqlite = self.sqlite_conn.cursor()
            cursor_mssql = self.mssql_conn.cursor()
            
            cursor_sqlite.execute("SELECT * FROM trips")
            rows = cursor_sqlite.fetchall()
            
            print(f"📦 Migrating {len(rows)} trips...")
            
            for row in rows:
                cursor_mssql.execute("""
                    INSERT INTO dbo.Trips 
                    (StartTime, EndTime, Duration, Distance, AverageSpeed, MaxSpeed, Route, Status, DriverId, CameraId)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    row['start_time'],
                    row['end_time'],
                    row['duration'],
                    row['distance'],
                    row['average_speed'],
                    row['max_speed'],
                    row['route'],
                    row['status'] or 'completed',
                    row['driver_id'],
                    row['camera_id']
                ))
                self.stats['trips'] += 1
            
            self.mssql_conn.commit()
            print(f"✅ Migrated {self.stats['trips']} trips")
        except Exception as e:
            print(f"❌ Error migrating trips: {e}")
            self.stats['errors'].append(f"Trips: {e}")
    
    def migrate_detections(self):
        """Migrate detections table"""
        try:
            cursor_sqlite = self.sqlite_conn.cursor()
            cursor_mssql = self.mssql_conn.cursor()
            
            cursor_sqlite.execute("SELECT * FROM detections ORDER BY timestamp DESC LIMIT 10000")
            rows = cursor_sqlite.fetchall()
            
            print(f"📦 Migrating {len(rows)} detections (last 10000)...")
            
            batch_size = 1000
            for i in range(0, len(rows), batch_size):
                batch = rows[i:i + batch_size]
                
                for row in batch:
                    cursor_mssql.execute("""
                        INSERT INTO dbo.Detections 
                        (ClassName, Confidence, BoundingBox, DistanceMeters, Timestamp, TripId, CameraId)
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                    """, (
                        row['class_name'],
                        row['confidence'],
                        row['bounding_box'],
                        row['distance_meters'],
                        row['timestamp'],
                        row['trip_id'],
                        row['camera_id']
                    ))
                    self.stats['detections'] += 1
                
                self.mssql_conn.commit()
                print(f"  ⏳ Migrated {self.stats['detections']}/{len(rows)} detections...")
            
            print(f"✅ Migrated {self.stats['detections']} detections")
        except Exception as e:
            print(f"❌ Error migrating detections: {e}")
            self.stats['errors'].append(f"Detections: {e}")
    
    def migrate_events(self):
        """Migrate events table"""
        try:
            cursor_sqlite = self.sqlite_conn.cursor()
            cursor_mssql = self.mssql_conn.cursor()
            
            cursor_sqlite.execute("SELECT * FROM events")
            rows = cursor_sqlite.fetchall()
            
            print(f"📦 Migrating {len(rows)} events...")
            
            for row in rows:
                cursor_mssql.execute("""
                    INSERT INTO dbo.Events 
                    (EventType, Description, Timestamp, Severity, Location, EventMetadata, TripId, CameraId, DriverId)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    row['event_type'],
                    row['description'],
                    row['timestamp'],
                    row['severity'],
                    row['location'],
                    row['event_metadata'] if 'event_metadata' in row.keys() else None,
                    row['trip_id'],
                    row['camera_id'],
                    row['driver_id']
                ))
                self.stats['events'] += 1
            
            self.mssql_conn.commit()
            print(f"✅ Migrated {self.stats['events']} events")
        except Exception as e:
            print(f"❌ Error migrating events: {e}")
            self.stats['errors'].append(f"Events: {e}")
    
    def migrate_ai_models(self):
        """Migrate AI models table"""
        try:
            cursor_sqlite = self.sqlite_conn.cursor()
            cursor_mssql = self.mssql_conn.cursor()
            
            cursor_sqlite.execute("SELECT * FROM ai_models")
            rows = cursor_sqlite.fetchall()
            
            print(f"📦 Migrating {len(rows)} AI models...")
            
            for row in rows:
                # Check if model already exists (from seed data)
                cursor_mssql.execute(
                    "SELECT COUNT(*) FROM dbo.AIModels WHERE Name = ?",
                    (row['name'],)
                )
                exists = cursor_mssql.fetchone()[0] > 0
                
                if not exists:
                    cursor_mssql.execute("""
                        INSERT INTO dbo.AIModels 
                        (Name, ModelType, Version, FilePath, FileSize, Accuracy, IsActive, IsDownloaded, Description, CreatedAt)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """, (
                        row['name'],
                        row['model_type'],
                        row['version'],
                        row['file_path'],
                        row['file_size'],
                        row['accuracy'],
                        row['is_active'] or 0,
                        row['is_downloaded'] or 0,
                        row['description'],
                        row['created_at'] or datetime.utcnow()
                    ))
                    self.stats['ai_models'] += 1
            
            self.mssql_conn.commit()
            print(f"✅ Migrated {self.stats['ai_models']} AI models")
        except Exception as e:
            print(f"❌ Error migrating AI models: {e}")
            self.stats['errors'].append(f"AI Models: {e}")
    
    def run_migration(self):
        """Run complete migration"""
        print("=" * 60)
        print("🚀 Starting ADAS Database Migration")
        print("=" * 60)
        print(f"📂 Source: SQLite ({SQLITE_DB_PATH})")
        print(f"📂 Target: SQL Server ({SQL_SERVER_CONFIG['server']}/{SQL_SERVER_CONFIG['database']})")
        print("=" * 60)
        print()
        
        # Connect to databases
        if not self.connect_sqlite():
            return False
        
        if not self.connect_mssql():
            return False
        
        print()
        
        # Run migrations in order (respecting foreign keys)
        self.migrate_drivers()
        self.migrate_cameras()
        self.migrate_ai_models()
        self.migrate_trips()
        self.migrate_events()
        self.migrate_detections()
        
        # Print summary
        print()
        print("=" * 60)
        print("📊 Migration Summary")
        print("=" * 60)
        print(f"✅ Drivers migrated:    {self.stats['drivers']}")
        print(f"✅ Cameras migrated:    {self.stats['cameras']}")
        print(f"✅ AI Models migrated:  {self.stats['ai_models']}")
        print(f"✅ Trips migrated:      {self.stats['trips']}")
        print(f"✅ Events migrated:     {self.stats['events']}")
        print(f"✅ Detections migrated: {self.stats['detections']}")
        print("=" * 60)
        
        if self.stats['errors']:
            print()
            print("⚠️  Errors encountered:")
            for error in self.stats['errors']:
                print(f"   - {error}")
        else:
            print("✅ Migration completed successfully!")
        
        # Close connections
        if self.sqlite_conn:
            self.sqlite_conn.close()
        if self.mssql_conn:
            self.mssql_conn.close()
        
        return True


if __name__ == "__main__":
    migration = DatabaseMigration()
    migration.run_migration()
