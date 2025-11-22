# Testing Guide

## Setup Testing Environment

### 1. Install test dependencies

```bash
pip install pytest pytest-asyncio httpx
```

### 2. Tạo test database

Sử dụng database riêng cho testing:

```env
# .env.test
SQL_SERVER=localhost
SQL_DATABASE=ADAS_TEST_DB
SQL_USERNAME=sa
SQL_PASSWORD=YourPassword
```

## Unit Tests

### Test Database Connection

Tạo file `test_database.py`:

```python
import pytest
from database import test_connection, get_db
from models import Base, Camera
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

def test_database_connection():
    """Test SQL Server connection"""
    assert test_connection() == True

def test_create_tables():
    """Test table creation"""
    from config import settings
    engine = create_engine(settings.get_database_url())
    Base.metadata.create_all(bind=engine)
    
    # Verify tables exist
    assert engine.dialect.has_table(engine.connect(), "cameras")
    assert engine.dialect.has_table(engine.connect(), "trips")
```

Chạy test:
```bash
pytest test_database.py -v
```

### Test Models

Tạo file `test_models.py`:

```python
import pytest
from datetime import datetime
from models import Camera, Driver, Trip

def test_camera_model():
    """Test Camera model creation"""
    camera = Camera(
        name="Test Camera",
        type="webcam",
        status="ready",
        url="http://test"
    )
    assert camera.name == "Test Camera"
    assert camera.type == "webcam"

def test_driver_model():
    """Test Driver model"""
    driver = Driver(
        name="Test Driver",
        license_number="TEST123",
        phone="1234567890"
    )
    assert driver.safety_score == 100.0
```

### Test Services

Tạo file `test_services.py`:

```python
import pytest
from services import CameraService, DriverService
from database import get_db

@pytest.fixture
def db_session():
    """Create test database session"""
    from database import SessionLocal
    db = SessionLocal()
    yield db
    db.close()

def test_camera_service(db_session):
    """Test CameraService"""
    service = CameraService(db_session)
    
    # Create camera
    camera = service.create_camera({
        "name": "Test Cam",
        "type": "webcam",
        "status": "ready"
    })
    
    assert camera.id is not None
    assert camera.name == "Test Cam"
    
    # Get camera
    found = service.get_camera(camera.id)
    assert found.id == camera.id
    
    # Delete
    service.delete_camera(camera.id)
```

## Integration Tests

### Test API Endpoints

Tạo file `test_api.py`:

```python
import pytest
from httpx import AsyncClient
from main import app

@pytest.mark.asyncio
async def test_health_endpoint():
    """Test health check"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"

@pytest.mark.asyncio
async def test_cameras_list():
    """Test cameras list endpoint"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/api/cameras/list")
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)

@pytest.mark.asyncio
async def test_create_camera():
    """Test create camera"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        payload = {
            "name": "API Test Camera",
            "type": "webcam",
            "status": "ready"
        }
        response = await client.post("/api/cameras", json=payload)
        assert response.status_code == 200
        data = response.json()
        assert data["name"] == payload["name"]
```

Chạy:
```bash
pytest test_api.py -v
```

## WebSocket Tests

Tạo file `test_websocket.py`:

```python
import pytest
import base64
from httpx import AsyncClient
from fastapi.testclient import TestClient
from main import app

def test_websocket_connection():
    """Test WebSocket inference connection"""
    client = TestClient(app)
    with client.websocket_connect("/ws/infer") as websocket:
        # Send test image
        test_image_b64 = "iVBORw0KGgoAAAANSUhEUg..." # truncated
        websocket.send_json({"frameB64": test_image_b64})
        
        # Receive response
        data = websocket.receive_json()
        assert "detections" in data
        assert isinstance(data["detections"], list)
```

## Load Testing

### Sử dụng Locust

Install:
```bash
pip install locust
```

Tạo file `locustfile.py`:

```python
from locust import HttpUser, task, between

class AdasUser(HttpUser):
    wait_time = between(1, 3)
    
    @task(3)
    def get_cameras(self):
        self.client.get("/api/cameras/list")
    
    @task(2)
    def get_trips(self):
        self.client.get("/api/trips/list")
    
    @task(1)
    def create_camera(self):
        self.client.post("/api/cameras", json={
            "name": f"Load Test Cam {self.user_id}",
            "type": "webcam",
            "status": "ready"
        })
```

Chạy:
```bash
locust -f locustfile.py --host http://localhost:8000
```

## Coverage Report

Install:
```bash
pip install pytest-cov
```

Chạy với coverage:
```bash
pytest --cov=. --cov-report=html
```

Xem report:
```bash
open htmlcov/index.html
```

## CI/CD Testing

### GitHub Actions

Tạo `.github/workflows/test.yml`:

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    
    services:
      sqlserver:
        image: mcr.microsoft.com/mssql/server:2019-latest
        env:
          ACCEPT_EULA: Y
          SA_PASSWORD: TestPassword123!
        ports:
          - 1433:1433
    
    steps:
    - uses: actions/checkout@v2
    
    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-asyncio pytest-cov
    
    - name: Run tests
      env:
        SQL_SERVER: localhost
        SQL_DATABASE: ADAS_TEST_DB
        SQL_USERNAME: sa
        SQL_PASSWORD: TestPassword123!
      run: |
        pytest --cov=. --cov-report=xml
    
    - name: Upload coverage
      uses: codecov/codecov-action@v2
```

## Manual Testing Checklist

- [ ] Health endpoint responds
- [ ] Database connection works
- [ ] Can create camera
- [ ] Can list cameras
- [ ] Can update camera status
- [ ] Can create trip
- [ ] Can end trip
- [ ] Can create event
- [ ] Can get analytics
- [ ] WebSocket connects
- [ ] WebSocket receives detections
- [ ] CORS headers present
- [ ] Error handling works (404, 500)

## Performance Testing

### Response time benchmarks

```bash
# Install ab (Apache Bench)
# Test cameras endpoint
ab -n 1000 -c 10 http://localhost:8000/api/cameras/list

# Test create endpoint
ab -n 100 -c 5 -p camera.json -T application/json http://localhost:8000/api/cameras
```

Expected results:
- GET endpoints: < 50ms average
- POST endpoints: < 100ms average
- WebSocket latency: < 30ms

---

Run all tests:
```bash
pytest -v --cov=. --cov-report=term-missing
```
