# 🧪 Test Docker Setup

## Bước 1: Test Build
```bash
cd backend-python
docker compose build
```

## Bước 2: Test Run
```bash
docker compose up
```

## Bước 3: Test API
```bash
# Terminal mới
curl http://localhost:8000/health

# Kết quả mong đợi:
# {"status":"healthy","version":"3.0.0"}
```

## Bước 4: Test WebSocket
```bash
# Test với wscat (nếu có)
npx wscat -c ws://localhost:8000/ws/inference

# Hoặc dùng Python
python3 -c "
import asyncio
import websockets
import json

async def test():
    async with websockets.connect('ws://localhost:8000/ws/inference') as ws:
        print('✅ WebSocket connected!')
        await ws.send(json.dumps({'model_id': 'yolo11n', 'test': True}))
        response = await ws.recv()
        print(f'Response: {response}')

asyncio.run(test())
"
```

## ✅ Checklist
- [ ] Docker build thành công
- [ ] Container chạy (docker compose ps)
- [ ] API health check OK
- [ ] WebSocket kết nối OK
- [ ] Logs không có lỗi (docker compose logs)
