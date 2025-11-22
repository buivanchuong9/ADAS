# Development Setup - VSCode

## Cài đặt môi trường development

### 1. Python Extension

Cài đặt extension:
- Python (ms-python.python)
- Pylance (ms-python.vscode-pylance)

### 2. Tạo và activate virtual environment

```bash
# Windows
python -m venv venv
venv\Scripts\activate

# Linux/Mac
python3 -m venv venv
source venv/bin/activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Chọn Python Interpreter trong VSCode

- Press `Ctrl+Shift+P` (Windows/Linux) hoặc `Cmd+Shift+P` (Mac)
- Gõ: "Python: Select Interpreter"
- Chọn: `./venv/Scripts/python.exe` (Windows) hoặc `./venv/bin/python` (Unix)

### 5. Setup .env file

```bash
cp .env.example .env
# Sửa .env với SQL Server credentials của bạn
```

---

## VSCode Settings (Optional)

Tạo file `.vscode/settings.json`:

```json
{
  "python.defaultInterpreterPath": "${workspaceFolder}/backend-python/venv/bin/python",
  "python.linting.enabled": true,
  "python.linting.pylintEnabled": false,
  "python.linting.flake8Enabled": true,
  "python.formatting.provider": "black",
  "editor.formatOnSave": true,
  "python.analysis.typeCheckingMode": "basic",
  "python.analysis.autoImportCompletions": true,
  "[python]": {
    "editor.defaultFormatter": "ms-python.black-formatter",
    "editor.codeActionsOnSave": {
      "source.organizeImports": true
    }
  }
}
```

---

## VSCode Launch Configuration

Tạo file `.vscode/launch.json`:

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Python: FastAPI",
      "type": "python",
      "request": "launch",
      "module": "uvicorn",
      "args": [
        "main:app",
        "--reload",
        "--host",
        "0.0.0.0",
        "--port",
        "8000"
      ],
      "jinja": true,
      "justMyCode": true,
      "envFile": "${workspaceFolder}/backend-python/.env",
      "cwd": "${workspaceFolder}/backend-python"
    },
    {
      "name": "Python: Seed Database",
      "type": "python",
      "request": "launch",
      "program": "${workspaceFolder}/backend-python/seed.py",
      "console": "integratedTerminal",
      "cwd": "${workspaceFolder}/backend-python",
      "envFile": "${workspaceFolder}/backend-python/.env"
    }
  ]
}
```

---

## Linting & Formatting

### Install dev tools

```bash
pip install black flake8 isort mypy
```

### Format code

```bash
# Black (formatter)
black main.py models.py schemas.py

# isort (import organizer)
isort main.py models.py schemas.py

# flake8 (linter)
flake8 main.py models.py schemas.py
```

---

## Fix Import Errors

Nếu VSCode vẫn hiện lỗi "Import could not be resolved":

### Option 1: Reload Window
- `Ctrl+Shift+P` → "Developer: Reload Window"

### Option 2: Restart Python Language Server
- `Ctrl+Shift+P` → "Python: Restart Language Server"

### Option 3: Verify Interpreter
```bash
# Trong terminal VSCode
which python  # Mac/Linux
where python  # Windows

# Should output: .../backend-python/venv/bin/python
```

---

## Run & Debug trong VSCode

### Run với debugger
1. Mở `main.py`
2. Press `F5`
3. Chọn "Python: FastAPI"
4. Server sẽ chạy với breakpoints enabled

### Set breakpoints
- Click vào gutter bên trái line number
- Màu đỏ = breakpoint active

### Debug console
- Khi hit breakpoint, mở Debug Console để inspect variables
- Gõ tên variable để xem giá trị

---

## Testing trong VSCode

### Configure pytest

Tạo file `pytest.ini`:

```ini
[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
```

### Enable Testing UI

1. Click Testing icon trong sidebar
2. Click "Configure Python Tests"
3. Chọn "pytest"
4. Chọn root directory

### Run tests
- Click ▶️ button trong Testing panel
- Hoặc `Ctrl+Shift+P` → "Testing: Run All Tests"

---

## Database Tools

### VSCode Extensions

Install:
- **SQL Server (mssql)** by Microsoft
- **SQLTools** by Matheus Teixeira

### Connect to SQL Server

1. Click SQL Server icon
2. Add connection:
   - Server: `localhost`
   - Database: `ADAS_DB`
   - Authentication: SQL Login
   - Username: `sa`
   - Password: your password

3. Test query trong `.sql` files

---

## Git Integration

### Recommended .gitignore

Already created in `backend-python/.gitignore`

### Commit conventions

```
feat: Add new camera endpoint
fix: Fix WebSocket connection issue
docs: Update README
refactor: Simplify database service
test: Add unit tests for models
```

---

## Common Issues

### Issue: Import errors
✅ Solution: Select correct Python interpreter

### Issue: Module not found
✅ Solution: `pip install -r requirements.txt`

### Issue: Linter errors
✅ Solution: Install `flake8` and `black`

### Issue: Debug không chạy
✅ Solution: Check `launch.json` config

---

## Useful VSCode Shortcuts

| Action | Windows/Linux | Mac |
|--------|---------------|-----|
| Run/Debug | `F5` | `F5` |
| Command Palette | `Ctrl+Shift+P` | `Cmd+Shift+P` |
| Format Document | `Shift+Alt+F` | `Shift+Option+F` |
| Go to Definition | `F12` | `F12` |
| Find All References | `Shift+F12` | `Shift+F12` |
| Rename Symbol | `F2` | `F2` |
| Toggle Terminal | `` Ctrl+` `` | `` Cmd+` `` |

---

**Development environment ready! 🚀**
