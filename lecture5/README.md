# Lecture 5：Kalman

## Windows 环境配置

### 1.安装uv
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```
安装完成后，关闭并重新打开 PowerShell，再检查：
```powershell
uv --version
```

如果仍提示找不到 `uv`，按默认安装位置将它加入当前终端的 PATH：

```powershell
$env:Path = "$env:USERPROFILE\.local\bin;$env:Path"
uv --version
```

### 2.安装项目 Python 和依赖

先进入 `lecture5` 文件夹，后执行下列命令
```powershell
uv python install 3.12
uv python pin 3.12
uv sync --dev
uv run python --version
```

### 3.启动仿真

在 `lecture5` 文件夹中运行主程序：

```powershell
# 匀速往返平移
uv run python main_translation.py
# 匀速旋转
uv run python main_rotation.py
```
