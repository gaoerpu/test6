@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo       VESC 4-Motor Build (Turbo Mode -j)
echo ========================================================

:: 0. 检查 Makefile 是否存在
if not exist Makefile (
    echo [ERROR] 找不到 Makefile！请将此脚本放在项目根目录下。
    pause
    exit /b 1
)

:: 1. 设置输出路径: build/all_build_6kw_20kw
set "OUT_DIR=build\all_build_6kw_20kw"

:: 如果 build 文件夹本身都不存在，先创建 build
if not exist build mkdir build

:: 在 build 里面创建汇总文件夹
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"

echo [状态] 启用多核编译 (-j)，速度将大幅提升！
echo [状态] 最终生成的8个文件将存放在: %OUT_DIR%
echo.

:: ==========================================================
:: 任务 1: 6KW Left (左后)
:: ==========================================================
echo [1/4] 正在编译 6KW Left (ikesc_6kw_left)...
:: -j%NUMBER_OF_PROCESSORS% 表示调用所有 CPU 核心
make -j%NUMBER_OF_PROCESSORS% ikesc_6kw_left

if exist build\ikesc_6kw_left\ikesc_6kw_left.bin (
    echo    - [OK] bin 复制中...
    copy /Y build\ikesc_6kw_left\ikesc_6kw_left.bin "%OUT_DIR%\ikesc_6kw_left.bin" >nul
    echo    - [OK] hex 复制中...
    copy /Y build\ikesc_6kw_left\ikesc_6kw_left.hex "%OUT_DIR%\ikesc_6kw_left.hex" >nul
) else (
    echo [WARNING] 未找到 6KW Left 的输出文件！
)
echo.

:: ==========================================================
:: 任务 2: 6KW Right (右后)
:: ==========================================================
echo [2/4] 正在编译 6KW Right (ikesc_6kw_right)...
make -j%NUMBER_OF_PROCESSORS% ikesc_6kw_right

if exist build\ikesc_6kw_right\ikesc_6kw_right.bin (
    echo    - [OK] bin 复制中...
    copy /Y build\ikesc_6kw_right\ikesc_6kw_right.bin "%OUT_DIR%\ikesc_6kw_right.bin" >nul
    echo    - [OK] hex 复制中...
    copy /Y build\ikesc_6kw_right\ikesc_6kw_right.hex "%OUT_DIR%\ikesc_6kw_right.hex" >nul
) else (
    echo [WARNING] 未找到 6KW Right 的输出文件！
)
echo.

:: ==========================================================
:: 任务 3: 20KW Left (左前)
:: ==========================================================
echo [3/4] 正在编译 20KW Left (ikesc_20kw_left)...
make -j%NUMBER_OF_PROCESSORS% ikesc_20kw_left

if exist build\ikesc_20kw_left\ikesc_20kw_left.bin (
    echo    - [OK] bin 复制中...
    copy /Y build\ikesc_20kw_left\ikesc_20kw_left.bin "%OUT_DIR%\ikesc_20kw_left.bin" >nul
    echo    - [OK] hex 复制中...
    copy /Y build\ikesc_20kw_left\ikesc_20kw_left.hex "%OUT_DIR%\ikesc_20kw_left.hex" >nul
) else (
    echo [WARNING] 未找到 20KW Left 的输出文件！
)
echo.

:: ==========================================================
:: 任务 4: 20KW Right (右前)
:: ==========================================================
echo [4/4] 正在编译 20KW Right (ikesc_20kw_right)...
make -j%NUMBER_OF_PROCESSORS% ikesc_20kw_right

if exist build\ikesc_20kw_right\ikesc_20kw_right.bin (
    echo    - [OK] bin 复制中...
    copy /Y build\ikesc_20kw_right\ikesc_20kw_right.bin "%OUT_DIR%\ikesc_20kw_right.bin" >nul
    echo    - [OK] hex 复制中...
    copy /Y build\ikesc_20kw_right\ikesc_20kw_right.hex "%OUT_DIR%\ikesc_20kw_right.hex" >nul
) else (
    echo [WARNING] 未找到 20KW Right 的输出文件！
)
echo.

:: ==========================================================
:: 结束
:: ==========================================================
echo ========================================================
echo [SUCCESS] 全部完成！
echo 请查看文件夹: build\all_build_6kw_20kw
echo ========================================================
pause
