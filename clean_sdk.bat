@echo off
rem @file clean_sdk.bat
rem @brief Windows cleanup script for Alicia-M-SDK generated artifacts.
rem @details
rem Removes Python caches, test caches, coverage files, packaging outputs,
rem documentation build output, and *.egg-info directories.
rem @note
rem logs/ and virtual environments are intentionally opt-in. Use /logs or /venv
rem when those directories should also be removed.
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
set "DRY_RUN=0"
set "CLEAN_LOGS=0"
set "CLEAN_VENV=0"

:parse_args
if "%~1"=="" goto after_args
if /I "%~1"=="/dry-run" (
    set "DRY_RUN=1"
    shift
    goto parse_args
)
if /I "%~1"=="--dry-run" (
    set "DRY_RUN=1"
    shift
    goto parse_args
)
if /I "%~1"=="/logs" (
    set "CLEAN_LOGS=1"
    shift
    goto parse_args
)
if /I "%~1"=="--logs" (
    set "CLEAN_LOGS=1"
    shift
    goto parse_args
)
if /I "%~1"=="/venv" (
    set "CLEAN_VENV=1"
    shift
    goto parse_args
)
if /I "%~1"=="--venv" (
    set "CLEAN_VENV=1"
    shift
    goto parse_args
)
if /I "%~1"=="/help" goto help
if /I "%~1"=="--help" goto help
echo Unknown option: %~1
echo.
goto help

:after_args
pushd "%ROOT%" >nul || (
    echo Failed to enter SDK root: "%ROOT%"
    exit /b 1
)

echo Alicia-M-SDK cleanup
echo Root: %CD%
if "%DRY_RUN%"=="1" echo Mode: dry-run, no files will be removed.
echo.

rem @brief Remove Python bytecode and cache directories.
call :delete_tree_by_name "__pycache__"
call :delete_tree_by_name ".pytest_cache"
call :delete_tree_by_name ".mypy_cache"
call :delete_tree_by_name ".ruff_cache"
call :delete_tree_by_name ".cache"
call :delete_tree_by_name ".hypothesis"

rem @brief Remove build, packaging, coverage, and documentation artifacts.
call :delete_dir "build"
call :delete_dir "dist"
call :delete_dir "htmlcov"
call :delete_dir "cover"
call :delete_dir ".tox"
call :delete_dir ".nox"
call :delete_dir ".eggs"
call :delete_dir "wheels"
call :delete_dir "docs\_build"
call :delete_dir ".pdm-build"
call :delete_dir "__pypackages__"

for /d %%D in (*.egg-info) do call :delete_dir "%%~fD"

rem @brief Remove Python and test output files.
call :delete_files "*.pyc"
call :delete_files "*.pyo"
call :delete_files "*.py.cover"
call :delete_files "*.cover"
call :delete_file ".coverage"
call :delete_files ".coverage.*"
call :delete_file "coverage.xml"
call :delete_file "nosetests.xml"
call :delete_file "pip-log.txt"
call :delete_file "pip-delete-this-directory.txt"

if "%CLEAN_LOGS%"=="1" goto clean_logs
echo.
echo Skipping logs. Use /logs to remove logs and *.log files.
goto after_logs

:clean_logs
echo.
echo Cleaning logs because /logs was provided.
call :delete_dir "logs"
call :delete_dir "examples\logs"
call :delete_files "*.log"

:after_logs
if "%CLEAN_VENV%"=="1" goto clean_venv
echo Skipping virtual environments. Use /venv to remove local env folders.
goto after_venv

:clean_venv
echo.
echo Cleaning virtual environments because /venv was provided.
call :delete_dir ".venv"
call :delete_dir "venv"
call :delete_dir "env"
call :delete_dir "ENV"

:after_venv

echo.
if "%DRY_RUN%"=="1" (
    echo Dry-run complete.
) else (
    echo Cleanup complete.
)

popd >nul
exit /b 0

:delete_tree_by_name
for /d /r %%D in (%~1) do call :delete_dir "%%~fD"
exit /b 0

:delete_dir
if not exist "%~1" exit /b 0
if "%DRY_RUN%"=="1" (
    echo [dry-run] rmdir /s /q "%~1"
) else (
    echo Removing directory: "%~1"
    rmdir /s /q "%~1"
)
exit /b 0

:delete_file
if not exist "%~1" exit /b 0
if "%DRY_RUN%"=="1" (
    echo [dry-run] del /f /q "%~1"
) else (
    echo Removing file: "%~1"
    del /f /q "%~1" >nul 2>nul
)
exit /b 0

:delete_files
for /r %%F in (%~1) do (
    if exist "%%~fF" (
        if "%DRY_RUN%"=="1" (
            echo [dry-run] del /f /q "%%~fF"
        ) else (
            echo Removing file: "%%~fF"
            del /f /q "%%~fF" >nul 2>nul
        )
    )
)
exit /b 0

:help
echo Alicia-M-SDK cleanup script
echo.
echo Usage:
echo   clean_sdk.bat [options]
echo.
echo Options:
echo   /dry-run   Show what would be removed without deleting anything.
echo   /logs      Also remove logs/, examples/logs/, and *.log files.
echo   /venv      Also remove .venv/, venv/, env/, and ENV/.
echo   /help      Show this help.
echo.
echo Default cleanup removes Python caches, test caches, coverage files,
echo build artifacts, dist artifacts, docs build output, and *.egg-info.
exit /b 0
