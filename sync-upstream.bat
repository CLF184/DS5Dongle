@echo off
rem ============================================================================
rem  sync-upstream.bat -- Windows (cmd) entry point, equivalent to sync-upstream.sh
rem
rem  Usage (same arguments as the .sh):
rem    sync-upstream.bat                  sync + push to fork
rem    sync-upstream.bat --check          report status only (incl. unfinished work)
rem    sync-upstream.bat --no-push        rebase locally, do not push
rem    sync-upstream.bat --build          only if a build toolchain exists on this PC
rem    sync-upstream.bat --repo DIR       repo to sync (default: this script's dir,
rem                                       or <this dir>\reverse-port if present)
rem    sync-upstream.bat --fork URL       fork to push to
rem    sync-upstream.bat --force          continue even with a dirty working tree
rem
rem  This file only does two things: locate the bash shipped with Git, then hand
rem  the arguments to sync-upstream.sh next to it. All logic (backup refs,
rem  unfinished-run recovery, mirror fallback, ...) lives in that one script.
rem  Keep both files in the same directory.
rem
rem  NOTE: this file is intentionally ASCII-only -- cmd.exe parses .bat files in
rem  the OEM codepage (GBK on Chinese Windows), so any UTF-8 text here would be
rem  misparsed. All Chinese output comes from the .sh via bash, which is fine.
rem ============================================================================
setlocal

chcp 65001 >nul

set "HERE=%~dp0"
set "SH=%HERE%sync-upstream.sh"

if not exist "%SH%" (
  echo [ERROR] cannot find "%SH%"
  echo         put sync-upstream.bat and sync-upstream.sh in the same folder.
  exit /b 2
)

rem ---- locate a usable bash (bundled with Git for Windows) ----
set "BASH="
for %%b in (bash.exe) do if not defined BASH if exist "%%~$PATH:b" set "BASH=%%~$PATH:b"
if not defined BASH if exist "%ProgramFiles%\Git\bin\bash.exe" set "BASH=%ProgramFiles%\Git\bin\bash.exe"
if not defined BASH if exist "%ProgramFiles(x86)%\Git\bin\bash.exe" set "BASH=%ProgramFiles(x86)%\Git\bin\bash.exe"
if not defined BASH if exist "%LOCALAPPDATA%\Programs\Git\bin\bash.exe" set "BASH=%LOCALAPPDATA%\Programs\Git\bin\bash.exe"

if not defined BASH (
  echo [ERROR] bash.exe not found ^(it ships with Git for Windows^).
  echo         install Git for Windows and retry: https://git-scm.com/download/win
  exit /b 2
)

rem ---- forward all arguments to the shell script ----
"%BASH%" "%SH%" %*
exit /b %ERRORLEVEL%
