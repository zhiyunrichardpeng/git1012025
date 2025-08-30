@echo off
set SOURCE="C:\keyLabLocal_D\games103 assignment\games103 assignment V3\My project\Assets\Rigid_Bunny.cs"
set DEST="C:\keyLabLocal_D\games103 assignment\a1 backup"

REM Create backup directory if it doesn't exist
if not exist %DEST% mkdir %DEST%

REM Copy file with timestamp and new name
for /f "tokens=1-3 delims=/" %%a in ('date /t') do set DATE=%%a-%%b-%%c
for /f "tokens=1-2 delims=: " %%a in ('time /t') do set TIME=%%a-%%b

copy %SOURCE% "%DEST%\a99_bunny_rigid_%DATE%_%TIME%.cs"

echo Backup completed!
pause