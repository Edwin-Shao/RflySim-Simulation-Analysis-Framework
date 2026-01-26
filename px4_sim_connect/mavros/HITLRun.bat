@ECHO OFF

REM Run script as administrator
NET SESSION >nul 2>&1 || powershell -Command "try {Start-Process cmd -ArgumentList '/c, ""%~f0""' -Verb RunAs} catch {}" && exit /b


REM The text start with 'REM' is annotation, the following options are corresponding to Options on CopterSim

REM Set the path of the RflySim tools
if not defined PSP_PATH (
    SET PSP_PATH=D:\study\matlabtools\PX4PSP
    SET PSP_PATH_LINUX=/mnt/d/study/matlabtools/PX4PSP
)

REM Start index of vehicle number (should larger than 0)
REM This option is useful for simulation with multi-computers
SET /a START_INDEX=1

REM Auto determine CopterID according to SysID of Pixhawk
SET /a IsSysID=0

REM Total vehicle Number to auto arrange position
REM SET /a TOTOAL_COPTER=8

REM Set the vehicleType/ClassID of vehicle 3D display in RflySim3D
SET /a CLASS_3D_ID=-1


REM Set use DLL model name or not, use number index or name string
REM This option is useful for simulation with other types of vehicles instead of multicopters
set DLLModel=0

REM Check if DLLModel is a name string, if yes, copy the DLL file to CopterSim folder
SET /A DLLModelVal=DLLModel
if %DLLModelVal% NEQ %DLLModel% (
    REM Copy the latest dll file to CopterSim folder
    copy /Y "%~dp0"\%DLLModel%.dll %PSP_PATH%\CopterSim\external\model\%DLLModel%.dll
)

REM Set the simulation mode on CopterSim, use number index or name string
REM e.g., 0:PX4_HITL, 1:PX4_SITL, 2:PX4_SITL_RFLY, 3:"Simulink&DLL_SIL", 4:PX4_HITL_NET, 5:EXT_HITL_COM, 6:EXT_SIM_NET
set SimMode=0

REM SET the baudrate for Pixhawk COM
SET /a BaudRate=921600

REM Set the map, use index or name of the map on CopterSim
REM e.g., UE4_MAP=1 equals to UE4_MAP=Grasslands
SET UE4_MAP=1

REM Set the origin x,y position (m) and yaw angle (degree) at the map
SET /a ORIGIN_POS_X=0
SET /a ORIGIN_POS_Y=0
SET /a ORIGIN_YAW=0

REM Set the interval between two vehicle, unit:m 
SET /a VEHICLE_INTERVAL=2


REM Set broadcast to other computer; IS_BROADCAST=0: only this computer, IS_BROADCAST=1: broadcast; 
REM or use IP address to increase speed, e.g., IS_BROADCAST=192.168.3.1
REM Note: in IP mode, IS_BROADCAST=0 equals to IS_BROADCAST=127.0.0.1, IS_BROADCAST=1 equals to IS_BROADCAST=255.255.255.255
REM You can also use a IP list with seperator "," or ";" to specify IPs to send, e.g., 127.0.0.1,192.168.1.4,192.168.1.5
SET IS_BROADCAST=0

REM Set UDP data mode; 0: UDP_FULL, 1:UDP_Simple, 2: Mavlink_Full, 3: Mavlink_simple. input number or string
REM 4:Mavlink_NoSend, 5:Mavlink_NoGPS, 6:Mavlink_Vision (NoGPS and set PX4 EKF)
SET UDPSIMMODE=Mavlink_Full

ECHO.
ECHO ---------------------------------------
REM Get the Com port number
for /f "delims=" %%t in ('%PSP_PATH%\CopterSim\GetComList.exe 3 %BaudRate%') do set ComInfoStr=%%t

for /f "tokens=1,2,3 delims=#" %%a in ("%ComInfoStr%") do (
    set ComNumExe=%%a
    set ComNameList=%%b
    set ComInfoList=%%c
)
echo Please input the Pixhawk COM port list for HIL
echo Use ',' as the separator if more than one Pixhawk
echo E.g., input 3 for COM3 of Pixhawk on the computer
echo Input 3,6,7 for COM3, COM6 and COM7 of Pixhawks 

ECHO ---------------------------------------
set remain=%ComInfoList%
echo All COM ports on this computer are:
echo.
:loopInfo
for /f "tokens=1* delims=;" %%a in ("%remain%") do (
    echo %%a
    set remain=%%b
)
if defined remain goto :loopInfo
echo.
ECHO ---------------------------------------

if %ComNumExe% EQU 0 (
    echo Warning: there is no available COM port
) else (
    echo Recommended COM list input is: %ComNameList%
)

ECHO.
ECHO ---------------------------------------
SET /P ComNum=My COM list for HITL simulation is:
SET string=%ComNum%
set subStr = ""
set /a VehicleNum=0
:split
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
    set subStr=%%i
    set string=%%j
    )
    set /a eValue=subStr
    if not %eValue% EQU %subStr% (
        echo Error: Input '%subStr%' is not a integer!
        goto EOF
    )
    set /a VehicleNum = VehicleNum +1
if not "%string%"=="" goto split
REM cho total com number is %VehicleNum%

SET /A VehicleTotalNum=%VehicleNum% + %START_INDEX% - 1
if not defined TOTOAL_COPTER (
    SET /A TOTOAL_COPTER=%VehicleTotalNum%
)

set /a sqrtNum=1
set /a squareNum=1
:loopSqrt
set /a squareNum=%sqrtNum% * %sqrtNum%
if %squareNum% EQU %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
if %squareNum% GTR %TOTOAL_COPTER% (
    goto loopSqrtEnd
)
set /a sqrtNum=%sqrtNum%+1
goto loopSqrt
:loopSqrtEnd


REM UE4Path
cd /d %PSP_PATH%\RflySim3D
tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe
choice /t 5 /d y /n >nul


tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
ECHO Kill all CopterSims


REM CptSmPath
cd /d %PSP_PATH%\CopterSim

set /a cntr = %START_INDEX%
set /a endNum = %VehicleTotalNum% +1

SET string=%ComNum%
:split1
    for /f "tokens=1,* delims=," %%i in ("%string%") do (
    set subStr=%%i
    set string=%%j
    )
    set /a PosXX=((%cntr%-1) / %sqrtNum%)*%VEHICLE_INTERVAL% + %ORIGIN_POS_X%
    set /a PosYY=((%cntr%-1) %% %sqrtNum%)*%VEHICLE_INTERVAL% + %ORIGIN_POS_Y%
    REM echo start CopterSim
    start /realtime CopterSim.exe 1 %cntr% %CLASS_3D_ID% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %PosXX% %PosYY% %ORIGIN_YAW% %subStr%:%BaudRate% %UDPSIMMODE% %IsSysID%
    choice /t 2 /d y /n >nul
    set /a cntr=%cntr%+1
    
    REM TIMEOUT /T 1
if not "%string%"=="" goto split1

REM QGCPath
tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe -noComPix
ECHO Start QGroundControl

pause

REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"

ECHO Start End.
