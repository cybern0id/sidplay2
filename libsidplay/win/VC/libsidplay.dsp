# Microsoft Developer Studio Project File - Name="libsidplay" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 5.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=libsidplay - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "libsidplay.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "libsidplay.mak" CFG="libsidplay - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "libsidplay - Win32 Release" (based on\
 "Win32 (x86) Dynamic-Link Library")
!MESSAGE "libsidplay - Win32 Debug" (based on\
 "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "libsidplay - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MT /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /I "../../include" /I "../../include/sidplay" /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /o NUL /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /o NUL /win32
# ADD BASE RSC /l 0x809 /d "NDEBUG"
# ADD RSC /l 0x809 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /dll /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /dll /machine:I386 /out:"../../../binaries/Release/libsidplay2.dll"

!ELSEIF  "$(CFG)" == "libsidplay - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MTd /W3 /Gm /GX /Zi /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /Zi /Od /I "../../include" /I "../../include/sidplay" /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /o NUL /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /o NUL /win32
# ADD BASE RSC /l 0x809 /d "_DEBUG"
# ADD RSC /l 0x809 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /dll /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /dll /debug /machine:I386 /out:"../../../binaries/Debug/libsidplay2.dll" /pdbtype:sept

!ENDIF 

# Begin Target

# Name "libsidplay - Win32 Release"
# Name "libsidplay - Win32 Debug"
# Begin Source File

SOURCE=..\..\include\sidplay\Buffer.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\conf6510.h
# End Source File
# Begin Source File

SOURCE=..\..\include\config.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\envelope.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\envelope.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\extfilt.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\extfilt.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\filter.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\filter.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\IconInfo.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\InfoFile.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mixer.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\mos6510.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\mos6510.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\cycle_based\mos6510c.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\cycle_based\mos6510c.i
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6526\mos6526.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6526\mos6526.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos656x\mos656x.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos656x\mos656x.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\mos6581.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\MUS.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\opcodes.h
# End Source File
# Begin Source File

SOURCE=..\..\src\player.bin
# End Source File
# Begin Source File

SOURCE=..\..\src\player.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\player.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\pot.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\pot.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\PP20.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\PP20.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\PP20_Defs.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\PSID.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\sid.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\sid.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\cycle_based\sid6510c.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6510\cycle_based\sid6510c.i
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\sidconfig.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\siddefs.h
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\sidendian.h
# End Source File
# Begin Source File

SOURCE=..\..\include\sidenv.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidplay2.cpp
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\sidplay2.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\SidTune.cpp
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\SidTune.h
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\SidTuneCfg.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\SidTuneTools.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\sidtune\SidTuneTools.h
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\sidtypes.h
# End Source File
# Begin Source File

SOURCE=..\..\include\sidplay\SmartPtr.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\spline.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\version.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\voice.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\voice.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave.h
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave6581__ST.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave6581_P_T.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave6581_PS_.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave6581_PST.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave8580__ST.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave8580_P_T.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave8580_PS_.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\mos6581\resid\wave8580_PST.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\xsid\xsid.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\xsid\xsid.h
# End Source File
# End Target
# End Project
