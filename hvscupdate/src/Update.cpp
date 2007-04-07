//
// HVSC Update Tool
//
// Copyright (C) 1998,1999 HVSC administrators.

#include "compconf.h"

#if defined(HAVE_AMIGAOS)
#include <dos/dos.h>
#include <proto/dos.h>
#include <unistd.h>
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fstream.h>
#include <iostream.h>
#include <iomanip.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#if defined(HAVE_MSWINDOWS)
#include <dir.h>
#endif

#if defined(HAVE_LINUX) || defined(HAVE_UNIX) || defined(HAVE_BEOS)
#include <unistd.h>
#endif

#include "FileExists.h"
#include "TextFile.h"
#include "PathCreator.h"
#include "PathSplitter.h"
#include "fastperc.h"
#include "max.h"
#include "fformat.h"
#include "sidtune.h"
#include "hvscver.h"

static const char versionString[] = "2.7.1";
static const char HVSCcontactEmail[] = "Warren Pilkington <hvscadmin@wazzaw.freeuk.com>";

HVSCVER HVSCversion_found;

#if defined(HAVE_AMIGAOS)
static const bool ALLOW_PRE_UPDATE7 = true;
#else
static const bool ALLOW_PRE_UPDATE7 = false;
#endif
static const bool FINDPATH_DEBUG = false;

// Platform-specific.
#if defined(HAVE_LINUX) || defined(HAVE_UNIX) || defined(HAVE_BEOS)
static const char PARENT_DIR[] = "..";
static const char PARENT_DIR_PREFIX[] = "../";
#elif defined(HAVE_MSWINDOWS)
static const char PARENT_DIR[] = "..";
static const char PARENT_DIR_PREFIX[] = "..\\";
#elif defined(HAVE_MACOS)
static const char PARENT_DIR[] = "::";
static const char PARENT_DIR_PREFIX[] = "::";
#elif defined(HAVE_AMIGAOS)
static const char PARENT_DIR[] = "/";
static const char PARENT_DIR_PREFIX[] = "/";
#endif

#if defined(FS_IS_BACKSLASH) || defined(FS_IS_COLON_AND_BACKSLASH)
static const char DIR_SEPARATOR[] = "\\";
#elif defined(FS_IS_SLASH) || defined(FS_IS_COLON_AND_SLASH)
static const char DIR_SEPARATOR[] = "/";
#elif defined(FS_IS_COLON)
static const char DIR_SEPARATOR[] = ":";
#else
#error See config.h
#endif

// HVSC top-level directories. Case does not matter because it
// will be determined by the update tool.
static const char UPDATE_DIR[] = "update";
//
// HVSC 2.0-post (long file names)
static const char LONG_DOCUMENTS_DIR[] = "DOCUMENTS";
static const char LONG_HUBBARD_ROB_DIR[] = "Hubbard_Rob";
//
// HVSC 1.x (short file names, MS-DOS 8.3 style)
static const char SHORT_DOCUMENTS_DIR[] = "DOCUMNTS";
static const char SHORT_HUBBARD_ROB_DIR[] = "HUBBARD";

// The main HVSC doc file is read by the revision check code.
static const char HVSIDS_TXT[] = "hv_sids.txt";

static const char* DOCUMENTS_DIR = LONG_DOCUMENTS_DIR;
static const char* HUBBARD_ROB_DIR = LONG_HUBBARD_ROB_DIR;

static const int fileCopyBufLen = 16*1024;
static char* fileCopyBuffer;  // via new[]

static const int maxUpdateNum = 1000;   // Should be plenty...
static const int maxSidInfoLen = 31;  // not including terminator

// Leave TITLE, AUTHOR, COPYRIGHT in exact order as enumerated below.
static const char* keywords[] =
{
    "TITLE", "AUTHOR", "COPYRIGHT", "SPEED", "SONGS",
    "CREDITS", "DELETE", "MOVE", "REPLACE", "MKDIR", "FIXLOAD",
    "INITPLAY", "MUSPLAYER", "PLAYSID", "CLOCK", "SIDMODEL",
    "FREEPAGES", "FLAGS",
    NULL
};
enum mode_type
{
    TITLE=0, AUTHOR=1, COPYRIGHT=2, SPEED, SONGS,
    CREDITS, DELETEMODE, MOVE, REPLACE, MKDIRMODE, FIXLOAD,
    INITPLAY, MUSPLAYER, PLAYSID, CLOCK, SIDMODEL,
    FREEPAGES, FLAGS,
    NO_MODE
};
static mode_type mode = NO_MODE;

// We include it here because it depends on above mode_type.
#include "mysidtune.h"

static TextFile updateFile(0);

bool getHVSCpath(PathCreator& dest, const char* source);

bool fileCopy(ofstream& errorFile, int& errorCount, int line,
              const char* inFileName, const char* outFileName);

bool makeHVSCdir(ofstream& errorFile, int& errorCount, int line,
                 const char* hvscPath);

void logError(ofstream& errorFile,
              const char* lineContent, const char* errorMessage,
              int iLineNum, mode_type mode, int& errorCount);

bool myMkDir(const char* dirName);

bool isDir(const char* fileName);

// --------------------------------------------------------------------------
// OS-dependent application exit() wrapper.

inline void appExit(int returnValue)
{
    if (fileCopyBuffer != 0)
        delete[] fileCopyBuffer;

#if defined(HAVE_MSWINDOWS)
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    DWORD fdwMode, fdwOldMode;
    bool newMode = true;
    if (!GetConsoleMode(hStdin, &fdwOldMode))
        newMode = false;
    fdwMode = fdwOldMode & ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT);
    fdwMode |= ENABLE_PROCESSED_INPUT;
    if (!SetConsoleMode(hStdin, fdwMode))
        newMode = false;
    // If we could not change console mode, better leave immediately.
    if ( newMode )
    {
        cout << endl
             << "--- PRESS <RETURN> TO EXIT ---" << endl << flush;
        FlushConsoleInputBuffer(hStdin);
        DWORD count;
        char readBuf[2];
        ReadConsole(hStdin,&readBuf,1,&count,0);
        SetConsoleMode(hStdin, fdwOldMode);
    }
#endif
#if defined(HAVE_AMIGAOS)
    if(returnValue!=0)
	 {
		exit(RETURN_FAIL);
	 }
#endif
    exit(returnValue);
}

// --------------------------------------------------------------------------

int main(int, char* argv[])
{
#if defined(HAVE_MSWINDOWS)
    SetConsoleTitle("HVSC Update Tool");
#endif

    cout
        << endl
        << "HVSC Update Tool   Version " << versionString << endl
        << endl;

    if ((fileCopyBuffer = new char[fileCopyBufLen]) == 0)
    {
        cerr << "Error: Not enough memory." << endl;
        appExit(-1);  // or ENOMEM
    }

    struct HVSCversionInfo
    {
        HVSCVER required;
        HVSCVER resulting;
        HVSCVER found;
        bool havePrevScript;
        bool isHVSC1X;
        bool isUpdate10;
        bool printWarning;
    } HVSCversion;

    HVSCversion.required = MAKE_HVSCVER(0,0);
    HVSCversion.resulting = MAKE_HVSCVER(0,0);
    HVSCversion.found = MAKE_HVSCVER(0,0);
    HVSCversion.isHVSC1X = false;
    HVSCversion.isUpdate10 = false;
    HVSCversion.printWarning = false;

    HVSCversion_found = MAKE_HVSCVER(0,0);

    // Check whether user is in correct directory inside the HVSC tree.
    // Search for "update" directory.
    PathCreator updatePath;
    bool foundUpdateDir = getHVSCpath(updatePath,UPDATE_DIR);

    // Search for "DOCUMENTS" directory.
    PathCreator documentsPath;
    bool foundDocumentsDir = false;
    if (getHVSCpath(documentsPath,LONG_DOCUMENTS_DIR))
    {
        foundDocumentsDir = true;
        DOCUMENTS_DIR = LONG_DOCUMENTS_DIR;
        HUBBARD_ROB_DIR = LONG_HUBBARD_ROB_DIR;
    }
    else if (ALLOW_PRE_UPDATE7 &&
             getHVSCpath(documentsPath,SHORT_DOCUMENTS_DIR))
    {
        foundDocumentsDir = HVSCversion.isHVSC1X = true;
        DOCUMENTS_DIR = SHORT_DOCUMENTS_DIR;
        HUBBARD_ROB_DIR = SHORT_HUBBARD_ROB_DIR;
    }

    cout
        << "For this tool to work correctly, you must unzip/copy the desired update" << endl
        << "directly within the HVSC main directory (named ``C64music'' if you haven't" << endl
        << "renamed it).  Once you have done this your directory structure should have" << endl
#if defined(HAVE_MSWINDOWS)
        << "a similar path as the following:    C:\\C64music\\update\\" << endl
#elif defined(HAVE_MACOS)
        << "a similar path as the following:    C64music:update:" << endl
#elif defined(HAVE_AMIGAOS)
        << "a similar path as the following:    work:C64music/update/" << endl
#else
        << "a similar path as the following:    C64music/update/" << endl
#endif
        << "The device can vary as well as the name of the main HVSC directory." << endl
        << "In addition, the main HVSC directory doesn't have to be at the root of" << endl
        << "the selected device." << endl
        << endl
        << "To see exactly what this tool will do to the HVSC, you can view the file" << endl
        << "``UpdateXY.hvs'' (where XY stands for the update number).  That file is" << endl
        << "used for input, but it is written for you to view as well.  It contains" << endl
        << "explanations as to why certain tunes are being replaced, deleted, etc." << endl
        << "The file ``UpdateXY.hvs'' will reside in the ``";
    if (foundDocumentsDir)
        cout << documentsPath.get();
    else
        cout << DOCUMENTS_DIR;
    cout
        << "'' directory after" << endl
        << "the tool terminates successfully.  All files in the update directory will" << endl
        << "be removed except the executables.  We recommend that after you unzip the" << endl
        << "collection that you first listen to the tunes before running this tool." << endl
        << "Otherwise, you might have to spend a little more time searching for them" << endl
        << "within the massive collection (you can always re-unzip it later though)." << endl
        << endl
        << "Finally, you must run the tool from within the update directory." << endl
        << "(Don't forget to backup your HVSC!)  ";

    cout << "Do you wish to continue? (y/n) : " << flush;

    char cContinue;
    cin >> cContinue;
    // Exit if character is anything but 'y' or 'Y'.
    if (tolower(cContinue) != 'y')
        appExit(0);
    cout << endl;

    // ----------------------------------------------------------------------

    if (!foundDocumentsDir || !foundUpdateDir)
    {
        if (!foundDocumentsDir)
        {
            cerr << "Could not locate HVSC documents directory." << endl;
        }
        if (!foundUpdateDir)
        {
            cerr << "Could not locate HVSC update directory." << endl;
        }
        cerr << endl
            << "Error: "
            << fileNameWithoutPath(argv[0])
            << " must be run from within the HVSC "
            << UPDATE_DIR << " directory." << endl;
		appExit(-1);
    }

    // ----------------------------------------------------------------------
    // Read the first 10 lines of the main HVSC documentation file
    // and check for the line which contains the version number.

    PathCreator hvsidsFileName, hvsidsFileNameTmp;
    sprintf(hvsidsFileNameTmp.getWritable(),
            "%s/%s",DOCUMENTS_DIR,HVSIDS_TXT);
    if (getHVSCpath(hvsidsFileName,hvsidsFileNameTmp.get()))
    {
        TextFile hvsidsFile(hvsidsFileName.get());
        int line = 0;
        while (!hvsidsFile.endOfFile() && line<=10)
        {
            hvsidsFile.readNextLine();
            line++;
            if (hvsidsFile.isKey("release")) {
                HVSCversion.found = atohvscver(hvsidsFile.getCurParseBuf());
                HVSCversion_found = HVSCversion.found;
            }
        }
        hvsidsFile.close();
    }
    else
    {
        cerr
            << "Warning: HVSC documentation file ``"
            << HVSIDS_TXT << "'' not found." << endl
        << endl;
        HVSCversion.printWarning = true;
    }

    // ----------------------------------------------------------------------
    // Extra HVSC structure sanity check.
    // Search for Hubbard directory.

    PathCreator hubbardPath;
    if (!getHVSCpath(hubbardPath,HUBBARD_ROB_DIR))
    {
        cerr
            << "Rob Hubbard directory not detected!" << endl
            << "Perhaps you did not follow the directions stated at the start of this "
            << "program?" << endl
            << "The update directory must reside directly in the HVSC directory." << endl
            << "Errors will likely occur if you continue (unless you deleted the ``"
            << HUBBARD_ROB_DIR << "'' directory and this update does not depend on it)." << endl
            << "Do you wish to continue? (y/n) : " << flush;
        cin >> cContinue;
        cout << endl;
        // Exit if character is anything but 'y' or 'Y'.
        if (tolower(cContinue) != 'y')
            appExit(0);
    }

    // ----------------------------------------------------------------------

    PathCreator errorsFileName;
    PathCreator updateFileName, updateFileNameCat;

    int updateNum;
    // Search for the update script.
    for (updateNum=1; updateNum <= maxUpdateNum; updateNum++)
    {
        // Update data file is named "UpdateXY.hvs".
        // Work with HVSC-style path names.
        if ( updateNum <= 99 ) {
		    sprintf(updateFileNameCat.getWritable(),"%s/Update%02d.hvs",UPDATE_DIR,updateNum);
		    sprintf(errorsFileName.getWritable(),"Errors%02d.hvs",updateNum);
		}
        else {
            sprintf(updateFileNameCat.getWritable(),"%s/Update%d.hvs",UPDATE_DIR,updateNum);
            sprintf(errorsFileName.getWritable(),"Errors%d.hvs",updateNum);
        }

		if (getHVSCpath(updateFileName,updateFileNameCat.get()))
        {
			break;  // the HVS file has been located
        }
		else if (updateNum == maxUpdateNum)
        {
			cerr
                << "Error: No HVSC update script file ``UpdateXY.hvs'' found." << endl;
            appExit(-1);
        }
    }

    // Count number of lines in file.
    updateFile.open(updateFileName.get());
    int lines = 0;
    while (!updateFile.endOfFile())  // line-by-line loop
    {
		updateFile.readNextLine();
        lines++;
        if (lines <= 10)
        {
            if (updateFile.isKey("#PreviousVersion:"))
                HVSCversion.required = atohvscver(updateFile.getCurParseBuf());
            else if (updateFile.isKey("#ResultingVersion:"))
                HVSCversion.resulting = atohvscver(updateFile.getCurParseBuf());
        }
    }
    updateFile.close();

    if (lines == 0)
    {
        cerr
            << "HVSC update script file ``" << updateFileName.get()
                << "'' is empty." << endl;
        appExit(-1);
    }

    if (updateNum > 1)
    {
        // Create the file name of the previous update script,
        // which should be in the DOCUMENTS directory.
        PathCreator prevUpdateFileName, prevUpdateFileNameCat;
        sprintf(prevUpdateFileNameCat.getWritable(),"%s/Update%02d.hvs",
                DOCUMENTS_DIR,updateNum-1);
        HVSCversion.havePrevScript = getHVSCpath(prevUpdateFileName,
                                                 prevUpdateFileNameCat.get());
    }
    else
    {
        HVSCversion.havePrevScript = true;  // Update #1 is first update
    }

    // Version check can be done since Update #7 (HVSC 2.0 => 2.1).
    if (hvscvercmp(HVSCversion.required,MAKE_HVSCVER(0,0))>0 &&
        hvscvercmp(HVSCversion.resulting,MAKE_HVSCVER(0,0))>0)
    {
        char ver[ 10 ];

        hvscvertoa(ver,HVSCversion.required);
        cout.setf(ios::fixed);
        cout << "This update requires HVSC version: "
            << ver << endl;
        cout << "The update tool has found version: ";
        if (hvscvercmp(HVSCversion.found,MAKE_HVSCVER(0,0))>0)
        {
            hvscvertoa(ver,HVSCversion.found);
            cout << ver;
        }
        else
        {
            cout << "CHECK FAILED";
        }
        cout << endl << endl;
        if (hvscvercmp(HVSCversion.found,HVSCversion.required)!=0)
        {
            HVSCversion.printWarning = true;
        }
    }

    if (!HVSCversion.havePrevScript)
    {
        cerr << "Could not find the script from Update #"
            << dec << setw(2) << setfill('0') << updateNum-1
            << " (previous update) in the " << endl
            << DOCUMENTS_DIR << " directory. You are about to "
            << "update an incomplete HVSC." << endl << endl;
        HVSCversion.printWarning = true;
    }

    if (HVSCversion.printWarning)
    {
        cout
            << "***************" << endl
            << "*** WARNING ***" << endl
            << "***************" << endl
            << endl
            << "Errors will likely occur if you continue." << endl
            << "Do you wish to continue? (y/n) : " << flush;
        cin >> cContinue;
        cout << endl;
        // Exit if character is anything but 'y' or 'Y'.
        if (tolower(cContinue) != 'y')
            appExit(0);
    }

    // If Update #10 is used, enable special compatibility mode.
    // Allow missing parameter lines in CREDITS command.
    if (hvscvercmp(HVSCversion.required,MAKE_HVSCVER(2,3))==0 &&
        hvscvercmp(HVSCversion.resulting,MAKE_HVSCVER(2,4))==0)
    {
        HVSCversion.isUpdate10 = true;
    }

    // This time we take care of file access errors.
    if (!updateFile.open(updateFileName.get()))
    {
        cerr << endl
            << "Error: Could not open ``" << updateFileName.get()
                << "''." << endl;
        appExit(-1);
    }

	ofstream errorFile(errorsFileName.get());

    fastPercent updateProgress(lines+1);
    cout << "Working : ";
    updateProgress.cout();

    // ----------------------------------------------------------------------

    int skipLinesAfterError = 0;
    int errorCount = 0;
    while (!updateFile.endOfFile())  // line-by-line loop
    {
        updateFile.readNextLine();
        updateProgress.update(updateFile.getLineNum());
        if (updateProgress.changed())
            updateProgress.coutUpdate();
        // Skip blank and comment lines.
        while (!updateFile.endOfFile() && updateFile.isBlank()
               || updateFile.isComment())
        {
            updateFile.readNextLine();
        };
        if (updateFile.isBlank())
            break;

        int x = -1;
        // Find the Mode.  Compare current line to all of the keywords
        // until found or end of keywords.
        while (NULL != keywords[++x])
        {
            if (!strcmp(keywords[x],updateFile.getParseBuf()))
            {
                mode = (mode_type) x;	// keyword found; set corresponding mode.
                break;
            }
        }
        // Not equal to NULL if keyword found;
        // If keyword found, get next line in file.
        if (NULL != keywords[x])
            continue;

        int line = updateFile.getLineNum();

// --------------------------------------------------------------------------

        // Based on the mode, take the proper action.
        switch (mode)
        {
         case     TITLE:
         case    AUTHOR:
         case COPYRIGHT:
         case   CREDITS:
         case     SPEED:
         case     SONGS:
         case   FIXLOAD:
         case  INITPLAY:
         case MUSPLAYER:
         case   PLAYSID:
         case     CLOCK:
         case  SIDMODEL:
         case FREEPAGES:
         case     FLAGS:
            {
                PathCreator tmpSource;
                if (!getHVSCpath(tmpSource,updateFile.getLineBuf()))
                {
                    logError(errorFile,updateFile.getLineBuf(),
                             "File not found or permission denied.",
                             line,mode,errorCount);
                    if (mode == CREDITS)
                        skipLinesAfterError = 3;
                    else if (mode == FLAGS)
                        skipLinesAfterError = 4;
                    else if (mode == FIXLOAD)
                        skipLinesAfterError = 0;
                    else
                        skipLinesAfterError = 1;
                }
                else
                {
                    char sidInfo[4][maxSidInfoLen+1];
                    sidInfo[0][0] = 0;
                    sidInfo[1][0] = 0;
                    sidInfo[2][0] = 0;
                    sidInfo[3][0] = 0;

                    // strcpy copies terminator.
                    // strncpy copies terminator if inside size n.
                    // strlen does not count terminator.

                    if (CREDITS == mode)
                    {
                        for (int n=0; n<3; n++)
                        {
                            updateFile.readNextLine();
                            if (!HVSCversion.isUpdate10)
                            {
                                if (updateFile.isBlank())
                                    logError(errorFile,tmpSource.get(),
                                             "Premature end of update script?",
                                             updateFile.getLineNum(),mode,errorCount);
                            }
                            if (updateFile.getLineLen() > maxSidInfoLen)
                                logError(errorFile,tmpSource.get(),
                                         "SID credit string too long.",
                                         updateFile.getLineNum(),mode,errorCount);
                            strncpy(sidInfo[n],updateFile.getLineBuf(),maxSidInfoLen+1);
                        }
                    }
                    else if (FLAGS == mode)
                    {
                        for (int n=0; n<4; n++)
                        {
                            updateFile.readNextLine();
                            if (!HVSCversion.isUpdate10)
                            {
                                if (updateFile.isBlank())
                                    logError(errorFile,tmpSource.get(),
                                             "Premature end of update script?",
                                             updateFile.getLineNum(),mode,errorCount);
                            }
                            strncpy(sidInfo[n],updateFile.getLineBuf(),maxSidInfoLen+1);
                        }
                    }
                    else if ((AUTHOR == mode) || (TITLE == mode) || (COPYRIGHT==mode))
                    {
                        updateFile.readNextLine();
                        if (updateFile.isBlank())
                            logError(errorFile,tmpSource.get(),
                                     "Premature end of update script?",
                                     updateFile.getLineNum(),mode,errorCount);
                        if (updateFile.getLineLen() > maxSidInfoLen)
                            logError(errorFile,tmpSource.get(),
                                     "SID credit string too long.",
                                     updateFile.getLineNum(),mode,errorCount);
                        strncpy(sidInfo[mode],updateFile.getLineBuf(),maxSidInfoLen+1);
                    }
                    else if (FIXLOAD == mode)
                    {
                        ;
                    }

                    // SPEED, SONGS, INITPLAY, MUSPLAYER, PLAYSID, CLOCK,
                    // SIDMODEL, FREEPAGES
                    else
                    {
                        updateFile.readNextLine();
                        if (updateFile.isBlank())
                            logError(errorFile,tmpSource.get(),
                                     "Premature end of update script?",
                                     updateFile.getLineNum(),mode,errorCount);
                        strncpy(sidInfo[0],updateFile.getLineBuf(),maxSidInfoLen+1);
                    }

                    mySidTune sidFile(tmpSource.get());
                    if (!sidFile)
                    {
                        logError(errorFile,tmpSource.get(),
                                 "Could not load sidtune file.",
                                 line,mode,errorCount);
                    }
                    if (sidFile.writeToSidTune(sidInfo,mode) == true)
                    {
                        if (!sidFile.savePSIDfile(tmpSource.get(),true))  // overwrite!
                        {
                            logError(errorFile,tmpSource.get(),
                                     "Could not save sidtune file.",
                                     line,mode,errorCount);
                        }
                    }
                    else
                    {
                        logError(errorFile,tmpSource.get(),
                                 "Problem writing new sidtune info.",
                                 line,mode,errorCount);
                    }
                }  // else
                break;
            }

// ------------------------------------------------------------------- DELETE

            // Delete files and directories. Directories being deleted must
            // not have files within it (this might be changed later).

         case DELETEMODE:
            {
                PathCreator dest;
                if (!getHVSCpath(dest,updateFile.getLineBuf()))
                {
                    logError(errorFile,updateFile.getLineBuf(),
                             "File or directory not found.",
                             line,mode,errorCount);
                }
                else
                {
                    if (isDir(dest.get()))
                    {
                        if (rmdir(dest.get()) != 0)
                        {
                            logError(errorFile,dest.get(),
                                     "Could not delete directory.",
                                     line,mode,errorCount);
                        }
                    }
                    else
                    {
                        if (remove(dest.get()) != 0)
                        {
                            logError(errorFile,dest.get(),
                                     "Could not delete file.",
                                     line,mode,errorCount);
                        }
                    }
                }
                break;
            }

// --------------------------------------------------------------------- MOVE

         case MOVE:
         case REPLACE:
            {
                const char* curLine = updateFile.getLineBuf();

                if (ALLOW_PRE_UPDATE7)
                {
                    // Here drop any wildcards that were used.
                    char* wildcardDropped;
                    if (HVSCversion.isHVSC1X)
                    {
                        wildcardDropped = new char[strlen(curLine)+1];
                        strcpy(wildcardDropped,curLine);
                        char* tmp = strstr(wildcardDropped,"*.dat");
                        if (tmp != NULL)
                            *tmp = 0;
                        tmp = strstr(wildcardDropped,"*.DAT");
                        if (tmp != NULL)
                            *tmp = 0;
                        tmp = strstr(wildcardDropped,"*.txt");
                        if (tmp != NULL)
                            *tmp = 0;
                        curLine = wildcardDropped;
                    }
                }

                // The path we fetch the source file name from
                // if it is not a directory.
                PathSplitter sourceSplitter(curLine);

                // Check whether source (dir or file) exists.
                PathCreator source;
                if (!getHVSCpath(source,curLine))
                {
                    logError(errorFile,curLine,
                             "(source error) No such path or permission denied.",
                             line,mode,errorCount);
                    // Escape from error further below, ``source'' is empty.
                    // First read next line of Update script to not get
                    // out of sync.
                }

                // Read destination directory/file from next line.
                updateFile.readNextLine();
                int destLine = updateFile.getLineNum();
                if (updateFile.isBlank())
                {
                    logError(errorFile,updateFile.getLineBuf(),
                             "Premature end of update Script?",
                             destLine,mode,errorCount);
                    break;
                }

                // Here escape from source error.
                if (source.isEmpty())
                    break;

                // Check whether destination (dir or file) exists.
                PathCreator dest;
                if (!getHVSCpath(dest,updateFile.getLineBuf()))
                {
                    // Destination path does not exist.

                    // If it should be a directory, create it.
                    int len = updateFile.getLineLen();
                    char lastChar = updateFile.getLineBuf()[len-1];
                    if (lastChar=='/' || lastChar=='\\')
                    {
                        makeHVSCdir(errorFile,errorCount,destLine,updateFile.getLineBuf());
                        // Now this should not return an error.
                        if (!getHVSCpath(dest,updateFile.getLineBuf()))
                        {
                            logError(errorFile,updateFile.getLineBuf(),
                                     "Creation of directory failed.",
                                     destLine,mode,errorCount);
                            break;
                        }
                    }
                    else
                    {
                        // Last name in path does not end with a slash.
                        // Destination file to be created is specified.
                        // We leave dest.isEmpty().
                    }
                }

                // Now dest exists, only if it is an existent directory
                // or file. And it is platform-specific as well.

                if (isDir(source.get()))
                {
                    // Destination has to be directory as well.
                    if (!isDir(dest.get()) || dest.isEmpty())
                    {
                        logError(errorFile,dest.get(),
                                 "Destination is not a directory.",
                                 destLine,mode,errorCount);
                        break;
                    }

                    dirent* entry;
                    DIR* directory;

                    if ((directory = opendir(source.get())) == NULL)
                    {
                        logError(errorFile,source.get(),
                                 "(source error) No such path or permission denied.",
                                 line,mode,errorCount);
                        break;
                    }

                    while ((entry = readdir(directory)) != NULL)
                    {
                        PathCreator sourceFile(source.get());
                        sourceFile.append(DIR_SEPARATOR);
                        sourceFile.append(entry->d_name);

                        if (!isDir(sourceFile.get()))  // omit directories
                        {
                            // REPLACE tries to delete old version of file.
                            // If old file is not there, we don`t report an error because
                            // we want to get rid of it anyway.
                            if (REPLACE == mode)
                            {
                                // REPLACE is case-insensitive as well.
                                // Hence we seek the source file in dest dir.
                                // Create HVSC-style path.
                                PathSplitter destLine(updateFile.getLineBuf());
                                PathCreator tmp(destLine.getFile());
                                while (destLine.nextFile())
                                {
                                    tmp.append("/");
                                    tmp.append(destLine.getFile());
                                }
                                // And add destination file name.
                                tmp.append("/");
                                tmp.append(entry->d_name);
                                // Determine platform-specific path
                                // and delete file if available.
                                PathCreator removeDest;
                                if (getHVSCpath(removeDest,tmp.get()))
                                    remove(removeDest.get());
                            }

                            // Create platform-specific destination path name
                            // from dest dir and source file name.
                            PathCreator destFile(dest.get());
                            destFile.append(DIR_SEPARATOR);
                            destFile.append(entry->d_name);

                            if (fileCopy(errorFile,errorCount,line,
                                         sourceFile.get(),destFile.get()))
                            {
                                if (remove(sourceFile.get()) != 0)
                                    logError(errorFile,sourceFile.get(),
                                             "Could not remove source file.",
                                             line,mode,errorCount);
                            }

                        }  // nodir
                    } // while direntry

                    closedir(directory);

                }
                else  // source is single file
                {     // source has platform-specific path

                    // Here, ``dest'' is empty if it is a file to be created.
                    // Else, it is not empty and contains a platform-specific
                    // path to a file.

                    // Anyway...
                    // We need to seek the source file name in the
                    // destination directory to be able to REPLACE
                    // a file case-insensitively or trigger an error
                    // (MOVE mode only).

                    // Destination path to be created.
                    PathCreator newDestFile;
                    // The file to be overwritten in REPLACE mode.
                    PathCreator oldDestFile;

                    // Skip to source file name in source path.
                    sourceSplitter.lastFile();

                    // Create HVSC-style destination path.
                    if (!dest.isEmpty() && isDir(dest.get()))
                    {
                        // Create name of possibly existing dest.file.
                        PathCreator tmp(updateFile.getLineBuf());
                        // Assumes slash is last char in line.
                        tmp.append(sourceSplitter.getFile());
                        // Determine platform-specific path.
                        getHVSCpath(oldDestFile,tmp.get());

                        // Dest.file is dest dir plus source file name.
                        newDestFile.append(dest.get());
                        // Append source file name.
                        newDestFile.append(DIR_SEPARATOR);
                        newDestFile.append(sourceSplitter.getFile());
                    }
                    else
                    {
                        // Although complete destination file name is given
                        // in the update script, we need to determine the
                        // case-insensitive path name in front of the file
                        // name and the name of a possibly existing
                        // destination file of different case.

                        // Create HVSC-style path without the file name.
                        PathSplitter tmp(updateFile.getLineBuf());
                        PathCreator destCheck(tmp.getFile());
                        while (tmp.nextFile() && !tmp.isLastFile())
                        {
                            destCheck.append("/");
                            destCheck.append(tmp.getFile());
                        }
                        // Check whether path exists.
                        if (!getHVSCpath(dest,destCheck.get()))
                        {
                            logError(errorFile,destCheck.get(),
                                     "(destination error) No such path or permission denied.",
                                     destLine,mode,errorCount);
                            break;
                        }

                        // Skip to dest file name in dest path.
                        tmp.lastFile();

                        // Now see whether case-insensitive destination
                        // file exists already?
                        destCheck.append("/");
                        destCheck.append(tmp.getFile());
                        getHVSCpath(oldDestFile,destCheck.get());
                        // oldDestFile.isEmpty() = true, if no such file

                        // Complete new platform-specific path name.
                        newDestFile.append(dest.get());
                        // Append dest file name.
                        newDestFile.append(DIR_SEPARATOR);
                        newDestFile.append(tmp.getFile());
                    }

                    // REPLACE tries to delete old version of file.
                    // If old file is not there, we don`t report an error because
                    // we want to get rid of it anyway.
                    if (REPLACE==mode && !oldDestFile.isEmpty())
                    {
                        remove(oldDestFile.get());
                    }

                    if (fileCopy(errorFile,errorCount,line,
                                 source.get(),newDestFile.get()))
                    {
                        if (remove(source.get()) != 0)
                            logError(errorFile,source.get(),
                                     "Could not remove source file.",
                                     line,mode,errorCount);
                    }
                }  // single file
                break;
            }

// -------------------------------------------------------------------- MKDIR

         // Obsolete, but deprecated.

         case MKDIRMODE:
            {
                makeHVSCdir(errorFile,errorCount,line,updateFile.getLineBuf());
                break;
            }

// ------------------------------------------------------------------ UNKNOWN

         // NO_MODE is active when no keywords have been read yet.

         case NO_MODE:
         default:
            {
                logError(errorFile,updateFile.getLineBuf(),
                         "Keyword/parameter mismatch?",
                         line,mode,errorCount);
                break;
            }

        }  // switch

        while (skipLinesAfterError-- > 0)
        {
            updateFile.readNextLine();
        };
        skipLinesAfterError = 0;

    };  // while !endOfFile

    // ----------------------------------------------------------------------

    updateProgress.end();  // 100%
    cout << endl
        << endl;

    updateFile.close();

	// Clean up.
	if (errorCount == 0)
    {
		// Move HVS file.
        PathSplitter docSrc(updateFileNameCat.get());
        docSrc.lastFile();
        PathCreator docDest(documentsPath.get());
        docDest.append(DIR_SEPARATOR);
        docDest.append(docSrc.getFile());
        fileCopy(errorFile,errorCount,0,
                 updateFileName.get(),docDest.get());
    }

    errorFile.close();

    if (errorCount == 0)
    {
        remove(updateFileName.get());
        remove(errorsFileName.get());

        cout
            << "Update was successful! Update directory is no longer needed." << endl
            << endl
            << "P.S. - Check the update file in the ``" << documentsPath.get()
            << "'' directory for" << endl
            << "answers as to why certain tunes were added (if concerned)." << endl;
        appExit(0);
    }
	else
    {
        cerr
            << errorCount << " errors encountered." << endl
            << "Please read ``" << errorsFileName.get()
            << "'', and ``Readme.1st'' in the update archive." << endl
            << "Email " << HVSCcontactEmail
            << " if you encounter any unsolvable problems." << endl;
        appExit(-1);
    }
}

// --------------------------------------------------------------------------

// Function searches HVSC tree for the specified directory or file passed
// in "path" without regard to upper or lower case, starting in parent
// directory. The correct case of the file or directory is determined and
// created in "outPath". If the path could not be found, false is returned
// and "outPath" is empty.
//
// - Ignores leading slash.
// - Starts searching in parent directory.
// - Does not work if source path contains platform-specific ".."
//   directories.
//
// Input: HVSC-style path (.., / or \)
// Output: platform-specific path

bool getHVSCpath(PathCreator& outPath, const char *path)
{
    outPath.empty();
    int pathLen = strlen(path);
    if (pathLen == 0)
        return false;

    if (FINDPATH_DEBUG)
        cout << "getHVSCpath(), path = " << path << endl;

    // We use this to analyze the path name.
    PathSplitter myPathSplitter(path);

#if defined(HAVE_AMIGAOS)
	/* AmigaOS specific code required to handle truncation of file
	names longer than 31 characters by the standard file systems.
	Lock() will ignore case, unless a third party file system is
	installed that allows case sensitive names to be enabled. */
	BPTR lock;
	bool found;

	do
	{
		outPath.append( "/" );
		outPath.append( myPathSplitter.getFile() );
	}
	while( myPathSplitter.nextFile() );

	if( FINDPATH_DEBUG )
	{
		cout << "outPath = " << outPath.get() << endl;
	}

	lock = Lock( (char *)outPath.get(), ACCESS_READ );
	if( lock != 0 )
	{
		UnLock( lock );
		found = true;
	}
	else
	{
		outPath.empty();
		found = false;
	}

	if( FINDPATH_DEBUG )
	{
		cout << "found = " << found << endl;
	}

	return found;
#else
    // Get to HVSC root directory.
    outPath.append(PARENT_DIR);

#if defined(HAVE_MACOS)
    bool parentHasBeenLast = true;
#endif

    if (FINDPATH_DEBUG)
        cout << "outPath = " << outPath.get() << endl;

    // Check existence of each directory in path.
    do
    {
        dirent* entry;
        DIR* directory;

        // Check availability of current directory/path.
        if ((directory = opendir(outPath.get())) == NULL)
        {
            // No such directory.
            outPath.empty();
            return false;
        }

        if (FINDPATH_DEBUG)
            cout << "nextDirToFind = " << myPathSplitter.getFile() << endl;

        // Dir exists. Now find next directory name to be opened,
        // and retrieve its name in correct case.
        bool found = false;
        while ((entry = readdir(directory)) != NULL)
        {
            if (stricmp(entry->d_name,myPathSplitter.getFile())==0)
            {
                // Append next directory name to temporary path.
#if defined(HAVE_MACOS)
                if (!parentHasBeenLast)
                    outPath.append(DIR_SEPARATOR);
                else
                    parentHasBeenLast = false;
#else
                outPath.append(DIR_SEPARATOR);
#endif
                outPath.append(entry->d_name);
                if (FINDPATH_DEBUG)
                    cout << "outPath = " << outPath.get() << endl;
                found = true;
                break;
            }
        }

        if (FINDPATH_DEBUG)
            cout << "found = " << found << endl;

        closedir(directory);
        if (!found)
        {
            outPath.empty();
            return false;
        }
    }
    while (myPathSplitter.nextFile());

    return true;
#endif
}

// --------------------------------------------------------------------------

bool makeHVSCdir(ofstream& errorFile, int& errorCount, int line,
                 const char* hvscPath)
{
    // Summary:
    //
    // If path already exists, ignore MKDIR request.
    // If path points to a file, log appropriate error.
    //
    // If path does not exist, create last directory in path
    // if this ought to be a directory.

    PathCreator tmpDest;
    // Check whether path exists.
    if (!getHVSCpath(tmpDest,hvscPath))  // path not found?
    {
        // We use this to analyze the input path.
        PathSplitter myPathSplitter(hvscPath);

        if (!myPathSplitter.isGood())
        {
            logError(errorFile,hvscPath,"Invalid argument.",
                     line,mode,errorCount);
            return false;
        }

        // The path we will construct.
        PathCreator dest;

        // Construct HVSC-style path.
        // dest = system-specific path, tmpDest = HVSC-style path

        // Special case: top-level directory only. Parent does exist
        // (= HVSC root), so we don`t need to check it.
        if (myPathSplitter.isLastFile())
        {
            // Not HVSC-style!
            dest.append(PARENT_DIR_PREFIX);
            dest.append(myPathSplitter.getFile());
        }
        else
        {
            tmpDest.empty();  // redundant
            tmpDest.append(myPathSplitter.getFile());
            // Omit last file/dir, so we can check the rest of the path.
            while (myPathSplitter.nextFile() && !myPathSplitter.isLastFile())
            {
                tmpDest.append("/");  // HVSC-style!
                tmpDest.append(myPathSplitter.getFile());
            }

            // Check whether contructed path exists.
            if (!getHVSCpath(dest,tmpDest.get()))  // path not found?
            {
                logError(errorFile,tmpDest.get(),"Path does not exist.",
                         line,mode,errorCount);
                return false;
            }

            // Append directory to create.
            dest.append(DIR_SEPARATOR);
            myPathSplitter.lastFile();
            dest.append(myPathSplitter.getFile());
        }

        bool success = myMkDir(dest.get());
        if (!success)
        {
            logError(errorFile,dest.get(),"Could not create directory.",
                     line,mode,errorCount);
        }
        return success;
    }
    else  // path found
    {
        bool success = isDir(tmpDest.get());
        if (!success)
        {
            logError(errorFile,tmpDest.get(),
                     "Trying to create directory on top of file.",
                     line,mode,errorCount);
        }
        return success;
    }
}

// --------------------------------------------------------------------------

bool fileCopy(ofstream& errorFile, int& errorCount, int line,
              const char* inFileName, const char* outFileName)
{
	// Open binary input file stream at end of file.
#if defined(HAVE_IOS_BIN)
	ifstream fIn(inFileName,ios::in|ios::bin);
#else
	ifstream fIn(inFileName,ios::in|ios::binary);
#endif
    fIn.seekg(0,ios::end);  // explicit ios::ate
	// As a replacement for !is_open(), bad() and the NOT-operator don`t seem
	// to work on all systems.
#if defined(DONT_HAVE_IS_OPEN)
    if ( !fIn )
#else
	if ( !fIn.is_open() || !fileExists(inFileName) )
#endif
	{
        logError(errorFile,inFileName,"Could not open source file.",
                 line,mode,errorCount);
        return false;
	}

    if ( fileExists(outFileName) )
    {
        logError(errorFile,outFileName,"Destination file already exists.",
                 line,mode,errorCount);
        return false;
    }
    // Open binary input file stream.
#if defined(HAVE_IOS_BIN)
    ofstream fOut(outFileName,ios::out|ios::bin);
#else
    ofstream fOut(outFileName,ios::out|ios::binary);
#endif
    // As a replacement for !is_open(), bad() and the NOT-operator don`t seem
    // to work on all systems.
#if defined(DONT_HAVE_IS_OPEN)
    if ( !fOut )
#else
    if ( !fOut.is_open() )
#endif
    {
        logError(errorFile,outFileName,"Could not open destination file.",
                 line,mode,errorCount);
        return false;
    }

    int fileLen;
#if defined(HAVE_SEEKG_OFFSET)
    fileLen = (fIn.seekg(0,ios::end)).offset();
#else
    fIn.seekg(0, ios::end);
    fileLen = (int)fIn.tellg();
#endif

    fIn.seekg(0,ios::beg);
    int restFileLen = fileLen;
    while (restFileLen > fileCopyBufLen)
    {
        fIn.read(fileCopyBuffer,fileCopyBufLen);
        restFileLen -= fileCopyBufLen;
        fOut.write(fileCopyBuffer,fileCopyBufLen);
    }
    if (restFileLen > 0)
    {
        fIn.read(fileCopyBuffer,restFileLen);
        fOut.write(fileCopyBuffer,restFileLen);
    }

    if (fIn.bad() || fOut.bad() || fOut.fail())
    {
        logError(errorFile,outFileName,"Stream error during file copy.",
                 line,mode,errorCount);
        fIn.close();
        fOut.close();
        return false;
    }

    fIn.close();
    fOut.close();
    return true;
}

// --------------------------------------------------------------------------

bool myMkDir(const char* dirName)
{
#if defined(HAVE_LINUX) || defined(HAVE_UNIX) || defined(BEOS) || defined(HAVE_AMIGAOS)
    return (mkdir(dirName,0777) == 0);
#else
    return (mkdir(dirName) == 0);
#endif
}

// Try to open a file using opendir(). If successful, it is a directory.
bool isDir(const char* fileName)
{
    if (fileName == 0)
        return false;
	DIR *directory = opendir(fileName);
    bool openFailed = (directory==NULL);
	if (!openFailed)
		closedir(directory);
    return !openFailed;
}

// --------------------------------------------------------------------------

void logError(ofstream& errorFile,
              const char* lineContent,
              const char* errorMessage,
              int iLineNum, mode_type mode, int& errorCount)
{
    errorFile << "Line " << iLineNum << ", ";

	switch (mode)
	{
     case DELETEMODE:
        errorFile << "DELETE: ";
        break;
     case MOVE:
        errorFile << "MOVE: ";
        break;
     case REPLACE:
        errorFile << "REPLACE: ";
        break;
     case MKDIRMODE:
        errorFile << "MKDIR: ";
        break;
     case AUTHOR:
        errorFile << "AUTHOR: ";
        break;
     case TITLE:
        errorFile << "TITLE: ";
        break;
     case COPYRIGHT:
        errorFile << "COPYRIGHT: ";
        break;
     case CREDITS:
        errorFile << "CREDITS: ";
        break;
     case FIXLOAD:
        errorFile << "FIXLOAD: ";
        break;
     case INITPLAY:
        errorFile << "INITPLAY: ";
        break;
     case MUSPLAYER:
        errorFile << "MUSPLAYER: ";
        break;
     case PLAYSID:
        errorFile << "PLAYSID: ";
        break;
     case CLOCK:
        errorFile << "CLOCK: ";
        break;
     case SIDMODEL:
        errorFile << "SIDMODEL: ";
        break;
     case FREEPAGES:
        errorFile << "FREEPAGES: ";
        break;
     case FLAGS:
        errorFile << "FLAGS: ";
        break;
      default:
        errorFile << "UNKNOWN: ";
        break;
    }

    errorFile << errorMessage << endl
        << "    " << lineContent << endl;

    errorCount++;
}
