#ifndef mysidtune_h
#define mysidtune_h


class mySidTune : public sidTune
{
 public:  // --------------------------------------------------------- public

    // Only derive the simple constructor.
    mySidTune(const char* fileName) : sidTune(fileName)
    {
    };

    bool writeToSidTune(char newInfoString[][maxSidInfoLen+1], mode_type mode)
    {
        // PSID-format can only handle up to 31 characters plus a terminating zero.
        //
        // The infoStrings are kept without any special order in a two-dimensional
        // array of the size [infoStringNum][infoStringLen].
        //
        // To make available publically readable copies for each infoString,
        // extra pointers are assigned: info.nameString, info.authorString,
        // info.copyrightString, and for compatibility to MUS files and a future
        // file format: info.infoString[0], ..., info.infoString[infoStringNum].
        // A copy of the private instance of the sidTuneInfo structure can be read
        // out using ::returnInfo().

        int infoStringIndex = 0; // Used for FLAGS case.

        switch(mode)
        {
         case TITLE:
         case AUTHOR:
         case RELEASED:
            {
                // Copy string to private array.
                strcpy(&infoString[(int) mode][0], newInfoString[(int) mode]);

                info.infoString[(int) mode] = &infoString[(int) mode][0];

                if (TITLE == mode)
                    // Assign pointer: First infoString usually is NAME.
                    info.nameString = &infoString[(int) mode][0];
                else if (AUTHOR == mode)
                    // Assign pointer: Second infoString usually is AUTHOR.
                    info.authorString = &infoString[(int) mode][0];
                else
                    // Assign pointer: Third infoString usually is RELEASED
                    // (original release information).
                    info.copyrightString = &infoString[(int) mode][0];
                break;
            }

         case CREDITS:
            {
                if ('*' != newInfoString[0][0])
                {
                    strcpy(&infoString[0][0], newInfoString[0]);
                    info.nameString = &infoString[0][0];
                    info.infoString[0] = &infoString[0][0];
                }
                if ('*' != newInfoString[1][0])
                {
                    strcpy(&infoString[1][0], newInfoString[1]);
                    info.authorString = &infoString[1][0];
                    info.infoString[1] = &infoString[1][0];
                }
                if ('*' != newInfoString[2][0])
                {
                    strcpy(&infoString[2][0], newInfoString[2]);
                    info.copyrightString = &infoString[2][0];
                    info.infoString[2] = &infoString[2][0];
                }
                break;
            }

         case SPEED:
            {
                unsigned long ulSpeed;

                // Only 32 song speed can be set with the below code assuming
                // an unsigned long stores 32 bits.
                if (sizeof(ulSpeed) >= 4)
                {
                    // Not modifiable!
                    if (info.compatibility == SIDTUNE_COMPATIBILITY_R64)
                        return false;
                    // SPEED string is in hex.
                    ulSpeed = strtoul(newInfoString[0], NULL, 16);
                    convertOldStyleSpeedToTables((udword) ulSpeed);
                    break;
                }
                else
                    return false;
            }

         case SONGS:  //SONGS string must have a comma!
            {
                size_t string_len = strlen(newInfoString[0]);

                // Terminate string at the comma
                // example, with SONGS=3,2 the string is "3"
                size_t index;
                for (index = 0; newInfoString[0][index]; index++)
                {
                    if (',' == newInfoString[0][index])
                    {
                        newInfoString[0][index] = '\0';
                        break;
                    }
                }

                // If a comma found, continue...else fail
                if (string_len >= index-1)
                {
                    info.songs = atoi(newInfoString[0]);
                    info.startSong = atoi(newInfoString[0]+index+1);
                    break;
                }
                else
                    return false;
            }

         case INITPLAY:  // INITPLAY string must have a comma!
            {
                size_t string_len = strlen(newInfoString[0]);

                // Terminate string at the comma
                // example, with INITPLAY=1000,1003 the string is "1000"
                size_t index;
                for (index = 0; newInfoString[0][index]; index++)
                {
                    if (',' == newInfoString[0][index])
                    {
                        newInfoString[0][index] = '\0';
                        break;
                    }
                }

                // If a comma found, continue...else fail
                if (string_len >= index-1)
                {
                    // The strings are in hex. Note that in RSID mode certain
                    // values of init are illegal.  This is not checked here
                    info.initAddr = (uword) strtoul(newInfoString[0], NULL, 16);
                    info.playAddr = (uword) strtoul(newInfoString[0]+index+1, NULL, 16);
                    // Not modifiable!
                    if (checkCompatibility() == false)
                        return false;
                    break;
                }
                else
                    return false;
            }

         case FREEPAGES:  //FREEPAGES string must have a comma!
            {
                size_t string_len = strlen(newInfoString[0]);

                // Terminate string at the comma
                // example, with FREEPAGES=20,03 the string is "20"
                size_t index;
                for (index = 0; newInfoString[0][index]; index++)
                {
                    if (',' == newInfoString[0][index])
                    {
                        newInfoString[0][index] = '\0';
                        break;
                    }
                }

                // If a comma found, continue...else fail
                if (string_len >= index-1)
                {
                    info.relocStartPage = (ubyte) strtoul(newInfoString[0], NULL, 16);
                    info.relocPages     = (ubyte) strtoul(newInfoString[0]+index+1, NULL, 16);
                    if (checkRelocInfo() == false)
		        return false;
                    break;
                }
                else
                    return false;
            }

         case FLAGS: // We'll fall thru the next 4 cases for this one.
         case MUSPLAYER:
            {
                if ((mode == FLAGS) && (newInfoString[infoStringIndex][0] == '*')) {
                    ; // Do nothing - this field is not to be changed.
                }
                else if (atoi(newInfoString[infoStringIndex]) == 0) {
                    info.musPlayer = false;
                }
                else if (atoi(newInfoString[infoStringIndex]) == 1) {
                    info.musPlayer = true;
                }
                else {
                    return false;
                }

                if (mode != FLAGS) {
                    break;
                }
                else {
                    // Fall through.
                    infoStringIndex++;
                }
            }

         case PLAYSID:
            {
                if ((mode == FLAGS) && (newInfoString[infoStringIndex][0] == '*')) {
                    ; // Do nothing - this field is not to be changed.
                }
                else if (atoi(newInfoString[infoStringIndex]) == 0) {
                    if ( (info.compatibility != SIDTUNE_COMPATIBILITY_C64) &&
                         (info.compatibility != SIDTUNE_COMPATIBILITY_PSID) )
                    {
                        return false;
                    }
                    info.compatibility = SIDTUNE_COMPATIBILITY_C64;
                }
                else if (atoi(newInfoString[infoStringIndex]) == 1) {
                    if ( (info.compatibility != SIDTUNE_COMPATIBILITY_C64) &&
                         (info.compatibility != SIDTUNE_COMPATIBILITY_PSID) )
                    {
                        return false;
                    }
                    info.compatibility = SIDTUNE_COMPATIBILITY_PSID;
                }
                else {
                    return false;
                }

                if (mode != FLAGS) {
                    break;
                }
                else {
                    // Fall through.
                    infoStringIndex++;
                }
            }

         case CLOCK:
            {
                if ((mode == FLAGS) && (newInfoString[infoStringIndex][0] == '*')) {
                    ; // Do nothing - this field is not to be changed.
                }
                else if (!strcmp(newInfoString[infoStringIndex], "UNKNOWN")) {
                    info.clockSpeed = SIDTUNE_CLOCK_UNKNOWN;
                }
                else if (!strcmp(newInfoString[infoStringIndex], "PAL")) {
                    info.clockSpeed = SIDTUNE_CLOCK_PAL;
                }
                else if (!strcmp(newInfoString[infoStringIndex], "NTSC")) {
                    info.clockSpeed = SIDTUNE_CLOCK_NTSC;
                }
                else if (!strcmp(newInfoString[infoStringIndex], "ANY") || !strcmp(newInfoString[infoStringIndex], "EITHER")) {
                    info.clockSpeed = SIDTUNE_CLOCK_ANY;
                }
                else {
                    return false;
                }

                if (mode != FLAGS) {
                    break;
                }
                else {
                    // Fall through.
                    infoStringIndex++;
                }
            }

         case SIDMODEL:
            {
                if ((mode == FLAGS) && (newInfoString[infoStringIndex][0] == '*')) {
                    ; // Do nothing - this field is not to be changed.
                }
                else if (!strcmp(newInfoString[infoStringIndex], "UNKNOWN")) {
                    info.sidModel = SIDTUNE_SIDMODEL_UNKNOWN;
                }
                else if (!strcmp(newInfoString[infoStringIndex], "6581")) {
                    info.sidModel = SIDTUNE_SIDMODEL_6581;
                }
                else if (!strcmp(newInfoString[infoStringIndex], "8580")) {
                    info.sidModel = SIDTUNE_SIDMODEL_8580;
                }
                else if (!strcmp(newInfoString[infoStringIndex], "ANY") || !strcmp(newInfoString[infoStringIndex], "EITHER")) {
                    info.sidModel = SIDTUNE_SIDMODEL_ANY;
                }
                else {
                    return false;
                }

                break; // The FLAGS directive stops here, too.
            }

         case FIXLOAD:
            {
                // Increase load address by 2 without verification.
                fixLoadAddress(true,info.initAddr,info.playAddr);
                break;
            }

         default:
            {
                return false;
            }
        } // switch

        return true;

    }; // writeToSidTune
}; // mySidTune

#endif  // mysidtune_h
