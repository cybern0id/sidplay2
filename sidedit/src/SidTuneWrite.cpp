#include "SidTuneWrite.h"

const char SidTuneWrite::txt_vbiClockSpeed[]        = "SIDTUNE ERROR: Clock speed must be VBI for compatibility mode";
const char SidTuneWrite::txt_invalidClockSpeed[]    = "SIDTUNE ERROR: Invalid clock speed";
const char SidTuneWrite::txt_invalidCompatibility[] = "SIDTUNE ERROR: Invalid sid model";
const char SidTuneWrite::txt_invalidSidModel[]      = "SIDTUNE ERROR: Invalid sid model";
const char SidTuneWrite::txt_invalidSongSpeed[]     = "SIDTUNE ERROR: Invalid song speed";

SidTuneWrite::SidTuneWrite (const char* fileName, const char **fileNameExt, const bool separatorIsSlash)
:SidTune(fileName, fileNameExt, separatorIsSlash)
{
}

SidTuneWrite::SidTuneWrite (const uint_least8_t* oneFileFormatSidtune, const uint_least32_t sidtuneLength)
:SidTune(oneFileFormatSidtune, sidtuneLength)
{
}

bool SidTuneWrite::setInfo (const SidTuneInfo &tuneInfo)
{
    SidTuneInfo info = tuneInfo;

    // Lets verify some of the data
    if ((info.currentSong >= info.songs) || (info.songs >= SIDTUNE_MAX_SONGS))
    {
        m_info.statusString = txt_songNumberExceed;
        return false;
    }

    if (info.loadAddr == 0)
    {
        m_info.statusString = txt_badAddr;
        return false;
    }

    if (info.sidModel1 > SIDTUNE_COMPATIBILITY_BASIC)
    {
        m_info.statusString = txt_invalidCompatibility;
        return false;
    }

    if (info.clockSpeed > SIDTUNE_CLOCK_ANY)
    {
        m_info.statusString = txt_invalidClockSpeed;
        return false;
    }

    if (info.sidModel1 > SIDTUNE_SIDMODEL_ANY)
    {
        m_info.statusString = txt_invalidSidModel;
        return false;
    }

    if (info.sidModel2 > SIDTUNE_SIDMODEL_ANY)
    {
        m_info.statusString = txt_invalidSidModel;
        return false;
    }

    switch (info.songSpeed)
    {
    case SIDTUNE_SPEED_CIA_1A:
        switch (info.compatibility)
        {
        case SIDTUNE_COMPATIBILITY_R64:
        case SIDTUNE_COMPATIBILITY_BASIC:
            m_info.statusString = txt_vbiClockSpeed;
            return false;
        }
        break;
    case SIDTUNE_SPEED_VBI:
        break;
    default:
        m_info.statusString = txt_invalidSongSpeed;
        return false;
    }

    if (!resolveAddrs(info, 0))
    {
        m_info.statusString = info.statusString;
        return false;
    }

    if (!checkRelocInfo(info))
    {
        m_info.statusString = info.statusString;
        return false;
    }

    if (!checkCompatibility(info))
    {
        m_info.statusString = info.statusString;
        return false;
    }
/*
    for ( uint_least16_t sNum = 0; sNum < SIDTUNE_MAX_CREDIT_STRINGS; sNum++ )
    {
        for ( uint_least16_t sPos = 0; sPos < SIDTUNE_MAX_CREDIT_STRLEN; sPos++ )
        {
            infoString[sNum][sPos] = info.infoString[sNum][sPos];
        }
    }
*/
    m_info = info;
}
