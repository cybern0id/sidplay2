#ifndef SIDTUNEWRITE_H
#define SIDTUNEWRITE_H

#include <SidTune.h>

class SidTuneWrite: public SidTune
{
public:
    SidTuneWrite (const char* fileName, const char **fileNameExt = 0,
                  const bool separatorIsSlash = false);

    // Load a single-file sidtune from a memory buffer.
    // Currently supported: PSID format
    SidTuneWrite (const uint_least8_t* oneFileFormatSidtune, const uint_least32_t sidtuneLength);

    // Error messages stored in getInfo().statusString
    virtual bool setInfo (const SidTuneInfo &info);

private:
    static const char SidTuneWrite::txt_vbiClockSpeed[];
    static const char SidTuneWrite::txt_invalidClockSpeed[];
    static const char SidTuneWrite::txt_invalidCompatibility[];
    static const char SidTuneWrite::txt_invalidSidModel[];
    static const char SidTuneWrite::txt_invalidSongSpeed[];
};

#endif // SIDTUNEWRITE_H
