/***************************************************************************
                          psiddrv.cpp  -  PSID Driver Installtion
                             -------------------
    begin                : Fri Jul 27 2001
    copyright            : (C) 2001 by Simon White
    email                : s_a_white@email.com
 ***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
/***************************************************************************
 *  $Log: not supported by cvs2svn $
 *  Revision 1.12  2002/03/03 22:02:36  s_a_white
 *  Sidplay2 PSID driver length now exported.
 *
 *  Revision 1.11  2002/02/17 12:41:18  s_a_white
 *  Fixed to not so easily break when C64 code is modified.
 *
 *  Revision 1.10  2002/02/04 23:50:48  s_a_white
 *  Improved compatibilty with older sidplay1 modes.
 *
 *  Revision 1.9  2002/01/29 21:50:33  s_a_white
 *  Auto switching to a better emulation mode.  tuneInfo reloaded after a
 *  config.  Initial code added to support more than two sids.
 *
 *  Revision 1.8  2001/12/21 21:54:14  s_a_white
 *  Improved compatibility if Sidplay1 bankswitching mode.
 *
 *  Revision 1.7  2001/12/13 08:28:08  s_a_white
 *  Added namespace support to fix problems with xsidplay.
 *
 *  Revision 1.6  2001/11/16 19:23:18  s_a_white
 *  Fixed sign of buf for reloc65 call.
 *
 *  Revision 1.5  2001/10/28 21:28:35  s_a_white
 *  For none real mode if play != 0 we now always jump to irqjob instead of
 *  playAddr.
 *
 *  Revision 1.4  2001/10/02 18:31:24  s_a_white
 *  No longer dies if relocStartPages != 0 byr relocPages == 0.  For none real
 *  evironment modes, play is always followed even if interrupt handlers are
 *  installed.
 *
 *  Revision 1.3  2001/09/01 11:13:18  s_a_white
 *  Fixes sidplay1 environment modes.
 *
 *  Revision 1.2  2001/07/27 12:52:12  s_a_white
 *  Removed warning.
 *
 *  Revision 1.1  2001/07/27 12:12:23  s_a_white
 *  Initial release.
 *
 ***************************************************************************/

// --------------------------------------------------------
// The code here is use to support the PSID Version 2NG
// (proposal B) file format for player relocation support.
// --------------------------------------------------------
#include "sidendian.h"
#include "player.h"

SIDPLAY2_NAMESPACE_START

const char *Player::ERR_PSIDDRV_NO_SPACE = "ERROR: No space to install psid driver in C64 ram"; 
const char *Player::ERR_PSIDDRV_RELOC    = "ERROR: Failed whilst relocating psid driver";

extern "C" int reloc65(unsigned char** buf, int* fsize, int addr);

int Player::psidDrvInstall (SidTuneInfo &tuneInfo)
{
    uint_least16_t relocAddr;
    
    // Check if we need to find the reloc addr
    if ((tuneInfo.relocStartPage == 0) ||
        (tuneInfo.relocPages     == 0))
    {
        psidRelocAddr (tuneInfo);
    }

    if ((tuneInfo.relocStartPage == 0xff) ||
        (tuneInfo.relocPages < 1))
    {
        m_errorString = ERR_PSIDDRV_NO_SPACE;
        return -1;
    }

    relocAddr = tuneInfo.relocStartPage << 8;

    {   // Place psid driver into ram
        uint8_t psid_driver[] = {
#          include "psiddrv.bin"
        };
        uint8_t *reloc_driver = psid_driver;
        int      reloc_size   = sizeof (psid_driver);

        if (!reloc65 (&reloc_driver, &reloc_size, relocAddr - 17))
        {
            m_errorString = ERR_PSIDDRV_RELOC;
            return -1;
        }

        // Adjust size to not included initialisation data.
        reloc_size -= 17;
        m_info.driverAddr   = relocAddr;
        m_info.driverLength = (uint_least16_t) reloc_size;
        // Round length to end of page
        m_info.driverLength += 0xff;
        m_info.driverLength &= 0xff00;

        m_ram[0x310] = JMPw;
        memcpy (&m_ram[0x0311],    &reloc_driver[4], 9);
        memcpy (&m_ram[relocAddr], &reloc_driver[17], reloc_size);

        // Setup hardware vectors
        switch (m_info.environment)
        {
        case sid2_envBS:
            memcpy (&m_rom[0xfffa], &m_ram[0x0318],   2);
            memcpy (&m_rom[0xfffc], &reloc_driver[0], 4);
        case sid2_envTP:
        case sid2_envPS:
            memcpy (&m_ram[0xfffa], &m_ram[0x0318],   2);
            memcpy (&m_ram[0xfffc], &reloc_driver[0], 4);
            break;
        case sid2_envR:
            memcpy (&m_rom[0xfffc], &reloc_driver[0], 2);
            break;
        }

        // Support older modes ability to ignore the IRQ
        // vectors if valid play
        if ((m_info.environment != sid2_envR) && tuneInfo.playAddr)
        {   // Get the addr of the sidplay vector
            uint_least16_t addr = endian_little16(&reloc_driver[13]);
            // Get the irqjob vector
            uint_least16_t vec  = endian_little16(&reloc_driver[15]);
            // Jump directly to irqjob instead of using interrupt
            // vector at 0x0134
            m_ram[addr] = JMPw;
            endian_little16 (&m_ram[addr + 1], vec);
        }
    }

    {   // Setup the Initial entry point
        uint_least16_t playAddr = tuneInfo.playAddr;
        uint_least16_t addr;

        // Check to make sure the play address is legal
        if (playAddr == 0xffff)
            playAddr  = 0;

        // Tell C64 about song, 1st 3 locations reserved for
        // contain jmp addr
        addr = relocAddr;
        m_ram[addr++] = (uint8_t) tuneInfo.currentSong;
        if (tuneInfo.songSpeed == SIDTUNE_SPEED_VBI)
            m_ram[addr] = 0;
        else // SIDTUNE_SPEED_CIA_1A
            m_ram[addr] = 1;

        addr++;
        endian_little16 (&m_ram[addr], tuneInfo.initAddr);
        addr += 2;
        endian_little16 (&m_ram[addr], playAddr);
    }
    return 0;
}


void Player::psidRelocAddr (SidTuneInfo &tuneInfo)
{
    // Start and end pages.
    int startp =  tuneInfo.loadAddr >> 8;
    int endp   = (tuneInfo.loadAddr + (tuneInfo.c64dataLen - 1)) >> 8;

    // Used memory ranges.
    bool pages[256];
    int  used[] = {0x00,   0x03,
                   0xa0,   0xbf,
                   0xd0,   0xff,
                   startp, endp};

    // Mark used pages in table.
    memset(pages, false, sizeof(pages));
    for (size_t i = 0; i < sizeof(used)/sizeof(*used); i += 2)
    {
        for (int page = used[i]; page <= used[i + 1]; page++)
            pages[page] = true;
    }

    {   // Find largest free range.
        int relocPages, lastPage = 0;
        tuneInfo.relocPages = 0;
        for (size_t page = 0; page < sizeof(pages)/sizeof(*pages); page++)
        {
            if (pages[page] == false)
                continue;
            relocPages = page - lastPage;
            if (relocPages > tuneInfo.relocPages)
            {
                tuneInfo.relocStartPage = lastPage;
                tuneInfo.relocPages     = relocPages;
            }
            lastPage = page + 1;
        }
    }

    if (tuneInfo.relocPages    == 0x00)
        tuneInfo.relocStartPage = 0xff;
}

SIDPLAY2_NAMESPACE_STOP
