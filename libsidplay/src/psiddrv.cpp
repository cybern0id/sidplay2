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
 *  Revision 1.1  2001/07/27 12:12:23  s_a_white
 *  Initial release.
 *
 ***************************************************************************/

// --------------------------------------------------------
// The code here is use to support the PSID Version 2B file
// format (player relocation support).
// --------------------------------------------------------
#include "player.h"

const char *Player::ERR_PSIDDRV_NO_SPACE = "ERROR: No space to install psid driver in C64 ram"; 
const char *Player::ERR_PSIDDRV_RELOC    = "ERROR: Failed whilst relocating psid driver";

extern "C" int reloc65(char** buf, int* fsize, int addr);

int Player::psidDrvInstall ()
{
    uint_least16_t relocAddr;
    
    // Check if we need to find the reloc addr
    if (!m_tuneInfo.relocStartPage)
        psidRelocAddr ();

    if ((m_tuneInfo.relocStartPage == 0xff) ||
        (m_tuneInfo.relocPages < 1))
    {
        m_errorString = ERR_PSIDDRV_NO_SPACE;
        return -1;
    }

    relocAddr = m_tuneInfo.relocStartPage << 8;
    m_info.driverAddr = relocAddr;

    {   // Place psid driver into ram
        uint8_t psid_driver[] = {
#          include "psiddrv.bin"
        };
        uint8_t *reloc_driver = psid_driver;
        int      reloc_size   = sizeof (psid_driver);

        if (!reloc65 ((char **) &reloc_driver, &reloc_size, relocAddr - 11))
        {
            m_errorString = ERR_PSIDDRV_RELOC;
            return -1;
        }

        m_ram[0x310] = JMPw;
        memcpy (&m_rom[0xfffc],    &reloc_driver[0], 2);
        memcpy (&m_ram[0x0311],    &reloc_driver[2], 9);
        memcpy (&m_ram[relocAddr], &reloc_driver[11], reloc_size - 11);
    }

    {   // Setup the Initial entry point
        uint_least16_t playAddr = m_tuneInfo.playAddr;
        uint_least16_t addr;

        // Check to make sure the play address is legal
        if (playAddr == 0xffff)
            playAddr  = 0;

        // Tell C64 about song, 1st 3 locations reserved for
        // contain jmp addr
        addr = relocAddr;
        m_ram[addr++] = (uint8_t) m_tuneInfo.currentSong;
        if (m_tuneInfo.songSpeed == SIDTUNE_SPEED_VBI)
            m_ram[addr] = 0;
        else // SIDTUNE_SPEED_CIA_1A
            m_ram[addr] = 1;

        addr++;
        endian_little16 (&m_ram[addr], m_tuneInfo.initAddr);
        addr += 2;
        endian_little16 (&m_ram[addr], playAddr);
    }
    return 0;
}


void Player::psidRelocAddr ()
{
    // Start and end pages.
    int startp =  m_tuneInfo.loadAddr >> 8;
    int endp   = (m_tuneInfo.loadAddr + (m_tuneInfo.c64dataLen - 1)) >> 8;

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
        m_tuneInfo.relocPages = 0;
        for (size_t page = 0; page < sizeof(pages)/sizeof(*pages); page++)
        {
            if (pages[page] == false)
                continue;
            relocPages = page - lastPage;
            if (relocPages > m_tuneInfo.relocPages)
            {
                m_tuneInfo.relocStartPage = lastPage;
                m_tuneInfo.relocPages     = relocPages;
            }
            lastPage = page + 1;
        }
    }

    if (m_tuneInfo.relocPages    == 0x00)
        m_tuneInfo.relocStartPage = 0xff;
}
