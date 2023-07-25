/***************************************************************************
             hardsid.cpp  -  Hardsid support interface.
                             -------------------
    begin                : Fri Dec 15 2000
    copyright            : (C) 2001-2001 by Jarno Paananen
    email                : jpaana@s2.org
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
 *  $Log: hardsid-unix.cpp,v $
 *  Revision 1.14  2004/05/27 21:18:28  jpaana
 *  The filter ioctl was reversed
 *
 *  Revision 1.13  2004/05/05 23:48:01  s_a_white
 *  Detect available sid devices on Unix system.
 *
 *  Revision 1.12  2004/04/29 23:20:01  s_a_white
 *  Optimisation to polling hardsid delay write to only access the hardsid
 *  if really necessary.
 *
 *  Revision 1.11  2003/10/28 00:15:16  s_a_white
 *  Get time with respect to correct clock phase.
 *
 *  Revision 1.10  2003/01/20 16:25:25  s_a_white
 *  Updated for new event scheduler interface.
 *
 *  Revision 1.9  2002/10/17 18:36:43  s_a_white
 *  Prevent multiple unlocks causing a NULL pointer access.
 *
 *  Revision 1.8  2002/08/14 16:03:54  jpaana
 *  Fixed to compile with new HardSID::lock method
 *
 *  Revision 1.7  2002/07/20 08:36:24  s_a_white
 *  Remove unnecessary and pointless conts.
 *
 *  Revision 1.6  2002/02/17 17:24:51  s_a_white
 *  Updated for new reset interface.
 *
 *  Revision 1.5  2002/01/30 01:47:47  jpaana
 *  Read ioctl used wrong parameter type and delay ioctl takes uint, not uint*
 *
 *  Revision 1.4  2002/01/30 00:43:50  s_a_white
 *  Added realtime delays even when there is no accesses to
 *  the sid.  Prevents excessive CPU usage.
 *
 *  Revision 1.3  2002/01/29 21:47:35  s_a_white
 *  Constant fixed interval delay added to prevent emulation going fast when
 *  there are no writes to the sid.
 *
 *  Revision 1.2  2002/01/29 00:32:56  jpaana
 *  Use the new read and delay IOCTLs
 *
 *  Revision 1.1  2002/01/28 22:35:20  s_a_white
 *  Initial Release.
 *
 *
 ***************************************************************************/


// ASID midi protocol hack
// by R. Hof , sid@k-n-p.org
// v0.1 2010/09/11
// still needs loads of cleaning up, but works (for me)
// v0.2 2010/09/27
// using the much more stable clock_nanosleep() for waiting.

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include "config.h"
#include "hardsid-emu.h"
#define __LINUX_ALSASEQ__
#include "RtMidi.cpp"

// Move these to common header file
#define HSID_IOCTL_RESET     _IOW('S', 0, int)
#define HSID_IOCTL_FIFOSIZE  _IOR('S', 1, int)
#define HSID_IOCTL_FIFOFREE  _IOR('S', 2, int)
#define HSID_IOCTL_SIDTYPE   _IOR('S', 3, int)
#define HSID_IOCTL_CARDTYPE  _IOR('S', 4, int)
#define HSID_IOCTL_MUTE      _IOW('S', 5, int)
#define HSID_IOCTL_NOFILTER  _IOW('S', 6, int)
#define HSID_IOCTL_FLUSH     _IO ('S', 7)
#define HSID_IOCTL_DELAY     _IOW('S', 8, int)
#define HSID_IOCTL_READ      _IOWR('S', 9, int*)


//#define ASID_DEBUG
// this prints out lots of debug stuff, and dumps the midi bytes into a file (out.syx)

int regmap[]={0,1,2,3,5,6,7,8,9,10,12,13,14,15,16,17,19,20,21,22,23,24,4,11,18,25,26,27};

extern int asid_midi_port;

bool       HardSID::m_sidFree[16] = {0};
const uint HardSID::voices = HARDSID_VOICES;
uint       HardSID::sid = 0;
char       HardSID::credit[];

HardSID::HardSID (sidbuilder *builder)
:sidemu(builder),
 Event("HardSID Delay"),
 m_handle(0),
 m_eventContext(NULL),
 m_phase(EVENT_CLOCK_PHI1),
 m_instance(sid++),
 m_status(false),
 m_locked(false)
{
    uint num = 16;
    std::vector<unsigned char> message;


    for ( uint i = 0; i < 16; i++ )
    {
        if(m_sidFree[i] == 0)
        {
            m_sidFree[i] = 1;
            num = i;
            break;
        }
    }

    for(uint i=0;i<28;i++)
    {
	    sid_register[i]=0;
	    sid_modified[i]=0;
    }
    // All sids in use?!?
    if ( num == 16 )
        return;

    m_instance = num;

    {
        char device[20];
        *m_errorBuffer = '\0';
#ifdef ASID_DEBUG
	printf("****** HARDSID %d*****\n",m_instance);
	printf(" midiport:%d\n",asid_midi_port);
#endif
	//printf("  %d \n\n",m_info.channels);
	/*
        sprintf (device, "/dev/sid%u", m_instance);
	m_handle = open (device, O_RDWR);
        if (m_handle < 0)
        {
            if (m_instance == 0)
            {
                m_handle = open ("/dev/sid", O_RDWR);
                if (m_handle < 0)
                {
                    sprintf (m_errorBuffer, "HARDSID ERROR: Cannot access \"/dev/sid\" or \"%s\"", device);
                    return;
                }
            }
            else
            {
                sprintf (m_errorBuffer, "HARDSID ERROR: Cannot access \"%s\"", device);
                return;
            }
        }*/
    }
	
	if(m_instance==0) // TODO: find out why/where it opens two instances
	{
#ifdef ASID_DEBUG
		f=fopen("out.syx","wb");
#endif
		midiout = new RtMidiOut();// TODO: clean up the whole midi port selection thing, shouldn't be done here anyway
		if(asid_midi_port==-1)
		{
			int nports=midiout->getPortCount();
			if(nports==0)
			{
				sprintf(m_errorBuffer,"no midi ports! go away!\n");
				return;
			}

			sprintf(m_errorBuffer,"\nNo midi port selected! Use --midiport=<num> to select one");

			for(int i=0;i<nports;i++)			
				printf("Port %d : %s\n",i,midiout->getPortName(i).c_str());
			return;
		
			
		}
		midiout->openPort(asid_midi_port); 
		message.clear();
		message.push_back(0xf0);
		message.push_back(0x2d);
		message.push_back(0x4c);
		message.push_back(0xf7);
		midiout->sendMessage(&message); //start sid play mode
	}

    clock_gettime(CLOCK_REALTIME,&time1);

    m_status = true;
    reset ();
}

HardSID::~HardSID()
{
    sid--;
    m_sidFree[m_instance] = 0;
    if(m_instance==0)
    {
#ifdef ASID_DEBUG
	    fclose(f);
#endif
	     std::vector<unsigned char> message;
		message.clear();
		message.push_back(0xf0);
		message.push_back(0x2d);
		message.push_back(0x4d);
		message.push_back(0xf7);
		midiout->sendMessage(&message); //exit sid play mode

	    delete midiout;
    }

    //if (m_handle)
    //    close (m_handle);
#ifdef ASID_DEBUG  
    printf("*** ~HardSID ***\n");
#endif
}

void HardSID::reset (uint8_t volume)
{
    for (uint i= 0; i < voices; i++)
        muted[i] = false;
    //ioctl(m_handle, HSID_IOCTL_RESET, volume);
    m_accessClk = 0;
    if (m_eventContext != NULL)
        m_eventContext->schedule (this, HARDSID_DELAY_CYCLES, m_phase);

#ifdef ASID_DEBUG
    printf("*** reset %d ***\n",m_instance);
#endif
    cycletimer=0;	
}

uint8_t HardSID::read (uint_least8_t addr)
{
    //if (!m_handle)
    //    return 0;

    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;

    while ( cycles > 0xffff )
    {
        /* delay */
        //ioctl(m_handle, HSID_IOCTL_DELAY, 0xffff);
	delay(cycles);
        cycles -= 0xffff;
    }

    uint packet = (( cycles & 0xffff ) << 16 ) | (( addr & 0x1f ) << 8 );
    //ioctl(m_handle, HSID_IOCTL_READ, &packet);
#ifdef ASID_DEBUG
    printf("***** read *****\n");
#endif
    cycles = 0;
    return (uint8_t) (packet & 0xff);
}

void HardSID::write (uint_least8_t addr, uint8_t data)
{
    //if (!m_handle)
    //    return;

    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    m_accessClk += cycles;

    cycletimer+=cycles; // number of cycles since last midi write

    // send data when it's been "a while" since we last sent anything
    if(cycletimer>6000) 
    {
	    sendreg();
#ifdef ASID_DEBUG
	    printf("** send data**\n");
#endif
	    cycletimer=0;
    } 

//    while ( cycles > 0xffff )
//    {
        /* delay */
//#ifdef ASID_DEBUG
//	    printf("*** delay %x\n",cycles);
//#endif
	    //usleep(cycles);
//	    delay(0xffff);
        //ioctl(m_handle, HSID_IOCTL_DELAY, 0xffff);
//        cycles -= 0xffff;
//    }

    uint packet = (( cycles & 0xffff ) << 16 ) | (( addr & 0x1f ) << 8 )
        | (data & 0xff);
    //cycles = 0;
    //::write (m_handle, &packet, sizeof (packet));
    //printf("**** write %d : %x : %x ***** %0x \n",m_instance,addr,data,packet);

    int reg=addr & 0x1f;

    // some sids write to the wrong places
    if(reg>0x18) 
    {
#ifdef ASID_DEBUG
	    printf("**** wrong register! %x ****\n",reg);
#endif
	    return;
    }
    
    // check how many times the register has been written to since the last sendreg,
    // if writing for the second time to 0x04,0x0b or 0x12, redirect write
    // TODO: some sids write 3 or more times to those registers, maybe integrate the writes into one instead of only using the last written version
    // sample does even more writes of course
    if(sid_modified[reg]==0)
    {
    	sid_register[reg]=data & 0xff;
	sid_modified[reg]++;
    }
    else
    {
		switch(reg)
		{
		    case 0x04:
			if(sid_modified[0x19]!=0) sid_register[0x04]=sid_register[0x19]; //if already written to secondary,move back to original one 
		    	sid_register[0x19]=data & 0xff;
		    	sid_modified[0x19]++;
			break;
		    case 0x0b:
			if(sid_modified[0x1a]!=0) sid_register[0x0b]=sid_register[0x1a]; 
			sid_register[0x1a]=data & 0xff;
 			sid_modified[0x1a]++;
			break;
		    case 0x12:
			if(sid_modified[0x1b]!=0) sid_register[0x12]=sid_register[0x1b]; 
		    	sid_register[0x1b]=data & 0xff;
		    	sid_modified[0x1b]++;
			break;

		    default:
		     	sid_register[reg]=data & 0xff;
			sid_modified[reg]++;
		}
    }
#ifdef ASID_DEBUG
    printf("**** write %d : %010d %06d %02x %02x",m_instance,m_accessClk,cycles,(addr & 0x1f),(data & 0xff));
	if(sid_modified[reg]!=1) printf(" mwrite:%d",sid_modified[reg]);
    printf("\n");
#endif
    cycles = 0;
}

// find changed registers, pack them into a sysex message, and send it
void HardSID::sendreg(void)
{
	int i;
	unsigned int r=0;
	unsigned int mask=0;
	unsigned int msb=0;
	std::vector<unsigned char> message;
	int j;
#ifdef ASID_DEBUG
	printf("**********sendreg  ");
	
	for(i=0;i<28;i++)
	{
		printf("%d",(r>>i) & 1);
		if((i % 4)==3) printf(" ");
	}
		
	printf("f0 2d 4e ");
	fputc(0xf0,f);fputc(0x2d,f);fputc(0x4e,f);
#endif
	// message header
	message.clear();
	message.push_back(0xf0);
	message.push_back(0x2d);
	message.push_back(0x4e);

	// set bits in mask for each register that has been written to
	// write last bit of each register into msb
	for(i=0;i<28;i++)
	{
		j=regmap[i];
		if(sid_modified[j]!=0)
		{
			mask=mask | (1<<i);
		}
		if(sid_register[j]>0x7f)
		{
			msb=msb | (1<<i);
		}
	}
#ifdef ASID_DEBUG
	printf("%02x %02x %02x %02x ",mask & 0x7f,(mask>>7) & 0x7f,(mask>>14) & 0x7f,(mask>>21) & 0x7f);
	printf("%02x %02x %02x %02x ",msb & 0x7f,(msb>>7) & 0x7f,(msb>>14) & 0x7f,(msb>>21) & 0x7f);
	
	for(i=0;i<4;i++)
		fputc((mask>>(i*7)) & 0x7f,f);
	for(i=0;i<4;i++)
		fputc((msb>>(i*7)) & 0x7f,f);
#endif
	// chop mask and msb in 7 bit pieces
	message.push_back(mask & 0x7f);
	message.push_back((mask>>7)&0x7f);
	message.push_back((mask>>14)&0x7f);
	message.push_back((mask>>21)&0x7f);
	message.push_back(msb & 0x7f);
	message.push_back((msb>>7)&0x7f);
	message.push_back((msb>>14)&0x7f);
	message.push_back((msb>>21)&0x7f);


	// append the register values (first 7 bits only)
	for(i=0;i<28;i++)
	{
		j=regmap[i];
		if(sid_modified[j]!=0)
		{
#ifdef ASID_DEBUG
			printf("%02x ",sid_register[j]&0x7f);
			fputc(sid_register[j]&0x7f,f);
#endif
			message.push_back(sid_register[j]&0x7f);
		}
	}
#ifdef ASID_DEBUG
	printf("f7\n");
	fputc(0xf7,f);
#endif
	// end sysex message
	message.push_back(0xf7);

	//wait for the right time
	delay(cycletimer);
	// send it
	midiout->sendMessage(&message);

	//old unaccurate timer
	//usleep(cycletimer);
	//
	
	//time2.tv_sec=time1.tv_sec;
	//time2.tv_nsec=time1.tv_nsec;
	//
	// now uses the much more accurate clock_gettime and nanosleep
	

	//delay(cycletimer);

	// reset modified counter
	for(i=0;i<28;i++)
	{
		sid_modified[i]=0;
	}

	
}


#define NSEC_PER_SEC    1000000000
				
void HardSID::delay(int clockcycles)
{

	time1.tv_nsec+=clockcycles*1000;  	//add time we have to wait to the timer
	if(time1.tv_nsec>NSEC_PER_SEC) 		//more than 1 second in total ?
	{
		time1.tv_nsec-=NSEC_PER_SEC; // -1 second
		time1.tv_sec++;		     // +1 second
	}
	//wakes us up when it's time
	clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&time1,NULL);

}


void HardSID::voice (uint_least8_t num, uint_least8_t volume,
                     bool mute)
{
    // Only have 3 voices!
    if (num >= voices)
        return;
    muted[num] = mute;
    
    int cmute = 0;
    for ( uint i = 0; i < voices; i++ )
        cmute |= (muted[i] << i);
    //ioctl (m_handle, HSID_IOCTL_MUTE, cmute);
}

void HardSID::event (void)
{
    event_clock_t cycles = m_eventContext->getTime (m_accessClk, m_phase);
    if (cycles < HARDSID_DELAY_CYCLES)
    {
        m_eventContext->schedule (this, HARDSID_DELAY_CYCLES - cycles,
                                EVENT_CLOCK_PHI1);
    }
    else
    {
        uint cdelay = (uint) cycles;
        m_accessClk += cycles;
	cycletimer+=cycles;
        //ioctl(m_handle, HSID_IOCTL_DELAY, delay);
//#ifdef ASID_DEBUG
//	printf("**** event delay %d %d\n",m_instance,cdelay);
//#endif
	if(m_instance==0) {sendreg();cycletimer=0;}// send changes before going to sleep
//        cycletimer+=cycles;
	//if(m_instance==0) {sendreg();cycletimer=0;}
	
	m_eventContext->schedule (this, HARDSID_DELAY_CYCLES, m_phase);
    }
}

void HardSID::filter(bool enable)
{
    //ioctl (m_handle, HSID_IOCTL_NOFILTER, !enable);
}

void HardSID::flush(void)
{
    //ioctl(m_handle, HSID_IOCTL_FLUSH);
    //printf("*** flush %d\n",m_instance);
}

bool HardSID::lock(c64env* env)
{
    if( env == NULL )
    {
	if (!m_locked)
	    return false;
        m_eventContext->cancel (this);
        m_locked = false;
        m_eventContext = NULL;
    }
    else
    {
	if (m_locked)
	    return false;
        m_locked = true;
        m_eventContext = &env->context();
        m_eventContext->schedule (this, HARDSID_DELAY_CYCLES, m_phase);
    }
    return true;
}

