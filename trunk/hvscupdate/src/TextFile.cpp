#include "config.h"
#include "compconf.h"

#include <ctype.h>
#if defined(HAVE_FSTREAM)
  #include <fstream>
  using std::ifstream;
#else
  #include <fstream.h>
#endif
#if defined(HAVE_IOMANIP)
  #include <iomanip>
  using std::ios;
#else
  #include <iomanip.h>
#endif
#include <string.h>

#include "fformat.h"  // strnicmp stuff
#include "TextFile.h"

TextFile::TextFile(const char* fileName) : maxLineLen(2048)
{
    lineBuf = new char[maxLineLen+1];
    parseBuf = new char[maxLineLen+1];
    status = (lineBuf!=0 && parseBuf!=0);
    inFile = 0;
    open(fileName);
}

TextFile::~TextFile()
{
    close();
    if (lineBuf != 0)
        delete[] lineBuf;
    if (parseBuf != 0)
        delete[] parseBuf;
}

bool TextFile::open(const char* fileName)
{
    close();  // always re-open
    
    lineBuf[maxLineLen] = 0;
    lineLen = 0;
    lineNum = 0;
    inBuffer = (moreInBuffer = 0);
    nextLine = 0;
    
    if (fileName == 0)
        return false;
    
	// Create input file stream.
	// Make it binary because of compatibility to text files from any
	// known operating system.
#if defined(HAVE_IOS_BIN)
	inFile = new ifstream(fileName,ios::in|ios::bin);
#else
	inFile = new ifstream(fileName,ios::in|ios::binary);
#endif
#if defined(HAVE_SEEKG_OFFSET)
	leftToLoad = (inFileLen = (inFile->seekg(0,ios::end)).offset());
#else
    inFile->seekg(0,ios::end);
	leftToLoad = (inFileLen = (unsigned long)inFile->tellg());
#endif
	inFile->seekg(0,ios::beg);
#if defined(DONT_HAVE_IS_OPEN)
    isGood = (inFile && inFile!=0 && !inFile->bad());
#else
	isGood = (inFile->is_open() && inFile!=0 && !inFile->bad());
#endif
    return isGood;
}

void TextFile::close()
{
    if (inFile != 0)
    {
        inFile->close();
        delete inFile;
        inFile = 0;
        isGood = false;
    }
}

const char* TextFile::getLineBuf() const
{ 
    return lineBuf;
}

const unsigned long TextFile::getLineNum() const
{
    return lineNum;
}

int TextFile::getLineLen() const
{
    return lineLen;
}

bool TextFile::endOfFile() const
{
    return ((leftToLoad==0)&&(moreInBuffer==0));
}

bool TextFile::readNextLine()
{
    // Next line (or just part of ...) also buffered.
	if ((nextLine != 0) && (moreInBuffer != 0))
    {
        // Now move the next line to the beginning of the buffer.
        for (int i = 0; i < (maxLineLen-(int)(nextLine-lineBuf)); i++ )
	    {
            lineBuf[i] = nextLine[i];
        }
        inBuffer = moreInBuffer;
        loadFromDisk(lineBuf+moreInBuffer,maxLineLen-moreInBuffer);
        zeroDelimiters();
        lineNum++;
        haveParseCopy = false;
        return true;
    }
	// Nothing left in buffer. Fill the buffer by loading from disk.
    else
    {
        inBuffer = (moreInBuffer = 0);
        if (!loadFromDisk(lineBuf,maxLineLen))
        {
            // No next line. Reset everything.
            lineLen = 0;
            lineBuf[0] = 0;
            nextLine = 0;
            lineNum = 0;
            haveParseCopy = false;
            return false;
        }
        else
	    {
            zeroDelimiters();
            lineNum++;
            haveParseCopy = false;
            return true;
        }
    }
}

bool TextFile::loadFromDisk(char* buf, const unsigned int maxLen)
{
	if (status && isGood)  // input file stream object status?
    {
        if (maxLen <= leftToLoad)
        {
            inFile->read(buf,maxLen);
            inBuffer += maxLen;
            leftToLoad -= maxLen;
        }
        else if (leftToLoad > 0)
        {
            inFile->read(buf,leftToLoad);
            inBuffer += leftToLoad;
            leftToLoad = 0;
        }
        else  // leftToLoad == 0
        {
            return false;
        }
        return true;
    }
    else  // ifstream object not created; cannot load from disk
    {
        return false;
    }
}

// Search input line buffer for next newline sequence. Replace newline
// sequence by a string-terminating zero. Skip it and save start of next
// line if available.
bool TextFile::zeroDelimiters()
{
    // Unix: LF = 0x0A
    // MS-Windows, MS-DOS: CR,LF = 0x0D,0x0A
    // MacOS: CR = 0x0D
    bool foundEOL = false;  // assume we do not find a delimiter
    char c;
    char* pCurPos = lineBuf;
    char* pDelimiter;
    for (int n = 0; n < inBuffer; n++)
    {
        c = *pCurPos;
        pDelimiter = pCurPos;
        pCurPos++;                             // skip read character
        if (c == 0x0A)
        {
            *pDelimiter = 0;		     // zero overstrike
            lineLen = pDelimiter - lineBuf;
            nextLine = pCurPos;
            // In case we read more than one line. Calc the remainder.
            moreInBuffer = (int) ((lineBuf+inBuffer)-pCurPos);
            foundEOL = true;
            break;                             // LF found
        }
        else if (c == 0x0D)
        {
            *pDelimiter = 0;                   // zero overstrike
            lineLen = pDelimiter - lineBuf;
            nextLine = pCurPos;
            if (*pCurPos == 0x0A)
            {
                *pCurPos = 0;		     // zero overstrike
                pCurPos++;                     // CR,LF found, skip LF
                nextLine = pCurPos;
            }
            // In case we read more than one line. Calc the remainder.
            moreInBuffer = (int) ((lineBuf+inBuffer)-pCurPos);
            foundEOL = true;
            break;                             // CR or CR,LF found
        }
    }
    // Here we decide to accept some unterminated characters at the end of a
    // file as a valid line (the last one).
    if ( !foundEOL && (inBuffer != 0))
    {
        lineBuf[inBuffer] = 0;  // terminate
        lineLen = inBuffer;
        moreInBuffer = 0;
        nextLine = 0;
    }
    return foundEOL;
}

bool TextFile::isComment()
{
    if ( !haveParseCopy )
    {
        createParseCopy();
    }
	return ((parseBuf[0]==';')||(parseBuf[0]=='#'));
}

bool TextFile::isBlank()
{
    if ( !haveParseCopy )
    {
        createParseCopy();
    }
    return (parseBuf[0]==0);
}

bool TextFile::isKey(const char *keyword, bool advance)
{
    if ( !haveParseCopy )
    {
        createParseCopy();
    }
	bool matches = (strnicmp(curParseBuf,keyword,strlen(keyword))==0);
    if (matches && advance)
    {
        curParseBuf += strlen(keyword);
        if (curParseBuf > (parseBuf+maxLineLen))
        {
            curParseBuf = parseBuf+maxLineLen;
        }
    }
    return matches;
}

bool TextFile::createParseCopy()
{
    int di = 0;
    for ( int i = 0; i < lineLen; i++ )
    {   // The code here uses extended ASCII characters > 127.
        // If char is signed these all appear as negative numbers to
        // isspace, which is illegal.
        char c = lineBuf[i];
        parseBuf[di] = c;
        if ( !isspace((unsigned char)c) )
        {
			di++;
        }
    }
    parseBuf[di] = 0;
    curParseBuf = parseBuf;
    return (haveParseCopy=true);
}

const char* TextFile::getParseBuf()
{ 
    if ( !haveParseCopy )
    {
        createParseCopy();
    }
    return parseBuf;
}

const char* TextFile::getCurParseBuf()
{ 
    if ( !haveParseCopy )
    {
        createParseCopy();
    }
    return curParseBuf;
}
