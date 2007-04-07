// A textfile class that does a little bit more than readline().

#ifndef TextFile_h
#define TextFile_h


class TextFile
{

 public:  // --------------------------------------------------------- public

	TextFile(const char* fileName);  // opens file!
	virtual ~TextFile();
	
    virtual bool open(const char* fileName);
    virtual void close();

    virtual const char* getLineBuf() const;
    virtual const unsigned long getLineNum() const;
    virtual int getLineLen() const;
    virtual bool endOfFile() const;
    virtual bool readNextLine();
	
    // Return a buffer that contains the current line without spaces.
    virtual const char* getParseBuf();

    // Return a buffer that contains the current line without spaces
    // at current parse position.
    virtual const char* getCurParseBuf();
	
    // Check whether the first non-space character is a ``#'' or ``;''.
    virtual bool isComment();
	
    // Check whether the line does not contain any non-space characters.
    virtual bool isBlank();
	
    // Check whether the first characters are equal to a given keyword string.
    // By default a possibly matching keyword will be skipped.
    virtual bool isKey(const char *key, bool advance = true);

 protected:  // --------------------------------------------------- protected

    virtual bool loadFromDisk(char* buf, const unsigned int maxLen);
    virtual bool zeroDelimiters();
    virtual bool createParseCopy();

    const int maxLineLen;

    ifstream* inFile;             // our input file stream
    unsigned long inFileLen;
    unsigned long leftToLoad;	  // how much is left to be loaded from disk
	
    char* lineBuf;                // current line buffer
    char* parseBuf;               // line without white-space (see flag below)
    char* curParseBuf;            // points to current pos in parseBuf
	
    int inBuffer;		        // number of chars read into buffer
    int moreInBuffer;	        // <> 0, if more than one line buffered
    int lineLen;                // actual number of chars till end of line
    char* nextLine;	            // pointer to start of next line, if available 
	
    unsigned long int lineNum;  // line number in file
	
    bool haveParseCopy;         // true, if line contents were copied to an
                                // extra buffer via a white-space eating
                                // string stream
    bool status;
    bool isGood;                // to prevent from using a bad inFile pointer

 private:  // ------------------------------------------------------- private

};

#endif
