#include "PathCreator.h"

PathCreator::PathCreator(void) : maxPathLen(maxCharBufferLen)
{
    path = new char[maxPathLen];
    path[0] = 0;
}

PathCreator::PathCreator(const char* inPath) : maxPathLen(maxCharBufferLen)
{
    path = new char[maxPathLen];
    path[0] = 0;
    if (inPath != 0)
        strcpy(path,inPath);
}

PathCreator::~PathCreator(void)
{
    if (path != 0)
        delete[] path;
}

const char* PathCreator::get(void)
{
    return path;
}

char* PathCreator::getWritable(void)
{
    return path;
}

void PathCreator::empty(void)
{
    path[0] = 0;
}

bool PathCreator::append(const char* file)
{
    strcat(path,file);
    return true;  //!!!
}
