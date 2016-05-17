#include "CFremenGridSet.h"

using namespace std;

CFremenGridSet::CFremenGridSet()
{
    numFremenGrids = 0;
    activeIndex = 0;
    active = NULL;
}

CFremenGridSet::~CFremenGridSet()
{
    for (int i = 0; i < numFremenGrids; i--)
        delete fremengrid[i];
}

int CFremenGridSet::add(const char* name, float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize)
{
    int exists = find(name);

    if (exists >= 0)
        return -1;
    else
    {
        fremengrid[numFremenGrids++] = new CFremenGrid(name, originX, originY, originZ, dimX, dimY, dimZ, cellSize);
        activeIndex = numFremenGrids-1;
        active = fremengrid[numFremenGrids-1];
    }

    return 0;
}

int CFremenGridSet::incorporate(const char *name, float *x,float *y,float *z,float *d,int size,uint32_t t)
{
    int exists = find(name);
    if (exists < 0) return -1;
    else return active->incorporate( x, y, z, d, size, t);

}

float CFremenGridSet::recalculate(const char *name, uint32_t timeStamp)
{
    find(name);
    return active->recalculate(timeStamp);

}

float CFremenGridSet::estimate(const char *name, unsigned int index, uint32_t timeStamp)
{
    find(name);
    return active->estimate(index, timeStamp);

}

float CFremenGridSet::retrieve(const char *name, unsigned int index, uint32_t timeStamp)
{
    find(name);
    return active->retrieve(index);

}

float CFremenGridSet::dominant(const char *name, unsigned int index, uint32_t period)
{
    find(name);
    return active->getDominant(index, period);

}


int CFremenGridSet::estimateEntropy(const char *name,float x,float y,float z,float range,uint32_t t)
{
    if (find(name) < 0) return -1;

    return active->estimateInformation(x, y, z, range, t);
}

int CFremenGridSet::find(const char *name)
{
    int i = 0;
    for (i =0;(i<numFremenGrids) && (strcmp(fremengrid[i]->id,name)!=0);i++) {}
    if (i==numFremenGrids) return -1;
    activeIndex = i;
    active = fremengrid[i];
    return activeIndex;
}

int CFremenGridSet::remove(const char *name)
{
    if (find(name) < 0) return -numFremenGrids;
    delete fremengrid[activeIndex];
    fremengrid[activeIndex] = fremengrid[--numFremenGrids];
    return numFremenGrids+1;
}

bool CFremenGridSet::update(const char* name,int order)
{
    if (find(name) < 0) return false;
    active->update();
    return true;
}

bool CFremenGridSet::print(bool verbosityLevel)
{
    for (int i = 0;i<numFremenGrids;i++) fremengrid[i]->print(verbosityLevel);
}

bool CFremenGridSet::load(const char* name, const char* filename)
{
}

bool CFremenGridSet::save(const char* name, const char* filename)
{

    if (find(name) < 0)
        return false;
    else
    {
        active->saveSmart(filename, false, 0);
        return true;
    }
}
