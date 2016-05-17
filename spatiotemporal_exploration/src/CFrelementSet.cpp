#include "CFrelementSet.h"

using namespace std;

CFrelementSet::CFrelementSet()
{
	numFrelements = 0;
	activeIndex = 0;
	active = NULL;
}

CFrelementSet::~CFrelementSet()
{
	for (int i=0;i<numFrelements;i++) delete frelements[i];
}

int CFrelementSet::add(const char *name,uint32_t times[],unsigned char states[],int length,bool createNew)
{
	int exists = find(name);
	if (exists < 0 && createNew == false) return -1;
	if (exists < 0 && createNew == true )
	{
		frelements[numFrelements++] = new CFrelement(name);
		activeIndex = numFrelements-1;
		active = frelements[numFrelements-1];
	}
	//printf("Add %i %s \n",activeIndex,active->id);
	return active->add(times,states,length);
}

int CFrelementSet::evaluate(const char *name,uint32_t times[],unsigned char states[],int length,int order,float errors[])
{
	if (find(name) < 0) return -1;
	return active->evaluate(times,states,length,order,errors);;
}

int CFrelementSet::estimate(const char *name,uint32_t times[],float probs[],int length,int order)
{
	if (find(name) < 0) return -1;
	//printf("Estimate %i %s \n",activeIndex,active->id);
	return active->estimate(times,probs,length,order);
}

int CFrelementSet::estimateEntropy(const char *name,uint32_t times[],float entropy[],int length,int order)
{
	if (find(name) < 0) return -1;
	//printf("Estimate %i %s \n",activeIndex,active->id);
	return active->estimateEntropy(times,entropy,length,order);
}

int CFrelementSet::find(const char *name)
{
	int i = 0;
	for (i =0;(i<numFrelements) && (strcmp(frelements[i]->id,name)!=0);i++) {}
	if (i==numFrelements) return -1;
	activeIndex = i;	
	active = frelements[i];
	return activeIndex;
}

int CFrelementSet::remove(const char *name)
{
	if (find(name) < 0) return -numFrelements;
	delete frelements[activeIndex];
	frelements[activeIndex] = frelements[--numFrelements];
	return numFrelements+1;
}

bool CFrelementSet::update(const char* name,int order)
{
	if (find(name) < 0) return false;
	active->update(order);
	return true;
}

bool CFrelementSet::print(int verbosityLevel)
{
	for (int i = 0;i<numFrelements;i++) frelements[i]->print();
}

bool CFrelementSet::load(const char* name)
{
}

bool CFrelementSet::save(const char* name)
{
	FILE* file = fopen(name,"w+");
	fwrite(&numFrelements,sizeof(int),1,file);
	for (int i = 0;i<numFrelements;i++) frelements[i]->save(file);
	fclose(file);
}
