#include "DataRecorder.hh"
#include <stdlib.h>
#include <string.h>


DataRecorder::DataRecorder(int records) : recsize(0),count(0) {
  // just take an initial guess at number of fields (32) in order
  // to size the initial buffer.  During the add() we'll make sure
  // we don't go past the end.
  bufsize=32*sizeof(double)*records;
  data=(double *)malloc(bufsize);
  if (!data) {
    throw "out of memory";
  }
}

void DataRecorder::add(const std::vector<double> &v) {
  if (v.size() == 0) {
    return;
  }
  if (recsize==0) {
    recsize=v.size();
  }
  if (v.size() != recsize) {
    throw "mismatched data sizes";
  }
  if ((count+1)*recsize*sizeof(double) < bufsize) {
    memcpy(data+count*recsize,&v[0],recsize*sizeof(double));
    ++count;
  }
}

void DataRecorder::reset() {
  count=0;
  recsize=0;
}

bool DataRecorder::dump(const char *fname) {
  FILE *csv = fopen(fname,"w");
  if (csv) {
    for (unsigned int i=0; i<count; ++i) {
      fprintf(csv,"%3.4f",data[i*recsize]);
      for (unsigned int j=1; j<recsize; ++j) {
	fprintf(csv,", %3.4f",data[i*recsize+j]);
      }
      fprintf(csv,"\n");
    }
    fclose(csv);
    return true;
  } else {
    return false;
  }
}
	    
