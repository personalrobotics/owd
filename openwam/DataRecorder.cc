#ifndef DATARECORDER_CC
#define DATARECORDER_CC

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

template<class T> class DataRecorder {
public:

  DataRecorder(int records);
  void add(const std::vector<T> &v);
  void reset();
  bool dump(const char *fname);

  unsigned int recsize, bufsize, count;
  T *data;
};

template<class T> inline DataRecorder<T>::DataRecorder (int records) : recsize(0),count(0) {
  // just take an initial guess at number of fields (32) in order
  // to size the initial buffer.  During the add() we'll make sure
  // we don't go past the end.
  bufsize=32*sizeof(T)*records;
  data=(T *)malloc(bufsize);
  if (!data) {
    throw "out of memory";
  }
}

template<class T> inline void DataRecorder<T>::add(const std::vector<T> &v) {
  if (v.size() == 0) {
    return;
  }
  if (recsize==0) {
    recsize=v.size();
  }
  if (v.size() != recsize) {
    throw "mismatched data sizes";
  }
  if ((count+1)*recsize*sizeof(T) < bufsize) {
    memcpy(data+count*recsize,&v[0],recsize*sizeof(T));
    ++count;
  }
}

template<class T> inline void DataRecorder<T>::reset() {
  count=0;
  recsize=0;
}

template<> inline bool DataRecorder<double>::dump (const char *fname) {
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
	    
#endif // DATARECORDER_CC
