#ifndef DATARECORDER_HH
#define DATARECORDER_HH

#include <vector>
#include <stdio.h>

class DataRecorder {
public:

  DataRecorder(int records);
  void add(const std::vector<double> &v);
  void reset();
  bool dump(const char *fname);

  unsigned int recsize, bufsize, count;
  double *data;
};

#endif // DATARECORDER_HH
