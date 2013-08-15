//
// Copyright 2013
//
// National Robotics Engineering Center, Carnegie Mellon University
// 10 40th Street, Pittsburgh, PA 15201
// www.rec.ri.cmu.edu
//
// Control Instructions (who can see this code):
// NREC Confidential.  Not for public release unless permission granted
// by program manager.
//
// Usage Rights (who can use this code):
// Usage allowed for all NREC programs with permissions from author and
// program manager.
//
// This notice must appear in all copies of this file and its derivatives.
//
// Created under Program: DRC
//
// History of Significant Contributions (don't put commit logs here):
// 2013-02-13 vandeweg@cmu.edu  Initial working version
// 

#ifndef MTLOGGER_H
#define MTLOGGER_H

#include <vector>

template <class value_t> class MTLogger {
public:
  MTLogger(unsigned int size=1000);
  bool append(const value_t &v, bool overwrite=false);
  bool remove(value_t *v);
private:
  std::vector<value_t> buffer;
  typename std::vector<value_t>::iterator next_append;
  typename std::vector<value_t>::iterator next_remove;
};

template <class value_t> MTLogger<value_t>::MTLogger(unsigned int size) :
    buffer(size),
    next_append(buffer.begin()),
    next_remove(buffer.begin())
  {
  }

template<class value_t> bool MTLogger<value_t>::append(const value_t &v,
						       bool overwrite) {
  typename std::vector<value_t>::iterator new_next_append = next_append+1;
  if (new_next_append == buffer.end()) {
    new_next_append=buffer.begin(); // wrap
  }
  if (new_next_append == next_remove) {
    // vector is full
    if (!overwrite) {
      return false;
    }
    // make room by throwing out the earliest one
    typename std::vector<value_t>::iterator new_next_remove = next_remove+1;
    if (new_next_remove == buffer.end()) {
      new_next_remove = buffer.begin(); // wrap
    }
    next_remove = new_next_remove;
  }
  *next_append = v;
  next_append = new_next_append;
  return true;
}

template<class value_t> bool MTLogger<value_t>::remove(value_t *v) {
  if (next_remove == next_append) {
    // vector is empty
    return false;
  }
  *v = *next_remove;
  if (++next_remove == buffer.end()) {
    next_remove = buffer.begin();
  }
  return true;
}
#endif //  MTLOGGER_H
