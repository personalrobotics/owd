#include "TrajType.hh"

TrajPoint& TrajPoint::operator=(const TrajPoint &rhs) {
    *(JointPos*)this = *(const JointPos*)&rhs;
    absolute_time = rhs.absolute_time;
    blend_radius = rhs.blend_radius;
    return *this;
}

bool TrajPoint::operator!=(const TrajPoint &rhs) {
  if (fabs(blend_radius - rhs.blend_radius) > 0.01f) {
    return true;
  }
  for (unsigned int i = 0; i < this->size(); ++i) {
    if (fabs(this->operator[](i) - rhs[i]) > 0.01f ) {
      return true;
    }
  }
  return false;
}

void TrajPoint::fprint(FILE *fout) const {
    fprintf(fout,"%2.8f",absolute_time);
    for (unsigned int i = 0; i < this->size(); ++i) {
        fprintf(fout,", % 2.8f",this->operator[](i));
    }
    fprintf(fout,"\n");
}

void JointPos::cpy(double *out) const {
  for (unsigned int i=0; i<size(); ++i) {
    out[i]=this->operator[](i);
  }
}

double JointPos::length() const {
  double sum=0;
  for (unsigned int i=0; i<size(); ++i) {
    sum += pow(this->operator[](i),2);
  }
  return sqrt(sum);
}

void JointPos::dump() const {
  printf("[ ");
  for (unsigned int i=0; i<size(); ++i) {
    printf("%2.3f ",this->operator[](i));
  }
  printf("]\n");
}

void JointPos::SetFromArray(const unsigned int arraysize, const double *p) {
  this->resize(arraysize);
  for (unsigned int i=0; i<arraysize; ++i) {
    this->operator[](i) = p[i];
  }
}

