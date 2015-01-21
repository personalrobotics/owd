#include "Motor.hh"
#include <stdio.h>
#include <algorithm>

bool Motor::cogging_data_lessthan(const std::pair<double,double> &p1, const std::pair<double,double> &p2) {
  return (p1.first < p2.first);
}

double Motor::cogging_comp_torque(double angle) {
  if (cogging_torques.size() == 0) {
    return 0;
  }
  // reverse the offset to get to the true rotor position
  angle -= offset;
  while (angle > 2*M_PI) {
    angle -= 2*M_PI;
  }
  while (angle < 0) {
    angle += 2*M_PI;
  }
  std::pair<double,double> lookup(angle,0);
  std::vector<std::pair<double,double> >::iterator it2
    =std::lower_bound(cogging_torques.begin(),
		      cogging_torques.end(),
		      lookup,
		      cogging_data_lessthan);
  if (it2 == cogging_torques.end()) {
    printf ("Error: could not find lower bound; angle is beyond range of loaded cogging data\n");
    throw -1;
  } else if (it2->first == angle) {
    if (fabs(it2->second) > 0.085/3.0) {
      printf("Illegal value for exact match of angle %f: %f\n",
	     angle,
	     it2->second);
      throw -1;
    }
    return -it2->second;
  } else if (it2 == cogging_torques.begin()) {
    printf("Error: angle is below range of loaded cogging data\n");
    throw -1;
  }
  std::vector<std::pair<double,double> >::iterator it1(it2-1);
  if (it1 == cogging_torques.end()) {
    // don't understand why this would ever happen, because we've already
    // checked to make sure that it2 is not pointing to the first element,
    // but I feel better checking for it anyway.
    printf("something weird happened with the cogging torque iterator\n");
    throw -1;
  }
  double angle_dist = it2->first - it1->first;
  double torque_dist = it2->second - it1->second;
  //printf("interpolating between t=%f at %f and t=%f at %f\n",
  //       it1->second,it1->first,it2->second,it2->first);
  double c=it1->second + torque_dist * ((angle - it1->first)/angle_dist);
  if (fabs(c) > 0.085/3.0) {
    printf("Illegal value for angle %f, interpolated from t=%f at %f and t=%f at %f\n",
	   angle,
	   it1->second,
	   it1->first,
	   it2->second,
	   it2->first);
    throw -1;
  }
  return -c;
}

double Motor::cogging_comp_torque() {
  return cogging_comp_torque(q);
}

bool Motor::load_cogging_data(const char *filename) {
  cogging_torques.clear();
  if (FILE *f = fopen(filename,"r")) {
    double angle, torque;
    int lines(0);
    char line[100];
    int count;
    while (fgets(line,100,f)) {
      if (sscanf(line,"%lf %lf %d\n",
		 &angle, &torque, &count) == 3) {
	++lines;
	while (angle < 0) {
	  angle += 2*M_PI;
	}
	while (angle > 2*M_PI) {
	  angle -= 2*M_PI;
	}
	cogging_torques.push_back(std::pair<double,double>(angle,torque));
      } else {
	fprintf(stderr,"Could not parse line \"%s\" from cogging compensation file \"%s\"\n",line, filename);
      }
    }
    fclose(f);
    if (cogging_torques.size() == 0) {
      printf("Warning: no cogging data read from file %s\n",filename);
      return false;
    } 
  } else {
    printf("Could not open cogging torque data file %s\n",filename);
    return false;
  }

  // add slots for the very beginning and very end before we sort, so that
  // we don't have to insert the first one afterwards.
  cogging_torques.push_back(std::pair<double,double>(-0.00001,0));
  cogging_torques.push_back(std::pair<double,double>(2*M_PI+0.00001,0));

  // sort by rotor angle
  std::sort(cogging_torques.begin(),
	    cogging_torques.end(),
	    cogging_data_lessthan);

  // calculate a value for the points at the beginning and end of the
  // range by interpolating between the first and last actual values as if
  // the angle wrapped around from 2pi to zero.
  std::vector<std::pair<double,double> >::iterator zero=cogging_torques.begin();
  if (zero->first != -0.00001) {
    printf("Something weird happened with the beginning of our vector, and the first point isn't what we expected\n");
    cogging_torques.clear();
    return false;
  }
  std::vector<std::pair<double,double> >::iterator two_pi=cogging_torques.end()-1;
  if (two_pi->first != 2*M_PI+0.00001) {
    printf("Something weird happened with the end of our vector, and the last point isn't what we expected\n");
    cogging_torques.clear();
    return false;
  }
  std::vector<std::pair<double,double> >::iterator firstpt=zero+1;
  std::vector<std::pair<double,double> >::iterator lastpt=two_pi-1;
  // this is the distance from the last point to the first point, if we
  // go forward and wrap at 2pi
  double angle_dist = 2*M_PI - lastpt->first + firstpt->first;
  // this is the difference between the two recorded torques
  double torque_dist = lastpt->second - firstpt->second;
  // calculate the value at exactly the 0 (or 2pi) point:
  double newvalue = firstpt->second + firstpt->first / angle_dist * torque_dist;
  // set our added first and last points to the new value
  zero->second = two_pi->second = newvalue;

  printf("Loaded %zd points of cogging compensation data for motor %d\n",
	   cogging_torques.size()-2, ID);
  return true;
}
