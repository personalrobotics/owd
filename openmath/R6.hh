/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openman is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openman is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <iostream>
#include <iomanip>
#include <string.h>

#include "R3.hh"

#ifndef __R6_HH__
#define __R6_HH__

using namespace std;

class R6{
private:
  double x[6];

public:

  static const int VX =  0;
  static const int VY =  1;
  static const int VZ =  2;
  static const int WX =  3;
  static const int WY =  4;
  static const int WZ =  5;

  R6(){clear();}
  R6(const R3& v, const R3& w)
  {x[0]=v[0];  x[1]=v[1];  x[2]=v[2];  x[3]=w[0];  x[4]=w[1];  x[5]=w[2];}
  R6(double x0, double x1, double x2, double x3, double x4, double x5)
  {x[0]=x0;    x[1]=x1;    x[2]=x2;    x[3]=x3;    x[4]=x4;    x[5]=x5;}

  double norm() const {return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] +
				   x[3]*x[3] + x[4]*x[4] + x[5]*x[5]);}

  R3 v() const { return R3(x[0],x[1],x[2]);  }
  R3 w() const { return R3(x[3],x[4],x[5]);  }

  void  clear(){bzero(x, 6*sizeof(double));}
  
  operator       double* ()        {return x;}
  operator const double* ()  const {return x;}
  
  double operator [] (int i) const {return x[i];}
  
  friend R6 operator * (double s, const R6& x)
  {return R6(s*x[0],  s*x[1],  s*x[2],  s*x[3],  s*x[4],  s*x[5]);}
  friend R6 operator * (const R6& r, double s) {return s*r;}
  friend R6 operator / (const R6& r, double s) {return (1.0/s)*r;}

  friend R6 operator + (const R6& r1, const R6& r2)
  {return R6(r1[0]+r2[0], r1[1]+r2[1], r1[2]+r2[2], 
	     r1[3]+r2[3], r1[4]+r2[4], r1[5]+r2[5]);}

  friend R6 operator - (const R6& r1, const R6& r2)
  {return R6(r1[0]-r2[0], r1[1]-r2[1], r1[2]-r2[2], 
	     r1[3]-r2[3], r1[4]-r2[4], r1[5]-r2[5]);}

  friend ostream& operator <<  (ostream& s, const R6& r){
    int p;
    p = cout.precision();
    cout.precision(12);
    cout.setf(ios::fixed, ios::floatfield);
    cout.setf(ios::right, ios::adjustfield);
    
    cout << setw(22) << r[0] 
	 << setw(22) << r[1] 
	 << setw(22) << r[2]
	 << setw(22) << r[3]
	 << setw(22) << r[4]
	 << setw(22) << r[5];
    
    cout.setf(ios::left, ios::adjustfield);
    cout.precision(p);
    return s;
  }
};

#endif
