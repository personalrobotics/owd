#ifndef MACACCELPULSE_H
#define MACACCELPULSE_H

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

class MacAccelElement {
protected:
  double start_p, dist;
  double start_v, a;
  double start_t, dur;

public:
  static const double DTMAX = 0.5;  // time to reach max accel (eq 3.4)
                                    // max jerk = PI*accel/(2*dtmax)
  static const double PI = 3.141592654;

  MacAccelElement() {};
  inline virtual double start_pos() const {return start_p;}
  inline virtual double start_vel() const {return start_v;}
  inline virtual double start_time() const {return start_t;}
  inline virtual double accel() const {return a;}
  inline virtual double duration() const {return dur;}
  inline virtual double distance() const {return dist;}
  inline virtual double end_time() const {return start_t + dur;}
  inline virtual double end_pos() const {return start_p + dist;}

  virtual double end_vel() const =0;
  virtual void set_constant_accel(double t) =0;
  virtual void extend_sustain_time_by(double t) =0;
  virtual void reset(double pos, double vel, double accel, double t) =0;
  virtual void eval(double *y, double *yd, double *ydd, double t) =0;
  virtual void dump() const {
    printf("     start_pos=%2.3f  dist=%2.3f  end_pos=%2.3f\n",
	   start_p,dist,start_p+dist);
    printf("     start_vel=%2.3f  accel=%2.3f\n",start_v,a);
    printf("     start_time=%2.3f  dur=%2.3f  end_time=%2.3f\n",
	   start_t,dur,start_t+dur);
  }

  virtual ~MacAccelElement() {}
};

class MacAccelPulse : public MacAccelElement {
protected:
  double sustain_t;
  double pa, sa, pb, sb; // intermediate positions and speeds
  double end_v;

  void recalc_curve_constants() {
    // total time
    dur = 2*DTMAX + sustain_t;

    // compute the total distance, in stages
    static const double DTMAX2 = pow(DTMAX,2);
    static const double PI2 = pow(PI,2);
    // first, the initial accel rise
    sa = start_v + a*DTMAX/2;
    dist = start_v*DTMAX + a*DTMAX2*(.25-1/PI2);
    pa=start_p + dist;

    // next, the sustain (if any)
    sb = sa + sustain_t*a;
    dist += sustain_t * (sa + 0.5*a*sustain_t);
    pb = start_p + dist;
    
    // finally, the accel fall
    end_v = sb + a*DTMAX/2;
    dist += sb*DTMAX + a*DTMAX2*(.25+1/PI2);
  }
  
public:

  MacAccelPulse(double start_pos, double start_vel, double delta_v,
		double accel, double start_time) {
    if (accel == 0 || delta_v == 0) {
      throw "MacAccelPulse requires non-zero accel and delta_v";
    }
    if ((accel > 0 && delta_v < 0) || (accel<0 && delta_v>0)) {
      throw "MacAccelPulse requires accel and delta_v of the same sign";
    }
    
    start_p = start_pos;
    start_v = start_vel;
    a = accel;
    start_t = start_time;

    if (fabs(delta_v) > fabs(a)*DTMAX) {
      // we need a sustain portion
      sustain_t = (fabs(delta_v) - fabs(a)*DTMAX) / fabs(a);
      printf("MacAccelPulse: adding a sustain of t= %2.3f to achieve delta_v\n",sustain_t);
    } else if (fabs(delta_v) < fabs(a)*DTMAX) {
      // calculate reduced acceleration to achieve velocity change
      printf("MacAccelPulse: reducing accel from %2.3f to %2.3f to achieve requested delta_v\n",
	     a, delta_v/DTMAX);
      a = delta_v/DTMAX;
      sustain_t=0;
    } else {
      // works out perfectly at max accel with no sustain
      printf("MacAccelPulse: no sustain necessary (a*DTMAX=%2.3f, delta_v only %2.3f)\n",fabs(a)*DTMAX,fabs(delta_v));
      sustain_t = 0;
    }
    recalc_curve_constants();
  }

  inline double end_vel() const {
    return end_v;
  }

  void set_constant_accel(double t) {
    sustain_t = t;
    recalc_curve_constants();
  }

  void extend_sustain_time_by(double t) {
    sustain_t += t;
    recalc_curve_constants();
  }

  void reset(double pos, double vel, double accel, double t) {
    start_p = pos; start_v = vel; a = accel; start_t = t;
    recalc_curve_constants();
  }

  void eval(double *y, double *yd, double *ydd, double t) {
    t -= start_t; // normalize to start at t=0
    if (t < 0) {
      t=0;
    }
    if (t > 2*DTMAX + sustain_t) {
      t = 2*DTMAX + sustain_t;
    }
    if ((t>=0) && (t<DTMAX)) {
      // initial rise
      if (y) *y = start_p
	+ start_v*t
	+ a*0.25*t*t
	- a*DTMAX*DTMAX/PI/PI * 0.5*(sin(t*PI/DTMAX - 0.5*PI) +1);
      if (yd) *yd = start_v
	 + a*0.5*t
	- a*0.5*DTMAX/PI * cos(t*PI/DTMAX - 0.5*PI);
      if (ydd) *ydd = a*0.5 * (sin(t*PI/DTMAX - 0.5*PI) +1);
      return;

    } else if (t<(DTMAX + sustain_t)) {
      // linear sustain
      t -= DTMAX;  // adjust t to be the time inside the sustain
      if (y) *y = pa
	+ t * (sa + 0.5*a*t);
      if (yd) *yd = sa + a*t;
      if (ydd) *ydd = a;
      return;

    } else {
      // final fall
      t -= DTMAX + sustain_t; // adjust t to be inside the fall
      if (y) *y = pb
	+ sb*t
	+ a*0.25*t*t
	- a*DTMAX*DTMAX/PI/PI * 0.5*(sin(t*PI/DTMAX + 0.5*PI) -1);
      if (yd) *yd = sb
	+ a*0.5*t
	- a*0.5*DTMAX/PI * cos(t*PI/DTMAX + 0.5*PI);
      if (ydd) *ydd = a*0.5 * (sin(t*PI/DTMAX + 0.5*PI) + 1);
      return;
    }
  }

  void dump() const {
    printf("   MacAccelPulse (a=%2.3f):\n",a);
    printf("     rise from t=%2.3f to t=%2.3f, pos=%2.3f to pos=%2.3f (d=%2.3f)\n",
	   start_t, start_t+DTMAX,start_p,pa,pa-start_p);
    printf("          v=%2.3f to v=%2.3f (delta_v=%2.3f)\n",
	   start_v, sa, sa-start_v);

    if (sustain_t > 0) {
      printf("     sustain from t=%2.3f to t=%2.3f, pos=%2.3f to pos=%2.3f (d=%2.3f)\n",
	     start_t+DTMAX, start_t+DTMAX+sustain_t, pa, pb, pb-pa);
      printf("          v=%2.3f to v=%2.3f (delta_v=%2.3f)\n",
	     sa, sb, sb-sa);
    }
    printf("     fall from t=%2.3f to t=%2.3f, pos=%2.3f to pos=%2.3f (d=%2.3f)\n",
	   start_t+DTMAX+sustain_t, start_t+2*DTMAX+sustain_t,pb,start_p+dist,start_p+dist-pb);
    printf("          v=%2.3f to v=%2.3f (delta_v=%2.3f)\n",
	   sb, end_v, end_v-sb);
  }
    
};

class MacZeroAccel : public MacAccelElement {
protected:
  double vel;

public:
  MacZeroAccel(double start_pos, double start_vel, double end_pos, double start_time) {
    start_p = start_pos;
    start_v = start_vel;
    dist = end_pos - start_pos;
    if (((dist < -0.001) && (start_v >= 0)) ||
	((dist > 0.001) && (start_v <= 0))) {
      printf("dist was %2.3f, start_v was %2.3f\n",dist,start_v);
      throw "MacZeroAccel: velocity sign does not match sign of end-start";
    }
    start_t = start_time;
    a=0;
    dur = dist/start_v;
  }
  
  inline double end_vel() const {
    return start_v;
  }

  void set_constant_accel(double t) {
    dur=t;
  }

  void extend_sustain_time_by(double t) {
    dur += t;
  }

  void reset(double pos, double vel, double accel, double t) {
    start_p = pos;
    start_v = vel;
    start_t = t;
    dur = dist/start_v;
    if (accel != 0) {
      throw "MacZeroAccel: cannot reset to a non-zero accel";
    }
    if (dur < 0) {
      throw "MacZeroAccel: new start velocity is the wrong sign";
    }
  }

  void eval(double *y, double *yd, double *ydd, double t) {
    t -= start_t;
    if (t<0) {
      t = 0;
    }
    if (t>dur) {
      t = dur;
    }
    if (y) *y = start_p + t*start_v;
    if (yd) *yd = start_v;
    if (ydd) *ydd = 0;
    return;
  }

  void dump() const {
    printf("   MacZeroAccel:\n");
    MacAccelElement::dump();
  }
};

#endif // MACACCELPULSE_H
