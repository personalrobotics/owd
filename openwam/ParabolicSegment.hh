/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#ifndef PARABOLICSEGMENT_H
#define PARABOLICSEGMENT_H

#define TRAJ_TOLERANCE 0.003f

class ParabolicSegment {
public:
    int start_index, end_index;
    double start_pos, end_pos;
    double start_time, end_time;
    double time_a, time_v;  // times spent in initial/final accel and vel
    double max_vel, accel;
    typedef enum {
        DOWN = -1,
        CONST = 0,
        UP = 1
    } Direction;
    Direction dir;

    ParabolicSegment(int start_i,
                     double start_t,
                     double first_p,
                     double second_p);

    void fit_curve(double mv, double a);

    void refit_curve(double max_v, double max_a, double new_end_time, double new_accel_time);

    void evaluate(double *y, double *yd, double *ydd, double t);

    double calc_time(double value);

    bool inflection(double current_pos, double next_pos);

    void dump();
};


#endif // PARABOLICSEGMENT_H

