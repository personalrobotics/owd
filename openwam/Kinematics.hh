/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Link.hh"
#include "../openmath/R6.hh"

#ifndef __KINEMATICS_HH__
#define __KINEMATICS_HH__

class WAM;

namespace OWD {

  class Kinematics {

    friend class ::WAM;
    
  public:

    static const int NJOINTS=Link::Ln;
    static const int NDIMS=6;

    /// The column-major Jacobian matrix in the EE frame (NJOINTS x 6)
    static double JacobianEE[][NDIMS];

    /// The column-major Jacobian matrix in the %Link 0 frame (NJOINTS x 6)
    static double Jacobian0[][NDIMS];

    /// \brief Multiply the base-frame Jacobian by the supplied vector
    ///
    /// \param q an array of NJOINTS doubles for the joint values
    /// \param {out} out A pointer to a previously-allocated array of
    ///              NDIMS doubles to hold the result
    ///
    /// This function is typically used to compute the workspace velocities
    /// resulting from the supplied joint velocities
    static void Jacobian0_times_vector(double *q, double *out);

    /// \brief Multiply the base-frame Jacobian Transpose by the
    /// supplied vector
    ///
    /// \param v an R6 vector
    /// \param {out} out A pointer to a previously-allocated array of
    ///              NJOINTS doubles to hold the result
    ///
    /// This function is typically used to compute the joint torques
    /// that will yield the requested workspace forces/torques
    static void Jacobian0Transpose_times_vector(R6 &v, double *out);

  private:

    /// \brief Calculate the endpoint pose
    ///
    /// Runs through the current links, multiplying their
    /// transformation matrices by one another.
    ///
    /// \param link A pointer to the first element in the array of links
    /// \returns The transformation matrix of link7 in terms of link0
    static SE3 forward_kinematics(Link* links);

    /// \brief Pre-calculate the Jacobian and friends
    ///
    /// Calculates the Jacobian and derived matrices based on the
    /// current link values, The resulting column-major matrices are
    /// stored so that multiple functions can ask for them without
    /// having to repeat the calculations each time.
    ///
    /// \param links A pointer to the first element in the array of links
    ///
    /// \attention This function should be called from WAM.cc every time
    ///            we receive updated joint values from the arm.
    static void update_jacobians(Link *links);

    /// \brief body Jacobian for Fortran (column major)
    static void Jacobians(double JEE[][NDIMS], double J0[][NDIMS], Link *links);

    /// \brief 3DOF body Jacobian (column major)
    static void JacobianDB(double J[][6], Link *links);

    /// \brief body Jacobian for C (row major) 
    static void JacobianN(double J[][7], Link *links);

    static char TRANST;
    static char TRANSN;
    static double ALPHA;
    static double BETA;
    static char UPLO;
    static int LD_Jacobian;
    static int LD_JacobianPseudoInverse;
    static int INC;
    static bool valid_Jacobian;
  };

}; // namespace OWD

#endif // __KINEMATICS_HH__
