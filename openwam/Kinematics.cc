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

#include "Kinematics.hh"
#include <stdlib.h>
#include "Plugin.hh" // for debugging the Jacobian

extern "C" {
  void dpotrf_(char *uplo, int *n, double *a, int *lda, int *info);
  void dpotri_(char *uplo, int *n, double *a, int *lda, int *info);
  void dgemm_(char *transa, char *transb, int *m, int *n, int *k,double *alpha,
	      double *a, int *lda, double *b, int *ldb, double *beta,double *c,
	      int *ldc);
  void dgemv_(char *trans, int *m, int *n, double *alpha, double *a, int *lda,
	      double *x, int *incx, double *beta, double *y, int *incy);
}

namespace OWD {

  // allocate and initialize the static members
  char Kinematics::TRANST = 'T';  
  char Kinematics::TRANSN = 'N';
  double Kinematics::ALPHA = 1.0; 
  double Kinematics::BETA = 0.0;
  char Kinematics::UPLO = 'L';
  int Kinematics::LD_Jacobian=NDIMS;
  int Kinematics::LD_JacobianPseudoInverse=7;
  int Kinematics::INC=1;
  bool Kinematics::valid_Jacobian(false);
  double Kinematics::JacobianEE[NJOINTS][NDIMS];
  double Kinematics::Jacobian0[NJOINTS][NDIMS];

  void Kinematics::Jacobian0_times_vector(double *q, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    // multiply by the supplied vector
    int M=NDIMS;
    int N=NJOINTS;
    dgemv_(&TRANSN, &M,  &N, &ALPHA,
	   &Jacobian0[0][0], &LD_Jacobian,
	   (double*)q,  &INC, &BETA,
	   out, &INC);
  }

  /*
  void Kinematics::JacobianPseudoInverse_times_vector(R6 &v, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    // multiply by the supplied vector
    int M=NJOINTS;
    int N=NDIMS;
    dgemv_(&TRANSN, &M,  &N, &ALPHA,
	   &JacobianPseudoInverse[0][0], &LD_JacobianPseudoInverse,
	   (double*)v,  &INC, &BETA,
	   out, &INC);
  }
  */

  void Kinematics::Jacobian0Transpose_times_vector(R6 &v, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    int M=NDIMS;
    int N=NJOINTS;
    dgemv_(&TRANST, &M,  &N, &ALPHA,
	   &Jacobian0 [0][0], &LD_Jacobian,
	   (double*)v,  &INC, &BETA,
	   out, &INC);
  }
    
  SE3 Kinematics::forward_kinematics(Link* link){
    SE3 H = (SE3)link[Link::L0];

    for(int l=Link::L1; l<=Link::Ln; l++)
      H = H * (SE3)link[l];

    return H;
  }

  void Kinematics::update_jacobians(Link *links) {
    // all the matrices are in column-major order, for Fortran

    Jacobians(JacobianEE, Jacobian0, links);

    /*


    double inv_J_JT[NDIMS][NDIMS];
    int LD_inv_J_JT=6;
    
    // compute J * Jtranspose, factorize, and invert
    {
      int M=NDIMS;
      int N=NDIMS;
      int K=NJOINTS;
      dgemm_(&TRANSN, &TRANST, &M, &N, &K, &ALPHA, 
	     &Jacobian [0][0], &LD_Jacobian, 
	     &Jacobian [0][0], &LD_Jacobian, &BETA, 
	     &inv_J_JT [0][0], &LD_inv_J_JT);
    }
    {
      int INFO;
      int N=NDIMS;
      dpotrf_(&UPLO, &N, &inv_J_JT[0][0], &LD_inv_J_JT, &INFO);
      if(INFO<0) {
	valid_Jacobian=false;
	throw "update_jacobian: bad argument to dpotrf";
      } else if(0<INFO) {
	valid_Jacobian=false;
	throw "update_jacobian: matrix passed to dpotrf is not positive definite";
      }
      dpotri_(&UPLO, &N, &inv_J_JT[0][0], &LD_Jacobian, &INFO);
      if(INFO<0) {
	valid_Jacobian=false;
	throw "update_jacobian: bad argument to dpotri";
      } else if(0<INFO) {
	valid_Jacobian=false;
	throw "update_jacobian: matrix passed to dpotrf is singular";
      }
    }

    // compute pseudo-inverse = Jtranspose * inv(J*Jtranspose)
    {
      int M=NJOINTS;
      int N=NDIMS;
      int K=NDIMS;
      dgemm_(&TRANST, &TRANSN, &M, &N, &K, &ALPHA, 
	     &Jacobian              [0][0], &LD_Jacobian,
	     &inv_J_JT              [0][0], &LD_inv_J_JT, &BETA, 
	     &JacobianPseudoInverse [0][0], &LD_JacobianPseudoInverse);
    }

*/
    valid_Jacobian=true;
  }


  /*
   * Body manipulator Jacobian
   * Paul IEEE SMC 11(6) 1981
   * 
   * BIG FAT WARNING: The jacobians are in column major (for Fortran)
   */
  void Kinematics::Jacobians(double JN[][Kinematics::NDIMS], double J0[][Kinematics::NDIMS], Link *links){
    SE3 U;

    for(int l=Link::Ln; l>=Link::L1; --l){
      U = ((SE3)links[l]) * U;

      if (JN) {
	JN[l-1][0] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
	JN[l-1][1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
	JN[l-1][2] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
	JN[l-1][3] = U[SE3::NZ];
	JN[l-1][4] = U[SE3::OZ];
	JN[l-1][5] = U[SE3::AZ];
      }
    }

    // Rotate the end-effector Jacobian to the base frame
    if (J0) {
      for (int i=0; i<Kinematics::NJOINTS; ++i) {
	// upper three rows, translation
	J0[i][0] = JN[i][0]*U[SE3::R11] + JN[i][1]*U[SE3::R12] + JN[i][2]*U[SE3::R13];
	J0[i][1] = JN[i][0]*U[SE3::R21] + JN[i][1]*U[SE3::R22] + JN[i][2]*U[SE3::R23];
	J0[i][2] = JN[i][0]*U[SE3::R31] + JN[i][1]*U[SE3::R32] + JN[i][2]*U[SE3::R33];

	// bottom three rows, rotation
	J0[i][3] = JN[i][3]*U[SE3::R11] + JN[i][4]*U[SE3::R12] + JN[i][5]*U[SE3::R13];
	J0[i][4] = JN[i][3]*U[SE3::R21] + JN[i][4]*U[SE3::R22] + JN[i][5]*U[SE3::R23];
	J0[i][5] = JN[i][3]*U[SE3::R31] + JN[i][4]*U[SE3::R32] + JN[i][5]*U[SE3::R33];
      }
    }
  }

  void Kinematics::JacobianDB(double J[][6], Link *links){
    SE3 U,EE;
    R3 v,axis,anchor,tEE;

    /*
      for(int l=Link::L1; l <= Link::Ln; l++)
      {        
      EE = (((SE3)links[l])^-1) * EE;

      }*/

    EE = forward_kinematics(links);
    tEE = R3(EE[3],EE[7],EE[11]);

    U = (SE3)links[Link::L0];
    for(int l=Link::L1; l <= Link::Ln; l++)
      {

        axis = R3(U[2],U[6],U[10]);
        anchor = R3(U[3],U[7],U[11]);
        
        v = axis^(anchor - tEE);

        J[l-1][0] = v[0];
        J[l-1][1] = v[1];
        J[l-1][2] = v[2];
        J[l-1][3] = 0;
        J[l-1][4] = 0;
        J[l-1][5] = 0;

        U = U * (SE3)links[l];

      }
  }



  // row-major version of the EE Jacobian

  void Kinematics::JacobianN(double J[][7], Link *links){
    SE3 U;

    for(int l=Link::Ln; Link::L1<=l; l--){
      U = ((SE3)links[l]) * U;

      J[0][l-1] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
      J[1][l-1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
      J[2][l-1] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
      J[3][l-1] = U[SE3::NZ];
      J[4][l-1] = U[SE3::OZ];
      J[5][l-1] = U[SE3::AZ];
    }
  }

}; // namespace OWD
