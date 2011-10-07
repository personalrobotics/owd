#include <deque>

template<class value_t> class Butterworth {
public:
  unsigned int order;
  double *A;
  double *B;
  std::deque<value_t> X;
  std::deque<value_t> Y;
  
  inline void reset() {
    X.resize(0);
    Y.resize(0);
  }

  Butterworth(unsigned int _order,
	      double cutoff) 
    : order(_order) {
    A=(double*)malloc(sizeof(double) * (order+1));
    B=(double*)malloc(sizeof(double) * (order+1));
    if ((order == 2) && (cutoff==10.0)) {
      // coefficients for a 2nd-order filter with a
      // cutoff of 10hz
      B[0]=0.0036;
      B[1]=0.0072;
      B[2]=0.0036;
      A[0]=1.0;
      A[1]=-1.8227;
      A[2]= 0.8372;
    } else {
      throw "Unable to find coefficients for that filter";
    }
    // coefficients for a 3rd-order filter with a
    // cutoff of 10hz
    /*
      static const unsigned int order=3;
      static const double B1=0.2196e-3;
      static const double B2=0.6588e-3;
      static const double B3=0.6588e-3;
      static const double B4=0.2196e-3;
      static const double A2=-2.7488;
      static const double A3= 2.5282;
      static const double A4=-0.7776;
    */
  }
  
  ~Butterworth() {
    free(A);
    free(B);
  }

  value_t eval(value_t current) {
    if (X.size() != order) {
      // initialize with all current values
      X.resize(order,current);
      Y.resize(order,current);
    } 
    
    value_t output = B[0] * current;
    
    for (unsigned int i=0; i<order; ++i) {
      output += B[i+1]*X[i] - A[i+1]*Y[i];
    }
    
    X.pop_back();
    Y.pop_back();
    X.push_front(current);
    Y.push_front(output);
    return output;
  }
  
};
