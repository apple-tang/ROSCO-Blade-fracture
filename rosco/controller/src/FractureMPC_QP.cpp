// SPDX-License-Identifier: Apache-2.0
#include <qpOASES.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#ifdef _WIN32
#define FMPC_EXPORT __declspec(dllexport)
#else
#define FMPC_EXPORT
#endif
extern "C" FMPC_EXPORT int fmpc_qp_solve(int n, int m, const double* h,
    const double* g, const double* a, const double* lb, const double* ub,
    const double* la, const double* ua, int iterations, double* x, double* violation) {
    using namespace qpOASES;
    if (n < 1 || n > 80 || m < 1 || m > 320 || iterations < 1) return -1;
    // Fortran column-major matrices are converted explicitly for qpOASES.
    std::vector<real_t> H(n*n), A(m*n), G(n), L(n), U(n), LA(m), UA(m), X(n);
    for (int i=0;i<n;++i) {
        G[i]=g[i]; L[i]=lb[i]; U[i]=ub[i];
        if (!std::isfinite(G[i]) || L[i]>U[i]) return -2;
        for (int j=0;j<n;++j) {
            H[i*n+j]=h[i+j*n]; if (!std::isfinite(H[i*n+j])) return -2;
        }
    }
    for (int i=0;i<m;++i) {
        LA[i]=la[i]; UA[i]=ua[i]; if (LA[i]>UA[i]) return -2;
        for (int j=0;j<n;++j) { A[i*n+j]=a[i+j*m]; if(!std::isfinite(A[i*n+j])) return -2; }
    }
    QProblem qp(n,m,HST_POSDEF);
    Options options; options.setToMPC(); options.printLevel=PL_NONE; qp.setOptions(options);
    int budget=iterations;
    const returnValue rc=qp.init(H.data(),G.data(),A.data(),L.data(),U.data(),LA.data(),UA.data(),budget);
    if (rc!=SUCCESSFUL_RETURN) return static_cast<int>(rc);
    if (qp.getPrimalSolution(X.data())!=SUCCESSFUL_RETURN) return -3;
    *violation=0;
    for(int j=0;j<n;++j) {
        x[j]=X[j]; if(!std::isfinite(x[j])) return -4;
        *violation=std::max(*violation,std::max(L[j]-x[j],x[j]-U[j]));
    }
    for(int i=0;i<m;++i) {
        double ax=0;for(int j=0;j<n;++j)ax+=A[i*n+j]*x[j];
        *violation=std::max(*violation,std::max(LA[i]-ax,ax-UA[i]));
    }
    return *violation>1e-6 ? -5 : 0;
}
