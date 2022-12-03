function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively

    C_t = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
           0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
           0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    R = [0.001;0.001;0.001;0.001;0.001;0.001];
    R_t = diag(R);

    K_t = covarEst*(C_t.')/(C_t*covarEst*(C_t.')+R_t);
    covar_curr = covarEst - K_t*C_t*covarEst;
    uCurr = uEst + K_t*(z_t-C_t*uEst);
end