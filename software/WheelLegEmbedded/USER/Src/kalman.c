#include "kalman.h"

void kalman_Init(kalman_filter_t *p, float T_Q, float T_R) {
    p->X_last = 0.0f;
    p->P_last = 0.0f;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1.0f;
    p->H = 1.0f;
    p->X_mid = p->X_last;
}

float Kalman_Filter(kalman_filter_t *p, float dat) {
    p->X_mid = p->A * p->X_last;                    //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A * p->P_last + p->Q;             //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);           //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;              //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                           //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}
