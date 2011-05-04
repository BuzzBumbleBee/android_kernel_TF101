#ifndef _PROXIMITY_H
#define _PROXIMITY_H

extern int prox_lds6202_enable(void);
extern int prox_lds6202_disable(void);

int init_proximity(void);
void free_proximity(void);
void ril_request_proxi(int state);

#endif

