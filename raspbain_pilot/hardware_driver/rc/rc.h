#ifndef RC_H_
#define RC_H_

int RC_RECIVED;
int RC[8];
void rc_calculate(struct timeval start,struct timeval end);
int RC_Init(void);
#endif /* RC_H_ */
