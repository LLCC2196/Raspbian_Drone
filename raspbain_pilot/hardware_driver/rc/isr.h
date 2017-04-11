#ifndef ISR_H_
#define ISR_H_

#define	INT_EDGE_SETUP		0
#define	INT_EDGE_FALLING	1
#define	INT_EDGE_RISING		2
#define	INT_EDGE_BOTH		3

extern int PiISR (int pin, int mode, void (*function)(void));
extern int  waitForInterrupt    (int pin, int mS) ;
void delay (unsigned int howLong);
int PiSetup (void);
void Interrupt_INT_EDGE_FALLING(void);
void Interrupt_INT_EDGE_RISING(void);
//static void initialiseEpoch (void);
#endif /* ISR_H_ */
