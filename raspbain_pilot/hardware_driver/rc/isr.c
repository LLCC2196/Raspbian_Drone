#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <poll.h>
#include "isr.h"
#include "rc.h"

int GPIO_ISR_handler = NULL;
struct timeval start;
struct timeval end;
#define	BLOCK_SIZE		(4*1024)
#define	TIMER_CONTROL	(0x408 >> 2)
#define	TIMER_PRE_DIV	(0x41C >> 2)
#define	TIMER_IRQ_RAW	(0x410 >> 2)
static pthread_mutex_t pinMutex ;
static volatile int    pinPass = -1 ;
static volatile unsigned long *gpio ;
static volatile unsigned long *pwm ;
static volatile unsigned long *clk ;
static volatile unsigned long *pads ;
static volatile unsigned int RASPBERRY_PI_PERI_BASE = 0x3F000000 ;
static volatile unsigned int GPIO_PADS ;
static volatile unsigned int GPIO_CLOCK_BASE;
static volatile unsigned int GPIO_BASE;
static volatile unsigned int GPIO_TIMER;
static volatile unsigned int GPIO_PWM ;
static void (*isrFunctions [4])(void) ;
static signed long long int epochMilli, epochMicro ;
static int sysFds [4] =
{
  -1, -1, -1, -1
} ;
static int pinToGpio[4] =
{
  17, 18, 27, 22
} ;

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

//static void initialiseEpoch (void)
//{
//  struct timeval tv ;
//
//  gettimeofday (&tv, NULL) ;
//  epochMilli = (signed long long int)tv.tv_sec * (signed long long int)1000    + (signed long long int)(tv.tv_usec / 1000) ;
//  epochMicro = (signed long long int)tv.tv_sec * (signed long long int)1000000 + (signed long long int)(tv.tv_usec) ;
//}

int PiSetup (void)
{
  int   fd ;
  int   boardRev ;
  int   model, rev, mem, maker, overVolted ;
  RASPBERRY_PI_PERI_BASE = 0x3F000000;
    if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    {	printf("Unable to open /dev/gpiomem.") ;
      return -1;
    }
  RASPBERRY_PI_PERI_BASE = 0;

  GPIO_PADS 	  = RASPBERRY_PI_PERI_BASE + 0x00100000 ;
  GPIO_CLOCK_BASE = RASPBERRY_PI_PERI_BASE + 0x00101000 ;
  GPIO_BASE	  = RASPBERRY_PI_PERI_BASE + 0x00200000 ;
  GPIO_TIMER	  = RASPBERRY_PI_PERI_BASE + 0x0000B000 ;
  GPIO_PWM	  = RASPBERRY_PI_PERI_BASE + 0x0020C000 ;

// Map the individual hardware components

//	GPIO:
  gpio = (unsigned long *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE) ;
  if ((signed long)gpio == -1)
  {	printf("mmap (GPIO) failed.") ;
    return -1;
  }
//	PWM

  pwm = (unsigned long *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PWM) ;
  if ((signed long)pwm == -1)
  {	printf("mmap (PWM) failed.") ;
    return -1;
  }

//	Clock control (needed for PWM)

  clk = (unsigned long *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_CLOCK_BASE) ;
  if ((signed long)clk == -1)
  {	printf("mmap (CLOCK) failed.") ;
    return -1;
  }

//	The drive pads

  pads = (unsigned long *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PADS) ;
  if ((signed long)pads == -1)
  {	printf("mmap (PADS) failed.") ;
    return -1;
  }
#ifdef	USE_TIMER
//	The system timer

  timer = (unsigned long *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER) ;
  if ((signed long)timer == -1)
  {
	  printf("mmap (TIMER) failed.");
	  return -1;
  }

// Set the timer to free-running, 1MHz.
//	0xF9 is 249, the timer divide is base clock / (divide+1)
//	so base clock is 250MHz / 250 = 1MHz.

  *(timer + TIMER_CONTROL) = 0x0000280 ;
  *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
  timerIrqRaw = timer + TIMER_IRQ_RAW ;
#endif

  return 0 ;
}

int waitForInterrupt (int pin, int mS)
{
  int fd, x ;
  unsigned char c ;
  struct pollfd polls ;
//  pin = pinToGpio[pin];
  if ((fd = sysFds [pin]) == -1)
    return -2 ;

// Setup poll structure

  polls.fd     = fd ;
  polls.events =  POLLPRI ;	// Urgent data!

// Wait for it ...

  x = poll (&polls, 1, mS) ;

// Do a dummy read to clear the interrupt
//	A one character read appars to be enough.
//	Followed by a seek to reset it.

  (void)read (fd, &c, 1) ;
  lseek (fd, 0, SEEK_SET) ;

  return x ;
}


static void *interruptHandler (void *arg)
{
  int myPin ;
  myPin   = pinPass ;
  pinPass = -1 ;

  for (;;)
    if (waitForInterrupt (myPin, -1) > 0)
      isrFunctions [myPin] () ;

  return NULL ;
}

int PiISR (int pin, int mode, void (*function)(void))
{
  pthread_t threadId ;
  const char *modeS ;
  char fName   [64] ;
  char  pinS [8] ;
  pid_t pid ;
  int   count, i ;
  char  c ;
  int   bcmGpioPin ;

    bcmGpioPin = pinToGpio[pin] ;

// Now export the pin and set the right edge
//	We're going to use the gpio program to do this, so it assumes
//	a full installation of wiringPi. It's a bit 'clunky', but it
//	is a way that will work when we're running in "Sys" mode, as
//	a non-root user. (without sudo)

  if (mode != INT_EDGE_SETUP)
  {
    /**/ if (mode == INT_EDGE_FALLING)
      modeS = "falling" ;
    else if (mode == INT_EDGE_RISING)
      modeS = "rising" ;
    else
      modeS = "both" ;

    sprintf (pinS, "%d", bcmGpioPin) ;

    if ((pid = fork ()) < 0)	// Fail
	{
        perror("fork failed.\n");
 	  return -1;
	}
    if (pid == 0)	// Child, exec
    {
      /**/ if (access ("/usr/local/bin/gpio", X_OK) == 0)
      {
	execl ("/usr/local/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
    perror("execl failed.\n");
	  return -1;
      }
      else if (access ("/usr/bin/gpio", X_OK) == 0)
      {
	execl ("/usr/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
    perror("execl failed.\n");
	  return -1;
      }
      else
	perror("Can't find gpio program.\n") ;
      return -1;
    }
    else		// Parent, wait
      wait (NULL) ;
  }

// Now pre-open the /sys/class node - but it may already be open if
//	we are in Sys mode...

  if (sysFds [pin] == -1)
  {
    sprintf (fName, "/sys/class/gpio/gpio%d/value", bcmGpioPin) ;
    if ((sysFds [pin] = open (fName, O_RDWR)) < 0)
	{
    	perror("unable to open gpio.\n") ;
          return -1;
	}
  }

// Clear any initial pending interrupt

  ioctl (sysFds [pin], FIONREAD, &count) ;
  for (i = 0 ; i < count ; ++i)
    read (sysFds [pin], &c, 1) ;

  isrFunctions [pin] = function ;

  pthread_mutex_lock (&pinMutex) ;
    pinPass = pin ;
    pthread_create (&threadId, NULL, interruptHandler, NULL) ;
    while (pinPass != -1)
      delay (1) ;
  pthread_mutex_unlock (&pinMutex) ;

  return 0 ;
}
void Interrupt_INT_EDGE_RISING(void)
{
	gettimeofday(&start,NULL);
}
void Interrupt_INT_EDGE_FALLING(void)
{
	gettimeofday(&end,NULL);	
	rc_calculate(start,end);
}
