#include "rotation.h"


#define HALF_SQRT_2 0.70710678118654757f
// rotate a vector by a standard rotation, attempting
// to use the minimum number of floating point operations
void rotate(enum Rotation rotation,float *x,float *y,float *z)
{
    float tmp;
    switch (rotation) {
    case ROTATION_NONE:
    case ROTATION_MAX:
        return;
    case ROTATION_YAW_45: {
				tmp = HALF_SQRT_2*(*x - *y);
				*y   = HALF_SQRT_2*(*x + *y);
				*x = tmp;
				return;
			}	
    case ROTATION_YAW_90: {
				tmp = *x; *x = -*y; *y = tmp;
				return;
			}
    case ROTATION_YAW_135: {
        tmp = -HALF_SQRT_2*(*x + *y);
        *y   =  HALF_SQRT_2*(*x - *y);
        *x = tmp;
				return;
			}        
    case ROTATION_YAW_180:
        *x = -*x; *y = -*y;
        return;
    case ROTATION_YAW_225: {
        tmp = HALF_SQRT_2*(*y - *x);
        *y  = -HALF_SQRT_2*(*x + *y);
        *x = tmp;
        return;
    }
    case ROTATION_YAW_270: {
        tmp = *x; *x = *y; *y = -tmp;
        return;
    }
    case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2*(*x + *y);
        *y  = HALF_SQRT_2*(*y - *x);
        *x = tmp;
        return;
    }
    case ROTATION_ROLL_180: {
        *y = -*y; *z = -*z;
        return;
    }
    case ROTATION_ROLL_180_YAW_45: {
        tmp = HALF_SQRT_2*(*x + *y);
        *y  = HALF_SQRT_2*(*x - *y);
        *x = tmp; *z = -*z;
        return;
    }
    case ROTATION_ROLL_180_YAW_90: {
        tmp = *x; *x = *y; *y = tmp; *z = -*z;
        return;
    }
    case ROTATION_ROLL_180_YAW_135: {
        tmp = HALF_SQRT_2*(*y - *x);
        *y   = HALF_SQRT_2*(*y + *x);
        *x = tmp; *z = -*z;
        return;
    }
    case ROTATION_PITCH_180: {
        *x = -*x; *z = -*z;
        return;
    }
    case ROTATION_ROLL_180_YAW_225: {
        tmp = -HALF_SQRT_2*(*x + *y);
        *y   =  HALF_SQRT_2*(*y - *x);
        *x = tmp; *z = -*z;
        return;
    }
    case ROTATION_ROLL_180_YAW_270: {
        tmp = *x; *x = -*y; *y = -tmp; *z = -*z;
        return;
    }
    case ROTATION_ROLL_180_YAW_315: {
        tmp =  HALF_SQRT_2*(*x - *y);
        *y   = -HALF_SQRT_2*(*x + *y);
        *x = tmp; *z = -*z;
        return;
    }
    case ROTATION_ROLL_90: {
        tmp = *z; *z = *y; *y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_45: {
        tmp = *z; *z = *y; *y = -tmp;
        tmp = HALF_SQRT_2*(*x - *y);
        *y   = HALF_SQRT_2*(*x + *y);
        *x = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_90: {
        tmp = *z; *z = *y; *y = -tmp;
        tmp = *x; *x = -*y; *y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_135: {
        tmp = *z; *z = *y; *y = -tmp;
        tmp = -HALF_SQRT_2*(*x + *y);
        *y   =  HALF_SQRT_2*(*x - *y);
        *x = tmp;
        return;
    }
    case ROTATION_ROLL_270: {
        tmp = *z; *z = -*y; *y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_45: {
        tmp = *z; *z = -*y; *y = tmp;
        tmp = HALF_SQRT_2*(*x - *y);
        *y   = HALF_SQRT_2*(*x + *y);
        *x = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_90: {
        tmp = *z; *z = -*y; *y = tmp;
        tmp = *x; *x = -*y; *y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_135: {
        tmp = *z; *z = -*y; *y = tmp;
        tmp = -HALF_SQRT_2*(*x + *y);
        *y  =  HALF_SQRT_2*(*x - *y);
        *x = tmp;
        return;
    }
    case ROTATION_PITCH_90: {
        tmp = *z; *z = -*x; *x = tmp;
        return;
    }
    case ROTATION_PITCH_270: {
        tmp = *z; *z = *x; *x = -tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_90: {
        *z = -*z;
        tmp = -*x; *x = -*y; *y = tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_270: {
        *x = -*x; *z = -*z;
        tmp = *x; *x = *y; *y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_90: {
        tmp = *z; *z = *y; *y = -tmp;
        tmp = *z; *z = -*x; *x = tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_90: {
        *y = -*y; *z = -*z;
        tmp = *z; *z = -*x; *x = tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_90: {
        tmp = *z; *z = -*y; *y = tmp;
        tmp = *z; *z = -*x; *x = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180: {
        tmp = *z; *z = *y; *y = -tmp;
        *x = -*x; *z = -*z;
        return;
    }
    case ROTATION_ROLL_270_PITCH_180: {
        tmp = *z; *z = -*y; *y = tmp;
        *x = -*x; *z = -*z;
        return;
    }
    case ROTATION_ROLL_90_PITCH_270: {
        tmp = *z; *z = *y; *y = -tmp;
        tmp = *z; *z = *x; *x = -tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_270: {
        *y = -*y; *z = -*z;
        tmp = *z; *z = *x; *x = -tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_270: {
        tmp = *z; *z = -*y; *y = tmp;
        tmp = *z; *z = *x; *x = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
        tmp = *z; *z = *y; *y = -tmp;
        *x = -*x; *z = -*z;
        tmp = *x; *x = -*y; *y = tmp;
        return;
    }
		case ROTATION_ROLL_90_YAW_270: {
        tmp = *z; *z = *y; *y = -tmp;
        tmp = *x; *x = *y; *y = -tmp;
        return;
    }
	}
}

