#ifndef __HOLONOMIC_H__
#define __HOLONOMIC_H__

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

typedef enum
{
	HOLD = 0x00,
	FRONT = 0x01,
	BACK = 0x02,
	RIGHT = 0x04,
	LEFT = 0x08,
	TURN = 0x03,
	FREE = 0x0f
}
ROBOT_DIR;

typedef struct
{
	int fd; ///< file descriptor to access to pca9685
	ROBOT_DIR dir; ///< move dir
	uint32_t steps; ///< nb steps need to be done
	bool fullStep; ///< half step or full step used ?
	struct
	{
		uint16_t value;
		struct
		{
			uint8_t right:3,
				left:3;
		}
		front, back;
	}
	stepper;

	pthread_cond_t doneCond;
	pthread_mutex_t doneMutex;

	pthread_cond_t startCond; ///< not used outside of lib
	pthread_mutex_t startMutex; ///< only for internal purposes

	pthread_mutex_t *busMutex;
	pthread_t thread;
}
robot_t;

int holonomicInit ( robot_t *r, bool useThread, pthread_mutex_t *busMutex );
void calcNextStep ( robot_t *r );

#endif