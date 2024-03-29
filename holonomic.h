#ifndef __HOLONOMIC_H__
#define __HOLONOMIC_H__

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

////////////////////////////////////////////////////////////////////////////////
/// \copiright ox223252, 2019
///
/// This program is free software: you can redistribute it and/or modify it
///     under the terms of the GNU General Public License published by the Free
///     Software Foundation, either version 2 of the License, or (at your
///     option) any later version.
///
/// This program is distributed in the hope that it will be useful, but WITHOUT
///     ANY WARRANTY; without even the implied of MERCHANTABILITY or FITNESS FOR
///     A PARTICULAR PURPOSE. See the GNU General Public License for more
///     details.
///
/// You should have received a copy of the GNU General Public License along with
///     this program. If not, see <http://www.gnu.org/licenses/>
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// \file freeOnExit.h
/// \brief lib created to manage an holonomic robot with four steppers motor and
///     four menaum well
/// \author ox223252
/// \date 2019-06
/// \copyright GPLv2
/// \version 1.0
/// \warning not tested
/// \bug NONE
////////////////////////////////////////////////////////////////////////////////

/// \tyepdef ROBOT_DIR
typedef enum
{
	HOLD = 0x00,
	FRONT = 0x01,
	BACK = 0x02,
	RIGHT = 0x04,
	LEFT = 0x08,
	TURN = 0x03,
	FREE = 0x0f ///< stop controling stepper wire
}
ROBOT_DIR;

/// \typedef robot_t
typedef struct
{
	int fd; ///< file descriptor to access to pca9685
	ROBOT_DIR dir; ///< \see ROBOT_DIR move dir
	uint32_t steps; ///< nb steps need to be done
	bool fullStep; ///< half step or full step used ?


	struct
	{
		uint16_t value; ///< value where stepper motor control is stored
			/// value & 0x0f = front right motor
			/// value >> 4 & 0x0f =  front left motor
			/// value >> 8 & 0x0f = back right motor
			/// value >> 12 & 0x0f = back left motor
		struct
		{
			uint8_t right:3,
				left:3;
		}
		front, back;
	}
	stepper;

	pthread_cond_t doneCond; ///< signal used by thread part to inform counter 
		/// reach zero
	pthread_mutex_t doneMutex; ///< mutex associated with doneCond

	struct
	{ ///< user should not use these var
		pthread_cond_t startCond; ///< not used outside of lib
		pthread_mutex_t startMutex; ///< only for internal purposes

		uint32_t delay; ///< time between to steps (µs)
		pthread_mutex_t delayMutex; ///< only for internal purposes
	}
	private;

	pthread_mutex_t *busMutex; ///< mutex used to avoid multiple access to
		/// same hardware bus.
	pthread_t thread;
}
robot_t;

///////////////////////////////////////////////////////////////////////////////
/// \note holonomicResetSteps, holonomicGetSteps, holonomicIni, holonomicSet
///     and holonomicSetDelay function should be used when you set threaded 
///     mode, else you can directly use calcNextStep() and its result stored in
///     r->steppe.value
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// \fn uint32_t holonomicSet ( robot_t * const r, const ROBOT_DIR dir,
///     const uint32_t steps, const bool fullStep );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \param[ in ] dir: new dir of robot \see ROBOT_DIR
/// \param[ in ] steps: number steps should be added to current counter
/// \param[ in ] fullStep: select type of steps should be done full or half
/// \biref this function should be used to update robot_t struct
/// \note thread safe
/// \return hte previous value of steps counter
///////////////////////////////////////////////////////////////////////////////
uint32_t holonomicSet ( robot_t * const r, const ROBOT_DIR dir, 
	const uint32_t steps, const bool fullStep );

///////////////////////////////////////////////////////////////////////////////
/// \fn void holonomicResetSteps ( robot_t * const r );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \biref this function should be used to reset step counter
/// \note thread safe
/// \return hte previous value of steps counter
///////////////////////////////////////////////////////////////////////////////
uint32_t holonomicResetSteps ( robot_t * const r );

///////////////////////////////////////////////////////////////////////////////
/// \fn uint32_t holonomicGetSteps ( robot_t * const r );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \biref this function should be used to get step counter
/// \note thread safe
/// \return thee steps counter value
///////////////////////////////////////////////////////////////////////////////
uint32_t holonomicGetSteps ( robot_t * const r );

///////////////////////////////////////////////////////////////////////////////
/// \fn uint32_t holonomicSetDelay ( robot_t * const r, const uint32_t delay );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \param[ in ] delay: time between two steps
/// \biref this function allow to change timming two step in threaded mode
/// \note thread safe
/// \return the previous delay
///////////////////////////////////////////////////////////////////////////////
uint32_t holonomicSetDelay ( robot_t * const r, const uint32_t delay );

///////////////////////////////////////////////////////////////////////////////
/// \fn int holonomicInit ( robot_t * const r, const bool useThread,
///     pthread_mutex_t * const busMutex, const int fd  );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \param[ in ] useThread: flag to determin if the system should use automatic
///     thread management
/// \param[ in ] busMutex: pointer to the communication bus with hardware mutex
/// \param[ in ] fd: bus file decriptor used to manage robot
/// \biref function used to set robot_t struct and init thread
/// \return o if ok else see errno
///////////////////////////////////////////////////////////////////////////////
int holonomicInit ( robot_t * const r, const bool useThread, 
	pthread_mutex_t * const busMutex, const int fd  );

///////////////////////////////////////////////////////////////////////////////
/// \fn void calcNextStep ( robot_t *r );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \biref fucntion used to calc the next step for your step motor
///////////////////////////////////////////////////////////////////////////////
void calcNextStep ( robot_t * const r );

///////////////////////////////////////////////////////////////////////////////
/// \fn void holonomicWait ( robot_t * const r );
/// \param[ in ] r: pointer on robot_t struct \see robot_t
/// \biref this function will return once last step will be done
//////////////////////////////////////////////////////////////////////////////
void holonomicWait ( robot_t * const r );

#endif