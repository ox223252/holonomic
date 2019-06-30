#include <unistd.h>
#include <stdio.h>

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

#include "holonomic.h"

// https://eskimon.fr/tuto-arduino-603-a-petits-pas-le-moteur-pas-à-pas#rotation-par-demi-pas
static const uint8_t stepValues[ ] = {
			// Etape     A1    B1    A2    B2
	0x08,	// Pas n°0   HIGH  LOW   -     - 
	0x0a,	// Pas n°1 ½ HIGH  LOW   HIGH  LOW 
	0x02,	// Pas n°2   -     -     HIGH  LOW 
	0x06,	// Pas n°3 ½ LOW   HIGH  HIGH  LOW 
	0x04,	// Pas n°4   LOW   HIGH  -     - 
	0x05,	// Pas n°5 ½ LOW   HIGH  LOW   HIGH 
	0x01,	// Pas n°6   -     -     LOW   HIGH 
	0x09	// Pas n°7 ½ HIGH  LOW   LOW   HIGH 
};

static void* holonomicMove( void* arg )
{
	robot_t *r = arg;
	
	while ( 1 )
	{
		// wait cond sart

		if ( r->steps )
		{
			calcNextStep ( r );
			r->steps--;

			if ( r->fd )
			{
				if ( r->busMutex )
				{
					pthread_mutex_lock ( r->busMutex );
					pthread_setcancelstate (PTHREAD_CANCEL_DISABLE, NULL );
					// secure write on bus
					printf ( "NO BUS SET : %6d %#06x\n", r->steps, r->stepper.value );
					pthread_mutex_unlock ( r->busMutex );
					pthread_setcancelstate ( PTHREAD_CANCEL_ENABLE, NULL );
				}
				else
				{
					// unsecured write on bus
					printf ( "NO BUS SET : %6d %#06x\n", r->steps, r->stepper.value );
				}
			}
			else
			{
				printf ( "NO FD SET : %6d %#06x\n", r->steps, r->stepper.value );
			}

			pthread_mutex_lock ( &(r->private.delayMutex) );
			uint32_t tmp = r->private.delay;
			pthread_mutex_unlock ( &(r->private.delayMutex) );
			usleep ( tmp );
		}
		else
		{
			pthread_cond_broadcast ( &(r->doneCond) );

			pthread_mutex_lock ( &(r->private.startMutex) );
			pthread_cond_wait ( &(r->private.startCond), &(r->private.startMutex) );
			pthread_mutex_unlock ( &(r->private.startMutex) );
		}
	}
	return ( NULL );
}

uint32_t holonomicSet ( robot_t * const r, const ROBOT_DIR dir, const uint32_t steps, const bool fullStep )
{
	if ( r->thread != 0 )
	{
		pthread_mutex_lock ( &(r->private.startMutex) );
	}

	uint32_t tmp = r->steps;
	r->steps += steps;
	r->fullStep = fullStep;
	r->dir = dir;

	if ( r->thread != 0 )
	{
		pthread_mutex_unlock ( &(r->private.startMutex) );
		pthread_cond_broadcast ( &(r->private.startCond) );
	}
	return ( tmp );
}

uint32_t holonomicResetSteps ( robot_t * const r )
{
	if ( r->thread != 0 )
	{
		pthread_mutex_lock ( &(r->private.startMutex) );
	}
	
	uint32_t tmp = r->steps = 0;

	if ( r->thread != 0 )
	{
		pthread_mutex_unlock ( &(r->private.startMutex) );
	}
	return ( tmp );
}

uint32_t holonomicGetSteps ( robot_t * const r )
{
	if ( r->thread != 0 )
	{
		pthread_mutex_lock ( &(r->private.startMutex) );
	}
	
	uint32_t step = r->steps;

	if ( r->thread != 0 )
	{
		pthread_mutex_unlock ( &(r->private.startMutex) );
	}
	return ( step );
}

uint32_t holonomicSetDelay ( robot_t * const r, const uint32_t delay )
{
	if ( r->thread != 0 )
	{
		pthread_mutex_lock ( &(r->private.delayMutex) );
	}

	uint32_t tmp = r->private.delay;
	r->private.delay = delay;

	if ( r->thread != 0 )
	{
		pthread_mutex_unlock ( &(r->private.delayMutex) );
	}
	return ( tmp );
}

int holonomicInit ( robot_t *const r, const bool useThread, pthread_mutex_t * const busMutex, const int fd )
{
	if ( !r )
	{
		return ( __LINE__ );
	}

	if ( useThread )
	{
		r->steps = 0;

		r->stepper.front.right = 0;
		r->stepper.front.left = 0;
		r->stepper.back.right = 0;
		r->stepper.back.left = 0;
		r->fd = fd;

		r->busMutex = busMutex;
		if ( busMutex )
		{
			*(r->busMutex) = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
		}

		r->doneCond = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
		r->doneMutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
		
		r->private.startCond = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
		r->private.startMutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
		
		r->private.delayMutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
		r->private.delay = 100000;
		return ( pthread_create( &r->thread, 0, holonomicMove, r ) );
	}
	else
	{
		r->thread = 0;
	}

	return ( 0 );
}

#pragma GCC diagnostic ignored "-Wswitch"
void calcNextStep ( robot_t * const r )
{
	if ( !r )
	{
		return;
	}

	uint8_t step = 1;

	if ( r->fullStep )
	{
		step = 2;
	}

	switch ( r->dir )
	{
		case FREE:
		{
			r->stepper.value = 0;
		}
		case HOLD:
		default:
		{
			return;
		}
		case FRONT:
		{
			r->stepper.front.right += step;
			r->stepper.front.left += step;
			r->stepper.back.right += step;
			r->stepper.back.left += step;
			break;
		}
		case BACK:
		{
			r->stepper.front.right -= step;
			r->stepper.front.left -= step;
			r->stepper.back.right -= step;
			r->stepper.back.left -= step;
			break;
		}
		case LEFT:
		{
			r->stepper.front.right += step;
			r->stepper.front.left -= step;
			r->stepper.back.right -= step;
			r->stepper.back.left += step;
			break;
		}
		case RIGHT:
		{
			r->stepper.front.right -= step;
			r->stepper.front.left += step;
			r->stepper.back.right += step;
			r->stepper.back.left -= step;
			break;
		}
		case FRONT | LEFT:
		{
			r->stepper.front.right += step;
			r->stepper.back.left += step;
			break;
		}
		case FRONT | RIGHT:
		{
			r->stepper.front.left += step;
			r->stepper.back.right += step;
			break;
		}
		case BACK | LEFT:
		{
			r->stepper.front.left -= step;
			r->stepper.back.right -= step;
			break;
		}
		case BACK | RIGHT:
		{
			r->stepper.front.right -= step;
			r->stepper.back.left -= step;
			break;
		}
		case TURN | LEFT:
		{
			r->stepper.front.right -= step;
			r->stepper.front.left += step;
			r->stepper.back.right -= step;
			r->stepper.back.left += step;
			break;
		}
		case TURN | RIGHT:
		{
			r->stepper.front.right += step;
			r->stepper.front.left -= step;
			r->stepper.back.right += step;
			r->stepper.back.left -= step;
			break;
		}
	}

	r->stepper.value = stepValues[ r->stepper.front.right ] +
		( stepValues[ r->stepper.front.left ] << 4 ) +
		( stepValues[ r->stepper.back.right ] << 8 ) +
		( stepValues[ r->stepper.back.left ] << 12 );
}
#pragma GCC diagnostic push

void holonomicWait ( robot_t * const r )
{
	pthread_mutex_lock ( &(r->doneMutex) );
	pthread_cond_wait ( &(r->doneCond), &(r->doneMutex) );
	pthread_mutex_unlock ( &(r->doneMutex) );
}