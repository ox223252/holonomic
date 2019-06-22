#include <unistd.h>
#include <stdio.h>

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
			}

			usleep ( 100000 );
		}
		else
		{
			pthread_cond_broadcast ( &(r->doneCond) );

			pthread_mutex_lock ( &(r->startMutex) );
			pthread_cond_wait ( &(r->startCond), &(r->startMutex) );
			pthread_mutex_unlock ( &(r->startMutex) );
		}
	}
}

int holonomicSet ( robot_t *r, ROBOT_DIR dir, uint32_t steps, bool fullStep )
{
	if ( r->thread != 0 )
	{
		pthread_mutex_lock ( &(r->startMutex) );
	}

	r->steps += steps;
	r->fullStep = fullStep;
	r->dir = dir;

	if ( r->thread != 0 )
	{
		pthread_mutex_unlock ( &(r->startMutex) );
		pthread_cond_broadcast ( &(r->startCond) );
	}
}

int holonomicInit ( robot_t *r, bool useThread, pthread_mutex_t *busMutex )
{
	if ( useThread )
	{
		r->busMutex = busMutex;
		if ( busMutex )
		{
			*(r->busMutex) = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
		}

		r->doneCond = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
		r->doneMutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
		
		r->startCond = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
		r->startMutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;

		return ( pthread_create( &r->thread, 0, holonomicMove, r ) );
	}

	return ( 0 );
}

#pragma GCC diagnostic ignored "-Wswitch"
void calcNextStep ( robot_t *r )
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