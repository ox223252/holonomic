# holonomic

## copyleft
GPLv2

## brief
I'm creating robot using mecanum weel and I will follow thit shematics (approximately):

![img](res/ROUE-MECANUM-REDOHM-010-600x470.jpg)
![img](res/ROUE-MECANUM-REDOHM-011.jpg)

## usage:
```C
#include <stdio.h>
#include <unistd.h>
#include "lib/holonomic/holonomic.h"

int main ( void )
{
	robot_t robot;

	// init robot struct and set threaded mode
	holonomicInit ( &robot, true, NULL, 0 );

	// set 5 full steps in the front direction
	holonomicSet ( &robot, FRONT, 5, true );
	sleep ( 1 );

	// set 5 half steps in the right direction
	holonomicSet ( &robot, RIGHT, 5, false );
	usleep ( 250 );

	// update the direction and the steps type
	// before the end of previous move
	holonomicSet ( &robot, FRONT, 5, true );

	// change steps speed
	holonomicSetDelay ( &robot, 500000 );

	sleep ( 2 );
	
	// close systeme before end of move
	pthread_cancel ( robot.thread );
	pthread_join ( robot.thread, NULL );
	return ( 0 );
}
```

## test:
```Shell
>  valgrind --leak-check=full ./bin/exec
==10341== Memcheck, a memory error detector
==10341== Copyright (C) 2002-2017, and GNU GPL'd, by Julian Seward et al.
==10341== Using Valgrind-3.13.0 and LibVEX; rerun with -h for copyright info
==10341== Command: ./bin/exec
==10341==
NO FD SET :      4 0x2222
NO FD SET :      3 0x4444
NO FD SET :      2 0x1111
NO FD SET :      1 0x8888
NO FD SET :      0 0x2222
NO FD SET :      4 0xa66a
NO FD SET :      8 0x6556
NO FD SET :      7 0x5995
NO FD SET :      6 0x9aa9
NO FD SET :      5 0xa66a
==10341==
==10341== HEAP SUMMARY:
==10341==     in use at exit: 0 bytes in 0 blocks
==10341==   total heap usage: 7 allocs, 7 frees, 2,990 bytes allocated
==10341==
==10341== All heap blocks were freed -- no leaks are possible
==10341==
==10341== For counts of detected and suppressed errors, rerun with: -v
==10341== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)
```

### explanations:
In this output, you can see a this line: `NO FD SET: 4 0xa66a`
`NO FD SET` means no motor is driven because no bus fd is set
`4` is the number of remaining step before `doneCond` was set
`0xa66a` each digit represents a motor
- `value & 0x0f` front right motor: `0b1010`
- `value >> 4 & 0x0f` front left motor: `0b0110`
- `value >> 8 & 0x0f` back right motor: `0b0110`
- `value '12 & 0x0f` back left motor: `0b1010`

Work in progress :)