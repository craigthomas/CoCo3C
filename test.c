/**
 * Copyright (C) 2013 Craig Thomas
 * This project uses an MIT style license - see the LICENSE file for details.
 * 
 * @file      trs80e.c
 * @brief     Yet Another Color Computer 3 emulator
 * @author    Craig Thomas
 *
 * The test file runs the main unit tests for the emulator.
 */

/* I N C L U D E S ***********************************************************/

#include <stdlib.h>
#include "yacoco3e.h"

/* M A I N *******************************************************************/

/**
 * Run the main unit tests.
 */
int
main(int argc, char **argv)
{
    run_tests();
    return 0;
}

/* E N D   O F   F I L E *****************************************************/
