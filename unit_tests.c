/**
 * Copyright (C) 2013 Craig Thomas
 * This project uses an MIT style license - see the LICENSE file for details.
 * 
 * @file     unit_tests.c
 * @brief    Defines unit tests for the emulator.
 * @author   Craig Thomas
 */

/* I N C L U D E S ************************************************************/

#include "yacoco3e.h"

/******************************************************************************/

void 
check_result(char *result) 
{
    if (result) {
        printf("FAIL: %s\n", result);
        printf("Tests run: %d\n", tests_run);
        exit(1);
    }
}

/******************************************************************************/

void 
run_tests(void) 
{
    printf("Running memory tests...\n");
    check_result(memory_tests());
    printf("%d tests succeeded\n", tests_run);

    tests_run = 0;
    printf("\nRunning CPU tests...\n");
    check_result(cpu_tests());
    printf ("%d tests succeeded\n", tests_run);
}

/* E N D   O F   F I L E ******************************************************/
