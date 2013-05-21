/**
 * Copyright (C) 2013 Craig Thomas
 *
 * @file      yacoco3e.c
 * @brief     Yet Another Color Computer 3 emulator
 * @author    Craig Thomas
 * @copyright MIT style license - see the LICENSE file for details
 * @copyright @verbinclude LICENSE
 *
 * This project represents a Color Computer 3 emulator.
 *
 * In addition to the emulator keys, there are several special keys: 
 *
 *     F4        Immediately exits the emulator
 *     F5        Debug mode - an overlay will display the current instruction
 *     F6        Trace mode - executes and displays instructions
 *     F7        Run mode - normal execution mode
 *     F8        Steps to the next instruction (while in debug mode)
 */

/* I N C L U D E S ***********************************************************/

#include <stdlib.h>
#include "yacoco3e.h"

/* F U N C T I O N S *********************************************************/

/**
 * Loads the specified ROM contained in the file `romfilename` into emulator 
 * memory at the memory offset indicated by `offset`. Returns FALSE on 
 * failure or TRUE on success.
 *
 * @param romfilename the name of the file to load
 * @param offset the memory offset at which to load the file
 * @returns TRUE on success or FALSE on failure
 */
int 
loadrom(char *romfilename, word address)
{
    FILE *fp;
    byte byte_read;
    int result = TRUE;

    fp = fopen(romfilename, "r");
    if (fp == NULL) {
        printf("Error: could not open ROM image: %s\n", romfilename);
    }
    else {
        while (!feof(fp)) {
            fread(&byte_read, 1, 1, fp);
            memory_write(address, byte_read);
            address.WORD++;
        } 
        fclose(fp);
    }
    return result;
}

/*****************************************************************************/

/**
 * Prints out the usage message.
 */
void 
print_help(void)
{
    printf("usage: yacoco3e [-h] [-s SCALE] [-t] rom\n\n");
    printf("Starts a simple Color Computer 3 emulator. See README.md for ");
    printf("more information, and\n LICENSE for terms of use.\n\n");
    printf("positional arguments:\n");
    printf("  rom          the ROM file to load on startup\n\n");
    printf("optional arguments:\n");
    printf("  -h           show this help message and exit\n");
    printf("  -s SCALE     the scale factor to apply to the display ");
    printf("(default is 2)\n");
    printf("  -t           starts the CPU up in trace mode\n");
}

/******************************************************************************/

/**
 * Parse the command-line options. There are currently 3 recognized options:
 *
 *   -h      prints out the usage message
 *   -s      applies a scale factor to the window
 *   -t      starts the emulator in debug mode
 *   -u      runs the unit tests and exits
 *
 * @param argc the number of arguments on the command line
 * @param argv a pointer to a pointer to the set of command line strings
 * @returns the name of the file to be loaded
 */
char *
parse_options(int argc, char **argv)
{
    int arg;
    char *filename = NULL;

    for (arg = 1; arg < argc; arg++) {
        if ((argv[arg][0] == '-') && (strlen (argv[arg]) != 2)) {
            printf("Unrecognized option: %s\n", argv[arg]);
            print_help();
            exit(1);
        }
        else if ((argv[arg][0] == '-') && (strlen(argv[arg]) == 2)) {
            switch (argv[arg][1]) {
                case ('h'):
                    print_help();
                    exit(0);
                    break;

                case ('s'):
                    arg++;
                    if (arg < argc) {
                        screen_scale_factor = atoi(argv[arg]);
                    }
                    break;

                case ('t'):
                    cpu.state = CPU_DEBUG;
                    break;

                default:
                    printf("Unrecognized option: %s\n", argv[arg]);
                    print_help();
                    exit(1);
                    break;
            }
        }
        else {
            if (filename == NULL) {
                filename = argv[arg];
            }
            else {
                printf("Unrecognized parameter: %s\n", argv[arg]);
                print_help();
                exit(1);
            }
        }
    }

    if (filename == NULL) {
        printf("ROM file not specified\n");
        print_help();
        exit(1);
    }

    return filename;
}

/******************************************************************************/

void 
start_cpu_loop(char *filename) 
{
    word starting_address;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Fatal: Unable to initialize SDL: %s\n", SDL_GetError());
        exit(1);
    }

    if (!memory_init(MEM_512K)) {
        printf("Fatal: could not initialize emulator memory\n");
        SDL_Quit();
        exit(1);
    }

    starting_address.WORD = ROM_DEFAULT;

    if (!loadrom(filename, starting_address)) {
        printf("Fatal: emulator shutdown due to errors\n");
        memory_destroy();
        SDL_Quit();
        exit(1);
    }

    if (!screen_init(SCREEN_WIDTH, SCREEN_HEIGHT)) {
        printf("Fatal: emulator shutdown due to errors\n");
        memory_destroy();
        SDL_Quit();
        exit(1);
    }

    if (!cpu_timerinit()) {
        printf("Fatal: emulator shutdown due to errors\n");
        memory_destroy();
        SDL_Quit();
        exit(1);
    }

    cpu_execute();
    memory_destroy();
    SDL_Quit();
}

/* M A I N *******************************************************************/

/*
 * Initalize all the main components of the emulator and start the CPU 
 * execution loop. On shutdown, destory any memory structures created as well 
 * as shut down the SDL library.
 */
int 
main(int argc, char **argv)
{
    char *filename;

    cpu_reset();
    cpu.state = CPU_RUNNING;
    screen_scale_factor = SCALE_FACTOR;
    filename = parse_options(argc, argv);
    start_cpu_loop(filename);
    return 0;
}

/* E N D   O F   F I L E *****************************************************/
