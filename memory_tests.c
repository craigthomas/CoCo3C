/**
 * Copyright (C) 2013 Craig Thomas
 * This project uses an MIT style license - see the LICENSE file for details.
 *
 * @file     memory_tests.c
 * @brief    Functions to test emulator memory
 * @author   Craig Thomas
 */

/* I N C L U D E S ************************************************************/

#include "yacoco3e.h"

/******************************************************************************/

static char *
test_memory_init_memory_not_null(void) 
{
    printf("test_memory_init_memory_not_null\n");
    mu_assert("Could not allocate memory", memory_init(4096));
    mu_assert("Memory returned was NULL", memory != NULL);
    memory_destroy();
    return 0;    
}

/******************************************************************************/

static char *
test_memory_init_4096_correct_bits_set(void) 
{
    printf("test_memory_init_4096_correct_bits_set\n");
    mu_assert("memory_size_bit_0 incorrect", memory_size_bit_0 == 0);
    mu_assert("memory_size_bit_1 incorrect", memory_size_bit_0 == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_init_zero_bytes_returns_false(void)
{
    printf("test_memory_init_zero_bytes_returns_false\n");
    mu_assert("Memory init succeeded", !memory_init(0));
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_init_negative_bytes_returns_false(void)
{
    printf("test_memory_init_negative_bytes_returns_false\n");
    mu_assert("Memory init succeeded", !memory_init(-10));
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_init_map_mode_set_to_zero(void)
{
    printf("test_memory_init_map_mode_set_to_zero\n");
    mu_assert("Memory init failed", memory_init(4096));
    mu_assert("memory_map_mode not zero", memory_map_mode == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_write_writes_correct_byte(void)
{
    word location;
    byte value = 0x12;
    location.WORD = 0x100;
    printf("test_memory_write_writes_correct_byte\n");
    mu_assert("Memory init failed", memory_init(4096));
    memory_write(location, value);
    mu_assert("Write failed(expected 0x12)", memory[location.WORD] == 0x12);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_write_writes_to_byte_zero(void)
{
    word location;
    location.WORD = 0;
    printf("test_memory_write_writes_to_byte_zero\n");
    mu_assert("Memory init failed", memory_init(4096));
    memory_write(location, 1);
    mu_assert("Incorrect byte written", memory[0] == 1);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_write16_writes_correct_word(void)
{
    word location, value;
    location.WORD = 0x100;
    printf("test_memory_write16_writes_correct_word\n");
    mu_assert("Memory init failed", memory_init(4096));
    value.BYTE.high = 0xE8;
    value.BYTE.low = 0x12;
    memory_write16(location, value);
    mu_assert("Write failed(expected 0xE8)", memory[0x100] == 0xE8);
    mu_assert("Write failed(expected 0x12)", memory[0x101] == 0x12);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_read_correct_byte(void)
{
    word location;
    location.WORD = 0x100;
    printf("test_memory_read_correct_byte\n");
    mu_assert("Memory init failed", memory_init(4096));
    memory[location.WORD] = 0x12;
    mu_assert("Read failed(expected 0x12)", memory[location.WORD] == 0x12);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_read16_read_correct_word(void)
{
    word location, value;
    location.WORD = 0x100;
    printf("test_memory_read16_read_correct_word\n");
    mu_assert("Memory init failed", memory_init(4096));
    memory[location.WORD] = 0x12;
    memory[location.WORD + 1] = 0x34;
    value = memory_read16(location);
    mu_assert("Read failed(expected 0x1234)", value.WORD == 0x1234);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_read_reset_vector_correct(void)
{
    word address;
    address.WORD = CPU_RESET_ADDR;

    printf("test_memory_read_reset_vector_correct\n");
    mu_assert("Could not allocate memory", memory_init(4096));
    mu_assert("Incorrect PC high byte start address", 
               memory_read(address) == CPU_PC_START_HI);
    address.WORD += 1;
    mu_assert("Incorrect PC low byte start address", 
               memory_read(address) == CPU_PC_START_LO);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_less_than_4096(void)
{
    printf("test_memory_less_than_4096\n");
    mu_assert("Memory init failed", memory_init(512));
    mu_assert("memory_size_bit_0 not 0", memory_size_bit_0 == 0);
    mu_assert("memory_size_bit_1 not 0", memory_size_bit_0 == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_16k(void)
{
    printf("test_memory_16k\n");
    mu_assert("Memory init failed", memory_init(16384));
    mu_assert("memory_size_bit_0 not 1", memory_size_bit_0 == 1);
    mu_assert("memory_size_bit_1 not 0", memory_size_bit_1 == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_more_than_16k(void)
{
    printf("test_memory_more_than_16k\n");
    mu_assert("Memory init failed", memory_init(16385));
    mu_assert("memory_size_bit_0 not 0", memory_size_bit_0 == 0);
    mu_assert("memory_size_bit_1 not 1", memory_size_bit_1 == 1);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_memory_read_memory_size_bit_0(void)
{
    printf("test_memory_read_memory_size_bit_0\n");
    mu_assert("Memory init failed", memory_init(16384));
    word location;
    location.WORD = 0xFFDA;
    mu_assert("Memory read at FFDA did not return 1", memory_read(location) == 1);
    memory_destroy();
    mu_assert("Memory init failed", memory_init(16385));
    mu_assert("Memory read at FFDA did not return 0", memory_read(location) == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

char *
memory_tests() 
{
    mu_run_test(test_memory_init_memory_not_null);
    mu_run_test(test_memory_init_4096_correct_bits_set);
    mu_run_test(test_memory_init_zero_bytes_returns_false);
    mu_run_test(test_memory_init_negative_bytes_returns_false);
    mu_run_test(test_memory_init_map_mode_set_to_zero);
    mu_run_test(test_memory_write_writes_correct_byte);
    mu_run_test(test_memory_write16_writes_correct_word);
    mu_run_test(test_memory_write_writes_to_byte_zero);
    mu_run_test(test_memory_read_correct_byte);
    mu_run_test(test_memory_read16_read_correct_word);
    mu_run_test(test_memory_read_reset_vector_correct);
    mu_run_test(test_memory_less_than_4096);
    mu_run_test(test_memory_16k);
    mu_run_test(test_memory_more_than_16k);
    mu_run_test(test_memory_read_memory_size_bit_0);
    return 0;
}

/* E N D   O F   F I L E ******************************************************/
