/**
 * Copyright (C) 2013 Craig Thomas
 * This project uses an MIT style license - see the LICENSE file for details.
 *
 * @file      cpu_tests.c
 * @brief     Functions used to test the cpu
 * @author    Craig Thomas
 */

/* I N C L U D E S ************************************************************/

#include "yacoco3e.h"

/* L O C A L S ****************************************************************/

word address;

/******************************************************************************/

static char *
test_cpu_get_immediate8(void)
{
    printf("test_cpu_get_immediate8\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0x100;
    cpu.bytesconsumed = 0;
    memory[0x100] = 0x34;
    mu_assert("Incorrect immediate value(expected 0x34)", 
        cpu_get_immediate8(cpu.pc) == 0x34);
    mu_assert("No bytes consumed", cpu.bytesconsumed == 1);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_get_immediate16(void)
{
    word result;
    printf("test_cpu_get_immediate16\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0x100;
    cpu.bytesconsumed = 0;
    memory[0x100] = 0x12;
    memory[0x101] = 0x34;
    result = cpu_get_immediate16(cpu.pc);
    mu_assert("Incorrect immediate value(expected 0x1234)",
        result.WORD == 0x1234);
    mu_assert("No bytes consumed", cpu.bytesconsumed == 2);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_get_direct_address_correct(void)
{
    word result;

    printf("test_cpu_get_direct_address_correct\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.dp = 0x12;
    cpu.pc.WORD = 0x100;
    memory[0x100] = 0x34;
    result = cpu_get_direct(cpu.pc);    
    mu_assert("Incorrect address returned", result.WORD == 0x1234);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_push_stack_reg_s(void)
{
    printf("test_cpu_push_stack_reg_s\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    memory[0x0] = 0;
    cpu.s.WORD = 1;
    cpu_push_stack(REG_S, 0x12);
    mu_assert("Incorrect memory value(expected 0x12)", memory[0x0] == 0x12);
    mu_assert("S stack not decremented", cpu.s.WORD == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_push_stack_reg_u(void)
{
    printf("test_cpu_push_stack_reg_u\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    memory[0x0] = 0;
    cpu.u.WORD = 1;
    cpu_push_stack(REG_U, 0x12);
    mu_assert("Incorrect memory value(expected 0x12)", memory[0x0] == 0x12);
    mu_assert("U stack not decremented", cpu.u.WORD == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_pull_stack_reg_s(void)
{
    printf("test_cpu_pull_stack_reg_s\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    memory[0x0] = 0x12;
    cpu.s.WORD = 0;
    mu_assert("Incorrect return value(expected 0x12)", cpu_pop_stack(REG_S));
    mu_assert("S stack not incremented", cpu.s.WORD == 1);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_pull_stack_reg_u(void)
{
    printf("test_cpu_pull_stack_reg_u\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    memory[0x0] = 0x12;
    cpu.u.WORD = 0;
    mu_assert("Incorrect return value(expected 0x12)", cpu_pop_stack(REG_U));
    mu_assert("S stack not incremented", cpu.u.WORD == 1);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_short_does_nothing_on_false(void)
{
    printf("test_cpu_branch_short_does_nothing_on_false\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    cpu_branch_short(0, 0x10);
    mu_assert("Program counter modified", cpu.pc.WORD == 0);
    mu_assert("bytesconsumed not 0", cpu.bytesconsumed == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_short_positive_offset_updates_pc_on_true(void)
{
    printf("test_cpu_branch_short_positive_offset_updates_pc_on_true\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    cpu_branch_short(1, 0x10);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0x10);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_short_zero_offset_updates_pc_on_true(void)
{
    printf("test_cpu_branch_short_zero_offset_updates_pc_on_true\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    cpu_branch_short(1, 0);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_short_maximum_offset(void)
{
    printf("test_cpu_branch_short_maximum_offset\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    cpu_branch_short(1, 0x7F);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0x7F);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_short_negative_offset(void)
{
    printf("test_cpu_branch_short_negative_offset\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0xA;
    cpu.bytesconsumed = 0;
    cpu_branch_short(1, 0x85);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0x5);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_long_does_nothing_on_false(void)
{
    word offset;
    printf("test_cpu_branch_long_does_nothing_on_false\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    offset.WORD = 0x10;
    cpu_branch_long(0, offset);
    mu_assert("Program counter modified", cpu.pc.WORD == 0);
    mu_assert("bytesconsumed not 0", cpu.bytesconsumed == 0);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_long_positive_offset_updates_pc_on_true(void)
{ 
    word offset;
    printf("test_cpu_branch_long_positive_offset_updates_pc_on_true\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    offset.WORD = 0x10;
    cpu_branch_long(1, offset);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0x10);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_long_zero_offset_updates_pc_on_true(void)
{
    word offset;
    printf("test_cpu_branch_long_zero_offset_updates_pc_on_true\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    offset.WORD = 0x0;
    cpu_branch_long(1, offset);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_long_maximum_offset(void)
{
    word offset;
    printf("test_cpu_branch_long_maximum_offset\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0;
    cpu.bytesconsumed = 0;
    offset.WORD = 0x7FFF;
    cpu_branch_long(1, offset);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0x7FFF);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_branch_long_negative_offset(void)
{
    word offset;
    printf("test_cpu_branch_long_negative_offset\n");
    mu_assert("Failed to initialize memory", memory_init(4096));
    cpu.pc.WORD = 0xA;
    cpu.bytesconsumed = 0;
    offset.WORD = 0x8005;
    cpu_branch_long(1, offset);
    mu_assert("Program counter incorrect", cpu.pc.WORD == 0x5);
    mu_assert("bytesconsumed not NO_PC_UPDATE", cpu.bytesconsumed == NO_PC_UPDATE);
    memory_destroy();
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_zero_values(void)
{
    printf("test_cpu_binary_add8_zero_values\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(0, 0, FALSE, FALSE, FALSE);
    mu_assert("Result not 0", result == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_halfcarry_not_set_on_zero_values(void) 
{
    printf("test_cpu_binary_add8_halfcarry_not_set_on_zero_values\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(0, 0, TRUE, FALSE, FALSE);
    mu_assert("Result not 0", result == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_carry_not_set_on_zero_values(void)
{
    printf("test_cpu_binary_add8_carry_not_set_on_zero_values\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(0, 0, FALSE, TRUE, FALSE);
    mu_assert("Result not 0", result == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_overflow_not_set_on_zero_values(void)
{
    printf("test_cpu_binary_add8_overflow_not_set_on_zero_values\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(0, 0, FALSE, FALSE, TRUE);
    mu_assert("Result not 0", result == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_correct_result_simple_addition(void)
{
    printf("test_cpu_binary_add8_correct_result_simple_addition\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(1, 2, FALSE, FALSE, FALSE);
    mu_assert("Result not 3", result == 3);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_overflow_not_set_on_simple_addition(void)
{
    printf("test_cpu_binary_add8_overflow_not_set_on_simple_addition\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(1, 2, FALSE, FALSE, TRUE);
    mu_assert("Result not 3", result == 3);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_overflow_set_on_overflow(void)
{
    printf("test_cpu_binary_add8_overflow_set_on_overflow\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(255, 1, FALSE, FALSE, TRUE);
    mu_assert("Result not 0", result == 0);
    mu_assert("Overflow not set", cpu.cc == REG_CC_V);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_carry_set_on_carry(void)
{
    printf("test_cpu_binary_add8_carry_set_on_carry\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(128, 128, FALSE, TRUE, FALSE);
    mu_assert("Result not 0", result == 0);
    mu_assert("Carry not set", cpu.cc == REG_CC_C);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_halfcarry_set_on_halfcarry(void)
{
    printf("test_cpu_binary_add8_halfcarry_set_on_halfcarry\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(8, 8, TRUE, FALSE, FALSE);
    mu_assert("Result not 16", result == 16);
    mu_assert("Halfcarry not set", cpu.cc == REG_CC_H);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add8_halfcarry_set_carried_from_bit_2(void)
{
    printf("test_cpu_binary_add8_halfcarry_set_carried_from_bit_2\n");
    cpu.cc = 0;
    byte result = cpu_binary_add8(0xC, 0x4, TRUE, FALSE, FALSE);
    mu_assert("Result not 16", result == 16);
    mu_assert("Halfcarry not set", cpu.cc == REG_CC_H);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_zero_values(void)
{
    printf("test_cpu_binary_add16_zero_values\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0;
    val2.WORD = 0;
    word result = cpu_binary_add16(val1, val2, FALSE, FALSE, FALSE);
    mu_assert("Result not 0", result.WORD == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_halfcarry_not_set_on_zero_values(void) 
{
    printf("test_cpu_binary_add16_halfcarry_not_set_on_zero_values\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0;
    val2.WORD = 0;
    word result = cpu_binary_add16(val1, val2, TRUE, FALSE, FALSE);
    mu_assert("Result not 0", result.WORD == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_carry_not_set_on_zero_values(void)
{
    printf("test_cpu_binary_add16_carry_not_set_on_zero_values\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0;
    val2.WORD = 0;
    word result = cpu_binary_add16(val1, val2, FALSE, TRUE, FALSE);
    mu_assert("Result not 0", result.WORD == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_overflow_not_set_on_zero_values(void)
{
    printf("test_cpu_binary_add16_overflow_not_set_on_zero_values\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0;
    val2.WORD = 0;
    word result = cpu_binary_add16(val1, val2, FALSE, FALSE, TRUE);
    mu_assert("Result not 0", result.WORD == 0);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_correct_result_simple_addition(void)
{
    printf("test_cpu_binary_add16_correct_result_simple_addition\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 1;
    val2.WORD = 2;
    word result = cpu_binary_add16(val1, val2, FALSE, FALSE, FALSE);
    mu_assert("Result not 3", result.WORD == 3);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_overflow_not_set_on_simple_addition(void)
{
    printf("test_cpu_binary_add16_overflow_not_set_on_simple_addition\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 1;
    val2.WORD = 2;
    word result = cpu_binary_add16(val1, val2, FALSE, FALSE, TRUE);
    mu_assert("Result not 3", result.WORD == 3);
    mu_assert("CC register not 0", cpu.cc == 0);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_overflow_set_on_overflow(void)
{
    printf("test_cpu_binary_add16_overflow_set_on_overflow\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0xFFFF;
    val2.WORD = 1;
    word result = cpu_binary_add16(val1, val2, FALSE, FALSE, TRUE);
    mu_assert("Result not 0", result.WORD == 0);
    mu_assert("Overflow not set", cpu.cc == REG_CC_V);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_carry_set_on_carry(void)
{
    printf("test_cpu_binary_add16_carry_set_on_carry\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0xFF;
    val2.WORD = 1;
    word result = cpu_binary_add16(val1, val2, FALSE, TRUE, FALSE);
    mu_assert("Result not 0", result.WORD == 0x100);
    mu_assert("Carry not set", cpu.cc == REG_CC_C);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_halfcarry_set_on_halfcarry(void)
{
    printf("test_cpu_binary_add16_halfcarry_set_on_halfcarry\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 8;
    val2.WORD = 8;
    word result = cpu_binary_add16(val1, val2, TRUE, FALSE, FALSE);
    mu_assert("Result not 16", result.WORD == 16);
    mu_assert("Halfcarry not set", cpu.cc == REG_CC_H);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_binary_add16_halfcarry_set_carried_from_bit_2(void)
{
    printf("test_cpu_binary_add16_halfcarry_set_carried_from_bit_2\n");
    cpu.cc = 0;
    word val1, val2;
    val1.WORD = 0xC;
    val2.WORD = 0x4;
    word result = cpu_binary_add16(val1, val2, TRUE, FALSE, FALSE);
    mu_assert("Result not 16", result.WORD == 16);
    mu_assert("Halfcarry not set", cpu.cc == REG_CC_H);
    return 0;
}

/******************************************************************************/

static char *
test_cpu_lda_immediate_correct_value(void)
{
    printf("test_cpu_lda_immediate_correct_value\n");
    cpu_reset();
    cpu.state = CPU_UNIT_TEST;
    memory_init(MEM_512K);
    cpu.a = 0;
    cpu.d.WORD = 0x0;
    address.WORD = 0x0;
    cpu.pc = address;
    memory_write(address, 0x86);
    address.WORD++;
    memory_write(address, 0x23);
    cpu_execute();
    mu_assert("Result not 0x23", cpu.a == 0x23);
    mu_assert("Zero flag set", !(cpu.cc & REG_CC_Z));
    mu_assert("Negative flag set", !(cpu.cc & REG_CC_N));
    mu_assert("D register not updated", (cpu.d.BYTE.high == 0x23) && (cpu.d.BYTE.low == 0x0));
    return 0;
}

/******************************************************************************/

static char *
test_cpu_lda_immediate_correct_value_zero_sets_correct_flags(void)
{
    printf("test_cpu_lda_immediate_correct_value_zero_sets_correct_flags\n");
    cpu_reset();
    cpu.state = CPU_UNIT_TEST;
    memory_init(MEM_512K);
    cpu.a = 0;
    cpu.d.WORD = 0x0;
    address.WORD = 0x0;
    cpu.pc = address;
    memory_write(address, 0x86);
    address.WORD++;
    memory_write(address, 0x0);
    cpu_execute();
    mu_assert("Result not 0x0", cpu.a == 0x0);
    mu_assert("Zero flag not set", (cpu.cc & REG_CC_Z));
    mu_assert("Negative flag set", !(cpu.cc & REG_CC_N));
    mu_assert("D register not updated", (cpu.d.BYTE.high == 0x0) && (cpu.d.BYTE.low == 0x0));
    return 0;
}

/******************************************************************************/

static char *
test_cpu_lda_immediate_correct_value_negative_sets_correct_flags(void)
{
    printf("test_cpu_lda_immediate_correct_value_negative_sets_correct_flags\n");
    cpu_reset();
    cpu.state = CPU_UNIT_TEST;
    memory_init(MEM_512K);
    cpu.a = 0;
    cpu.d.WORD = 0x0;
    address.WORD = 0x0;
    cpu.pc = address;
    memory_write(address, 0x86);
    address.WORD++;
    memory_write(address, 0x83);
    cpu_execute();
    mu_assert("Result not 0x83", cpu.a == 0x83);
    mu_assert("Zero flag set", !(cpu.cc & REG_CC_Z));
    mu_assert("Negative flag not set", (cpu.cc & REG_CC_N));
    mu_assert("D register not updated", (cpu.d.BYTE.high == 0x83) && (cpu.d.BYTE.low == 0x0));
    return 0;
}

/******************************************************************************/

char *
cpu_tests() 
{
    mu_run_test(test_cpu_get_immediate8);
    mu_run_test(test_cpu_get_immediate16);
    mu_run_test(test_cpu_get_direct_address_correct);
    mu_run_test(test_cpu_push_stack_reg_s);
    mu_run_test(test_cpu_push_stack_reg_u);
    mu_run_test(test_cpu_pull_stack_reg_s);
    mu_run_test(test_cpu_pull_stack_reg_u);
    mu_run_test(test_cpu_branch_short_does_nothing_on_false);
    mu_run_test(test_cpu_branch_short_positive_offset_updates_pc_on_true);
    mu_run_test(test_cpu_branch_short_zero_offset_updates_pc_on_true);
    mu_run_test(test_cpu_branch_short_maximum_offset);
    mu_run_test(test_cpu_branch_short_negative_offset);
    mu_run_test(test_cpu_branch_long_does_nothing_on_false);
    mu_run_test(test_cpu_branch_long_positive_offset_updates_pc_on_true);
    mu_run_test(test_cpu_branch_long_zero_offset_updates_pc_on_true);
    mu_run_test(test_cpu_branch_long_maximum_offset);
    mu_run_test(test_cpu_branch_long_negative_offset);
    mu_run_test(test_cpu_binary_add8_zero_values);
    mu_run_test(test_cpu_binary_add8_halfcarry_not_set_on_zero_values);
    mu_run_test(test_cpu_binary_add8_carry_not_set_on_zero_values);
    mu_run_test(test_cpu_binary_add8_overflow_not_set_on_zero_values);
    mu_run_test(test_cpu_binary_add8_correct_result_simple_addition);
    mu_run_test(test_cpu_binary_add8_overflow_not_set_on_simple_addition);
    mu_run_test(test_cpu_binary_add8_overflow_set_on_overflow);
    mu_run_test(test_cpu_binary_add8_carry_set_on_carry);
    mu_run_test(test_cpu_binary_add8_halfcarry_set_on_halfcarry);
    mu_run_test(test_cpu_binary_add8_halfcarry_set_carried_from_bit_2);
    mu_run_test(test_cpu_binary_add16_zero_values);
    mu_run_test(test_cpu_binary_add16_halfcarry_not_set_on_zero_values);
    mu_run_test(test_cpu_binary_add16_carry_not_set_on_zero_values);
    mu_run_test(test_cpu_binary_add16_overflow_not_set_on_zero_values);
    mu_run_test(test_cpu_binary_add16_correct_result_simple_addition);
    mu_run_test(test_cpu_binary_add16_overflow_not_set_on_simple_addition);
    mu_run_test(test_cpu_binary_add16_overflow_set_on_overflow);
    mu_run_test(test_cpu_binary_add16_carry_set_on_carry);
    mu_run_test(test_cpu_binary_add16_halfcarry_set_on_halfcarry);
    mu_run_test(test_cpu_binary_add16_halfcarry_set_carried_from_bit_2);
    mu_run_test(test_cpu_lda_immediate_correct_value);
    mu_run_test(test_cpu_lda_immediate_correct_value_zero_sets_correct_flags);
    mu_run_test(test_cpu_lda_immediate_correct_value_negative_sets_correct_flags);
    return 0;
}

/* E N D   O F   F I L E ******************************************************/
