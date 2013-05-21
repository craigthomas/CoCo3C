/**
 * Copyright (C) 2013 Craig Thomas
 *
 * @file      cpu.c
 * @brief     Functions used to implement the Motorola 6809E processor
 * @author    Craig Thomas
 * @copyright MIT style license - see the LICENSE file for details
 * @copyright @verbinclude LICENSE
 *
 * This project represents a Color Computer 3 emulator.
 *
 * This file contains the code necessary to implement the Motorola 6809E
 * processor. It contains a full specification of each of the documented
 * 6809E opcodes, along with the correct set of registers.
 */

/* I N C L U D E S ***********************************************************/

#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "yacoco3e.h"

/* L O C A L S ***************************************************************/

long cpu_optime;     /**< Stores operation execution time in ns */
Uint32 cpu_ticks;    /**< Stores how many operations have occurred */

/* F U N C T I O N S *********************************************************/

/** 
 * @brief Initialize the CPU timer.
 * 
 * Intiailize the CPU timer and return whether or not the desired timer
 * could be created.
 *
 * @return TRUE on success, FALSE otherwise.
 */
int 
cpu_timerinit(void) 
{
    SDL_InitSubSystem(SDL_INIT_TIMER);
    cpu_timer = SDL_AddTimer(16, cpu_timerinterrupt, NULL);
    if (cpu_timer == NULL) {
        printf("Error: could not create timer: %s\n", SDL_GetError());
        return FALSE;
    }
    return TRUE;
}

/*****************************************************************************/

/**
 * @brief Fire a timer interrupt based upon the initialized timer value.
 * 
 * Every interval number of ticks, fire the timer interrupt. 
 *
 * @param [in] interval The interval at which to fire the interrupt
 * @param [in] parameters Additional parameters for the timer interrupt.
 */
Uint32 
cpu_timerinterrupt(Uint32 interval, void *parameters) 
{      
    screen_refresh_flag = TRUE;
    if (TRUE) {
        cpu_timer_countdown();
    }
    if (cpu_irq_enabled) {
        cpu_fire_interrupt = TRUE;
        cpu_irq_flag = TRUE;
        cpu_irq_value |= 0x08;
    }
    if (cpu_firq_enabled) {
        cpu_fire_interrupt = TRUE;
        cpu_firq_flag = TRUE;
        cpu_firq_value |= 0x08;
    }  
    return interval;
}

/*****************************************************************************/

/**
 * @brief counts down the current value of the timer.
 *
 * Subtract one from the current value of the timer. 
 */
void 
cpu_timer_countdown(void) 
{

}

/*****************************************************************************/

/**
 * @brief Update the clockrate based upon the clockrate bits.
 *
 * If both the high and low clock rate bit are set to 0, then run in slow 
 * mode, otherwise, change the CPU frequency to turbo mode.
 */
void 
cpu_set_clockrate(void) 
{
    cpu_optime = (cpu.clockrate == 0) ? CPU_89MHZ : CPU_178MHZ;
}

/*****************************************************************************/

/**
 * @brief Pushes the specified byte value onto the specified stack (U or S).
 * 
 * Given a register flag, pushes the specified value onto that stack. Values
 * for \a reg are either REG_S or REG_U.
 * 
 * @param [in] reg The register flag to use.
 * @param [in] value The value to push on to the stack.
 */
inline void 
cpu_push_stack(int reg, byte value)
{
    if (reg == REG_S) {
        cpu.s.WORD--;
        memory_write(cpu.s, value);
    }
    else {
        cpu.u.WORD--;
        memory_write(cpu.u, value);
    }
}

/*****************************************************************************/

/**
 * @brief Pop a value off of the specified stack (U or S).
 * 
 * Given a register flag, pops off a single value from that stack. Values for
 * \a reg are either REG_S or REG_U.
 *
 * @param [in] reg The register flag to use.
 * @return The single byte popped off of the specified stack.
 */
inline byte 
cpu_pop_stack(int reg)
{
    byte value;
    if (reg == REG_S) {
        value = memory_read(cpu.s);
        cpu.s.WORD++;
    }
    else {
        value = memory_read(cpu.u);
        cpu.u.WORD++;
    }
    return value;
}

/*****************************************************************************/

/**
 * @brief Performs an 8-bit binary addition.
 * 
 * Performs an 8-bit binary addition of the two specified values, returning
 * the result. The flags \a hflag, \a cflag, and \a vflag, tells the function
 * whether to set the half-carry (H), carry (C), or overflow (V) bits in the 
 * condition code register during the addition. 
 *
 * @param [in] val1 The first value to add.
 * @param [in] val2 The second value to add.
 * @param [in] hflag Set the half-carry bit in the CC register if a half carry occurs.
 * @param [in] cflag Set the carry bit in the CC register if a full carry occurs.
 * @param [in] vflag Set the overflow bit in the CC register on overflow.
 * @return The result of the addition.
 */
inline byte 
cpu_binary_add8(byte val1, byte val2, int hflag, int cflag, int vflag)
{
    if ((((val1 & 0xF) + (val2 & 0xF)) & 0x10) && hflag) {
        cpu.cc |= REG_CC_H;
    }

    if ((((val1 & 0xFF) + (val2 & 0xFF)) & 0x100) && cflag) {
        cpu.cc |= REG_CC_C;
    }

    if (((val1 + val2) > 255) && vflag) {
        cpu.cc |= REG_CC_V;
    }

    return (val1 + val2) & 0xFF;
}

/*****************************************************************************/

/**
 * @brief Performs a 16-bit binary addition.
 * 
 * Performs an 16-bit binary addition of the two specified values, returning
 * the result. The flags \a hflag, \a cflag, and \a vflag, tells the function
 * whether to set the half-carry (H), carry (C), or overflow (V) bits in the 
 * condition code register during the addition. 
 *
 * @param [in] val1 The first value to add.
 * @param [in] val2 The second value to add.
 * @param [in] hflag Set the half-carry bit in the CC register if a half carry occurs.
 * @param [in] cflag Set the carry bit in the CC register if a full carry occurs.
 * @param [in] vflag Set the overflow bit in the CC register on overflow.
 * @return The result of the addition.
 */
inline word 
cpu_binary_add16(word val1, word val2, int hflag, int cflag, int vflag)
{
    word result;

    if ((((val1.WORD & 0xF) + (val2.WORD & 0xF)) & 0x10) && hflag) {
        cpu.cc |= REG_CC_H;
    }

    if ((((val1.WORD & 0xFF) + (val2.WORD & 0xFF)) & 0x100) && cflag) {
        cpu.cc |= REG_CC_C;
    }

    if (((val1.WORD + val2.WORD) > 0xFFFF) && vflag) {
        cpu.cc |= REG_CC_V;
    }

    result.WORD = (val1.WORD + val2.WORD) & 0xFFFF;
    return result;
}

/************************************************************************/

/**
 * @brief Performs the two's compliment on the specified 8-bit byte.
 *
 * Calculates the two's compliment on the specified byte. The two's 
 * compliment is calculated by inverting the bits of the byte, and adding
 * 1 to the result. No condition codes are set by the operation.
 *
 * @param [in] val The 8-bit value to run the two's compliment on.
 * @return The 8-bit value of the two's compliment.
 */
inline byte 
cpu_twos_comp8(byte val)
{
    return ~val + 1;
}

/*****************************************************************************/

/**
 * @brief Performs the two's compliment on the specified 16-bit word.
 *
 * Calculates the two's compliment on the specified word. The two's 
 * compliment is calculated by inverting the bits of the word, and adding
 * 1 to the result. No condition codes are set by the operation.
 *
 * @param [in] val The 16-bit value to run the two's compliment on.
 * @return The 16-bit value of the two's compliment.
 */
inline word 
cpu_twos_comp16(word val)
{
    word result;
    result.WORD = ~val.WORD + 1;
    return result;
}

/*****************************************************************************/

/**
 * @brief Compute the signed value of the 16-bit word.
 *
 * Given a 16-bit value, will return the signed version of the word.
 *
 * @param [in] value The 16-bit value to check.
 * @return The signed value of the input.
 */
int 
cpu_get_signed16(word value)
{
    if (value.WORD & 0x8000) {
        value = cpu_twos_comp16(value);
        return -value.WORD;
    }
    else {
        return value.WORD;
    }
}

/*****************************************************************************/

/**
 * @brief Read and return the next 16-bit value offset from \a ptr.
 * 
 * Returns the immediate 16-bit value in memory starting from the
 * current value of the \a ptr passed. As a side-effect, updates the
 * number of bytes read in cpu.bytesconsumed.
 *
 * @note As a side-effect, updates the number of bytes read in cpu.bytesconsumed.
 * 
 * @param [in] ptr The pointer into memory to read from.
 * @return The 16-bit value read.
 */
inline word 
cpu_get_immediate16(word ptr)
{
    cpu.bytesconsumed += 2;
    return memory_read16(ptr);
}

/*****************************************************************************/

/**
 * @brief Read and return the next 8-bit value offset from \a ptr.
 * 
 * Returns the immediate 8-bit value in memory starting from the
 * current value of the \a ptr passed.
 *
 * @note As a side-effect, updates the number of bytes read in 
 * cpu.bytesconsumed.
 * 
 * @param [in] ptr The pointer into memory to read from.
 * @return The 8-bit value read.
 */
inline byte 
cpu_get_immediate8(word ptr)
{
    cpu.bytesconsumed++;
    return memory_read(ptr);
}

/************************************************************************/

/**
 * @brief Return the 16-bit address defined by DP:[ptr]
 *
 * Returns the 16-bit address defined by the Direct Page register 
 * combined with the 8-bit contents of the memory location defined by the 
 * 16-bit pointer. For example, if DP is set to FE and \a ptr is set to 
 * 129A, then the byte in memory location 129A is read (for example, 54)
 * and combined with the DP register to form the memory location FE54. 
 * 
 * @note As a side-effect, updates the number of bytes read in 
 * cpu.bytesconsumed.
 * 
 * @param [in] ptr The 16-bit pointer to a memory location.
 * @return The 16-bit address computed.
 */
inline word 
cpu_get_direct(word ptr)
{
    word address;
    address.BYTE.high = cpu.dp;
    address.BYTE.low = memory_read(ptr);
    cpu.bytesconsumed++;
    return address;
}

/************************************************************************/

/**
 * @brief Compute the indexed address based on the value in memory.
 * 
 * Given a pointer to memory beyond the op byte, the function will return 
 * a 16-bit pointer into memory where the indexed address is located.
 *
 * @param [in] ptr A 16-bit memory address pointer.
 * @return A 16-bit memory address pointer.
 */
inline word 
cpu_get_indexed(word ptr)
{
   word tword;
   word tword1;
   byte tbyte;
   word reg;
   int tint;
   int regflag;
   int addtoreg;
   word address;
   byte postbyte;

   addtoreg = 0;
   postbyte = memory_read(ptr);
   cpu.bytesconsumed++;

   /* Check for specific registers in bits 5 and 6*/
   switch (postbyte & 0x60) {

       case 0x00:
           reg.WORD = cpu.x.WORD;
           regflag = REG_X;
           break;

       case 0x20:
           reg.WORD = cpu.y.WORD;
           regflag = REG_Y;
           break;

       case 0x40:
           reg.WORD = cpu.u.WORD;
           regflag = REG_U;
           break;

       case 0x60:
           reg.WORD = cpu.s.WORD;
           regflag = REG_S;
           break;

       default:
           reg.BYTE.high = 0;
           reg.BYTE.low = 0;
           regflag = REG_UNK;
           break;
    }

    /* Check to see if we have a 5-bit signed offset */
    if ((postbyte & 0x80) == 0) {

        /* Calculate signed offset */
        tbyte = postbyte;
        tbyte = tbyte & 0x1F;

        /* Check to see if we are negative */
        tint = (tbyte & 0x10) ? -(tbyte & 0xF) : tbyte;
        address.WORD = reg.WORD + tint;
        return address;
    }

    /* Check the postbyte for the offset codes */
    switch (postbyte & 0xF) {

        case 0x00:
            /* ,R+ */
            tword.WORD = reg.WORD;
            addtoreg = 1;
            break;

        case 0x01:
            /* ,R++ */
            tword.WORD = reg.WORD;
            addtoreg = 2;
            break;

        case 0x02:
            /* ,-R */
            tword.WORD = reg.WORD;
            addtoreg = -1;
            break;

        case 0x03:
            /* ,--R */
            tword.WORD = reg.WORD;
            addtoreg = -2;
            break;

        case 0x04:
            /* ,R */
            tword.WORD = reg.WORD;
            break;

        case 0x05:
            /* B,R */
            tword.WORD = reg.WORD + (sbyte)cpu.b;
            break;

        case 0x06:
            /* A,R */
            tword.WORD = reg.WORD + (sbyte)cpu.a;
            break;
 
        case 0x08:
            /* nn,R - 8-bit offset */
            tbyte = memory_read(ptr);
            tword.WORD = reg.WORD + (sbyte)tbyte;
            cpu.bytesconsumed++;
            break;

        case 0x09:
            /* nnnn,R - 16-bit offset */
            tword1 = memory_read16(ptr);
            tbyte = memory_read(tword1);
            tword.WORD = reg.WORD + (sbyte)tbyte;
            cpu.bytesconsumed += 2;
            break;

        case 0x0B:
            /* D,R */
            tword1.BYTE.high = cpu.a;
            tword1.BYTE.low = cpu.b;
            tword = cpu_binary_add16(reg, tword1, FALSE, FALSE, FALSE);
            break;
     
        case 0x0C:
            /* nn,PC - 8-bit offset */
            tbyte = memory_read(ptr);
            tword.WORD = cpu.pc.WORD + (sbyte)tbyte;
            cpu.bytesconsumed = 1;
            break;

        case 0x0D:
            /* nnnn,PC - 16-bit offset */               
            tword1.BYTE.high = memory_read(ptr);
            ptr.WORD++;
            tword1.BYTE.low = memory_read(ptr);
            tword.WORD = cpu.pc.WORD + cpu_get_signed16(tword);
            cpu.bytesconsumed = 2;
            break;

        default:
            printf("Invalid indexed postbyte: [%X]", postbyte);
            tword.WORD = 0;
            break;    
    }
  
    /* If the 5-bit code starts with 1, then we do indirect addressing */
    if (postbyte & 0x10) {
        address.BYTE.high = memory_read(tword);
        tword.WORD++;
        address.BYTE.low = memory_read(tword);
        return address;
    }

    address.WORD = tword.WORD;
     
    /* Add the appropriate amount to the specified register */
    switch (regflag) {
        case REG_X:
            cpu.x.WORD += addtoreg;
            break;

        case REG_Y:
            cpu.y.WORD += addtoreg;
            break;

        case REG_U:
            cpu.u.WORD += addtoreg;
            break;

        case REG_S:
            cpu.s.WORD += addtoreg;
            break;
    }
    return address;
}

/************************************************************************/

/**
 * @brief Branch relative to the 8-bit offset if the test is true.
 *
 * This function will perform a relative branch to the specified 8-bit 
 * offset if the test condition is true. If offset is interpreted as a
 * signed integer. Thus, the topmost bit of offset is used to specify
 * whether the offset is positive or negative relative to the current
 * value of the program counter.
 *
 * @param [in] test The value to test for.
 * @param [in] offset The 8-bit offset to branch to.
 */
inline void 
cpu_branch_short(int test, byte offset)
{
    if (test) {
        cpu.pc.WORD += (offset & 0x80) ? -(offset & 0x7F) : offset;
        cpu.bytesconsumed = NO_PC_UPDATE;
    }
}

/************************************************************************/

/**
 * @brief Branch relative to the 16-bit offset if the test is true.
 *
 * This function will perform a relative branch to the specified 16-bit 
 * offset if the test condition is true. Returns TRUE (1) if the offset 
 * is a negative value, FALSE (0) if it is positive.
 *
 * @param [in] test The value to test for.
 * @param [in] offset The 16-bit offset to branch to.
 * @return TRUE (1) if the offset is negative, FALSE (0) otherwise.
 */
inline int 
cpu_branch_long(int test, word offset)
{
    if (test) { 
        cpu.pc.WORD += (offset.WORD & 0x8000) ? -(offset.WORD & 0x7FFF) : offset.WORD;
        cpu.bytesconsumed = NO_PC_UPDATE;
    }
    return offset.WORD & 0x8000;
}

/************************************************************************/

/**
 * @brief Reset the CPU.
 *
 * This function is responsible for resetting the CPU registers. It 
 * resets the program counter at PC_START. 
 */
void 
cpu_reset(void)
{
    int i;

    /* Reset the program counter */
/*
    start_loc.WORD = CPU_RESET_ADDR;
    cpu.pc = cpu_get_immediate16( start_loc );
*/
    cpu.pc.WORD = 0;

    /* Reset all general purpose registers */
    cpu.a = 0;
    cpu.b = 0;
    cpu.d.WORD = 0;
    cpu.dp = 0;
    cpu.cc = 0 | REG_CC_I | REG_CC_F;
    cpu.x.WORD = 0;
    cpu.y.WORD = 0;
    cpu.u.WORD = REG_U_START;
    cpu.s.WORD = REG_S_START;

    /* See the random number generator */
    srand(time(0));

    /* Set the execution time */
    cpu_optime = CPU_89MHZ;
    cpu.clockrate = 0;
    cpu_irq_enabled = 0;
    cpu_firq_enabled = 0;
    cpu_timer_value.WORD = 0;

    /* Set the CPU state */
    cpu.state = CPU_RUNNING;

    /* Reset the video starting address */
    screen_memory_ptr.BYTE.high = 0x2;
    screen_memory_ptr.BYTE.low = 0;
    screen_vdg_old_mode = 0;
    screen_low_res_enabled = TRUE;
    screen_buffer_size = 0;
    screen_border_color = 0;
    screen_alt_palette = FALSE;

    /* Reset the keypress state */
    keypress.BYTE.high = 255;
    keypress.BYTE.low = 255;

    /* Reset interrupt states */
    cpu_irq_value = 0;
    cpu_firq_value = 0;
    cpu_irq_flag = FALSE;
    cpu_firq_flag = FALSE;
    cpu_nmi_flag = FALSE;
    cpu_reset_flag = FALSE;

    /* Reset the number of CPU ticks that have passed */
    cpu_ticks = 0;
    cpu.opdesc = (char *)malloc(MAXSTRSIZE);
    cpu.opshortdesc = (char *)malloc(MAXSTRSIZE);
    sprintf(cpu.opdesc, "(null)");
    sprintf(cpu.opshortdesc, "(null)");

    /* Reset the MMU registers */
    for (i = 0; i < NUM_MMU_REGS; i++) {
        memory_mmu_task_set[i] = 0x38 + i;
        memory_mmu_exec_set[i] = 0x38 + i;
    }

    /* Initialize the video display */
    vdg_init();
}

/*****************************************************************************/

/**
 * @brief Proces SDL events.
 *
 * Process any events inside the SDL event queue. Will not block waiting for
 * events. Any event not processed by the emulator will be discarded.
 */
void 
cpu_process_sdl_events(void)
{
    if (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                cpu.state = CPU_STOP;
                break;

            case SDL_KEYDOWN:
                if (event.key.keysym.sym == QUIT_KEY) {
                    cpu.state = CPU_STOP;
                }
                else if (event.key.keysym.sym == DEBUG_KEY) {
                    cpu.state = CPU_DEBUG;
                }
                else if (event.key.keysym.sym == TRACE_KEY) {
                    cpu.state = CPU_TRACE;
                }
                else if (event.key.keysym.sym == NORMAL_KEY) {
                    cpu.state = CPU_RUNNING;
                }
                else if (event.key.keysym.sym == STEP_KEY) {
                    cpu.state = CPU_STEP;
                }
                break;

            default:
                break;
        }
    }
}

/*****************************************************************************/

/**
 * @brief Start execution of the CPU, starting at the current PC.
 *
 * This function contains the main CPU execution loop. It is responsible for 
 * checking the SDL event structure for input events. It then fetches and 
 * decodes the next instruction, executes it and restarts the loop. This
 * process continues until the cpu.state flag is set to STOP.
 */
void 
cpu_execute(void)
{
    byte opcode;        /* Stores the current opcode */
    byte operand1;      /* Stores the first operand */
    byte tbyte;         /* A temporary byte store */
    byte tbyte1;        /* A temporary byte store */
    byte tbyte2;        /* A temporary byte store */
    word tword;         /* A temporary word store */
    word tword1;        /* A temporary word store */
    word tword2;        /* A temporary word store */
    word tword3;        /* A temporary word store */
    word ptr;           /* A temporary memory pointer */
    int operationticks; /* Stores the number of ticks a CPU operation should take */
    word pc_save;       /* Stores the start of the operation */ 

    while (cpu.state != CPU_STOP) {

        cpu.oldpc.WORD = cpu.pc.WORD;
        pc_save.WORD = cpu.pc.WORD;
        opcode = memory_read(cpu.pc);
        cpu.operand = opcode;
        cpu.bytesconsumed = 1;
        ptr.WORD = cpu.pc.WORD + 1;

        switch (opcode) {

            /* NEG - Negate M */
            case 0x00:
            case 0x60:
            case 0x70:
                if (opcode == 0x00) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opdesc, "NEGM, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x60) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opdesc, "NEGM, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x70) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opdesc, "NEGM, EXT [%04X]", tword.WORD);
                }
                tbyte1 = memory_read(tword);
                tbyte = cpu_twos_comp8(tbyte1);
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                cpu.cc |= (tbyte & 0x80) ? REG_CC_V : 0;
                cpu.cc |= (tbyte == 0) ? REG_CC_C | REG_CC_N : 0;
                cpu.cc |= ((sbyte) tbyte < 0) ? REG_CC_N : 0;
                memory_write(tword, tbyte);
                sprintf(cpu.opdesc, "M' [%02X] = Negate M (twos comp)", tbyte);
                break;

            /* COM - Complement M */
            case 0x03:
            case 0x63:
            case 0x73:
                if (opcode == 0x03) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "COMM, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x63) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "COMM, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x73) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "COMM, EXT [%04X]", tword.WORD);
                }
                tbyte = ~(memory_read(tword));
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
                cpu.cc |= REG_CC_C;
                cpu.cc |= ((sbyte) tbyte < 0) ? REG_CC_N : 0;
                cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
                memory_write(tword, tbyte);
                sprintf(cpu.opdesc, "M' [%02X] = Complement M [%02X]", tbyte, ~tbyte);
                break;

            /* LSR - Logical Shift Right */
            case 0x04:
            case 0x64:
            case 0x74:
                if (opcode == 0x04) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "LSRM, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x64) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "LSRM, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x74) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "LSRM, IND [%04X]", tword.WORD);
                }
                tbyte = tbyte1 = memory_read(tword);
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
                cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
                tbyte = tbyte >> 1;
                cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
                memory_write(tword, tbyte);
                sprintf(cpu.opdesc, "M' [%02X] = Logical Shift Right M [%02X]", tbyte, tbyte1);
                break;

            /* ROR - Rotate Right */
            case 0x06:
            case 0x66:
            case 0x76:
                if (opcode == 0x06) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "RORM, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x66) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "RORM, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x76) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "RORM, EXT [%04X]", tword.WORD);
                }
                tbyte = memory_read(tword);
                tbyte1 = (cpu.cc & REG_CC_C) == REG_CC_C;
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
                cpu.cc |= (tbyte & 1) ? REG_CC_C : 0;
                tbyte2 = tbyte >> 1;
                cpu.cc |= (tbyte2 == 0) ? REG_CC_Z : 0;
                cpu.cc |= ((sbyte) tbyte2 < 0) ? REG_CC_N : 0;
                memory_write(tword, tbyte2);
                sprintf (cpu.opdesc, "M' [%02X] = Rotate Right M [%02X]", tbyte2, tbyte);
                break;

            /* ASR - Arithmetic Shift Right */
            case 0x07:
            case 0x67:
            case 0x77:
                if (opcode == 0x07) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "ASRM, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x67) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "ASRM, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x77) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "ASRM, EXT [%04X]", tword.WORD);
                }
                tbyte = memory_read(tword);
                tbyte1 = tbyte;
                tbyte = tbyte >> 1;
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
                cpu.cc |= (tbyte1 & 1) ? REG_CC_C : cpu.cc;
                cpu.cc |= (tbyte == 0) ? REG_CC_Z : cpu.cc;
                cpu.cc |= (tbyte < 0)  ? REG_CC_N : cpu.cc;
                tbyte |= (tbyte1 & 0x80) ? 0x80 : tbyte;
                memory_write(tword, tbyte);
                sprintf(cpu.opdesc, "M' [%02X] = Shift Right M [%02X]", tbyte, tbyte1);
                break;

            /* ASL - Arithmetic Shift Left */
            case 0x08:
            case 0x68:
            case 0x78:
                if (opcode == 0x08) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "ASL, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x68) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "ASL, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x78) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "ASL, EXT [%04X]", tword.WORD);
                }
                tbyte = memory_read(tword);
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                tbyte1 = tbyte;
                cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
                cpu.cc |= ((tbyte & 0x80) && (tbyte & 0x40)) ? REG_CC_V : 0;
                tbyte = tbyte << 1;
                cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
                cpu.cc |= ((sbyte)tbyte < 0 ) ? REG_CC_N : 0;
                memory_write(tword, tbyte);
                sprintf(cpu.opdesc, "M' [%02X] = Shift Left M [%02X]", tbyte, tbyte1);
                break;

            /* ROL - Rotate Left */
            case 0x09:
            case 0x69:
            case 0x79:
                if (opcode == 0x09) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "ROLM, DIR [%04X]", tword.WORD); 
                }
                else if (opcode == 0x69) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "ROLM, IND [%04X]", tword.WORD); 
                }
                else if (opcode == 0x79) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "ROLM, EXT [%04X]", tword.WORD); 
                }
                tbyte2 = memory_read(tword);
                tbyte1 = cpu.cc & REG_CC_C;
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                tbyte = tbyte2;
                cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
                cpu.cc |= ((tbyte & 0x80 ) ^ (tbyte & 0x40)) ? REG_CC_V : 0;
                tbyte = tbyte << 1;
                tbyte += (tbyte1) ? 1 : 0;
                cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
                cpu.cc |= ((sbyte) tbyte < 0) ? REG_CC_N : 0;
                memory_write(tword, tbyte);
                sprintf(cpu.opdesc, "M' [%02X] = Rotate Left M [%02X]", tbyte, tbyte2);
                break;

            /* DEC - Decrement M */
            case 0x0A:
            case 0x6A:
            case 0x7A:
                if (opcode == 0x0A) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "DEC M, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x6A) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "DEC M, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x7A) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "DEC M, EXT [%04X]", tword.WORD);
                }
                tbyte = memory_read(tword);
                tbyte2 = cpu_binary_add8(tbyte, 0xFF, FALSE, FALSE, FALSE);
                cpu.cc &= ~(REG_CC_V | REG_CC_Z | REG_CC_N);
                cpu.cc |= (tbyte == 0x80) ? REG_CC_V : 0;
                cpu.cc |= ((sbyte) tbyte2 < 0) ? REG_CC_N : 0;
                cpu.cc |= (tbyte2 == 0) ? REG_CC_Z : 0;
                memory_write(tword, tbyte2);
                sprintf(cpu.opdesc, "M' [%02X] = M [%02X] - 1", tbyte2, tbyte);
                break;

            /* INC - Increment M */
            case 0x0C:
            case 0x6C:
            case 0x7C:
                if (opcode == 0x0C) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "INC M, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x6C) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "INC M, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x7C) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "INC M, EXT [%04X]", tword.WORD);
                }
                tbyte = memory_read(tword);
                cpu.cc &= ~(REG_CC_V | REG_CC_Z | REG_CC_N);
                cpu.cc |= (tbyte == 0x7F) ? REG_CC_V : 0;
                tbyte1 = 1;
                tbyte2 = cpu_binary_add8(tbyte, tbyte1, FALSE, FALSE, FALSE);
                cpu.cc |= (tbyte2 == 0) ? REG_CC_Z : 0;
                cpu.cc |= ((sbyte) tbyte2 < 0) ? REG_CC_N : 0;
                memory_write(tword, tbyte2);
                sprintf(cpu.opdesc, "M' [%02X] = M [%02X] + 1", tbyte2, tbyte);
                break;

            /* TST - Test M */
            case 0x0D:
            case 0x6D:
            case 0x7D:
                if (opcode == 0x0D) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "TST M, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x6D) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "TST M, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x7D) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "TST M, EXT [%04X]", tword.WORD);
                }
                tbyte = memory_read(tword);
                cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
                cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
                cpu.cc |= ((sbyte) tbyte < 0) ? REG_CC_N : 0;
                sprintf(cpu.opdesc, "Test M [%02X]", tbyte);
                break;

            /* JMP - Jump to effective address */
            case 0x0E:
            case 0x6E:
            case 0x7E:
                if (opcode == 0x0E) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 3;
                    sprintf(cpu.opshortdesc, "JMP, DIR [%04X]", tword.WORD);
                }
                else if (opcode == 0x6E) {
                    tword = cpu_get_indexed(ptr);
                    operationticks = 1 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "JMP, IND [%04X]", tword.WORD);
                }
                else if (opcode == 0x7E) {
                    tword = ptr;
                    operationticks = 4;
                    sprintf(cpu.opshortdesc, "JMP, EXT [%04X]", tword.WORD);
                }
                cpu.pc = memory_read16(tword);
                cpu.bytesconsumed = NO_PC_UPDATE;
                sprintf(cpu.opshortdesc, "Jump to [%04X]", cpu.pc.WORD);
                break;

            /* CLR - Clear M */
            case 0x0F:
            case 0x6F:
            case 0x7F:
                if (opcode == 0x0F) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 6;
                    sprintf(cpu.opshortdesc, "CLR M, DIR [%04X]", tword.WORD);
                }
                if (opcode == 0x6F) {
                    tword = cpu_get_direct(ptr);
                    operationticks = 4 + cpu.bytesconsumed;
                    sprintf(cpu.opshortdesc, "CLR M, IND [%04X]", tword.WORD);
                }
                if (opcode == 0x7F) {
                    tword = memory_read16(cpu_get_immediate16(ptr));
                    operationticks = 7;
                    sprintf(cpu.opshortdesc, "CLR M, EXT [%04X]", tword.WORD);
                }
                memory_write(tword, 0);
                cpu.cc |= REG_CC_Z;
                cpu.cc &= ~(REG_CC_N | REG_CC_V | REG_CC_C);
                sprintf(cpu.opdesc, "Clear M");
                break;

            case 0x10: {
                opcode = cpu_get_immediate8(ptr);
                ptr.WORD++;
                switch (opcode) {

                    /* LBRN - Long branch never */
                    case 0x21:
                        tword = cpu_get_immediate16(ptr);
                        operationticks = 5;
                        sprintf(cpu.opshortdesc, "LBRN, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "Long branch never"); 
                        break;
 
                    /* LBHI - Long branch on higher */
                    case 0x22:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_C) == REG_CC_C;
                        tbyte1 = (cpu.cc & REG_CC_Z) == REG_CC_Z;
                        cpu_branch_long((tbyte | tbyte1) == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBHI, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF C [%X] OR Z [%X] = 0 THEN PC' [%04X] = PC [%04X] + M [%04X]",
                                tbyte, tbyte1, cpu.pc.WORD, pc_save.WORD,
                                tword.WORD & 0x7FFF);
                        break;
 
                    /* LBLS - Long branch on lower or same */
                    case 0x23:
                        tword = cpu_get_immediate16 (ptr);
                        tbyte = (cpu.cc & REG_CC_C) == REG_CC_C;
                        tbyte1 = (cpu.cc & REG_CC_Z) == REG_CC_Z;
                        cpu_branch_long((tbyte | tbyte1) == 1, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBLS, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF C [%X] OR Z [%X] = 1 THEN PC' [%04X] = PC [%04X] + M [%04X]",
                                tbyte, tbyte1, cpu.pc.WORD, pc_save.WORD, 
                                tword.WORD & 0x7FFF);
                        break;
 
                    /* LBCC - Long branch on carry clear */
                    case 0x24:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_C) == REG_CC_C;
                        cpu_branch_long(tbyte == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBCC, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF C [%X] = 0 THEN PC' [%04X] = PC [%04X] + M [%04X]",
                                tbyte, cpu.pc.WORD, pc_save.WORD, tword.WORD & 0x7FFF);
                        break;
 
                    /* LBCS - Long branch on carry set */
                    case 0x25:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_C) == REG_CC_C;
                        cpu_branch_long(tbyte, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBCS, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF C [%X] = 1 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBNE - Long branch not equal */
                    case 0x26:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_Z) == REG_CC_Z;
                        cpu_branch_long(tbyte == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBEQ, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF Z [%X] = 0 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBEQ - Long branch on equal */
                    case 0x27:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_Z) == REG_CC_Z;
                        cpu_branch_long(tbyte, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBEQ, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF Z [%X] = 1 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD );
                        break;
 
                    /* LBVC - Long branch on Overflow Clear */
                    case 0x28:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_V) == REG_CC_V;
                        cpu_branch_long(tbyte == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBVC, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF V [%X] = 0 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBVS - Long branch on Overflow Set */
                    case 0x29:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_V) == REG_CC_V;
                        cpu_branch_long(tbyte, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBVC, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF V [%X] = 1 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD);
                        break;
                 
                    /* LBPL - Long branch on plus */
                    case 0x2A:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
                        cpu_branch_long(tbyte == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBMI, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF N [%X] = 0 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBMI - Long branch on minus */
                    case 0x2B:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
                        cpu_branch_long(tbyte, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBMI, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF N [%X] = 1 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBGE - Long branch on greater than or equal to zero */
                    case 0x2C:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
                        tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
                        cpu_branch_long((tbyte ^ tbyte1) == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBGE, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF N [%X] XOR V [%X] = 0 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, tbyte1, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBLT - Long branch on less than zero */
                    case 0x2D:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
                        tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
                        cpu_branch_long((tbyte ^ tbyte1) == 1, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBGE, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF N [%X] XOR V [%X] = 1 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte, tbyte1, cpu.pc.WORD, pc_save.WORD );
                        break;
 
                    /* LBGT - Long branch on greater than zero */
                    case 0x2E:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
                        tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
                        tbyte2 = (cpu.cc & REG_CC_Z) == REG_CC_Z;
                        cpu_branch_long(((tbyte2) & (tbyte ^ tbyte1)) == 0, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBGT, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF Z [%X] AND [N [%X] XOR V [%X]] = 0 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte2, tbyte, tbyte1, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* LBLE - Long branch on less than or equal to zero */
                    case 0x2F:
                        tword = cpu_get_immediate16(ptr);
                        tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
                        tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
                        tbyte2 = (cpu.cc & REG_CC_Z) == REG_CC_Z;
                        cpu_branch_long(((tbyte2) | (tbyte ^ tbyte1)) == 1, tword);
                        operationticks = (cpu.bytesconsumed == NO_PC_UPDATE) ? 5 : 6;
                        sprintf(cpu.opshortdesc, "LBLE, REL [%04X]", tword.WORD);
                        sprintf(cpu.opdesc, "IFF Z [%X] OR [N [%X] XOR V [%X]] = 1 THEN PC' [%04X] = PC [%04X] + TEMP",
                                tbyte2, tbyte, tbyte1, cpu.pc.WORD, pc_save.WORD);
                        break;
 
                    /* SWI3- Software Interrupt 3 */
                    case 0x3F:
                        cpu.cc |= REG_CC_E;
                        cpu_push_stack(REG_S, cpu.pc.BYTE.low);
                        cpu_push_stack(REG_S, cpu.pc.BYTE.high);
                        cpu_push_stack(REG_S, cpu.u.BYTE.low);
                        cpu_push_stack(REG_S, cpu.u.BYTE.high);
                        cpu_push_stack(REG_S, cpu.y.BYTE.low);
                        cpu_push_stack(REG_S, cpu.y.BYTE.high);
                        cpu_push_stack(REG_S, cpu.x.BYTE.low);
                        cpu_push_stack(REG_S, cpu.x.BYTE.high);
                        cpu_push_stack(REG_S, cpu.dp);
                        cpu_push_stack(REG_S, cpu.b);
                        cpu_push_stack(REG_S, cpu.a);
                        cpu_push_stack(REG_S, cpu.cc);
                        tword.WORD = SWI3_INT;
                        cpu.pc = memory_read16(tword);
                        sprintf(cpu.opshortdesc, "SWI3");
                        sprintf(cpu.opdesc, "PC' [%04X] pushed PC U Y X DP B A CC\n", cpu.pc.WORD);
                        break;
 
                    /* CMPD - Compare D */
                    case 0x83:
                    case 0x93:
                    case 0xA3:
                    case 0xB3:
                        if (opcode == 0x83) {
                            tword3 = cpu_get_immediate16(ptr);
                            operationticks = 5;
                            sprintf(cpu.opshortdesc, "CMPD, IMM");
                        }
                        else if (opcode == 0x93) {
                            tword = cpu_get_direct(ptr);
                            tword3 = memory_read16(tword);
                            operationticks = 7;
                            sprintf(cpu.opshortdesc, "CMPD, DIR [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xA3) {
                            tword = cpu_get_indexed(ptr);
                            tword3 = memory_read16(tword);
                            operationticks = 5 + cpu.bytesconsumed;
                            sprintf(cpu.opshortdesc, "CMPD, IND [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xB3) {
                            tword = cpu_get_immediate16(ptr);
                            tword3 = memory_read16(tword);
                            operationticks = 8;
                            sprintf(cpu.opshortdesc, "CMPD, EXT [%04X]", tword.WORD);
                        }
                        tword1 = cpu_twos_comp16(tword3);
                        tword2 = cpu_binary_add16(cpu.d, tword1, FALSE, TRUE, TRUE);
                        cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                        cpu.cc |= (tword2.WORD == 0) ? REG_CC_Z : 0;
                        cpu.cc |= (cpu_get_signed16(tword2) < 0) ? REG_CC_N : 0;
                        sprintf(cpu.opdesc, "TEMP [%04X] = D [%04X] - %04X\n", tword2.WORD, cpu.d.WORD, tword3.WORD);
                        break;
   
                    /* CMPY - Compare Y */
                    case 0x8C:
                    case 0x9C:
                    case 0xAC:
                    case 0xBC:
                        if (opcode == 0x8C) {
                            tword3 = cpu_get_immediate16(ptr);
                            operationticks = 5;
                            sprintf(cpu.opshortdesc, "CMPY, IMM");
                        }
                        else if (opcode == 0x9C) {
                            tword = cpu_get_direct(ptr);
                            tword3 = memory_read16(tword);
                            operationticks = 7;
                            sprintf(cpu.opshortdesc, "CMPY, DIR [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xAC) {
                            tword = cpu_get_indexed(ptr);
                            tword3 = memory_read16(tword);
                            operationticks = 5 + cpu.bytesconsumed;
                            sprintf(cpu.opshortdesc, "CMPY, IND [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xBC) {
                            tword = cpu_get_immediate16(ptr);
                            tword3 = memory_read16(tword);
                            operationticks = 8;
                            sprintf(cpu.opshortdesc, "CMPY, EXT [%04X]", tword.WORD);
                        }
                        tword1 = cpu_twos_comp16(tword3);
                        tword2 = cpu_binary_add16(cpu.y, tword1, FALSE, TRUE, TRUE);
                        cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                        cpu.cc |= (tword2.WORD == 0) ? REG_CC_Z : 0;
                        cpu.cc |= (cpu_get_signed16(tword2) < 0) ? REG_CC_N : 0;
                        sprintf(cpu.opdesc, "TEMP [%04X] = Y [%04X] - %04X\n", tword2.WORD, cpu.y.WORD, tword3.WORD);
                        break;
   
                    /* LDY - Load Y */
                    case 0x8E:
                    case 0x9E:
                    case 0xAE:
                    case 0xBE:
                        if (opcode == 0x8E) {
                            cpu.y = cpu_get_immediate16(ptr);
                            operationticks = 4;
                            sprintf(cpu.opshortdesc, "LDY, IMM");
                        }
                        else if (opcode == 0x9E) {
                            tword = cpu_get_direct(ptr);
                            cpu.y = memory_read16(tword);
                            operationticks = 6;
                            sprintf(cpu.opshortdesc, "LDY, DIR [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xAE) {
                            tword = cpu_get_indexed(ptr);
                            cpu.y = memory_read16(tword);
                            operationticks = 4 + cpu.bytesconsumed;
                            sprintf(cpu.opshortdesc, "LDY, IND [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xBE) {
                            tword = cpu_get_immediate16(ptr);
                            cpu.y = memory_read16(tword);
                            operationticks = 7;
                            sprintf(cpu.opshortdesc, "LDY, EXT [%04X]", tword.WORD);
                        }
                        cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
                        cpu.cc |= (cpu.y.WORD == 0) ? REG_CC_C : 0;
                        cpu.cc |= (cpu_get_signed16(cpu.y) < 0) ? REG_CC_N : 0;
                        sprintf(cpu.opdesc, "Y' = [%04X]", cpu.y.WORD);
                        break;
   
                    /* STY - Store Y */
                    case 0x9F:
                    case 0xAF:
                    case 0xBF:
                        if (opcode == 0x9F) {
                            tword = cpu_get_direct(ptr);
                            operationticks = 6;
                            sprintf(cpu.opshortdesc, "STY, DIR [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xAF) {
                            tword = cpu_get_indexed(ptr);
                            operationticks = 4 + cpu.bytesconsumed;
                            sprintf(cpu.opshortdesc, "STY, IND [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xBF) {
                            tword = cpu_get_immediate16(ptr);
                            operationticks = 7;
                            sprintf(cpu.opshortdesc, "STY, EXT [%04X]", tword.WORD);
                        }
                        memory_write16(tword, cpu.y);
                        cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
                        cpu.cc |= (cpu.y.WORD == 0) ? REG_CC_C : 0;
                        cpu.cc |= (cpu_get_signed16(cpu.y) < 0) ? REG_CC_N : 0;
                        sprintf(cpu.opdesc, "M = Y [%04X]", cpu.y.WORD);
                        break;
   
                    /* LDS - Load S */
                    case 0xCE:
                    case 0xDE:
                    case 0xEE:
                    case 0xFE:
                        if (opcode == 0xCE) {
                            cpu.s = cpu_get_immediate16(ptr);
                            operationticks = 4;
                            sprintf(cpu.opshortdesc, "LDS, IMM");
                        }
                        else if (opcode == 0xDE) {
                            tword = cpu_get_direct(ptr);
                            cpu.s = memory_read16(tword);
                            operationticks = 6;
                            sprintf(cpu.opshortdesc, "LDS, DIR [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xEE) {
                            tword = cpu_get_indexed(ptr);
                            cpu.s = memory_read16(tword);
                            operationticks = 4 + cpu.bytesconsumed;
                            sprintf(cpu.opshortdesc, "LDS, IND [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xFE) {
                            tword = cpu_get_immediate16(ptr);
                            cpu.s = memory_read16(tword);
                            operationticks = 7;
                            sprintf(cpu.opshortdesc, "LDS, EXT [%04X]", tword.WORD);
                        }
                        cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
                        cpu.cc |= (cpu.s.WORD == 0) ? REG_CC_C : 0;
                        cpu.cc |= (cpu_get_signed16(cpu.s) < 0) ? REG_CC_N : 0;
                        sprintf(cpu.opdesc, "S' = [%04X]", cpu.s.WORD);
                        break;
   
                    /* STS - Store S */
                    case 0xDF:
                    case 0xEF:
                    case 0xFF:
                        if (opcode == 0xDF) {
                            tword = cpu_get_direct(ptr);
                            operationticks = 6;
                            sprintf(cpu.opshortdesc, "STS, DIR [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xEF) {
                            tword = cpu_get_indexed(ptr);
                            operationticks = 4 + cpu.bytesconsumed;
                            sprintf(cpu.opshortdesc, "STS, IND [%04X]", tword.WORD);
                        }
                        else if (opcode == 0xFF) {
                            tword = cpu_get_immediate16(ptr);
                            operationticks = 7;
                            sprintf(cpu.opshortdesc, "STS, EXT [%04X]", tword.WORD);
                        }
                        memory_write16(tword, cpu.s);
                        cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
                        cpu.cc |= (cpu.s.WORD == 0) ? REG_CC_C : 0;
                        cpu.cc |= (cpu_get_signed16(cpu.s) < 0) ? REG_CC_N : 0;
                        sprintf(cpu.opdesc, "M = Y [%04X]", cpu.s.WORD);
                        break;
   
                    default: 
                        sprintf(cpu.opshortdesc, "Illegal Opcode");
                        sprintf(cpu.opdesc, "Unimplemented opcode [%02X:%02X]", 16, opcode);
                        break;
            }
        }
        break;

        case 0x11: {
            opcode = cpu_get_immediate8(ptr);
            ptr.WORD++;
            cpu.bytesconsumed++;

            switch (opcode) {

                /* SWI2- Software Interrupt 2 */
                case 0x3F:
                    cpu.cc |= REG_CC_E;
                    cpu_push_stack(REG_S, cpu.pc.BYTE.low);
                    cpu_push_stack(REG_S, cpu.pc.BYTE.high);
                    cpu_push_stack(REG_S, cpu.u.BYTE.low);
                    cpu_push_stack(REG_S, cpu.u.BYTE.high);
                    cpu_push_stack(REG_S, cpu.y.BYTE.low);
                    cpu_push_stack(REG_S, cpu.y.BYTE.high);
                    cpu_push_stack(REG_S, cpu.x.BYTE.low);
                    cpu_push_stack(REG_S, cpu.x.BYTE.high);
                    cpu_push_stack(REG_S, cpu.dp);
                    cpu_push_stack(REG_S, cpu.b);
                    cpu_push_stack(REG_S, cpu.a);
                    cpu_push_stack(REG_S, cpu.cc);
                    tword.WORD = SWI2_INT;
                    cpu.pc = memory_read16(tword);
                    operationticks = 20;
                    sprintf(cpu.opshortdesc, "SWI2");
                    sprintf(cpu.opdesc, "PC' [%04X] pushed PC U Y X DP B A CC", cpu.pc.WORD);
                    break;

                /* CMPU - Compare U */
                case 0x83:
                case 0x93:
                case 0xA3:
                case 0xB3:
                    if (opcode == 0x83) {
                        tword3 = cpu_get_immediate16(ptr);
                        operationticks = 5;
                        sprintf(cpu.opshortdesc, "CMPU, IMM");
                    }
                    else if (opcode == 0x93) {
                        tword = cpu_get_direct(ptr);
                        tword3 = memory_read16(tword);
                        operationticks = 7;
                        sprintf(cpu.opshortdesc, "CMPU, DIR [%04X]", tword.WORD);
                    }
                    else if (opcode == 0xA3) {
                        tword = cpu_get_indexed(ptr);
                        tword3 = memory_read16(tword);
                        operationticks = 5 + cpu.bytesconsumed;
                        sprintf(cpu.opshortdesc, "CMPU, IND [%04X]", tword.WORD);
                    }
                    else if (opcode == 0xB3) {
                        tword = cpu_get_immediate16(ptr);
                        tword3 = memory_read16(tword);
                        operationticks = 8;
                        sprintf(cpu.opshortdesc, "CMPU, EXT [%04X]", tword.WORD);
                    }
                    tword1 = cpu_twos_comp16(tword3);
                    tword2 = cpu_binary_add16(cpu.u, tword1, FALSE, TRUE, TRUE);
                    cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                    cpu.cc |= (tword2.WORD == 0) ? REG_CC_Z : 0;
                    cpu.cc |= (cpu_get_signed16(tword2) < 0) ? REG_CC_N : 0;
                    sprintf(cpu.opdesc, "TEMP [%04X] = U [%04X] - %04X", tword2.WORD, cpu.u.WORD, tword3.WORD );
                    break;

                /* CMPS - Compare S */
                case 0x8C:
                case 0x9C:
                case 0xAC:
                case 0xBC:
                    if (opcode == 0x8C) {
                        tword3 = cpu_get_immediate16(ptr);
                        operationticks = 5;
                        sprintf(cpu.opshortdesc, "CMPS, IMM");
                    }
                    else if (opcode == 0x9C) {
                        tword = cpu_get_direct(ptr);
                        tword3 = memory_read16(tword);
                        operationticks = 7;
                        sprintf(cpu.opshortdesc, "CMPS, DIR [%04X]", tword.WORD);
                    }
                    else if (opcode == 0xAC) {
                        tword = cpu_get_indexed(ptr);
                        tword3 = memory_read16(tword);
                        operationticks = 5 + cpu.bytesconsumed;
                        sprintf(cpu.opshortdesc, "CMPS, IND [%04X]", tword.WORD);
                    }
                    else if (opcode == 0xBC) {
                        tword = cpu_get_immediate16(ptr);
                        tword3 = memory_read16(tword);
                        operationticks = 8;
                        sprintf(cpu.opshortdesc, "CMPS, EXT [%04X]", tword.WORD);
                    }
                    tword1 = cpu_twos_comp16(tword3);
                    tword2 = cpu_binary_add16(cpu.s, tword1, FALSE, TRUE, TRUE);
                    cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
                    cpu.cc |= (tword2.WORD == 0) ? REG_CC_Z : 0;
                    cpu.cc |= (cpu_get_signed16(tword2) < 0) ? REG_CC_N : 0;
                    sprintf(cpu.opdesc, "TEMP [%04X] = S [%04X] - %04X\n", tword2.WORD, cpu.s.WORD, tword3.WORD );
                    break;

                default:
                    sprintf(cpu.opshortdesc, "Illegal Opcode");
                    sprintf(cpu.opdesc, "Unimplemented opcode [%02X:%02X]", 17, opcode);
                    break;
                }
            }
            break;

        /* NOP - No Operation */
        case 0x12:
            operationticks = 2;
            sprintf(cpu.opshortdesc, "NOP, INH");
            sprintf(cpu.opdesc, "NOP");
            break;
  
        /* SYNC - Sync */
        case 0x13:
            sprintf(cpu.opshortdesc, "SYNC, INH");
            sprintf(cpu.opdesc, "SYNC");
            break;
 
        /* LBRA - Long branch always */
        case 0x16:
            tword = cpu_get_immediate16(ptr);
            cpu_branch_long(TRUE, tword);
            operationticks = 5;
            sprintf(cpu.opshortdesc, "LBRA, REL [%04X]", tword.WORD);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;
        
        /* DAA - Decimal Addition Adjust */
        case 0x19:
            tbyte = cpu.a & 0xF0;
            tbyte1 = cpu.a & 0x0F;
            tbyte = tbyte >> 4;
            tbyte2 = 0;
            if ((cpu.cc & REG_CC_C) || (tbyte > 9) || (tbyte > 8 && tbyte1 > 9)) {
                tbyte2 += 6;
            }
            tbyte2 = tbyte2 << 4; 
            if ((cpu.cc & REG_CC_C) || (tbyte1 > 9)) {
                tbyte2 += 6;
            }
            tbyte = cpu.cc & REG_CC_C;
            cpu.a = cpu_binary_add8(cpu.a, tbyte2, FALSE, TRUE, FALSE);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.cc |= (tbyte) ? REG_CC_C : 0;
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.high = cpu.a;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "DAA, INH");
            sprintf(cpu.opdesc, "A' [%02X]", cpu.a);
            break;

        /* ORCC - Logical OR */
        case 0x1A:
            operand1 = cpu_get_immediate8(ptr);
            cpu.cc |= operand1;
            operationticks = 3;
            sprintf(cpu.opshortdesc, "ORCC, IMM [%02X]", operand1);
            sprintf(cpu.opdesc, "CC' [%02X]", cpu.cc);
            break;

        /* ANDCC - Logical AND */
        case 0x1C:
            operand1 = cpu_get_immediate8(ptr);
            cpu.cc &= operand1;
            operationticks = 3;
            sprintf(cpu.opshortdesc, "ANDCC, IMM [%02X]", operand1);
            sprintf(cpu.opdesc, "CC' [%02X]", cpu.cc);
            break;
 
        /* SEX - sign extend */
        case 0x1D:
            tbyte = cpu.a;
            cpu.a = (cpu.b & 0x80) ? 0xFF : 0x00;
            cpu.d.BYTE.high = cpu.a;
            cpu.d.BYTE.low = cpu.b;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "SEX, INH");
            sprintf(cpu.opdesc, "D' [%04X], A' [%02X] <- A [%02X]", cpu.d.WORD, cpu.a, tbyte);
            break;

        /* EXG - Exchange registers */
        case 0x1E:
            tbyte1 = cpu_get_immediate8(ptr);
            sprintf(cpu.opshortdesc, "EXG, IMM");
            switch (tbyte1) {

                /* A:B <-> X */
                case 0x01: 
                case 0x10:
                    cpu.d.WORD = cpu.x.WORD;
                    cpu.x.BYTE.high = cpu.a;
                    cpu.x.BYTE.low = cpu.b;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "A:B [%04X] <-> X [%04X]", cpu.x.WORD, cpu.d.WORD);
                    break;

                /* A:B <-> Y */
                case 0x02:
                case 0x20:
                    cpu.d.WORD = cpu.y.WORD;
                    cpu.y.BYTE.high = cpu.a;
                    cpu.y.BYTE.low = cpu.b;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "A:B [%04X] <-> Y [%04X]", cpu.y.WORD, cpu.d.WORD);
                    break;

                /* A:B <-> U */
                case 0x03:
                case 0x30:
                    cpu.d.WORD = cpu.u.WORD;
                    cpu.u.BYTE.high = cpu.a;
                    cpu.u.BYTE.low = cpu.b;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "A:B [%04X] <-> U [%04X]", cpu.u.WORD, cpu.d.WORD );
                    break;

                /* A:B <-> S */
                case 0x04:
                case 0x40:
                    cpu.d.WORD = cpu.s.WORD;
                    cpu.s.BYTE.high = cpu.a;
                    cpu.s.BYTE.low = cpu.b;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "A:B [%04X] <-> S [%04X]", cpu.s.WORD, cpu.d.WORD);
                    break;

                /* A:B <-> PC */
                case 0x05:
                case 0x50:
                    cpu.d.WORD = cpu.pc.WORD;
                    cpu.pc.BYTE.high = cpu.a;
                    cpu.pc.BYTE.low = cpu.b;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "A:B [%04X] <-> PC [%04X]", cpu.pc.WORD, cpu.d.WORD);
                    break;

                /* X <-> Y */
                case 0x12:
                case 0x21:
                    tword.WORD = cpu.x.WORD;
                    cpu.x.WORD = cpu.y.WORD;
                    cpu.y.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "X [%04X] <-> Y [%04X]", cpu.y.WORD, cpu.x.WORD);
                    break;
 
                /* X <-> U */
                case 0x13:
                case 0x31:
                    tword.WORD = cpu.x.WORD;
                    cpu.x.WORD = cpu.u.WORD;
                    cpu.u.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "X [%04X] <-> U [%04X]", cpu.u.WORD, cpu.x.WORD);
                    break;

                /* X <-> S */
                case 0x14:
                case 0x41:
                    tword.WORD = cpu.x.WORD;
                    cpu.x.WORD = cpu.s.WORD;
                    cpu.s.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "X [%04X] <-> S [%04X]", cpu.s.WORD, cpu.x.WORD);
                    break;

                /* X <-> PC */
                case 0x15:
                case 0x51:
                    tword.WORD = cpu.x.WORD;
                    cpu.x.WORD = cpu.pc.WORD;
                    cpu.pc.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "X [%04X] <-> PC [%04X]", cpu.pc.WORD, cpu.x.WORD);
                    break;

                /* Y <-> U */
                case 0x23:
                case 0x32:
                    tword.WORD = cpu.y.WORD;
                    cpu.y.WORD = cpu.u.WORD;
                    cpu.u.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "Y [%04X] <-> U [%04X]", cpu.y.WORD, cpu.u.WORD);
                    break;

                /* Y <-> S */
                case 0x24:
                case 0x42:
                    tword.WORD = cpu.y.WORD;
                    cpu.y.WORD = cpu.s.WORD;
                    cpu.s.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "Y [%04X] <-> S [%04X]", cpu.y.WORD, cpu.s.WORD);
                    break;

                /* Y <-> PC */
                case 0x25:
                case 0x52:
                    tword.WORD = cpu.y.WORD;
                    cpu.y.WORD = cpu.pc.WORD;
                    cpu.pc.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "Y [%04X] <-> PC [%04X]", cpu.y.WORD, cpu.pc.WORD);
                    break;

                /* U <-> S */
                case 0x34:
                case 0x43:
                    tword.WORD = cpu.u.WORD;
                    cpu.u.WORD = cpu.s.WORD;
                    cpu.s.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "U [%04X] <-> S [%04X]", cpu.u.WORD, cpu.s.WORD);
                    break;

                /* U <-> PC */
                case 0x35:
                case 0x53:
                    tword.WORD = cpu.u.WORD;
                    cpu.u.WORD = cpu.pc.WORD;
                    cpu.pc.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "U [%04X] <-> PC [%04X]", cpu.u.WORD, cpu.pc.WORD);
                    break;

                /* S <-> PC */
                case 0x45:
                case 0x54:
                    tword.WORD = cpu.s.WORD;
                    cpu.s.WORD = cpu.pc.WORD;
                    cpu.pc.WORD = tword.WORD;
                    sprintf(cpu.opdesc, "S [%04X] <-> PC [%04X]", cpu.s.WORD, cpu.pc.WORD);
                    break;

                /* A <-> B */ 
                case 0x89:
                case 0x98:
                    tbyte = cpu.a;
                    cpu.a = cpu.b;
                    cpu.b = tbyte;
                    cpu.d.BYTE.high = cpu.a;
                    cpu.d.BYTE.low = cpu.b;
                    sprintf(cpu.opdesc, "A [%02X] <-> B [%02X]", cpu.a, cpu.b);
                    break;

                /* A <-> CC */
                case 0x8A:
                case 0xA8:
                    tbyte = cpu.a;
                    cpu.a = cpu.cc;
                    cpu.cc = tbyte;
                    cpu.d.BYTE.high = cpu.a;
                    sprintf(cpu.opdesc, "A [%02X] <-> CC [%02X]", cpu.a, cpu.cc);
                    break;
                 
                /* A <-> DP */
                case 0x8B:
                case 0xB8:
                    tbyte = cpu.a;
                    cpu.a = cpu.dp;
                    cpu.dp = tbyte;
                    cpu.d.BYTE.high = cpu.a;
                    sprintf(cpu.opdesc, "A [%02X] <-> DP [%02X]", cpu.a, cpu.dp);
                    break;
                 
                /* B <-> CC */
                case 0x9A:
                case 0xA9:
                    tbyte = cpu.b;
                    cpu.b = cpu.cc;
                    cpu.cc = tbyte;
                    cpu.d.BYTE.low = cpu.b;
                    sprintf(cpu.opdesc, "B [%02X] <-> CC [%02X]", cpu.b, cpu.cc);
                    break;
                 
                /* B <-> DP */
                case 0x9B:
                case 0xB9:
                    tbyte = cpu.b;
                    cpu.b = cpu.dp;
                    cpu.dp = tbyte;
                    cpu.d.BYTE.low = cpu.b;
                    sprintf(cpu.opdesc, "B [%02X] <-> DP [%02X]", cpu.b, cpu.dp);
                    break;
                 
                /* CC <-> DP */
                case 0xAB:
                case 0xBA:
                    tbyte = cpu.cc;
                    cpu.cc = cpu.dp;
                    cpu.dp = tbyte;
                    sprintf(cpu.opdesc, "CC [%02X] <-> DP [%02X]", cpu.cc, cpu.dp);
                    break;

                case 0x00:
                case 0x11:
                case 0x22:
                case 0x33:
                case 0x44:
                case 0x55:
                case 0x88:
                case 0x99:
                case 0xAA:
                case 0xBB:
                    sprintf(cpu.opdesc, "Self to self ignored");
                    break;

                default: 
                    sprintf(cpu.opdesc, "[%02X] illegal!", tbyte1);
                    break;
            }     
            operationticks = 8;
            break;

        /* TFR - Transfer between registers */
        case 0x1F:
            tbyte1 = cpu_get_immediate8(ptr);
            sprintf(cpu.opshortdesc, "TFR, IMM");
            switch (tbyte1) {
 
                case 0x01:
                    cpu.x.WORD = cpu.d.WORD;
                    sprintf(cpu.opdesc, "A:B -> X [%04X]", cpu.x.WORD);
                    break;

                case 0x10:
                    cpu.d.WORD = cpu.x.WORD;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "X -> A:B [%04X]", cpu.d.WORD);
                    break;

                case 0x02:
                    cpu.y.WORD = cpu.d.WORD;
                    sprintf(cpu.opdesc, "A:B -> Y [%04X]", cpu.y.WORD);
                    break;

                case 0x20:
                    cpu.d.WORD = cpu.y.WORD;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "Y -> D [%04X]", cpu.d.WORD);
                    break;

                case 0x03:
                    cpu.u.WORD = cpu.d.WORD;
                    sprintf(cpu.opdesc, "A:B -> U [%04X]", cpu.u.WORD);
                    break;

                case 0x30:
                    cpu.d.WORD = cpu.u.WORD;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "U -> A:B [%04X]", cpu.d.WORD);
                    break;

                case 0x04:
                    cpu.s.WORD = cpu.d.WORD;
                    sprintf(cpu.opdesc, "A:B -> S [%04X]", cpu.s.WORD);
                    break;

                case 0x40:
                    cpu.d.WORD = cpu.s.WORD;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "S -> A:B [%04X]", cpu.d.WORD);
                    break;

                case 0x05:
                    cpu.pc.WORD = cpu.d.WORD;
                    sprintf(cpu.opdesc, "A:B -> PC [%04X]", cpu.pc.WORD);
                    break;

                case 0x50:
                    cpu.d.WORD = cpu.pc.WORD;
                    cpu.a = cpu.d.BYTE.high;
                    cpu.b = cpu.d.BYTE.low;
                    sprintf(cpu.opdesc, "PC -> A:B [%04X]", cpu.d.WORD);
                    break;

                case 0x12:
                    cpu.y.WORD = cpu.x.WORD;
                    sprintf(cpu.opdesc, "X -> Y [%04X]",  cpu.y.WORD);
                    break;

                case 0x21:
                    cpu.x.WORD = cpu.y.WORD;
                    sprintf(cpu.opdesc, "Y -> X [%04X]", cpu.x.WORD);
                    break;

                case 0x13:
                    cpu.u.WORD = cpu.x.WORD;
                    sprintf(cpu.opdesc, "X -> U [%04X]", cpu.u.WORD);
                    break;

                case 0x31:
                    cpu.x.WORD = cpu.u.WORD;
                    sprintf(cpu.opdesc, "U <-> X [%04X]", cpu.x.WORD);
                    break;

                case 0x14:
                    cpu.s.WORD = cpu.x.WORD;
                    sprintf(cpu.opdesc, "X -> S [%04X]", cpu.s.WORD);
                    break;

                case 0x41:
                    cpu.x.WORD = cpu.s.WORD;
                    sprintf(cpu.opdesc, "S -> X [%04X]", cpu.x.WORD);
                    break;

                case 0x15:
                    cpu.pc.WORD = cpu.x.WORD;
                    sprintf(cpu.opdesc, "X -> PC [%04X]", cpu.pc.WORD);
                    break;

                case 0x51:
                    cpu.x.WORD = cpu.pc.WORD;
                    sprintf(cpu.opdesc, "PC -> X [%04X]", cpu.x.WORD);
                    break;

                case 0x23:
                    cpu.u.WORD = cpu.y.WORD;
                    sprintf(cpu.opdesc, "Y -> U [%04X]", cpu.u.WORD);
                    break;

                case 0x32:
                    cpu.y.WORD = cpu.u.WORD;
                    sprintf(cpu.opdesc, "U -> Y [%04X]", cpu.y.WORD);
                    break;

                case 0x24:
                    cpu.s.WORD = cpu.y.WORD;
                    sprintf(cpu.opdesc, "Y -> S [%04X]", cpu.y.WORD);
                    break;

                case 0x42:
                    cpu.y.WORD = cpu.s.WORD;
                    sprintf(cpu.opdesc, "S -> Y [%04X]", cpu.y.WORD);
                    break;

                case 0x25:
                    cpu.pc.WORD = cpu.y.WORD;
                    sprintf(cpu.opdesc, "Y -> PC [%04X]", cpu.pc.WORD);
                    break;

                case 0x52:
                    cpu.y.WORD = cpu.pc.WORD;
                    sprintf(cpu.opdesc, "PC <-> Y [%04X]", cpu.y.WORD);
                    break;

                case 0x34:
                    cpu.s.WORD = cpu.u.WORD;
                    sprintf(cpu.opdesc, "U -> S [%04X]", cpu.s.WORD);
                    break;

                case 0x43:
                    cpu.u.WORD = cpu.s.WORD;
                    sprintf(cpu.opdesc, "S -> U [%04X]", cpu.u.WORD);
                    break;

                case 0x35:
                    cpu.pc.WORD = cpu.u.WORD;
                    sprintf(cpu.opdesc, "U -> PC [%04X]", cpu.pc.WORD);
                    break;

                case 0x53:
                    cpu.u.WORD = cpu.pc.WORD;
                    sprintf(cpu.opdesc, "PC -> U [%04X]", cpu.u.WORD);
                    break;

                case 0x45:
                    cpu.pc.WORD = cpu.s.WORD;
                    sprintf(cpu.opdesc, "S -> PC [%04X]", cpu.pc.WORD);
                    break;

                case 0x54:
                    cpu.s.WORD = cpu.pc.WORD;
                    sprintf(cpu.opdesc, "PC -> S [%04X]", cpu.s.WORD);
                    break;

                case 0x89:
                    cpu.b = cpu.a;
                    cpu.d.BYTE.low = cpu.b;
                    sprintf(cpu.opdesc, "A -> B [%02X]", cpu.b);
                    break;

                case 0x98:
                    cpu.a = cpu.b;
                    cpu.d.BYTE.high = cpu.a;
                    sprintf(cpu.opdesc, "B -> A [%02X]", cpu.a);
                    break;

                case 0x8A:
                    cpu.cc = cpu.a;
                    sprintf(cpu.opdesc, "A -> CC [%02X]", cpu.cc);
                    break;

                case 0xA8:
                    cpu.a = cpu.cc;
                    cpu.d.BYTE.high = cpu.a;
                    sprintf(cpu.opdesc, "CC -> A [%02X]", cpu.a);
                    break;

                case 0x8B:
                    cpu.dp = cpu.a;
                    sprintf(cpu.opdesc, "A -> DP [%02X]", cpu.dp);
                    break;

                case 0xB8:
                    cpu.a = cpu.dp;
                    cpu.d.BYTE.high = cpu.a;
                    sprintf(cpu.opdesc, " DP -> A [%02X]", cpu.a);
                    break;

                case 0x9A:
                    cpu.cc = cpu.b;
                    sprintf(cpu.opdesc, "B -> CC [%02X]", cpu.cc);
                    break;

                case 0xA9:
                    cpu.b = cpu.cc;
                    cpu.d.BYTE.low = cpu.b;
                    sprintf(cpu.opdesc, "CC -> B [%02X]", cpu.b);
                    break;

                case 0x9B:
                    cpu.dp = cpu.b;
                    sprintf(cpu.opdesc, "B -> DP [%02X]", cpu.dp);
                    break;

                case 0xB9:
                    cpu.b = cpu.dp;
                    cpu.d.BYTE.low = cpu.b;
                    sprintf(cpu.opdesc, "DP -> B [%02X]", cpu.b);
                    break;

                case 0xAB:
                    cpu.dp = cpu.cc;
                    sprintf(cpu.opdesc, "CC -> DP [%02X]", cpu.dp);
                    break;
  
                case 0xBA:
                    cpu.cc = cpu.dp;
                    sprintf(cpu.opdesc, "DP -> CC [%02X]", cpu.cc);
                    break;

                case 0x00:
                case 0x11:
                case 0x22:
                case 0x33:
                case 0x44:
                case 0x55:
                case 0x88:
                case 0x99:
                case 0xAA:
                case 0xBB:
                    sprintf(cpu.opdesc, "Self to self ignored");
                    break;

                default:
                    sprintf(cpu.opdesc, "[%02X] illegal!", tbyte1);
                    break;
            }
            operationticks = 6;
            break;

        /* BRA - Branch always */
        case 0x20:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short(TRUE, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BRA, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BRN - Branch never */
        case 0x21:
            operand1 = cpu_get_immediate8(ptr);
            operationticks = 3; 
            sprintf(cpu.opshortdesc, "BRN, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BHI - Branch if Higher */
        case 0x22:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short(((cpu.cc & REG_CC_C) | (cpu.cc & REG_CC_Z)) == 0, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BHI, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BLE - Branch if Less than or Equal to Zero */
        case 0x23:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short(((cpu.cc & REG_CC_C) | (cpu.cc & REG_CC_Z)), operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BLE, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BCC - Branch on Carry Clear */
        case 0x24:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_C) == 0, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BCC, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BCS - Branch on Carry Set */
        case 0x25:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_C), operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BCS, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BNE - Branch Not Equal */
        case 0x26:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_Z) == 0, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BNE, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BEQ - Branch on Equal */
        case 0x27:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_Z), operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BEQ, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BVC - Branch on Overflow Clear */
        case 0x28:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_V) == 0, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BVC, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BVS - Branch on Overflow Set */
        case 0x29:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_V), operand1 );
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BVS, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BPL - Branch on Plus */
        case 0x2A:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_N) == 0, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BPL, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BMI - Branch on Minus */
        case 0x2B:
            operand1 = cpu_get_immediate8(ptr);
            cpu_branch_short((cpu.cc & REG_CC_N), operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BMI, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BGE - Branch on Greater than or Equal to Zero */
        case 0x2C:
            operand1 = cpu_get_immediate8(ptr);
            tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
            tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
            cpu_branch_short((tbyte ^ tbyte1) == 0, operand1); 
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BGE, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BLT - Branch on Less than Zero */
        case 0x2D:
            operand1 = cpu_get_immediate8(ptr);
            tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
            tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
            cpu_branch_short((tbyte ^ tbyte1) == 1, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BLT, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BGT - Branch on Greater */
        case 0x2E:
            operand1 = cpu_get_immediate8(ptr);
            tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
            tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
            tbyte2 = (cpu.cc & REG_CC_Z) == REG_CC_Z;
            cpu_branch_short((tbyte2 & (tbyte ^ tbyte1)) == 0, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BGT, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* BLE - Branch on Less than or Equal to Zero */
        case 0x2F:
            operand1 = cpu_get_immediate8(ptr);
            tbyte = (cpu.cc & REG_CC_N) == REG_CC_N;
            tbyte1 = (cpu.cc & REG_CC_V) == REG_CC_V;
            tbyte2 = (cpu.cc & REG_CC_Z) == REG_CC_Z;
            cpu_branch_short((tbyte2 | (tbyte ^ tbyte1)) == 1, operand1);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "BLE, REL [%02X]", operand1);
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;

        /* LEAX - Load Effective Address into X register */
        case 0x30:
            tword = cpu_get_indexed(ptr);
            cpu.x = memory_read16(tword);
            cpu.cc &= ~(REG_CC_Z);
            cpu.cc |= (cpu.x.WORD == 0) ? REG_CC_Z : 0;
            operationticks = 2 + cpu.bytesconsumed;
            sprintf(cpu.opshortdesc, "LEAX, IND [%04X]", tword.WORD);
            sprintf(cpu.opdesc, "X' [%04X]", cpu.x.WORD);
            break;

        /* LEAY - Load Effective Address into Y register */
        case 0x31:
            tword = cpu_get_indexed(ptr);
            cpu.y = memory_read16(tword);
            cpu.cc &= ~(REG_CC_Z);
            cpu.cc |= (cpu.y.WORD == 0) ? REG_CC_Z : 0;
            operationticks = 2 + cpu.bytesconsumed;
            sprintf(cpu.opshortdesc, "LEAY, IND [%04X]", tword.WORD);
            sprintf(cpu.opdesc, "Y' [%04X]", cpu.y.WORD);
            break;

        /* LEAS - Load Effective Address into S register */
        case 0x32:
            tword = cpu_get_indexed(ptr);
            cpu.s = memory_read16(tword);
            operationticks = 2 + cpu.bytesconsumed;
            sprintf(cpu.opshortdesc, "LEAS, IND [%04X]", tword.WORD);
            sprintf(cpu.opdesc, "S' [%04X]", cpu.s.WORD);
            break;

        /* LEAU - Load Effective Address into U register */
        case 0x33:
            tword = cpu_get_indexed(ptr);
            cpu.u = memory_read16(tword);
            operationticks = 2 + cpu.bytesconsumed;
            sprintf(cpu.opshortdesc, "LEAU, IND [%04X]", tword.WORD);
            sprintf(cpu.opdesc, "U' [%04X]", cpu.u.WORD);
            break;

        /* PSHS, PSHU - Push Registers on stack */
        case 0x34:
        case 0x36:
            tbyte = cpu_get_immediate8(ptr);
            if (opcode == 0x34) {
                tbyte1 = REG_S;
                sprintf(cpu.opshortdesc, "PSHS, IMM postbyte [%02X]", tbyte);
            }
            else if (opcode == 0x36) {
                tbyte1 = REG_U;
                sprintf(cpu.opshortdesc, "PSHU, IMM postbyte [%02X]", tbyte);
            }
            operationticks = 5;
            sprintf(cpu.opdesc, "Pushed ");
            if (tbyte & 0x80) {
               cpu_push_stack(tbyte1, cpu.pc.BYTE.low);
               cpu_push_stack(tbyte1, cpu.pc.BYTE.high);
               operationticks += 2;
               strcat(cpu.opdesc, " PC");
            }
            if (tbyte & 0x40) {
               cpu_push_stack(tbyte1, cpu.u.BYTE.low);
               cpu_push_stack(tbyte1, cpu.u.BYTE.high);
               operationticks += 2;
               strcat(cpu.opdesc, " U");
            }
            if (tbyte & 0x20) {
               cpu_push_stack(tbyte1, cpu.y.BYTE.low);
               cpu_push_stack(tbyte1, cpu.y.BYTE.high);
               operationticks += 2;
               strcat(cpu.opdesc, " Y");
            }
            if (tbyte & 0x10) {
               cpu_push_stack(tbyte1, cpu.x.BYTE.low);
               cpu_push_stack(tbyte1, cpu.x.BYTE.high);
               operationticks += 2;
               strcat(cpu.opdesc, " X");
            }
            if (tbyte & 0x08) {
               cpu_push_stack(tbyte1, cpu.dp);
               operationticks += 1;
               strcat(cpu.opdesc, " DP");
            }
            if (tbyte & 0x04) {
               cpu_push_stack(tbyte1, cpu.b);
               operationticks += 1;
               strcat(cpu.opdesc, " B");
            }
            if (tbyte & 0x02) {
               cpu_push_stack(tbyte1, cpu.a);
               operationticks += 1;
               strcat(cpu.opdesc, " A");
            }
            if (tbyte & 0x01) {
               cpu_push_stack(tbyte1, cpu.cc);
               operationticks += 1;
               strcat(cpu.opdesc, " CC");
            }
            break;

        /* PULS, PULU - Pull Registers from the stack */
        case 0x35:
        case 0x37:
            tbyte = cpu_get_immediate8(ptr);
            if (opcode == 0x35) {
                tbyte1 = REG_S;
                sprintf(cpu.opshortdesc, "PULS, IMM postbyte [%02X]", tbyte);
            }
            else if (opcode == 0x37) {
                tbyte1 = REG_S;
                sprintf(cpu.opshortdesc, "PULU, IMM postbyte [%02X]", tbyte);
            }
            operationticks = 5;
            sprintf(cpu.opdesc, "Pulled ");
            if (tbyte & 0x01) {
               cpu.cc = cpu_pop_stack(tbyte1);
               operationticks += 1;
               strcat(cpu.opdesc, " CC");
            }
            if (tbyte & 0x02) {
               cpu.a = cpu_pop_stack(tbyte1);
               cpu.d.BYTE.high = cpu.a;
               operationticks += 1;
               strcat(cpu.opdesc, " A");
            }
            if (tbyte & 0x04) {
               cpu.b = cpu_pop_stack(tbyte1);
               cpu.d.BYTE.low = cpu.b;
               operationticks += 1;
               strcat(cpu.opdesc, " B");
            }
            if (tbyte & 0x08) {
               cpu.dp = cpu_pop_stack(tbyte1);
               operationticks += 1;
               strcat(cpu.opdesc, " DP");
            }
            if (tbyte & 0x10) {
               cpu.x.BYTE.high = cpu_pop_stack(tbyte1);
               cpu.x.BYTE.low = cpu_pop_stack(tbyte1);
               operationticks += 2;
               strcat(cpu.opdesc, " X");
            }
            if (tbyte & 0x20) {
               cpu.y.BYTE.high = cpu_pop_stack(tbyte1);
               cpu.y.BYTE.low = cpu_pop_stack(tbyte1);
               operationticks += 2;
               strcat(cpu.opdesc, " Y");
            }
            if (tbyte & 0x40) {
               cpu.u.BYTE.high = cpu_pop_stack(tbyte1);
               cpu.u.BYTE.low = cpu_pop_stack(tbyte1);
               operationticks += 2;
               strcat(cpu.opdesc, " U");
            }
            if (tbyte & 0x80) {
               cpu.pc.BYTE.high = cpu_pop_stack(tbyte1);
               cpu.pc.BYTE.low = cpu_pop_stack(tbyte1);
               operationticks += 2;
               strcat(cpu.opdesc, " PC");
            }
            break;

        /* RTS - Return from subroutine */
        case 0x39:
            cpu.pc.BYTE.high = cpu_pop_stack(REG_S);
            cpu.pc.BYTE.low = cpu_pop_stack(REG_S);
            operationticks = 5;
            sprintf(cpu.opshortdesc, "RTS, INH");
            sprintf(cpu.opdesc, "PC' [%04X]", cpu.pc.WORD);
            break;
 
        /* ABX - Add accumulator B into index register X */
        case 0x3A:
            tword.BYTE.high = 0x00;
            tword.BYTE.low = cpu.b;
            cpu.x = cpu_binary_add16(cpu.x, tword, FALSE, FALSE, FALSE);
            operationticks = 3;
            sprintf(cpu.opshortdesc, "ABX, INH");
            sprintf(cpu.opdesc, "X' [%04X] = X [%04X] + B [%02X]", cpu.x.WORD, cpu.x.WORD - cpu.b, cpu.b);
            break;

        /* RTI - Return from Interrupt */
        case 0x3B:
            operationticks = 0;
            sprintf(cpu.opdesc, "Pulled ");
            if (cpu.cc & REG_CC_E) {
               operationticks += 9;
               cpu.a = cpu_pop_stack(REG_S);
               cpu.d.BYTE.high = cpu.a;
               cpu.b = cpu_pop_stack(REG_S);
               cpu.d.BYTE.low = cpu.b;
               cpu.dp = cpu_pop_stack(REG_S);
               cpu.x.BYTE.high = cpu_pop_stack(REG_S);
               cpu.x.BYTE.low = cpu_pop_stack(REG_S);
               cpu.y.BYTE.high = cpu_pop_stack(REG_S);
               cpu.y.BYTE.low = cpu_pop_stack(REG_S);
               cpu.u.BYTE.high = cpu_pop_stack(REG_S);
               cpu.u.BYTE.low = cpu_pop_stack(REG_S);
               strcat(cpu.opdesc, " A B DP X Y U");
            }
            cpu.pc.BYTE.high = cpu_pop_stack(REG_S);
            cpu.pc.BYTE.low = cpu_pop_stack(REG_S);
            strcat(cpu.opdesc, " PC");
            operationticks += 6;
            sprintf(cpu.opshortdesc, "RTI, INH");
            break;
       
        /* CWAI - Clear and Wait for Interrupt */
        case 0x3C:
            operand1 = cpu_get_immediate8(ptr);
            cpu.cc &= operand1;
            cpu.cc |= REG_CC_E;
            cpu_push_stack(REG_S, cpu.pc.BYTE.low);
            cpu_push_stack(REG_S, cpu.pc.BYTE.high);
            cpu_push_stack(REG_S, cpu.u.BYTE.low);
            cpu_push_stack(REG_S, cpu.u.BYTE.high);
            cpu_push_stack(REG_S, cpu.y.BYTE.low);
            cpu_push_stack(REG_S, cpu.y.BYTE.high);
            cpu_push_stack(REG_S, cpu.x.BYTE.low);
            cpu_push_stack(REG_S, cpu.x.BYTE.high);
            cpu_push_stack(REG_S, cpu.dp);
            cpu_push_stack(REG_S, cpu.b);
            cpu_push_stack(REG_S, cpu.a);
            cpu_push_stack(REG_S, cpu.cc);
            operationticks = 20;
            sprintf(cpu.opshortdesc, "CWAI");
            sprintf(cpu.opdesc, "Pushed PC U Y X DP B A CC");
            break;

        /* MUL - Multiply, unsigned */
        case 0x3D:
            cpu.d.WORD = (cpu.a * cpu.b) & 0xFFFF;
            tbyte = cpu.a;
            tbyte1 = cpu.b;
            cpu.cc &= ~(REG_CC_Z | REG_CC_C);
            cpu.cc |= (cpu.d.WORD == 0) ? REG_CC_Z : 0;
            cpu.cc |= (cpu.b & 0x80) ? REG_CC_C : 0;
            cpu.a = cpu.d.BYTE.high;
            cpu.b = cpu.d.BYTE.low;
            operationticks = 11;
            sprintf(cpu.opshortdesc, "MUL, INH");
            sprintf(cpu.opdesc, "D' [%04X] = A [%02X] x B [%02X]", cpu.d.WORD, tbyte, tbyte1);
            break;

        /* SWI - Software Interrupt 1 */
        case 0x3F:
            cpu.cc |= REG_CC_E;
            cpu_push_stack(REG_S, cpu.pc.BYTE.low);
            cpu_push_stack(REG_S, cpu.pc.BYTE.high);
            cpu_push_stack(REG_S, cpu.u.BYTE.low);
            cpu_push_stack(REG_S, cpu.u.BYTE.high);
            cpu_push_stack(REG_S, cpu.y.BYTE.low);
            cpu_push_stack(REG_S, cpu.y.BYTE.high);
            cpu_push_stack(REG_S, cpu.x.BYTE.low);
            cpu_push_stack(REG_S, cpu.x.BYTE.high);
            cpu_push_stack(REG_S, cpu.dp);
            cpu_push_stack(REG_S, cpu.b);
            cpu_push_stack(REG_S, cpu.a);
            cpu_push_stack(REG_S, cpu.cc);
            tword.WORD = SWI_INT;
            cpu.pc = memory_read16(tword);
            operationticks = 19;
            sprintf(cpu.opshortdesc, "SWI");
            sprintf(cpu.opdesc, "PC' [%04X] pushed PC U Y X DP B A CC", cpu.pc.WORD);
            break;

        /* NEGA - Negate A */
        case 0x40:
            tbyte1 = cpu.a;
            cpu.a = cpu.d.BYTE.high = cpu_twos_comp8(cpu.a);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.cc |= (cpu.a != 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu.a == 0x80) ? REG_CC_V : 0;
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "NEGA, INH");
            sprintf(cpu.opdesc, "A' [%02X] = Negate A (twos comp) [%02X]", cpu.a, tbyte1);
            break;

        /* NEGB - Negate B */
        case 0x50:
            tbyte1 = cpu.b;
            cpu.b = cpu.d.BYTE.low = cpu_twos_comp8(cpu.b);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.cc |= (cpu.b != 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu.b == 0x80) ? REG_CC_V : 0;
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "NEGB, INH");
            sprintf(cpu.opdesc, "B' [%02X] = Negate B (twos comp) [%02X]", cpu.b, tbyte1);
            break;

        /* COMA - Complement A */
        case 0x43:
            cpu.a = cpu.d.BYTE.high = ~cpu.a;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_Z);
            cpu.cc |= REG_CC_C;
            cpu.cc |= ((sbyte) cpu.a <0) ? REG_CC_N : cpu.cc;
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : cpu.cc;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "COMA, INH");
            sprintf(cpu.opdesc, "A' [%02X] = Complement A [%02X]", cpu.a, ~cpu.a);
            break;

        /* COMB - Complement B */
        case 0x53:
            cpu.b = cpu.d.BYTE.low = ~cpu.b;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_Z);
            cpu.cc |= REG_CC_C;
            cpu.cc |= ((sbyte)cpu.b <0) ? REG_CC_N : cpu.cc;
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : cpu.cc;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "COMB, INH");
            sprintf(cpu.opdesc, "B' [%02X] = Complement B [%02X]", cpu.b, ~cpu.b);
            break;

        /* LSRA - Logical Shift Right A */
        case 0x44:
            cpu.a = cpu.a >> 1;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (cpu.a & 0x1) ? REG_CC_C : 0;
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.d.BYTE.high = cpu.a;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "LSRA, INH");
            sprintf(cpu.opdesc, "A' [%02X]", cpu.a);
            break;

        /* LSRB - Logical Shift Right B */
        case 0x54:
            cpu.b = cpu.b >> 1;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (cpu.b & 0x1) ? REG_CC_C : 0;
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            cpu.d.BYTE.low = cpu.b;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "LSRB, INH");
            sprintf(cpu.opdesc, "B' [%02X]", cpu.b);
            break;

        /* RORA - Rotate Right A */
        case 0x46:
            tbyte = cpu.a;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (tbyte & 0x01) ? REG_CC_C : 0;
            tbyte2 = tbyte >> 1;
            cpu.cc |= (tbyte2 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte2 < 0) ? REG_CC_N : 0;
            cpu.a = cpu.d.BYTE.high = tbyte2;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "RORA, INH");
            sprintf(cpu.opdesc, "A' [%02X]", cpu.a);
            break;

        /* RORB - Rotate Right B */
        case 0x56:
            tbyte = cpu.b;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (tbyte & 0x01) ? REG_CC_C : 0;
            tbyte2 = tbyte >> 1;
            cpu.cc |= (tbyte2 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte2 < 0) ? REG_CC_N : 0;
            cpu.b = cpu.d.BYTE.low = tbyte2;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "RORB, INH");
            sprintf(cpu.opdesc, "B' [%02X]", cpu.b);
            break;

        /* ASRA - Arithmetic Shift Right A */
        case 0x47:
            tbyte = cpu.a;
            tbyte1 = tbyte;
            tbyte = tbyte >> 1;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (tbyte1 & 0x1) ? REG_CC_C : 0;
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte < 0) ? REG_CC_N : 0;
            tbyte |= (tbyte1 & 0x80) ? 0x80 : 0;
            cpu.a = cpu.d.BYTE.high = tbyte;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "ASRA, INH");
            sprintf(cpu.opdesc, "A' [%02X] = Shift Right", cpu.a);
            break;

        /* ASRB - Arithmetic Shift Right B */
        case 0x57:
            tbyte = cpu.b;
            tbyte1 = tbyte;
            tbyte = tbyte >> 1;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_C);
            cpu.cc |= (tbyte1 & 0x1) ? REG_CC_C : 0;
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte < 0) ? REG_CC_N : 0;
            tbyte |= (tbyte1 & 0x80) ? 0x80 : 0;
            cpu.b = cpu.d.BYTE.low = tbyte;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "ASRB, INH");
            sprintf(cpu.opdesc, "B' [%02X] = Shift Right", cpu.b);
            break;

        /* ASLA - Arithmetic Shift Left A */
        case 0x48:
            tbyte = cpu.a;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte1 = tbyte;
            cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
            cpu.cc |= (((tbyte & 0x80) == 0x80) ^ ((tbyte & 0x40) == 0x40)) ? REG_CC_V : 0;
            tbyte = tbyte << 1;
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte < 0) ? REG_CC_N : 0;
            cpu.a = cpu.d.BYTE.high = tbyte;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "ASLA, INH");
            sprintf(cpu.opdesc, "A' [%02X], Shift Left", cpu.a);
            break;

        /* ASLB - Arithmetic Shift Left B */
        case 0x58:
            tbyte = tbyte1 = cpu.b;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
            cpu.cc |= (((tbyte & 0x80) == 0x80) ^ ((tbyte & 0x40) == 0x40)) ? REG_CC_V : 0;
            tbyte = tbyte << 1;
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte < 0) ? REG_CC_N : 0;
            cpu.b = cpu.d.BYTE.low = tbyte;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "ASLB, INH");
            sprintf(cpu.opdesc, "B' [%02X], Shift Left", cpu.b);
            break;

        /* ROLA - Rotate Left A */
        case 0x49:
            tbyte2 = tbyte = cpu.a;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
            cpu.cc |= (((tbyte & 0x80) == 0x80) ^ (((tbyte & 0x40) == 0x40))) ? REG_CC_V : 0;
            tbyte = tbyte << 1;
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.high = cpu.a;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "ROLA, INH");
            sprintf(cpu.opdesc, "A' [%02X] = Rotate Left", cpu.a);
            break;

        /* ROLB - Rotate Left B */
        case 0x59:
            tbyte2 = tbyte = cpu.b;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.cc |= (tbyte & 0x80) ? REG_CC_C : 0;
            cpu.cc |= (((tbyte & 0x80) == 0x80) ^ (((tbyte & 0x40) == 0x40))) ? REG_CC_V : 0;
            tbyte = tbyte << 1;
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.low = cpu.b;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "ROLB, INH");
            sprintf(cpu.opdesc, "B' [%02X] = Rotate Left", cpu.b);
            break;

        /* DECA - Decrement A */
        case 0x4A:
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a & 0x80) ? REG_CC_V : 0;
            cpu.a = cpu_binary_add8(cpu.a, 0xFF, FALSE, FALSE, FALSE);
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            operationticks = 2;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opshortdesc, "DECA, INH");
            sprintf(cpu.opdesc, "A' [%02X] = A - 1", cpu.a);
            break;

        /* DECB - Decrement B */
        case 0x5A:
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b & 0x80) ? REG_CC_V : 0;
            cpu.b = cpu_binary_add8(cpu.b, 0xFF, FALSE, FALSE, FALSE);
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            operationticks = 2;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opshortdesc, "DECB, INH");
            sprintf(cpu.opdesc, "B' [%02X] = B - 1", cpu.b);
            break;

        /* INCA - Increment A */
        case 0x4C:
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a & 0x7F) ? REG_CC_V : 0;
            cpu.a = cpu_binary_add8(cpu.a, 0x01, FALSE, FALSE, FALSE);
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            operationticks = 2;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opshortdesc, "INCA, INH");
            sprintf(cpu.opdesc, "A' [%02X] = A + 1", cpu.a);
            break;

        /* INCB - Increment B */
        case 0x5C:
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b & 0x7F) ? REG_CC_V : 0;
            cpu.b = cpu_binary_add8(cpu.b, 0x01, FALSE, FALSE, FALSE);
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            operationticks = 2;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opshortdesc, "INCB, INH");
            sprintf(cpu.opdesc, "B' [%02X] = B + 1", cpu.b);
            break;

        /* TSTA - Test A */
        case 0x4D:
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "TSTA, INH");
            sprintf(cpu.opdesc, "TEST A (see CC for results)");
            break;

        /* TSTB - Test B */
        case 0x5D:
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            operationticks = 2;
            sprintf(cpu.opshortdesc, "TSTB, INH");
            sprintf(cpu.opdesc, "TEST B (see CC for results)");
            break;

        /* CLRA - Clear A */
        case 0x4F:
            cpu.a = cpu.d.BYTE.high = 0;
            cpu.cc |= REG_CC_Z;
            cpu.cc &= ~(REG_CC_N | REG_CC_V | REG_CC_C);
            operationticks = 2;
            sprintf(cpu.opshortdesc, "CLRA, INH");
            sprintf(cpu.opdesc, "A' = 0");
            break;

        /* CLRB - Clear B */
        case 0x5F:
            cpu.b = cpu.d.BYTE.low = 0;
            cpu.cc |= REG_CC_Z;
            cpu.cc &= ~(REG_CC_N | REG_CC_V | REG_CC_C);
            operationticks = 2;
            sprintf(cpu.opshortdesc, "CLRB, INH");
            sprintf(cpu.opdesc, "B' = 0");
            break;

        /* SUBA - Subtract M from A */
        case 0x80:
        case 0x90:
        case 0xA0:
        case 0xB0:
            if (opcode == 0x80) {
                tbyte = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "SUBA, IMM [%02X]", tbyte);
            }
            else if (opcode == 0x90) {
                tword = cpu_get_direct(ptr);
                tbyte = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "SUBA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA0) {
                tword = cpu_get_indexed(ptr);
                tbyte = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "SUBA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB0) {
                tword = cpu_get_immediate16(ptr);
                tbyte = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "SUBA, EXT [%04X]", tword.WORD);
            }
            tbyte1 = cpu_twos_comp8(tbyte);
            tbyte2 = cpu.a;
            cpu.cc &= ~(REG_CC_N | REG_CC_V | REG_CC_Z | REG_CC_C);
            cpu.a = cpu_binary_add8(cpu.a, tbyte1, FALSE, TRUE, TRUE);
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' [%02X]", cpu.a);
            break;

        /* SUBB - Subtract M from B */
        case 0xC0:
        case 0xD0:
        case 0xE0:
        case 0xF0:
            if (opcode == 0xC0) {
                tbyte = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "SUBB, IMM [%02X]", tbyte);
            }
            else if (opcode == 0xD0) {
                tword = cpu_get_direct(ptr);
                tbyte = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "SUBB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE0) {
                tword = cpu_get_indexed(ptr);
                tbyte = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "SUBB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF0) {
                tword = cpu_get_immediate16(ptr);
                tbyte = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "SUBB, EXT [%04X]", tword.WORD);
            }
            tbyte1 = cpu_twos_comp8(tbyte);
            tbyte2 = cpu.b;
            cpu.cc &= ~(REG_CC_N | REG_CC_V | REG_CC_Z | REG_CC_C);
            cpu.a = cpu_binary_add8(cpu.b, tbyte1, FALSE, TRUE, TRUE);
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' [%02X]", cpu.b);
            break;


        /* CMPA - Compare A */
        case 0x81: 
        case 0x91: 
        case 0xA1: 
        case 0xB1: 
            if (opcode == 0x81) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "CMPA, IMM");
            }
            else if (opcode == 0x91) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "CMPA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA1) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "CMPA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xB1) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "CMPA, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu_twos_comp8(operand1);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte1 = cpu_binary_add8(cpu.a, tbyte, FALSE, TRUE, TRUE);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "TEMP [%02X] = A [%02X] - %02X", tbyte1, cpu.a, operand1);
            break;

        /* CMPB - Compare B */
        case 0xC1: 
        case 0xD1: 
        case 0xE1: 
        case 0xF1: 
            if (opcode == 0xC1) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "CMPB, IMM");
            }
            else if (opcode == 0xD1) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "CMPB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE1) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "CMPB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xF1) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "CMPB, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu_twos_comp8(operand1);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte1 = cpu_binary_add8(cpu.b, tbyte, FALSE, TRUE, TRUE);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "TEMP [%02X] = B [%02X] - %02X", tbyte1, cpu.b, operand1);
            break;


        /* SBCA - Subtract M and C from A */
        case 0x82:
        case 0x92:
        case 0xA2:
        case 0xB2:
            if (opcode == 0x82) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "SBCA, IMM");
            }
            else if (opcode == 0x92) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "SBCA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA2) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "SBCA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB2) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "SBCA, EXT [%04X]", tword.WORD);
            }
            tbyte2 = cpu.a;
            tbyte = cpu_twos_comp8(operand1);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte1 = cpu_binary_add8(cpu.a, tbyte, TRUE, TRUE, TRUE);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0;
            if (cpu.cc & REG_CC_C) {
                cpu.cc &= ~(REG_CC_C);
            }
            else {
                cpu.cc |= REG_CC_C;
            }             
            cpu.a = tbyte1;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' [%02X] = A [%02X] - %02X - C", cpu.a, tbyte2, operand1);
            break;

        /* SBCB - Subtract M and C from B */
        case 0xC2:
        case 0xD2:
        case 0xE2:
        case 0xF2:
            if (opcode == 0xC2) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "SBCB, IMM");
            }
            else if (opcode == 0xD2) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "SBCB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE2) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "SBCB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF2) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "SBCB, EXT [%04X]", tword.WORD);
            }
            tbyte2 = cpu.b;
            tbyte = cpu_twos_comp8(operand1);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte1 = cpu_binary_add8(cpu.b, tbyte, TRUE, TRUE, TRUE);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0;
            if (cpu.cc & REG_CC_C) {
                cpu.cc &= ~(REG_CC_C);
            }
            else {
                cpu.cc |= REG_CC_C;
            }             
            cpu.b = tbyte1;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' [%02X] = B [%02X] - %02X - C", cpu.b, tbyte2, operand1);
            break;

        /* SUBD - Subtract M from D */
        case 0x83:
        case 0x93:
        case 0xA3:
        case 0xB3:
            if (opcode == 0x83) {  
                tword = cpu_get_immediate16(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "SUBD, IMM");
            }
            else if (opcode == 0x93) {  
                tword = cpu_get_direct(ptr);
                operationticks = 6;
                sprintf(cpu.opshortdesc, "SUBD, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA3) {  
                tword = cpu_get_indexed(ptr);
                operationticks = 4 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "SUBD, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB3) {  
                tword = memory_read16(cpu_get_immediate16(ptr));
                operationticks = 7;
                sprintf(cpu.opshortdesc, "SUBD, EXT [%04X]", tword.WORD);
            }
            cpu.cc &= ~(REG_CC_N | REG_CC_V | REG_CC_Z | REG_CC_C);
            tword1 = cpu_twos_comp16(tword);
            tword2.WORD = cpu.d.WORD;
            cpu.d = cpu_binary_add16(cpu.d, tword1, FALSE, TRUE, TRUE); 
            cpu.cc |= (cpu_get_signed16(cpu.d) < 0) ? REG_CC_N : 0;
            cpu.cc |= (cpu.d.WORD == 0) ? REG_CC_Z : 0;
            cpu.a = cpu.d.BYTE.high;
            cpu.b = cpu.d.BYTE.low;
            sprintf(cpu.opdesc, "D' [%04X]", cpu.d.WORD);
            break;

        /* ANDA - Logical AND */
        case 0x84:
        case 0x94:
        case 0xA4:
        case 0xB4:
            if (opcode == 0x84) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ANDA, IMM");
            }
            else if (opcode == 0x94) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ANDA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA4) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ANDA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB4) {
                tword = cpu_get_immediate16(ptr);
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ANDA, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.a;
            cpu.a = cpu.a & operand1;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) cpu.a < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' [%02X] = A [%02X] AND [%02X]", cpu.a, tbyte, operand1);
            break;

        /* ANDB - Logical AND */
        case 0xC4:
        case 0xD4:
        case 0xE4:
        case 0xF4:
            if (opcode == 0xC4) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ANDB, IMM");
            }
            else if (opcode == 0xD4) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ANDB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE4) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ANDB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF4) {
                tword = cpu_get_immediate16(ptr);
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ANDB, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.b;
            cpu.b = cpu.b & operand1;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' [%02X] = B [%02X] AND [%02X]", cpu.b, tbyte, operand1);
            break;

        /* BITA - Bit test */
        case 0x85:
        case 0x95:
        case 0xA5:
        case 0xB5:
            if (opcode == 0x85) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "BITA, IMM");
            }
            else if (opcode == 0x95) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "BITA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA5) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "BITA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB5) {
                tword = cpu_get_immediate16(ptr);
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "BITA, EXT [%04X]", tword.WORD);
            }
            tbyte1 = cpu.a & operand1;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) tbyte1 < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "TEMP [%02X] = A [%02X] AND [%02X]", tbyte1, cpu.a, operand1);
            break;

        /* BITB - Bit test */
        case 0xC5:
        case 0xD5:
        case 0xE5:
        case 0xF5:
            if (opcode == 0xC5) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "BITB, IMM");
            }
            else if (opcode == 0xD5) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "BITB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE5) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "BITB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF5) {
                tword = cpu_get_immediate16(ptr);
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "BITB, EXT [%04X]", tword.WORD);
            }
            tbyte1 = cpu.b & operand1;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) tbyte1 < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "TEMP [%02X] = B [%02X] AND [%02X]", tbyte1, cpu.b, operand1);
            break;

        /* LDA - Load A */
        case 0x86:
        case 0x96:
        case 0xA6:
        case 0xB6:
            if (opcode == 0x86) {
                cpu.a = cpu.d.BYTE.high = cpu_get_immediate8(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "LDA, IMM");
            }
            else if (opcode == 0x96) {
                tword = cpu_get_direct(ptr);
                cpu.a = memory_read(tword);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "LDA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA6) {
                tword = cpu_get_indexed(ptr);
                cpu.a = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "LDA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB6) {
                tword = cpu_get_immediate16(ptr);
                cpu.a = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "LDA, EXT [%04X]", tword.WORD);
            }
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) cpu.a < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' = [%02X]", cpu.a);
            break;

        /* LDB - Load B */
        case 0xC6:
        case 0xD6:
        case 0xE6:
        case 0xF6:
            if (opcode == 0xC6) {
                cpu.b = cpu.d.BYTE.low = cpu_get_immediate8(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "LDB, IMM");
            }
            else if (opcode == 0xD6) {
                tword = cpu_get_direct(ptr);
                cpu.b = memory_read(tword);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "LDB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE6) {
                tword = cpu_get_indexed(ptr);
                cpu.b = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "LDB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF6) {
                tword = cpu_get_immediate16(ptr);
                cpu.b = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "LDB, EXT [%04X]", tword.WORD);
            }
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) cpu.b < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' = [%02X]", cpu.b);
            break;

        /* EORA - Logical Exclusive OR */
        case 0x88:
        case 0x98:
        case 0xA8:
        case 0xB8:
            if (operand1 == 0x88) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "EORA, IMM");
            }
            else if (opcode == 0x98) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "EORA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA8) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "EORA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB8) {
                tword = cpu_get_immediate16(ptr);
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "EORA, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.a;
            cpu.a = cpu.a ^ operand1;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) cpu.a < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' = [%02X] = A [%02X] XOR [%02X]", cpu.a, tbyte, operand1);
            break;

        /* EORB - Logical Exclusive OR */
        case 0xC8:
        case 0xD8:
        case 0xE8:
        case 0xF8:
            if (operand1 == 0xC8) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "EORB, IMM");
            }
            else if (opcode == 0xD8) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "EORB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE8) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "EORB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF8) {
                tword = cpu_get_immediate16(ptr);
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "EORB, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.b;
            cpu.b = cpu.b ^ operand1;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) cpu.b < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' = [%02X] = B [%02X] XOR [%02X]", cpu.b, tbyte, operand1);
            break;

        /* ADCA - Add with Carry into Register */
        case 0x89:
        case 0x99:
        case 0xA9:
        case 0xB9:
            if (opcode == 0x89) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ADCA, IMM");
            }
            else if (opcode == 0x99) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ADCA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA9) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ADCA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB9) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ADCA, EXT [%04X]", tword.WORD);
            }
            cpu.cc &= ~(REG_CC_H | REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte2 = (cpu.cc & REG_CC_C) == REG_CC_C;
            tbyte = cpu.a;
            tbyte1 = cpu_binary_add8(cpu.a, operand1, TRUE, TRUE, TRUE);
            tbyte1 = cpu_binary_add8(tbyte1, tbyte2, TRUE, TRUE, TRUE);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0;
            cpu.a = tbyte1;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' [%02X] = A [%02X] + %02X + C [%01X]", cpu.a, tbyte, operand1, tbyte2 );
            break;

        /* ADCB - Add with Carry into Register */
        case 0xC9:
        case 0xD9:
        case 0xE9:
        case 0xF9:
            if (opcode == 0xC9) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ADCB, IMM");
            }
            else if (opcode == 0xD9) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ADCB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE9) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ADCB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF9) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ADCB, EXT [%04X]", tword.WORD);
            }
            cpu.cc &= ~(REG_CC_H | REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tbyte2 = (cpu.cc & REG_CC_C) == REG_CC_C;
            tbyte = cpu.b;
            tbyte1 = cpu_binary_add8(cpu.b, operand1, TRUE, TRUE, TRUE);
            tbyte1 = cpu_binary_add8(tbyte1, tbyte2, TRUE, TRUE, TRUE);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0;
            cpu.b = tbyte1;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' [%02X] = B [%02X] + %02X + C [%01X]", cpu.b, tbyte, operand1, tbyte2 );
            break;

        /* ORA - Logical OR */
        case 0x8A:
        case 0x9A:
        case 0xAA:
        case 0xBA:
            if (opcode == 0x8A) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ORA, IMM");
            }
            else if (opcode == 0x9A) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ORA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xAA) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ORA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xBA) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ORA, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.a;
            tbyte1 = cpu.a | operand1;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0; 
            cpu.d.BYTE.high = tbyte1;
            cpu.a = tbyte1;
            sprintf(cpu.opdesc, "A' [%02X] = A [%02X] OR [%02X]", cpu.a, tbyte, operand1);
            break;

        /* ORB - Logical OR */
        case 0xCA:
        case 0xDA:
        case 0xEA:
        case 0xFA:
            if (opcode == 0xCA) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ORB, IMM");
            }
            else if (opcode == 0xDA) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ORB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xEA) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ORB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xFA) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ORB, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.b;
            tbyte1 = cpu.b | operand1;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
            cpu.cc |= (tbyte1 == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)tbyte1 < 0) ? REG_CC_N : 0; 
            cpu.d.BYTE.low = tbyte1;
            cpu.b = tbyte1;
            sprintf(cpu.opdesc, "B' [%02X] = B [%02X] OR [%02X]", cpu.b, tbyte, operand1);
            break;

        /* ADDA - Add A */
        case 0x8B:
        case 0x9B:
        case 0xAB:
        case 0xBB:
            if (opcode == 0x8B) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ADDA, IMM");
            }
            else if (opcode == 0x9B) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ADDA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xAB) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ADDA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xBB) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ADDA, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.a;
            cpu.cc &= ~(REG_CC_H | REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.a = cpu_binary_add8(operand1, cpu.a, TRUE, TRUE, TRUE);
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte) cpu.a < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.high = cpu.a;
            sprintf(cpu.opdesc, "A' [%02X] = A [%02X] + %02X", cpu.a, tbyte, operand1);
            break;

        /* ADDB - Add B */
        case 0xCB:
        case 0xDB:
        case 0xEB:
        case 0xFB:
            if (opcode == 0xCB) {
                operand1 = cpu_get_immediate8(ptr);
                operationticks = 2;
                sprintf(cpu.opshortdesc, "ADDB, IMM");
            }
            else if (opcode == 0xDB) {
                tword = cpu_get_direct(ptr);
                operand1 = memory_read(tword);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ADDB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xEB) {
                tword = cpu_get_indexed(ptr);
                operand1 = memory_read(tword);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ADDB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xFB) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operand1 = memory_read(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "ADDB, EXT [%04X]", tword.WORD);
            }
            tbyte = cpu.b;
            cpu.cc &= ~(REG_CC_H | REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.b = cpu_binary_add8(operand1, cpu.b, TRUE, TRUE, TRUE);
            cpu.cc |= (tbyte == 0) ? REG_CC_Z : 0;
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            cpu.d.BYTE.low = cpu.b;
            sprintf(cpu.opdesc, "B' [%02X] = B [%02X] + %02X", cpu.b, tbyte, operand1);
            break;

        /* CMPX - Compare X */
        case 0x8C:
        case 0x9C:
        case 0xAC:
        case 0xBC:
            if (opcode == 0x8C) {
                tword = cpu_get_immediate16(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "CMPX, IMM");
            }
            else if (opcode == 0x9C) {
                tword3 = cpu_get_direct(ptr);
                tword = memory_read16(tword);
                operationticks = 6;
                sprintf(cpu.opshortdesc, "CMPX, DIR [%04X]", tword3.WORD);
            }
            else if (opcode == 0xAC) {
                tword3 = cpu_get_indexed(ptr);
                tword = memory_read16(tword);
                operationticks = 4 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "CMPX, IND [%04X]", tword3.WORD);
            }
            else if (opcode == 0xBC) {
                tword3 = cpu_get_immediate16(ptr);
                tword = memory_read16(tword);
                operationticks = 7;
                sprintf(cpu.opshortdesc, "CMPX, EXT [%04X]", tword3.WORD);
            }
            tword1 = cpu_twos_comp16(tword);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            tword2 = cpu_binary_add16(cpu.x, tword1, FALSE, TRUE, TRUE);
            cpu.cc |= (tword2.WORD == 0) ? REG_CC_Z : 0;
            cpu.cc |= (cpu_get_signed16(tword2) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "X [%04X] - %04X, CC [%02X]", cpu.x.WORD, tword.WORD, cpu.cc);
            break;

        /* BSR - Branch to Subroutine */
        case 0x8D:
            operand1 = cpu_get_immediate8(ptr);
            cpu_push_stack(REG_S, cpu.pc.BYTE.low);
            cpu_push_stack(REG_S, cpu.pc.BYTE.high);
            tword1.WORD = cpu.pc.WORD;
            tword.BYTE.high = 0;
            tword.BYTE.low = operand1;
            cpu.pc = cpu_binary_add16(cpu.pc, tword, FALSE, FALSE, FALSE);
            operationticks = 7;
            sprintf(cpu.opshortdesc, "BSR, REL");
            sprintf(cpu.opdesc, "PC' [%04X] = PC [%04X] + TEMP [%04X]\n", cpu.pc.WORD, tword1.WORD, tword.WORD);
            break;

        /* LDD - Load D */
        case 0xCC:
        case 0xDC:
        case 0xEC:
        case 0xFC:
            if (opcode == 0xCC) {
                tword1 = cpu_get_immediate16(ptr);
                operationticks = 3;
                sprintf(cpu.opshortdesc, "LDD, IMM");
            }
            else if (opcode == 0xDC) {
                tword = cpu_get_direct(ptr);
                tword1 = memory_read16(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "LDD, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xEC) {
                tword = cpu_get_indexed(ptr);
                tword1 = memory_read16(tword);
                operationticks = 3 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "LDD, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xFC) {
                tword = cpu_get_immediate16(ptr);
                tword1 = memory_read16(tword);
                operationticks = 6;
                sprintf(cpu.opshortdesc, "LDD, EXT [%04X]", tword.WORD);
            }
            cpu.d = tword1; 
            cpu.a = cpu.d.BYTE.high;
            cpu.b = cpu.d.BYTE.low;
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.d.WORD == 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu_get_signed16(cpu.d) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "D' = [%04X]", cpu.d.WORD);
            break;

        /* LDX - Load X */
        case 0x8E:
        case 0x9E:
        case 0xAE:
        case 0xBE:
            if (opcode == 0x8E) {
                tword1 = cpu_get_immediate16(ptr);
                operationticks = 3;
                sprintf(cpu.opshortdesc, "LDX, IMM");
            }
            else if (opcode == 0x9E) {
                tword = cpu_get_direct(ptr);
                tword1 = memory_read16(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "LDX, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xAE) {
                tword = cpu_get_indexed(ptr);
                tword1 = memory_read16(tword);
                operationticks = 3 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "LDX, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xBE) {
                tword = cpu_get_immediate16(ptr);
                tword1 = memory_read16(tword);
                operationticks = 6;
                sprintf(cpu.opshortdesc, "LDX, EXT [%04X]", tword.WORD);
            }
            cpu.x = tword1; 
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.x.WORD == 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu_get_signed16(cpu.x) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "X' = [%04X]", cpu.x.WORD);
            break;

        /* LDU - Load U */
        case 0xCE:
        case 0xDE:
        case 0xEE:
        case 0xFE:
            if (opcode == 0xCE) {
                tword1 = cpu_get_immediate16(ptr);
                operationticks = 3;
                sprintf(cpu.opshortdesc, "LDU, IMM");
            }
            else if (opcode == 0xDE) {
                tword = cpu_get_direct(ptr);
                tword1 = memory_read16(tword);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "LDU, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xEE) {
                tword = cpu_get_indexed(ptr);
                tword1 = memory_read16(tword);
                operationticks = 3 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "LDU, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xFE) {
                tword = cpu_get_immediate16(ptr);
                tword1 = memory_read16(tword);
                operationticks = 6;
                sprintf(cpu.opshortdesc, "LDU, EXT [%04X]", tword.WORD);
            }
            cpu.u = tword1; 
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.u.WORD == 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu_get_signed16(cpu.u) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "U' = [%04X]", cpu.u.WORD);
            break;

        /* STA - Store A */
        case 0x97:
        case 0xA7:
        case 0xB7:
            if (opcode == 0x97) {
                tword = cpu_get_direct(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "STA, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xA7) {
                tword = cpu_get_indexed(ptr);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "STA, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xB7) {
                tword = cpu_get_immediate16(ptr);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "STA, EXT [%04X]", tword.WORD);
            }
            memory_write(tword, cpu.a);
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.a == 0) ? REG_CC_C : 0;
            cpu.cc |= ((sbyte)cpu.a < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "M' [%02X]", cpu.a);
            break;

        /* STB - Store B */
        case 0xD7:
        case 0xE7:
        case 0xF7:
            if (opcode == 0xD7) {
                tword = cpu_get_direct(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "STB, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xE7) {
                tword = cpu_get_indexed(ptr);
                operationticks = 2 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "STB, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xF7) {
                tword = cpu_get_immediate16(ptr);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "STB, EXT [%04X]", tword.WORD);
            }
            memory_write(tword, cpu.b);
            cpu.cc &= ~(REG_CC_V | REG_CC_N | REG_CC_Z);
            cpu.cc |= (cpu.b == 0) ? REG_CC_C : 0;
            cpu.cc |= ((sbyte)cpu.b < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "M' [%02X]", cpu.b);
            break;

        /* JSR - Jump to subroutine */
        case 0x9D:
        case 0xAD: 
        case 0xBD:
            if (opcode == 0x9D) {
                tword = cpu_get_direct(ptr);
                operationticks = 7;
                sprintf(cpu.opshortdesc, "JSR, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xAD) {
                tword = cpu_get_direct(ptr);
                operationticks = 5 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "JSR, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xBD) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operationticks = 8;
                sprintf(cpu.opshortdesc, "JSR, EXT [%04X]", tword.WORD);
            }
            cpu_push_stack(REG_S, cpu.pc.BYTE.low);
            cpu_push_stack(REG_S, cpu.pc.BYTE.high);
            cpu.pc.BYTE.high = tword.BYTE.high;
            cpu.pc.BYTE.low = tword.BYTE.low;
            sprintf(cpu.opdesc, "PC' [%04X], pushed PC", cpu.pc.WORD);
            break;

        /* STD - Store D */
        case 0xDD:
        case 0xED:
        case 0xFD:
            if (opcode == 0xDD) {
                tword = cpu_get_direct(ptr);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "STD, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xED) {
                tword = cpu_get_indexed(ptr);
                operationticks = 3 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "STD, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xFD) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operationticks = 6;
                sprintf(cpu.opshortdesc, "STD, EXT [%04X]", tword.WORD);
            }
            memory_write16(tword, cpu.d);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
            cpu.cc |= (cpu.d.WORD == 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu_get_signed16(cpu.d) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "M = X [%04X]", cpu.d.WORD);
            break;

        /* STX - Store X */
        case 0x9F:
        case 0xAF:
        case 0xBF:
            if (opcode == 0x9F) {
                tword = cpu_get_direct(ptr);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "STX, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xAF) {
                tword = cpu_get_indexed(ptr);
                operationticks = 3 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "STX, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xBF) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operationticks = 6;
                sprintf(cpu.opshortdesc, "STX, EXT [%04X]", tword.WORD);
            }
            memory_write16(tword, cpu.x);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
            cpu.cc |= (cpu.x.WORD == 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu_get_signed16(cpu.x) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "M = X [%04X]", cpu.x.WORD);
            break;

        /* STU - Store U */
        case 0xDF:
        case 0xEF:
        case 0xFF:
            if (opcode == 0xDF) {
                tword = cpu_get_direct(ptr);
                operationticks = 5;
                sprintf(cpu.opshortdesc, "STU, DIR [%04X]", tword.WORD);
            }
            else if (opcode == 0xEF) {
                tword = cpu_get_indexed(ptr);
                operationticks = 3 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "STU, IND [%04X]", tword.WORD);
            }
            else if (opcode == 0xFF) {
                tword = memory_read16(cpu_get_immediate16(ptr));
                operationticks = 6;
                sprintf(cpu.opshortdesc, "STU, EXT [%04X]", tword.WORD);
            }
            memory_write16(tword, cpu.u);
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V);
            cpu.cc |= (cpu.u.WORD == 0) ? REG_CC_C : 0;
            cpu.cc |= (cpu_get_signed16(cpu.u) < 0) ? REG_CC_N : 0;
            sprintf(cpu.opdesc, "M = U [%04X]", cpu.u.WORD);
            break;

        /* ADDD - Add D */
        case 0xC3:
        case 0xD3:
        case 0xE3:
            if (opcode == 0xC3) {
                tword = cpu_get_immediate16(ptr);
                operationticks = 4;
                sprintf(cpu.opshortdesc, "ADDD, IMM");
            }
            else if (opcode == 0xD3) {
                tword3 = cpu_get_direct(ptr);
                tword = memory_read16(tword3);
                operationticks = 6;
                sprintf(cpu.opshortdesc, "ADDD, DIR [%04X]", tword3.WORD);
            }
            else if (opcode == 0xE3) {
                tword3 = cpu_get_indexed(ptr);
                tword = memory_read16(tword3);
                operationticks = 6 + cpu.bytesconsumed;
                sprintf(cpu.opshortdesc, "ADDD, IND [%04X]", tword3.WORD);
            }
            else if (opcode == 0xF3) {
                tword3 = memory_read16(cpu_get_indexed(ptr));
                tword = memory_read16(tword3);
                operationticks = 7;
                sprintf(cpu.opshortdesc, "ADDD, IND [%04X]", tword3.WORD);
            }
            tword1.WORD = cpu.d.WORD;
            cpu.cc &= ~(REG_CC_N | REG_CC_Z | REG_CC_V | REG_CC_C);
            cpu.d = cpu_binary_add16(cpu.d, tword, FALSE, TRUE, TRUE);
            cpu.cc |= (cpu.d.WORD == 0) ? REG_CC_Z : 0;
            cpu.cc |= (cpu_get_signed16(cpu.d) < 0) ? REG_CC_N : 0;
            cpu.a = cpu.d.BYTE.high;
            cpu.b = cpu.d.BYTE.low;
            operationticks = 4;
            sprintf(cpu.opshortdesc, "ADDD, IMM");
            sprintf(cpu.opdesc, "D' [%04X] = D [%04X] + %04X",
                       cpu.d.WORD, tword1.WORD, tword.WORD );
            break;

        default:
            sprintf(cpu.opshortdesc, "Illegal opcode!");
            sprintf(cpu.opdesc, "Unimplemented opcode [%02X]", opcode);
            break;
        }

        /* Update the PC if necessary */
        cpu.pc.WORD += (cpu.bytesconsumed != NO_PC_UPDATE) ? cpu.bytesconsumed : 0;

        /* Check to see if there was a keypress */
        keypress = keyboard_returnstatusbytes();
        if (keypress.WORD) { 
            cpu_irq_value |= (cpu_irq_enabled) ? 0x2 : 0;
            cpu_firq_value |= (cpu_firq_enabled) ? 0x2 : 0;
            cpu_fire_interrupt = TRUE;
        }

        /* Check to see if more than 63.5 microseconds have passed */
        cpu_ticks += operationticks; 
        if (cpu_ticks > 64) {
            /* TODO: Fire an interrupt if need be */
            cpu_ticks = 0;
        }

        if (cpu_irq_flag && cpu_fire_interrupt) {
            cpu_push_stack(REG_S, cpu.pc.BYTE.low);
            cpu_push_stack(REG_S, cpu.pc.BYTE.high);
            cpu_push_stack(REG_S, cpu.u.BYTE.low);
            cpu_push_stack(REG_S, cpu.u.BYTE.high);
            cpu_push_stack(REG_S, cpu.y.BYTE.low);
            cpu_push_stack(REG_S, cpu.y.BYTE.high);
            cpu_push_stack(REG_S, cpu.x.BYTE.low);
            cpu_push_stack(REG_S, cpu.x.BYTE.high);
            cpu_push_stack(REG_S, cpu.dp);
            cpu_push_stack(REG_S, cpu.b);
            cpu_push_stack(REG_S, cpu.a);
            cpu.cc |= REG_CC_E;
            cpu_push_stack(REG_S, cpu.cc);
            cpu.cc |= REG_CC_I;
            tword.WORD = IRQ_INT;
            cpu.pc.BYTE.high = memory_read(tword); 
            tword.WORD++;
            cpu.pc.BYTE.low = memory_read(tword);
            cpu_fire_interrupt = FALSE;
        }

        if (cpu_firq_flag && cpu_fire_interrupt) {
            cpu_push_stack(REG_S, cpu.pc.BYTE.low);
            cpu_push_stack(REG_S, cpu.pc.BYTE.high);
            cpu.cc |= REG_CC_E;
            cpu_push_stack(REG_S, cpu.cc);
            cpu.cc |= REG_CC_I | REG_CC_F;
            tword.WORD = FIRQ_INT;
            cpu.pc.BYTE.high = memory_read(tword);  
            tword.WORD++;
            cpu.pc.BYTE.low = memory_read(tword);
            cpu_fire_interrupt = FALSE;
        }

        if (cpu.state == CPU_TRACE) {
            SDL_Delay(1000);
        }

        if (cpu.state == CPU_UNIT_TEST) {
            return;
        }
      
        if ((cpu.state == CPU_DEBUG) || (cpu.state == CPU_TRACE)) {
            vdg_refresh();
            screen_refresh(TRUE);
            while (cpu.state == CPU_DEBUG) {
                cpu_process_sdl_events();
                SDL_Delay(20);
            }
            cpu.state = (cpu.state == CPU_STEP) ? CPU_DEBUG : cpu.state;
        }
        else {
            if (screen_refresh_flag) {
                vdg_refresh();
                screen_refresh(FALSE);
                screen_refresh_flag = FALSE;
            }
        }

        cpu_process_sdl_events();
    }
}

/* E N D   O F   F I L E ******************************************************/
