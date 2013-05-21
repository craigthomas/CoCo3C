/**
 * Copyright (C) 2013 Craig Thomas
 * This project uses an MIT style license - see the LICENSE file for details.
 *
 * @file     memory.c
 * @brief    Functions to access emulator memory
 * @author   Craig Thomas
 *
 * This file contains functions that are used to create, access, and
 * destroy the segments of emulator allocated memory. Memory access in 
 * the Color Computer 3 is complicated by the fact that the architecture
 * makes use of memory-mapped I/O. This means that certain memory
 * addresses are used to report on the state of the hardware, and are also
 * used to instruct the hardware to do various operations. See the comments
 * below for more information.
 */

/* I N C L U D E S *******************************************************/

#include <stdio.h>
#include <malloc.h>
#include "yacoco3e.h"

/* F U N C T I O N S *****************************************************/

/**
 * @brief Read or write a byte from or to the specified address.
 *
 * Will either write a byte of data to the specified address,
 * or will return the value read from the specified address. The flag
 * \a wflag controls whether the byte is read (0 - FALSE), or written
 * (1 - TRUE). On a read, the function will return 0.
 *
 * @param[in] address The 16-bit memory address to access.
 * @param[in] value The 8-bit value to write.
 * @param[in] wflag Whether to read (0) or write (1) to the memory location.
 * @return The 8-bit value read or written.
 */
inline byte 
memory_access(register word address, byte value, int wflag)
{
    byte newbyte;
    word mask;
    byte virtual_page;
    byte physical_page;
    word real_address;
  
    newbyte = 0;
    mask.BYTE.low = 0;
    mask.BYTE.high = 0;

    switch (address.WORD) {

        /* Keyboard row identifier */
        case 0xFF00:
        case 0xFF04:
        case 0xFF08:
        case 0xFF0C:
            newbyte = keypress.BYTE.high;
            break;
  
        /* Keyboard column identifier */ 
        case 0xFF02:
        case 0xFF06:
        case 0xFF0A:
        case 0xFF0E:
            newbyte = keypress.BYTE.low;
            break;
  
        /* ACVC - VDG */
        case 0xFF22:
           screen_vdg_old_mode = (wflag) ? value : screen_vdg_old_mode;
           newbyte = screen_vdg_old_mode;
           break;
  
        /* ACVC - Init0 */
        case 0xFF90:
            if (wflag) {
                screen_low_res_enabled = (value & 0x80) == 0x80;
                memory_mmu_enabled     = (value & 0x40) == 0x40;
                cpu_irq_enabled        = (value & 0x20) == 0x20;
                cpu_firq_enabled       = (value & 0x10) == 0x10;
  
                cpu_irq_value = (!cpu_irq_enabled) ? 0 : cpu_irq_value;
                cpu_firq_value = (!cpu_firq_enabled) ? 0 : cpu_firq_value;
            }
  
            /* Set the return value based on what is written */
            newbyte |= (screen_low_res_enabled) ? 0x80 : 0;
            newbyte |= (memory_mmu_enabled)     ? 0x40 : 0;
            newbyte |= (cpu_irq_enabled)        ? 0x20 : 0;
            newbyte |= (cpu_firq_enabled)       ? 0x10 : 0;
            break;
  
        /* ACVC - Init1 */
        case 0xFF91:
            if (wflag) {
                cpu_timer_rate          = (value & 0x20) == 0x20;
                memory_mmu_task_or_exec = (value & 0x01) == 0x01;
            }
            newbyte |= (cpu_timer_rate)          ? 0x20 : 0;
            newbyte |= (memory_mmu_task_or_exec) ? 0x01 : 0;
            break;
  
        /* ACVC - IRQ Enable */
        case 0xFF92:
            cpu_irq_value = (wflag) ? value : cpu_irq_value;
            newbyte = cpu_irq_value;
            if (!wflag && cpu_irq_enabled) {
                cpu_irq_value = 0;
                cpu_irq_flag = FALSE;
            }
            break;
  
        /* ACVC - Fast IRQ Enable */
        case 0xFF93:
            cpu_firq_value = (wflag) ? value : cpu_firq_value;
            newbyte = cpu_firq_value;
            if (!wflag && cpu_firq_enabled) {
                cpu_firq_value = 0;
                cpu_firq_flag = FALSE;
            }
            break;
  
        /* ACVC - Timer 1 */
        case 0xFF94:
            if (wflag) {
                cpu_timer_value.BYTE.high = value;
                cpu_timer_count = cpu_timer_value.WORD;
            }
            newbyte = cpu_timer_value.BYTE.high;
            break;
  
        /* ACVC - Timer 0 */
        case 0xFF95:
            if (wflag) {
                cpu_timer_value.BYTE.low = value;
                cpu_timer_count = cpu_timer_value.WORD;
            }
            newbyte = cpu_timer_value.BYTE.low;
            break;
  
        /* ACVC - Video Mode */
        case 0xFF98:
            if (wflag) {
                screen_low_res_enabled = (value & 0x80) == 0x80;
                screen_alt_palette     = (value & 0x20) == 0x20;
            }
            newbyte |= (screen_low_res_enabled) ? 0x80 : 0;
            newbyte |= (screen_alt_palette)     ? 0x20 : 0;
            break;
  
        /* ACVC - Border Color */
        case 0xFF9A:
            screen_border_color = (wflag) ? value : screen_border_color;
            newbyte = screen_border_color;
            break;
  
        /* ACVC - Vertical Scroll */
        case 0xFF9C:
            if (wflag) {
                screen_vert_scroll_en  = (value & 0x08) == 0x08;
                screen_vert_scroll_amt = value & 0x07;
            }
            newbyte |= (screen_vert_scroll_en) ? 0x08 : 0;
            newbyte |= screen_vert_scroll_amt;
            break;
  
        /* ACVC - Vertical Offset 1 */
        case 0xFF9D:
            screen_vertical_offset.BYTE.low = (wflag) ? value : screen_vertical_offset.BYTE.low;
            newbyte = screen_vertical_offset.BYTE.low;
            break;
  
        /* ACVC - Vertical Offset 0 */
        case 0xFF9E:
            screen_vertical_offset.BYTE.high = (wflag) ? value : screen_vertical_offset.BYTE.high;
            newbyte = screen_vertical_offset.BYTE.high;
            break;
  
        /* ACVC - Horizontal Offset */
        case 0xFF9F:
            if (wflag) {
                screen_horiz_offset_size = (value & 0x80) == 0x80;
                screen_horiz_shift = value & 0x7F;
            }
            newbyte = (screen_horiz_offset_size == 256) ? 0x80 : newbyte;
            newbyte = newbyte | screen_horiz_shift;
            break;
  
        /* MMU registers - Executive set */
        case 0xFFA0:
        case 0xFFA1:
        case 0xFFA2:
        case 0xFFA3:
        case 0xFFA4:
        case 0xFFA5:
        case 0xFFA6:
        case 0xFFA7:
            if (wflag) {
                memory_mmu_exec_set[address.WORD - 0xFFA0] = value;
            } 
            newbyte = memory_mmu_exec_set[address.WORD - 0xFFA0];
            break;
  
        /* MMU registers - Task set */
        case 0xFFA8:
        case 0xFFA9:
        case 0xFFAA:
        case 0xFFAB:
        case 0xFFAC:
        case 0xFFAD:
        case 0xFFAE:
        case 0xFFAF:
            if (wflag) {
                memory_mmu_task_set[address.WORD - 0xFFA8] = value;
            }
            newbyte = memory_mmu_task_set[address.WORD - 0xFFA8];
            break;
  
        /* Palette registers */
        case 0xFFB0:
        case 0xFFB1:
        case 0xFFB2:
        case 0xFFB3:
        case 0xFFB4:
        case 0xFFB5:
        case 0xFFB6:
        case 0xFFB7:
        case 0xFFB8:
        case 0xFFB9:
        case 0xFFBA:
        case 0xFFBB:
        case 0xFFBC:
        case 0xFFBD:
        case 0xFFBE:
        case 0xFFBF:
            if (wflag) {
                screen_palette_reg[address.WORD - 0xFFB0] = value;
            }
            newbyte = screen_palette_reg[address.WORD - 0xFFB0];
            break;
  
        /* V0 - Video display buffer size */
        case 0xFFC0:     /* Set */
            screen_buffer_size_reg |= (wflag) ? 0x1 : 0;
            newbyte = screen_buffer_size_reg & 0x1;
            break;
  
        case 0xFFC1:     /* Clear */
            screen_buffer_size_reg &= (wflag) ? ~(0x1) : 0xFF;
            newbyte = screen_buffer_size_reg & 0x1;
            break;
  
        /* V1 - Video display mode */
        case 0xFFC2:     /* Set */
            screen_buffer_size_reg |= (wflag) ? 0x2 : 0;
            newbyte = (screen_buffer_size_reg & 0x2) == 0x2;
            break;
  
        case 0xFFC3:     /* Clear */
            screen_buffer_size_reg &= (wflag) ? ~(0x2) : 0;
            newbyte = (screen_buffer_size_reg & 0x2) == 0x2;
            break;
  
        /* V2 - Video display mode */
        case 0xFFC4:     /* Set */
            screen_buffer_size_reg |= (wflag) ? 0x4 : 0;
            newbyte = (screen_buffer_size_reg & 0x4) == 0x4;
            break;
  
        case 0xFFC5:     /* Clear */
            screen_buffer_size_reg &= (wflag) ? ~(0x4) : 0xFF;
            newbyte = (screen_buffer_size_reg & 0x4) == 0x4;
            break;
  
        /* F0 - Video display starting address */
        case 0xFFC6:     /* Set */
            mask.BYTE.high = 0x2;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFC7:     /* Clear */
            mask.BYTE.high = 0x2;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* F1 - Video display starting address */
        case 0xFFC8:     /* Set */
            mask.BYTE.high = 0x4;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFC9:     /* Clear */
            mask.BYTE.high = 0x4;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* F2 - Video display starting address */
        case 0xFFCA:     /* Set */
            mask.BYTE.high = 0x8;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFCB:     /* Clear */
            mask.BYTE.high = 0x8;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* F3 - Video display starting address */
        case 0xFFCC:     /* Set */
            mask.BYTE.high = 0x10;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFCD:     /* Clear */
            mask.BYTE.high = 0x10;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* F4 - Video display starting address */
        case 0xFFCE:     /* Set */
            mask.BYTE.high = 0x20;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFCF:     /* Clear */
            mask.BYTE.high = 0x20;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* F5 - Video display starting address */
        case 0xFFD0:     /* Set */
            mask.BYTE.high = 0x40;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFD1:     /* Clear */
            mask.BYTE.high = 0x40;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* F6 - Video display starting address */
        case 0xFFD2:     /* Set */
            mask.BYTE.high = 0x80;
            screen_memory_ptr.WORD |= (wflag) ? mask.WORD : 0;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        case 0xFFD3:     /* Clear */
            mask.BYTE.high = 0x80;
            screen_memory_ptr.WORD &= (wflag) ? ~(mask.WORD) : 0xFFFF;
            newbyte = (screen_memory_ptr.WORD & mask.WORD) == mask.WORD;
            break;
  
        /* SAM - R0 - MPU cycle rate */
        case 0xFFD6:     /* Clear */
            cpu.clockrate &= (wflag) ? ~0x1 : cpu.clockrate;
            newbyte = (cpu.clockrate & 0x1) > 0;
            cpu_set_clockrate();
            break;
  
        case 0xFFD7:     /* Set */
            cpu.clockrate |= (wflag) ? 0x1 : 0;
            newbyte = (cpu.clockrate & 0x1) > 0;
            cpu_set_clockrate();
            break;
  
        /* SAM - R1 - MPU cycle rate */
        case 0xFFD8:     /* Clear */
            cpu.clockrate &= (wflag) ? ~0x2 : cpu.clockrate;
            newbyte = (cpu.clockrate & 0x2) > 0;
            cpu_set_clockrate();
            break;
  
        case 0xFFD9:     /* Set */
            cpu.clockrate |= (wflag) ? 0x2 : 0;
            newbyte = (cpu.clockrate & 0x2) > 0;
            cpu_set_clockrate();
            break;
  
        /* SAM - M0 - Memory size */
        case 0xFFDA:     /* Clear */
            memory_size_bit_0 = (wflag) ? 0 : memory_size_bit_0;
            newbyte = memory_size_bit_0;
            break;
  
        case 0xFFDB:     /* Set */
            memory_size_bit_0 = (wflag) ? 1 : memory_size_bit_0;
            newbyte = memory_size_bit_0;
            break;
  
        /* SAM - M1 - Memory size */
        case 0xFFDC:     /* Clear */
            memory_size_bit_1 = (wflag) ? 0 : memory_size_bit_1;
            newbyte = memory_size_bit_1;
            break;
  
        case 0xFFDD:     /* Set */
            memory_size_bit_1 = (wflag) ? 1 : memory_size_bit_1;
            newbyte = memory_size_bit_1;
            break;
  
        /* SAM - TY - Memory map mode */
        case 0xFFDE:     /* Clear */
            memory_map_mode = (wflag) ? 0 : memory_map_mode;
            newbyte = memory_map_mode;
            break;
  
        case 0xFFDF:     /* Set */
            memory_map_mode = (wflag) ? 1 : memory_map_mode;
            newbyte = memory_map_mode;
            break;
  
        /* Reset routine vector */
        case 0xFFFE:
            newbyte = CPU_PC_START_HI;
            break;
  
        case 0xFFFF:
            newbyte = CPU_PC_START_LO;
            break;
  
        /* Anything outside of the I/O range needs to be decoded to a physical */
        /* memory address somewhere in the 512K range (this emulator will not  */
        /* attempt to simulate a 128K machine, since it involves wrapping      */
        /* memory in strange ways). The physical memory of the computer is     */
        /* divided up into 8K chunks. When the MMU is turned off, then the     */
        /* 64K of the computer defaults to the topmost 8K pages, with virtual  */
        /* page 0 starting at physical page 0x38. When the MMU is turned on,   */
        /* then the physical page accessed is dependant on which MMU set is to */
        /* be used, the TASK set or the EXECUTIVE set.                         */
        default:
            virtual_page = address.WORD / 0x2000;
            if (memory_mmu_enabled) {
                if (memory_mmu_task_or_exec) {
                    physical_page = memory_mmu_task_set[virtual_page];
                }
                else {
                    physical_page = memory_mmu_exec_set[virtual_page];
                }
            }
            else {
              physical_page = 0x38 + virtual_page;
            }
            
            /* Figure out where in memory the actual address lies */
            real_address.WORD = address.WORD + (physical_page * 0x2000) 
                                + (virtual_page * 0x2000);

            /*
            printf( "memory_access: virtual_page = %d, physical_page = 
                     %d, new address = %04X\n",
                     virtual_page, physical_page, real_address.WORD);
            */

            if (wflag) {
                memory[real_address.WORD] = value;
            }
            newbyte = memory[real_address.WORD];
            break;
    }
    return newbyte;
}

/*****************************************************************************/

/**
 * @brief Read a single byte from the specified memory location.
 *
 * Read the contents specified by the 16-bit memory location specified in 
 * \a address, and return the value of the byte read.
 * 
 * @param [in] address The 16-bit memory address to read from.
 * @return The 8-bit value read.
 */
inline byte 
memory_read(register word address)
{
    return memory_access(address, 0, FALSE);
}

/*****************************************************************************/

/**
 * @brief Read a word from the specified memory location.
 *
 * Read the contents specified by the 16-bit memory location specified in
 * \a address, and return the value of the two bytes read.
 *
 * @param [in] address The 16-bit memory address to read from.
 * @return The 16-bit value read.
 */
inline word 
memory_read16(register word address)
{
    word newword;
    newword.BYTE.high = memory_access(address, 0, FALSE);
    address.WORD++;
    newword.BYTE.low = memory_access(address, 0, FALSE);
    return newword;
}

/*****************************************************************************/

/**
 * @brief Write a byte to the specified memory location.
 *
 * Given an 8-bit value, writes the value to the 16-bit memory location
 * specified in \a address.
 *
 * @param [in] address The 16-bit memory address to write to.
 * @param [in] value The 8-bit value to write.
 */
inline void 
memory_write(register word address, register byte value)
{
    memory_access(address, value, TRUE);
}

/*****************************************************************************/

/**
 * @brief Write a word to the specified memory location.
 *
 * Given an 16-bit value, writes the value to the 16-bit memory location
 * specified in \a address.
 *
 * @param [in] address The 16-bit memory address to write to.
 * @param [in] value The 16-bit value to write.
 */
inline void 
memory_write16(register word address, register word value)
{
    memory_access(address, value.BYTE.high, TRUE);
    address.WORD++;
    memory_access(address, value.BYTE.low, TRUE);
}

/*****************************************************************************/

/**
 * @brief Destroys (frees) emulator memory.
 *
 * Frees all of the memory that has been allocated to the emulator.
 */
void 
memory_destroy(void)
{
    free(memory);
    memory = NULL;
}

/*****************************************************************************/

/**
 * @brief Initializes emulator memory to the specified size in bytes.
 *
 * Attempts to malloc a memory block of the specified size for use as
 * emulator memory. Returns TRUE (1) on success or FALSE (0) on failure
 * to allocate.
 * 
 * @param [in] memorysize The size of memory to allocate in bytes.
 */
int 
memory_init(int memorysize)
{
    if (memorysize <= 0) {
        return (FALSE);
    }

    memory = (byte *)malloc(sizeof (byte) * memorysize);

    if (!memory) {
        return FALSE;
    }

    memset((void *)memory, 128, memorysize);
    memory_phys_size = memorysize;

    /* Set the memory size bits according to the physical memorysize */
    if (memory_phys_size <= 4096) {
        memory_size_bit_0 = 0;
        memory_size_bit_1 = 0;
    }
    else if (memory_phys_size == 16384) {
        memory_size_bit_0 = 1;
        memory_size_bit_1 = 0;
    }
    else {
        memory_size_bit_0 = 0;
        memory_size_bit_1 = 1;
    }

    /* Set the memory map mode to RAM/ROM */
    memory_map_mode = 0;

    return TRUE;
}

/* E N D   O F   F I L E *****************************************************/
