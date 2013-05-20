/*!
 * @file      trs80e.h
 * @brief     Global variables and structures defined for the emulator.
 * @author    Craig Thomas
 * @copyright MIT style license - see the LICENSE file for details
 * @copyright @verbinclude LICENSE
 *
 * This file contains the global variables and structures that are used
 * throughout the emulator. It provides prototype definitions for each of
 * the functions used throughout the emulator project, as well as the
 * hard coded definitions needed for the emulator to function.
 */

#ifndef GLOBALS_H_ 
#define GLOBALS_H_

/* I N C L U D E S ***********************************************************/

#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include "minunit.h"

/* D E F I N E S *************************************************************/

/* Memory */
#define MEM_4K         0x1000  /**< Defines a 4K memory size */
#define MEM_16K        0x4000  /**< Defines a 16K memory size */
#define MEM_32K        0x8000  /**< Defines a 32K memory size */
#define MEM_64K        0x10000 /**< Defines a 64K memory size */
#define MEM_128K       0x20000 /**< Defines a 128K memory size */
#define MEM_512K       0x80000 /**< Defines a 512K memory size */
#define ROM_DEFAULT    0x0     /**< Defines the default load point for ROMs */
#define   M0           0       /**< Memory size bit 0 */
#define   M1           1       /**< Memory size bit 1 */
#define NUM_MMU_REGS   8       /**< Number of MMU registers */

/* Screen */
#define SCREEN_HEIGHT      240 /**< Default screen height */
#define SCREEN_WIDTH       320 /**< Default screen width */
#define SCREEN_DEPTH       32  /**< Colour depth in Bites Per Pixel (BPP) */
#define SCALE_FACTOR       3   /**< Scaling for the window size */
#define PIXEL_COLOR        250 /**< Color to use for drawing pixels */
#define SCREEN_VERTREFRESH 60  /**< Vertical refresh of the screen in Hz */
#define NUM_PALETTE_REGS   16  /**< Stores the number of palette registers */

/* CPU state definitions */
#define CPU_RUNNING    1       /**< Continues CPU execution */
#define CPU_PAUSED     2       /**< Pauses the CPU */
#define CPU_DEBUG      3       /**< Sets the debugging mode */
#define CPU_TRACE      4       /**< Sets trace without debug */
#define CPU_STEP       5       /**< Wait for keypress */
#define CPU_UNIT_TEST  6       /**< Runs a single CPU operation and exits */
#define CPU_STOP       0       /**< Halts the CPU and quits */
#define CPU_89MHZ      1123    /**< Sets the nanos associated with .89 MHz */
#define CPU_178MHZ     561     /**< Sets the nanos associated with 1.78 MHz */

#define REG_U_START     0x0    /**< The start address of the U stack */
#define REG_S_START     0x0    /**< The start address of the S stack */
#define CPU_RESET_ADDR  0xFFFE /**< The start address of the PC */
#define CPU_PC_START_HI 0x8C   /**< The start location of the PC */
#define CPU_PC_START_LO 0x1B   /**< The start location of the PC */

/* Flags to set the register we are looking at */
#define REG_A   1        /**< A register flag */
#define REG_B   2        /**< B register flag */
#define REG_D   3        /**< D register flag */
#define REG_DP  4        /**< DP register flag */
#define REG_CC  5        /**< CC register flag */
#define REG_PC  6        /**< PC register flag */
#define REG_X   7        /**< X register flag */
#define REG_Y   8        /**< Y register flag */
#define REG_U   9        /**< U register flag */
#define REG_S   0        /**< S register flag */
#define REG_UNK -1       /**< Unknown register flag */

/* Flags to use for condition code register */
#define REG_CC_C 0x01    /**< Carry bit */
#define REG_CC_V 0x02    /**< Overflow bit */
#define REG_CC_Z 0x04    /**< Zero bit */
#define REG_CC_N 0x08    /**< Negative bit */
#define REG_CC_I 0x10    /**< Interrupt Request Mask bit */
#define REG_CC_H 0x20    /**< Half Carry bit */
#define REG_CC_F 0x40    /**< Fast Interrupt Request Mask bit */
#define REG_CC_E 0x80    /**< Everything bit */

#define NO_PC_UPDATE -1  /**< Tell the CPU that the PC should not be updated */
#define NO_CPU_BRANCH 0  /**< Tell the CPU that no PC branch should occur */

#define R0            0  /**< CPU clock rate bit 0 */
#define R1            1  /**< CPU clock rate bit 1 */

/* Keyboard */
#define KEYBOARD_BUFFERSIZE   8    /**< Number of keys to store in the buffer */
#define KEYBOARD_DEBUG        1    /**< Only accept debug keystrokes */
#define KEYBOARD_NORMAL       0    /**< Only accept normal keystrokes */
#define KEYBOARD_NUMBEROFKEYS 15   /**< Number of keys on the keyboard */
#define KEYBOARD_NOKEY        -999 /**< Sets the no keypress value */

/* Keyboard special keys */
#define QUIT_KEY   SDLK_F4     /**< Quits the emulator                        */
#define DEBUG_KEY  SDLK_F5     /**< Puts emulator into debug mode             */
#define TRACE_KEY  SDLK_F6     /**< Puts emulator into trace mode             */
#define NORMAL_KEY SDLK_F7     /**< Returns emulator to normal running mode   */
#define STEP_KEY   SDLK_F8     /**< Runs next instruction (in debug mode)     */

/* Other generic definitions */
#define TRUE           1          /**< Sets the value for true */
#define FALSE          0          /**< Sets the value for false */
#define MAXSTRSIZE     300        /**< Maximum buffer size for strings */

/* Interrupt Vectors - essentially PC = [INT_VAL] */
#define RESET_INT 0xFFFE   /**< Reset interrupt , default value = 0xA027 */
#define NMI_INT   0xFFFC   /**< Non-Maskable interrupt, BASIC value = 0x0109 */
#define IRQ_INT   0xFFF8   /**< Interrupt Request, BASIC value = 0x010C */
#define FIRQ_INT  0xFFF6   /**< Fast Interrupt Request, BASIC value = 0x010F */
#define SWI_INT   0xFFFA   /**< Software Interrupt 1, BASIC value = 0x0106 */
#define SWI2_INT  0xFFF4   /**< Software Interrupt 2, BASIC value = 0x0103 */
#define SWI3_INT  0xFFF2   /**< Software Interrupt 3, BASIC value = 0x0100 */

/* T Y P E D E F S ***********************************************************/

/*!
 * @brief Defines an emulator byte.
 *
 * A byte in the emulator is an 8-bit byte. To accomplish using exactly 8-bits,
 * an unsigned character is used.
 * @note If the emulator is going to be ported to a platform where unsigned
 * char is greater than 8-bits in length, then this definition will need to 
 * change accordingly.
 */
typedef unsigned char byte;

/*!
 * @brief Defines a signed byte.
 *
 * A signed byte is still an 8-bit byte, however, the sign bit is used in 
 * determining the sign of the value.
 */
typedef signed char sbyte;

/*!
 * @brief Define a word in terms of two bytes.
 *
 * A word is a combination of two bytes. Each word has a high byte and a low
 * byte. The value of a word can be accessed either through the WORD value
 * alone, or via the .high or .low attribute.
 */
typedef union
{
   unsigned short int WORD;     /**< The combined high and low bytes */
   struct
   {
      #ifdef WORDS_BIGENDIAN
         byte high, low;
      #else
         byte low, high;
      #endif
   } BYTE;                      /**< A struct of the high and low bytes */
} word;

/*!
 * @brief Defines a keyboard key.
 * 
 * \a KEYSPEC structures encode a single keypress. 
 */
typedef struct
{
   int keycode;        /**< The keycode value of the keypress */
   SDLKey symbol;      /**< The SDL value of the keypress */
} KEYSPEC;

/*!
 * @brief Stores a color in RGB format.
 */
typedef struct
{
   Uint8 r;
   Uint8 g;
   Uint8 b;
} COLOR;

typedef struct 
{
    byte a;               /**< General purpose A register */
    byte b;               /**< General purpose B register */
    word d;               /**< A and B registers combined */
    byte dp;              /**< Direct Page register */
    byte cc;              /**< Condition Code register */
    word pc;              /**< Program Counter register */
    word oldpc;           /**< The last Program Counter value */
    word x;               /**< Index register X */
    word y;               /**< Index register Y */
    word s;               /**< Stack pointer S */
    word u;               /**< Stack pointer U */

    int state;            /**< Current state of the CPU */

    char *opdesc;         /**< Describes the current operation */
    char *opshortdesc;    /**< Short description of the current operation */

    byte operand;         /**< The current operand */
    int bytesconsumed;   /**< Stores how many bytes the opcode took */

    byte clockrate;       /**< The current clockrate bits */

} mc6809e_regset;

/* G L O B A L S **************************************************************/

/* Memory */
byte *memory;                  /**< Pointer to emulator memory region */
int memory_phys_size;          /**< The physical size of memory */
byte memory_size_bit_0;        /**< The state of memory size bit 0 */
byte memory_size_bit_1;        /**< The state of memory size bit 1 */
byte memory_map_mode;          /**< The currently set memory mapping mode */
byte memory_mmu_task_set[NUM_MMU_REGS]; /* Task MMU set */
byte memory_mmu_exec_set[NUM_MMU_REGS]; /* Executive MMU set */
byte memory_mmu_enabled;       /**< Whether the MMU is turned on or not */
byte memory_mmu_task_or_exec;  /**< Whether TASK (1) or EXECUTIVE (0) MMU set */

/* Screen */
SDL_Surface *screen;           /**< Stores the main screen SDL structure */
SDL_Surface *virtscreen;       /**< Stores the main screen SDL structure */
SDL_Surface *overlay;          /**< Stores the main screen SDL structure */
int screen_width;              /**< Stores the width of the screen in pixels */
int screen_height;             /**< Stores the height of the screen in pixels */
int screen_depth;              /**< Stores the colour depth in BPP */
int screen_scale_factor;       /**< Stores the display scaling factor */
word screen_vertical_offset;   /**< Stores the vertical offset */
byte screen_vert_scroll_en;    /**< Stores vertical scrolling enabled */
byte screen_vert_scroll_amt;   /**< Stores how much scrolling occurs */
byte screen_horiz_offset_size; /**< Stores the horizontal buffer size */
byte screen_horiz_shift;       /**< Stores current horizontal of the image */
word screen_memory_ptr;        /**< Stores active RAM address for display */
byte screen_buffer_size_reg;   /**< Stores the buffer size register */
byte screen_vdg_old_mode;      /**< Stores the VDG mode written to 0xFF22 */
byte screen_low_res_enabled;   /**< Stores low-resolution video modes flag */
byte screen_border_color;      /**< Stores the border color */
byte screen_buffer_size;       /**< Stores the buffer size bits */
byte screen_palette_reg[NUM_PALETTE_REGS]; /**< The palette registers */
byte screen_alt_palette;       /**< Stores flag to display alternate palette */
int screen_refresh_flag;

/* CPU */
mc6809e_regset cpu;            /**< The Color Computer 3 processor (mc6809e) */
int cpu_state;                 /**< Current state of the CPU */
SDL_TimerID cpu_timer;         /**< A CPU tick timer */
unsigned long cpu_interrupt;   /**< The CPU interrupt routine */
int cpu_timer_count;           /**< The 6809 timer max value */
byte cpu_fire_interrupt;       /**< Whether an interrupt should be raised */
byte cpu_irq_enabled;          /**< Whether IRQ is enabled - set in 0xFF92 */
byte cpu_firq_enabled;         /**< Whether or not FIRQ is enabled */
byte cpu_timer_rate;           /**< How fast timer0 and timer1 count down */
word cpu_timer_value;          /**< Stores the CPU timer */
byte cpu_irq_value;            /**< Stores what device triggered the irq */
byte cpu_firq_value;           /**< Stores what device triggered the firq */
byte cpu_irq_flag;             /**< Interrupt flag */
byte cpu_firq_flag;            /**< Fast interrupt flag */
byte cpu_nmi_flag;             /**< Non-maskable interrupt flag */
byte cpu_reset_flag;           /**< Stores whether a reset state occurred */
byte cpu_swi_flag;             /**< Stores whether a user interrupt occurred */

/* Disk */
byte disk_motor_on;            /**< Whether or not the disk drive motor is on */

/* Event captures */
SDL_Event event;               /**< Stores SDL events */

/* Keyboard */
word keypress;                 /**< Stores the set of keypresses */

/* Unit Tests */
int tests_run;                 /**< Stores how many unit tests were run */

/* P R O T O T Y P E S *******************************************************/

/* memory.c */
inline byte memory_read(register word address);
inline word memory_read16(register word address);
inline void memory_write(register word address, register byte value);
inline void memory_write16(register word address, register word value);
int memory_init(int memorysize);
void memory_destroy(void);

/* cpu.c */
void cpu_set_clockrate(void);
int cpu_timerinit(void);
Uint32 cpu_timerinterrupt(Uint32 interval, void *parameters);
void cpu_timer_countdown(void);
inline void cpu_push_stack(int reg, byte value);
inline byte cpu_pop_stack(int reg);
inline void cpu_branch_short(int test, byte offset);
inline int cpu_branch_long(int test, word offset);
inline byte cpu_binary_add8(byte val1, byte val2, int hflag, 
                             int cflag, int vflag);
inline word cpu_binary_add16(word val1, word val2, int hflag, 
                             int cflag, int vflag);
inline byte cpu_twos_comp8(byte val);
inline word cpu_twos_comp16(word val);
int cpu_get_signed16(word value);
inline word cpu_get_immediate16(word ptr);
inline byte cpu_get_immediate8(word ptr);
inline word cpu_get_direct(word ptr);
inline void cpu_set_irq_enabled(byte enabled);
inline void cpu_get_irq_enabled(void);
void cpu_process_sdl_events(void);
void cpu_reset(void);
void cpu_execute(void);

/* screen.c */
int screen_drawpixel(int x, int y, Uint8 r, Uint8 g, Uint8 b);
int screen_getpixel(int x, int y);
void screen_refresh(int overlay_on);
void screen_draw(int x, int y, Uint8 r, Uint8 g, Uint8 b);
int screen_init(int width, int height);

/* keyboard.c */
word keyboard_returnstatusbytes(void);

/* vdg_mc6847.c */
void vdg_init(void);
void vdg_refresh(void);

/* unit_tests.c */
void run_tests(void);
void check_result(char *result);
char *memory_tests(void);
char *cpu_tests(void);

#endif

/* E N D   O F   F I L E *****************************************************/
