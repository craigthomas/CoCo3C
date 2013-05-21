/**
 * Copyright (C) 2013 Craig Thomas
 * This project uses an MIT style license - see the LICENSE file for details.
 *
 * @file      keyboard.c
 * @brief     Functions to map keypresses to emulator keys
 * @author    Craig Thomas
 * 
 * This file contains functions that are used to access the keyboard status
 * from within the emulator.
 */

/* I N C L U D E S ***********************************************************/

#include <stdio.h>
#include <malloc.h>
#include <SDL.h>
#include "yacoco3e.h"

/* F U N C T I O N S *********************************************************/

/**
 * @brief Returns a 2 byte array that describes the set of keys pressed
 *
 * The high byte of the result describes which ROW of the keyboard was
 * pressed, while the low byte of the word describes which COLUMN of the
 * keyboard was pressed. A keypress is detected via a LOW or 0 signal,
 * instead of a 1. This means that the complement of the actual value is 
 * returned in the word.
 *
 * @return The two byte description of the keypress.
 */
word 
keyboard_returnstatusbytes(void)
{
   word result;
   Uint8 *keystate;
   SDLMod modifier;

   /* The keys on the COCO3 keyboard are as follows (symbols in brackets      */
   /* represent shifted values)                                               */
   /*                                                                         */
   /* 1(!)  2(")  3(#)  4($)  5(%)  6(&)  7(')  8(()  9())  0( )  :(*)  -(=)  */
   /* ALT   Q     W     E     R     T     Y     U     I     O     P     @     */
   /* CTRL  A     S     D     F     G     H     J     L     ;(+)  ENTER       */
   /* SHIFT Z     X     C     V     B     N     M     ,(<)  .(>)  /(?)  SHIFT */
   /* UP    DOWN  LEFT  RIGHT F1    F2    BREAK ESC                           */
   /*                                                                         */
   /* A single word is used to encode what keys are pressed. The HIGH byte of */
   /* the word represents the ROW of the key in the table below. The LOW byte */
   /* of the word represents the COLUMN of the key in the table below. To get */
   /* the value of the keyboard, this keyboard word is available at memory    */
   /* addresses: FF00 (HIGH) and FF02 (LOW).                                  */
   /*                                                                         */
   /* HIGH                                                                    */
   /*   7                                                                     */
   /*   6   SHIFT   F2    F1   CTRL   ALT   BRK   CLR   ENTER                 */
   /*   5     /     .     -     ,      ;     :     9      8                   */
   /*   4     7     6     5     4      3     2     1      0                   */
   /*   3   SPACE   RIGHT LEFT  DOW    UP    z     y      x                   */
   /*   2     w     v     u     t      s     r     q      p                   */
   /*   1     o     n     m     l      k     j     i      h                   */
   /*   0     g     f     e     d      c     b     a      @                   */
   /*                                                                         */
   /*         7     6     5     4      3     2     1      0                   */
   /*                          LOW                                            */
   keystate = SDL_GetKeyState(NULL);
   modifier = SDL_GetModState();

   result.BYTE.low = 0;
   result.BYTE.high = 0;

   if (modifier & KMOD_CTRL) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x10;
   }

   if (modifier & KMOD_ALT) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x08;
   }

   if (keystate[SDLK_F1]) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x20;
   }

   if (keystate[SDLK_F2]) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x40;
   }

   if (keystate[SDLK_ESCAPE]) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x04;
   }

   if (keystate[SDLK_RETURN]) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x01;
   }

   if (keystate[SDLK_HOME]) {
      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x02;
   }

   if (keystate[SDLK_QUESTION]   ||
       keystate[SDLK_EXCLAIM]    ||
       keystate[SDLK_QUOTEDBL]   ||
       keystate[SDLK_HASH]       ||
       keystate[SDLK_DOLLAR]     ||
       keystate[SDLK_AMPERSAND]  ||
       keystate[SDLK_QUOTE]      ||
       keystate[SDLK_LEFTPAREN]  ||
       keystate[SDLK_RIGHTPAREN] ||
       keystate[SDLK_ASTERISK]   ||
       keystate[SDLK_EQUALS]     ||
       keystate[SDLK_PLUS]       ||
       keystate[SDLK_LESS]       ||
       keystate[SDLK_GREATER]) {

      result.BYTE.high |= 0x40;
      result.BYTE.low |= 0x80;

      if (keystate[SDLK_QUESTION]) {
          result.BYTE.high |= 0x20; 
          result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_EXCLAIM]) {
         result.BYTE.high |= 0x10;
         result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_QUOTEDBL]) {
         result.BYTE.high |= 0x10;
         result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_HASH]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_DOLLAR]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_AMPERSAND]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_QUOTE]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_LEFTPAREN]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x01;
      }

      if (keystate[SDLK_RIGHTPAREN]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_ASTERISK]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_EQUALS]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_PLUS]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_LESS]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_GREATER]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x40;
      }
   }
   else {
      if (keystate[SDLK_SLASH]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_PERIOD]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_MINUS]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_COMMA]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_SEMICOLON]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_COLON]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_9]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_8]) {
          result.BYTE.high |= 0x20;
          result.BYTE.low |= 0x01;
      }

      if (keystate[SDLK_7]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_6]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_5]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_4]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_3]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_2]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_1]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_0]) {
          result.BYTE.high |= 0x10;
          result.BYTE.low |= 0x01;
      }

      if (keystate[SDLK_SPACE]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_RIGHT]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_LEFT]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_DOWN]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_UP]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_z]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_y]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_x]) {
         result.BYTE.high |= 0x08;
         result.BYTE.low |= 0x01;
      }

      if (keystate[SDLK_w]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_v]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_u]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_t]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_s]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_r]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_q]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_p]) {
         result.BYTE.high |= 0x04;
         result.BYTE.low |= 0x01;
      }

      if (keystate[SDLK_o]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_n]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_m]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_l]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_k]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_j]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_i]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_h]) {
         result.BYTE.high |= 0x02;
         result.BYTE.low |= 0x01;
      }

      if (keystate[SDLK_g]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x80;
      }

      if (keystate[SDLK_f]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x40;
      }

      if (keystate[SDLK_e]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x20;
      }

      if (keystate[SDLK_d]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x10;
      }

      if (keystate[SDLK_c]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x08;
      }

      if (keystate[SDLK_b]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x04;
      }

      if (keystate[SDLK_a]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x02;
      }

      if (keystate[SDLK_AT]) {
          result.BYTE.high |= 0x01;
          result.BYTE.low |= 0x01;
      }

      if (modifier & KMOD_SHIFT) {
         result.BYTE.high |= 0x40;
         result.BYTE.low |= 0x80;
      }
    }

    /* The values are active low, meaning that we want the inverse of the */
    /* bit sequences that we had before */
    result.BYTE.high = ~result.BYTE.high;
    result.BYTE.low = ~result.BYTE.low;

    return result;
}

/* E N D   O F   F I L E *****************************************************/
