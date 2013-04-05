/*!
 * @file     screen.c
 * @brief    Functions and routines for drawing items to the screen.
 * @author    Craig Thomas
 * @copyright MIT style license - see the LICENSE file for details
 * @copyright @verbinclude LICENSE
 * 
 * This file contains the functions needed to draw and manipulate pixels on
 * the emulator display.
 */

/* I N C L U D E S ***********************************************************/

#include <SDL.h>
#include <SDL_ttf.h>
#include <stdlib.h>
#include "trs80e.h"

/* L O C A L S ***************************************************************/

TTF_Font *message_font;
Uint32 COLOR_DGREEN;
Uint32 COLOR_LGREEN;
Uint32 COLOR_BLACK;
SDL_Color COLOR_TEXT;

/* F U N C T I O N S *********************************************************/

/**
 * @brief Gets the color value of the pixel at the specified location
 *
 * Looks up the pixel at location \a x and \a y, and returns the value of
 * the pixel color at that location.
 *
 * @param [in] x The x location for the pixel
 * @param [in] y The y location for the pixel
 * @return The color value for that pixel
 */
int 
screen_getpixel(int x, int y)
{
    Uint8 r, g, b;
    Uint32 pixelcolor = 0;
    x = x * screen_scale_factor;
    y = y * screen_scale_factor;
    Uint32 *pixels = (Uint32 *)virtscreen->pixels;
    Uint32 pixel = pixels[(virtscreen->w * y) + x];
    SDL_GetRGB(pixel, virtscreen->format, &r, &g, &b);
    pixelcolor = SDL_MapRGB(virtscreen->format, r, g, b);
    return pixelcolor;
}

/******************************************************************************/

/**
 * Helper routine to blit one surface onto another surface at the specified
 * location. This routine blits the entire source surface onto the destination.
 *
 * @param src the source surface to blit
 * @param dest the destination surface to blit to
 * @param x the x location on the destination to blit to
 * @param y the y location on the destination to blit to
 */
void 
screen_blit_surface(SDL_Surface *src, SDL_Surface *dest, int x, int y)
{
    SDL_Rect location;
    location.x = x;
    location.y = y;
    SDL_BlitSurface(src, NULL, dest, &location);
}

/*****************************************************************************/

/**
 * Helper routine to create an SDL surface that contains a text message.
 *
 * @param msg the message to blit
 * @param text_color the color of the text to draw
 * @return the SDL_Surface with the message text
 */
SDL_Surface *
screen_render_message(char *msg, SDL_Color text_color)
{
    return TTF_RenderText_Solid(message_font, msg, text_color);
}

/******************************************************************************/

/**
 * Generates the current state of the CPU in an overlay on the main screen.
 * All of the CPU registers, along with the operand and a description of the
 * operand will be printed to the overlay. The overlay has an alpha channel
 * set such that it will not overwrite the Chip 8 screen - it will appear to 
 * be semi-transparent. 
 */
void 
screen_trace_message(void)
{
    SDL_Surface *msg_surface;
    SDL_Rect box;

    box.x = 5;
    box.y = screen_height - 58;
    box.w = 602;
    box.h = 53;
    SDL_FillRect(overlay, &box, COLOR_LGREEN);

    box.x = 6;
    box.y = screen_height - 57;
    box.w = 600;
    box.h = 51;
    SDL_FillRect(overlay, &box, COLOR_DGREEN);

    char *buffer = (char *)malloc(MAXSTRSIZE);

    sprintf(buffer, "PC:%04X - op %02X - %s",
            cpu.oldpc.WORD, cpu.operand, cpu.opshortdesc);
    msg_surface = screen_render_message(buffer, COLOR_TEXT);
    screen_blit_surface(msg_surface, overlay, 10, screen_height - 53);
    SDL_FreeSurface(msg_surface);

    sprintf(buffer,
             "A:%02X B:%02X D:%04X X:%04X Y:%04X U:%04X S:%04X DP:%02X E:%d F:%d H:%d I:%d N:%d Z:%d V:%d C:%d",
             cpu.a, cpu.b, cpu.d.WORD, cpu.x.WORD, cpu.y.WORD, 
             cpu.u.WORD, cpu.s.WORD, cpu.dp, cpu_bit_mask (cpu.cc, REG_CC_E),
             cpu_bit_mask(cpu.cc, REG_CC_F),
             cpu_bit_mask(cpu.cc, REG_CC_H),
             cpu_bit_mask(cpu.cc, REG_CC_I),
             cpu_bit_mask(cpu.cc, REG_CC_N),
             cpu_bit_mask(cpu.cc, REG_CC_Z),
             cpu_bit_mask(cpu.cc, REG_CC_V),
             cpu_bit_mask(cpu.cc, REG_CC_C));
    msg_surface = screen_render_message(buffer, COLOR_TEXT);
    screen_blit_surface(msg_surface, overlay, 10, screen_height - 38);
    SDL_FreeSurface(msg_surface);

    sprintf(buffer, "%s", cpu.opdesc); 
    msg_surface = screen_render_message(buffer, COLOR_TEXT);
    screen_blit_surface(msg_surface, overlay, 10, screen_height - 23);
    SDL_FreeSurface(msg_surface);

    free(buffer);
}

/******************************************************************************/

/**
 * Refreshes the screen. If overlay_on is a non-zero value, then the debug 
 * overlay will be turned on and will be painted with the refresh.
 */
void 
screen_refresh(int overlay_on)
{
    screen_blit_surface(virtscreen, screen, 0, 0);
    if (overlay_on) {
        screen_trace_message();
        screen_blit_surface(overlay, screen, 0, 0);
    }
    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

/*****************************************************************************/

/**
 * @brief Draws a pixel on to the screen with the specified color.
 *
 * This is the main pixel drawing function to be used by the emulator. It
 * takes into account the scaling factor applied to the display. It will write
 * the pixel to the specified location with the specified color value.
 *
 * @param [in] x The x location of the pixel
 * @param [in] y The y location of the pixel
 * @param [in] r The red value for the pixel
 * @param [in] g The green value for the pixel
 * @param [in] b The blue value for the pixel
 */
void 
screen_draw(int x, int y, Uint8 r, Uint8 g, Uint8 b)
{
    SDL_Rect pixel;
    Uint32 pixelcolor = SDL_MapRGB(virtscreen->format, r, g, b);

    pixel.x = x * screen_scale_factor;
    pixel.y = y * screen_scale_factor;
    pixel.w = screen_scale_factor;
    pixel.h = screen_scale_factor;
    SDL_FillRect(virtscreen, &pixel, pixelcolor);
}

/*****************************************************************************/

/**
 * Creates a new SDL surface with the specified dimensions, per-surface alpha
 * value, and the specified color_key. If color_key is equal to -1, then color
 * keys are turned off for the surface. The new surface will be exclusively a
 * software surface. Returns NULL if there was a problem creating the surface.
 * Note that it is also up to the user to call SDL_FreeSurface when they are
 * done with the surface. 
 *
 * @param width the width of the new surface, in pixels
 * @param height the height of the new surface, in pixels
 * @param alpha the alpha blend value of the entire surface
 * @param color_key the key to use as a color key for the surface
 * @returns the newly created surface
 */
SDL_Surface *
screen_create_surface(int width, int height, int alpha, Uint32 color_key)
{
    Uint32 rmask, gmask, bmask, amask;
    SDL_Surface *tempsurface, *newsurface;

    #if SDL_BYTEORDER == SDL_BIG_ENDIAN
        rmask = 0xff000000;
        gmask = 0x00ff0000;
        bmask = 0x0000ff00;
        amask = 0x000000ff;
    #else
        rmask = 0x000000ff;
        gmask = 0x0000ff00;
        bmask = 0x00ff0000;
        amask = 0xff000000;
    #endif

    tempsurface = SDL_CreateRGBSurface(SDL_SWSURFACE | SDL_SRCALPHA,
                                    screen_width, screen_height, SCREEN_DEPTH,
                                    rmask, gmask, bmask, amask);
    newsurface = SDL_DisplayFormat(tempsurface);
    SDL_FreeSurface(tempsurface);

    if (newsurface == NULL) {
        printf("Error: unable to create new surface\n");
    }
    else {
        SDL_SetAlpha(newsurface, SDL_SRCALPHA, alpha);
        if (color_key != -1) {
            SDL_SetColorKey(newsurface, SDL_SRCCOLORKEY, color_key);
        }
    }

    return newsurface;
}

/*****************************************************************************/

/**
 * @brief Initializes the emulator display.
 *
 * Initializes the emulator display with the specified \a width, \a height,
 * \a depth, \a screen_scale_factor. 
 *
 * @param [in] width The width of the screen in pixels
 * @param [in] height The height of the screen in pixels
 * @return TRUE (1) if the screen could be initialized, FALSE (0) otherwise
 */
int 
screen_init(int width, int height)
{
    int result = FALSE;

    TTF_Init();
    message_font = TTF_OpenFont("VeraMono.ttf", 11);
    screen_width = width * screen_scale_factor;
    screen_height = height * screen_scale_factor;

    screen = SDL_SetVideoMode(screen_width, 
                              screen_height, 
                              SCREEN_DEPTH, 
                              SDL_SWSURFACE);

    if (screen == NULL) {
        printf("Error: Unable to set video mode: %s\n", SDL_GetError());
    }
    else {
        SDL_SetAlpha(screen, SDL_SRCALPHA, 255);
        SDL_WM_SetCaption("TRS80 Emulator", NULL);

        /* Initialize colors for debug overlay */
        COLOR_TEXT.r = 255;
        COLOR_TEXT.g = 255;
        COLOR_TEXT.b = 255;
        COLOR_DGREEN = SDL_MapRGB(screen->format, 0, 70, 0);
        COLOR_LGREEN = SDL_MapRGB(screen->format, 0, 200, 0);
        COLOR_BLACK = SDL_MapRGB(screen->format, 0, 0, 0);

        virtscreen = screen_create_surface(screen_width, screen_height,
                                           255, -1);
        overlay = screen_create_surface(screen_width, screen_height,
                                        200, COLOR_BLACK);

        result = TRUE;
    }

    return ((virtscreen != NULL) && (overlay != NULL) && result);
}

/* E N D   O F   F I L E *****************************************************/
