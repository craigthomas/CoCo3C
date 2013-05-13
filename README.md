# Yet Another Color Computer 3 Emulator

## What is it?

This project is a Color Computer 3 emulator written in C. Note that I cannot 
distribute ROM files with the emulator, as they are copyright their respective
owners. 

The Color Computer 3 is the third incarnation of the Tandy Radio Shack 
Color Computer line (TRS-80). The CoCo 3 offered several improvements over the
original CoCo 1 and CoCo 2, most notably the introduction of a memory
management unit (MMU) and a new Advanced Color Video Chip (ACVC) - also known
as the Graphics Interrupt Memory Enhancer (GIME). 

While the official name of the computer was the TRS-80 Color Computer 3,
the Color Computer family was quite different from the line of business 
machines such as the TRS-80 Model I, II, III, and 4. While that family
of computers used a Zilog Z80 microprocessor, the Color Computer family used 
a Motorola 6809E processor running at 0.89 MHz. 

## Current Status - May 13, 2013

Please note that this project is not yet complete. As such, there is currently 
no complete set of sourcefiles available online. 


## License

This project makes use of an MIT style license. Please see the file called 
LICENSE for more information.


## Compiling

Simply copy the source files to a directory of your choice. In addition to
the source, you will need the following required software packages:

* [GNU C Compiler](http://gcc.gnu.org/) 4.6.3 or later
* [GNU Make](http://www.gnu.org/software/make/) 3.81 or later
* [SDL](http://www.libsdl.org/) 1.2.14 or later
* [SDL TTF Extension](http://www.libsdl.org/projects/SDL_ttf/) 2.0.11 or later

Note that other C compilers make work as well. To compile the project, open a
command prompt, switch to the source directory, and type:

    make

Assuming that the required packages are installed and available on your search
path, the project should compile without errors. The standard build variables
are available if you wish to customize the build further. For example, to use
an ARM based C compiler, simply override the `CC` variable during the build:

    CC=arm-c-compiler-bin make

Regardless of the compiler, a successful build should result in a single binary
in the source directory called:

    trs80e

The binary stands for "Yet Another TRS-80 Emulator".


## Further Documentation

Comments in the source code are written to conform to Doxygen conventions. 
A Doxygen configuration file, along with an associated make target have been
supplied. Simply type:

    make doc

This will create a directory called `doc` with `html` and `latex` directories.
Under `html`, open the `index.html` file in a web browser for more information.


## Third Party Copyrights

This project makes use of an unmodified "Vera Mono" font under the 
following license agreement:

    Copyright (c) 2003 by Bitstream, Inc. All Rights Reserved. Bitstream
    Vera is a trademark of Bitstream, Inc.

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of the fonts accompanying this license ("Fonts") and associated
    documentation files (the "Font Software"), to reproduce and distribute
    the Font Software, including without limitation the rights to use,
    copy, merge, publish, distribute, and/or sell copies of the Font
    Software, and to permit persons to whom the Font Software is furnished
    to do so, subject to the following conditions:
    
    The above copyright and trademark notices and this permission notice
    shall be included in all copies of one or more of the Font Software
    typefaces.
    
    The Font Software may be modified, altered, or added to, and in
    particular the designs of glyphs or characters in the Fonts may be
    modified and additional glyphs or characters may be added to the
    Fonts, only if the fonts are renamed to names not containing either
    the words "Bitstream" or the word "Vera".
    
    This License becomes null and void to the extent applicable to Fonts
    or Font Software that has been modified and is distributed under the
    "Bitstream Vera" names.
    
    The Font Software may be sold as part of a larger software package but
    no copy of one or more of the Font Software typefaces may be sold by
    itself.
    
    THE FONT SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT
    OF COPYRIGHT, PATENT, TRADEMARK, OR OTHER RIGHT. IN NO EVENT SHALL
    BITSTREAM OR THE GNOME FOUNDATION BE LIABLE FOR ANY CLAIM, DAMAGES OR
    OTHER LIABILITY, INCLUDING ANY GENERAL, SPECIAL, INDIRECT, INCIDENTAL,
    OR CONSEQUENTIAL DAMAGES, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    OTHERWISE, ARISING FROM, OUT OF THE USE OR INABILITY TO USE THE FONT
    SOFTWARE OR FROM OTHER DEALINGS IN THE FONT SOFTWARE.
    
    Except as contained in this notice, the names of Gnome, the Gnome
    Foundation, and Bitstream Inc., shall not be used in advertising or
    otherwise to promote the sale, use or other dealings in this Font
    Software without prior written authorization from the Gnome Foundation
    or Bitstream Inc., respectively. For further information, contact:
    fonts at gnome dot org.

This project makes use of the MinUnit C testing framework under the
following license agreement (from http://www.jera.com/techinfo/jtns/jtn002.html ):

    You may use the code in this tech note for any purpose, with the 
    understanding that it comes with NO WARRANTY.
