/*
 * cderror.h
 *
 * Copyright (C) 1994-1997, Thomas G. Lane.
 * Modified 2009 by Guido Vollbeding.
 * This file is part of the Independent JPEG Group's software.
 * For conditions of distribution and use, see the accompanying README.ijg
 * file.
 *
 * This file defines the error and message codes for the cjpeg/djpeg
 * applications.  These strings are not needed as part of the JPEG library
 * proper.
 * Edit this file to add new codes, or to translate the message strings to
 * some other language.
 */

/*
 * To define the enum list of message codes, include this file without
 * defining macro JMESSAGE.  To create a message string table, include it
 * again with a suitable JMESSAGE definition (see jerror.c for an example).
 */
#ifndef JMESSAGE
#ifndef CDERROR_H
#define CDERROR_H
/* First time through, define the enum list */
#define JMAKE_ENUM_LIST
#else
/* Repeated inclusions of this file are no-ops unless JMESSAGE is defined */
#define JMESSAGE(code, string)
#endif /* CDERROR_H */
#endif /* JMESSAGE */

#ifdef JMAKE_ENUM_LIST

typedef enum {

#define JMESSAGE(code, string) code,

#endif /* JMAKE_ENUM_LIST */

    MESSAGE(JMSG_FIRSTADDONCODE = 1000, NULL) /* Must be first entry! */

#ifdef BMP_SUPPORTED
    MESSAGE(JERR_BMP_BADCMAP, "Unsupported BMP colormap format") MESSAGE(
        JERR_BMP_BADDEPTH, "Only 8- and 24-bit BMP files are supported") MESSAGE(JERR_BMP_BADHEADER,
                                                                                  "Invalid BMP file: bad header length")
        MESSAGE(JERR_BMP_BADPLANES, "Invalid BMP file: biPlanes not equal to 1") MESSAGE(
            JERR_BMP_COLORSPACE, "BMP output must be grayscale or RGB") MESSAGE(JERR_BMP_COMPRESSED,
                                                                                 "Sorry, compressed BMPs not yet "
                                                                                 "supported") MESSAGE(JERR_BMP_EMPTY,
                                                                                                       "Empty BMP "
                                                                                                       "image")
            MESSAGE(JERR_BMP_NOT, "Not a BMP file - does not start with BM") MESSAGE(JTRC_BMP, "%ux%u 24-bit BMP "
                                                                                                 "image") MESSAGE(
                JTRC_BMP_MAPPED,
                "%ux%u 8-bit colormapped BMP image") MESSAGE(JTRC_BMP_OS2,
                                                              "%ux%u 24-bit OS2 BMP image") MESSAGE(JTRC_BMP_OS2_MAPPED,
                                                                                                     "%ux%u 8-bit "
                                                                                                     "colormapped OS2 "
                                                                                                     "BMP image")
#endif /* BMP_SUPPORTED */

#ifdef GIF_SUPPORTED
                MESSAGE(JERR_GIF_BUG, "GIF output got confused") MESSAGE(
                    JERR_GIF_CODESIZE, "Bogus GIF codesize %d") MESSAGE(JERR_GIF_COLORSPACE,
                                                                         "GIF output must be grayscale or RGB")
                    MESSAGE(JERR_GIF_IMAGENOTFOUND, "Too few images in GIF file") MESSAGE(
                        JERR_GIF_NOT,
                        "Not a GIF file") MESSAGE(JTRC_GIF, "%ux%ux%d GIF image") MESSAGE(JTRC_GIF_BADVERSION,
                                                                                            "Warning: unexpected GIF "
                                                                                            "version number '%c%c%c'")
                        MESSAGE(JTRC_GIF_EXTENSION, "Ignoring GIF extension block of type 0x%02x") MESSAGE(
                            JTRC_GIF_NONSQUARE,
                            "Caution: nonsquare pixels in input") MESSAGE(JWRN_GIF_BADDATA, "Corrupt data in GIF file")
                            MESSAGE(JWRN_GIF_CHAR, "Bogus char 0x%02x in GIF file, ignoring") MESSAGE(
                                JWRN_GIF_ENDCODE, "Premature end of GIF image") MESSAGE(JWRN_GIF_NOMOREDATA,
                                                                                         "Ran out of GIF bits")
#endif /* GIF_SUPPORTED */

#ifdef PPM_SUPPORTED
                                MESSAGE(JERR_PPM_COLORSPACE, "PPM output must be grayscale or RGB") MESSAGE(
                                    JERR_PPM_NONNUMERIC, "Nonnumeric data in PPM file")
                                    MESSAGE(JERR_PPM_TOOLARGE, "Integer value too large in PPM file") MESSAGE(
                                        JERR_PPM_NOT, "Not a PPM/PGM file") MESSAGE(JTRC_PGM, "%ux%u PGM image")
                                        MESSAGE(JTRC_PGM_TEXT, "%ux%u text PGM image") MESSAGE(
                                            JTRC_PPM, "%ux%u PPM image") MESSAGE(JTRC_PPM_TEXT, "%ux%u text PPM image")
#endif /* PPM_SUPPORTED */

#ifdef RLE_SUPPORTED
                                            MESSAGE(JERR_RLE_BADERROR, "Bogus error code from RLE library") MESSAGE(
                                                JERR_RLE_COLORSPACE,
                                                "RLE output must be grayscale or RGB") MESSAGE(JERR_RLE_DIMENSIONS,
                                                                                                "Image dimensions "
                                                                                                "(%ux%u) too large for "
                                                                                                "RLE")
                                                MESSAGE(JERR_RLE_EMPTY, "Empty RLE file") MESSAGE(
                                                    JERR_RLE_EOF,
                                                    "Premature EOF in RLE header") MESSAGE(JERR_RLE_MEM,
                                                                                            "Insufficient memory for "
                                                                                            "RLE header")
                                                    MESSAGE(JERR_RLE_NOT, "Not an RLE file") MESSAGE(
                                                        JERR_RLE_TOOMANYCHANNELS, "Cannot handle %d output channels "
                                                                                  "for RLE") MESSAGE(JERR_RLE_UNSUPPORTED,
                                                                                                      "Cannot handle "
                                                                                                      "this RLE setup")
                                                        MESSAGE(JTRC_RLE, "%ux%u full-color RLE file") MESSAGE(
                                                            JTRC_RLE_FULLMAP, "%ux%u full-color RLE file with map of "
                                                                              "length %d") MESSAGE(JTRC_RLE_GRAY,
                                                                                                    "%ux%u grayscale "
                                                                                                    "RLE file")
                                                            MESSAGE(JTRC_RLE_MAPGRAY, "%ux%u grayscale RLE file with "
                                                                                       "map of length %d") MESSAGE(
                                                                JTRC_RLE_MAPPED,
                                                                "%ux%u colormapped RLE file with map of length %d")
#endif /* RLE_SUPPORTED */

#ifdef TARGA_SUPPORTED
                                                                MESSAGE(JERR_TGA_BADCMAP, "Unsupported Targa colormap "
                                                                                           "format") MESSAGE(
                                                                    JERR_TGA_BADPARMS, "Invalid or unsupported Targa "
                                                                                       "file") MESSAGE(JERR_TGA_COLORSPACE,
                                                                                                        "Targa output "
                                                                                                        "must be "
                                                                                                        "grayscale or "
                                                                                                        "RGB")
                                                                    MESSAGE(JTRC_TGA, "%ux%u RGB Targa image")
                                                                        MESSAGE(JTRC_TGA_GRAY, "%ux%u grayscale Targa "
                                                                                                "image") MESSAGE(
                                                                            JTRC_TGA_MAPPED,
                                                                            "%ux%u colormapped Targa image")
#else
JMESSAGE(JERR_TGA_NOTCOMP, "Targa support was not compiled")
#endif /* TARGA_SUPPORTED */

                                                                            MESSAGE(JERR_BAD_CMAP_FILE,
                                                                                     "Color map file is invalid or of "
                                                                                     "unsupported format")
                                                                                MESSAGE(JERR_TOO_MANY_COLORS,
                                                                                         "Output file format cannot "
                                                                                         "handle %d colormap entries")
                                                                                    MESSAGE(JERR_UNGETC_FAILED,
                                                                                             "ungetc failed")
#ifdef TARGA_SUPPORTED
                                                                                        MESSAGE(
                                                                                            JERR_UNKNOWN_FORMAT,
                                                                                            "Unrecognized input file "
                                                                                            "format --- perhaps you "
                                                                                            "need -targa")
#else
JMESSAGE(JERR_UNKNOWN_FORMAT, "Unrecognized input file format")
#endif
                                                                                            MESSAGE(
                                                                                                JERR_UNSUPPORTED_FORMAT,
                                                                                                "Unsupported output "
                                                                                                "file format")

#ifdef JMAKE_ENUM_LIST

                                                                                                JMSG_LASTADDONCODE
} ADDON_MESSAGE_CODE;

#undef JMAKE_ENUM_LIST
#endif /* JMAKE_ENUM_LIST */

/* Zap JMESSAGE macro so that future re-inclusions do nothing by default */
#undef JMESSAGE
