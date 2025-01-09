// /* Copyright 1997, 1998, 1999 University Corporation for Atmospheric Research

// 	This software is distributed in the hope that it will be useful,
// 	but WITHOUT ANY WARRANTY; without even the implied warranty of
// 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

// 	Author: Louis H. Estey <lou@unavco.ucar.edu>
// 	Date:   July 1999
// 		part of the BINEX source code release, see also:

// 	http://www.unavco.ucar.edu/software/binex
// */
// #ifndef _BINEX_H
// #define _BINEX_H

// /* enum to specify type of BINEX data streams
// */
// enum {	BINEX_STREAM,
// 	no_of_BINEX_DATA_STREAMS };

// #define BINEX_F_stx_LE			(0xc2)
// #define BINEX_F_stx_BE			(0xe2)
// #define BINEX_FBstx_LE			(0xd2)
// #define BINEX_FBstx_BE			(0xf2)
// #define BINEX_FBetx_LE			(0xb4)
// #define BINEX_FBetx_BE			(0xb0)

// #define BINEX_MAX_REC_SIZE		BUFSIZ

// #define BINEX_SITE			(0x00000000)
// #define BINEX_GNSS_NAV			(0x00000001)
// #define BINEX_GNSS_DATA			(0x00000002)
// #define BINEX_SITE_DATA			(0x00000003)
// #define BINEX_GNSS_DATA_PROTO		(0x0000007f)
// #define BAD_BINEX_CHECKSUM		(0xfffffffc)	/* there will never be a BINEX record/subrecord/length with this
// value */ #define BAD_BINEX_READ			(0xfffffffd)	/* there will never be a BINEX
// record/subrecord/length with this value */ #define NORMAL_BINEX_EOF		(0xfffffffe)	/* there will never be a
// BINEX record/subrecord/length with this value */ #define NO_KNOWN_BINEX			(0xffffffff)	/* there
// will never be a BINEX record/subrecord/length with this value */

// #define BIT_0				(0x01)
// #define BIT_1				(0x02)
// #define BIT_2				(0x04)
// #define BIT_3				(0x08)
// #define BIT_4				(0x10)
// #define BIT_5				(0x20)
// #define BIT_6				(0x40)
// #define BIT_7				(0x80)

// #if KR_C
// /* K&R C function prototypes:
// */
// extern void		BINEX ();
// extern unsigned long	next_BINEX_record ();
// extern unsigned long	decompose_binex_00 ();
// extern unsigned long	decompose_binex_01 ();
// extern unsigned long	decompose_binex_02 ();
// extern unsigned long	decompose_binex_03 ();
// extern unsigned long	decompose_binex_7f ();
// extern void		binex_01_01_ephemeris ();
// extern unsigned char	binex_7f_00_constellation ();
// extern void		binex_7f_00_obs ();
// extern void		nav_binex_out ();
// extern void		obs_binex_out ();
// extern void		binex_observables_7f_00 ();
// extern void		binex_nav_message_01_01 ();

// #else
// /* ANSI C function prototypes:
// */
// extern void		BINEX (FILE_INFO *);
// extern unsigned long	next_BINEX_record (FILE_INFO *, unsigned char *);
// extern unsigned long	decompose_binex_00 (FILE_INFO *, unsigned char *);
// extern unsigned long	decompose_binex_01 (FILE_INFO *, unsigned char *);
// extern unsigned long	decompose_binex_02 (FILE_INFO *, unsigned char *);
// extern unsigned long	decompose_binex_03 (FILE_INFO *, unsigned char *);
// extern unsigned long	decompose_binex_7f (FILE_INFO *, unsigned char *);
// extern void		binex_01_01_ephemeris (unsigned char *);
// extern unsigned char	binex_7f_00_constellation (unsigned char *, bool);
// extern void		binex_7f_00_obs (unsigned char *);
// extern void		nav_binex_out (FILE *);
// extern void		obs_binex_out (FILE *);
// extern void		binex_observables_7f_00 (unsigned char *, unsigned long *);
// extern void		binex_nav_message_01_01 (unsigned char *, unsigned long *);
// #endif

// #endif