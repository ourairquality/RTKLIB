/*------------------------------------------------------------------------------
 * rtklib.h : RTKLIB constants, types and function prototypes
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DENAGLO   enable GLONASS
 *           -DENAGAL   enable Galileo
 *           -DENAQZS   enable QZSS
 *           -DENACMP   enable BeiDou
 *           -DENAIRN   enable IRNSS
 *           -DNFREQ=n  set number of obs codes/frequencies
 *           -DNEXOBS=n set number of extended obs codes
 *           -DMAXOBS=n set max number of obs data in an epoch
 *           -DWIN32    use WIN32 API
 *           -DWIN_DLL  generate library as Windows DLL
 *
 * Version : $Revision:$ $Date:$
 * History : 2007/01/13 1.0  RTKLIB ver.1.0.0
 *           2007/03/20 1.1  RTKLIB ver.1.1.0
 *           2008/07/15 1.2  RTKLIB ver.2.1.0
 *           2008/10/19 1.3  RTKLIB ver.2.1.1
 *           2009/01/31 1.4  RTKLIB ver.2.2.0
 *           2009/04/30 1.5  RTKLIB ver.2.2.1
 *           2009/07/30 1.6  RTKLIB ver.2.2.2
 *           2009/12/25 1.7  RTKLIB ver.2.3.0
 *           2010/07/29 1.8  RTKLIB ver.2.4.0
 *           2011/05/27 1.9  RTKLIB ver.2.4.1
 *           2013/03/28 1.10 RTKLIB ver.2.4.2
 *           2020/11/30 1.11 RTKLIB ver.2.4.3 b34
 *----------------------------------------------------------------------------*/
#ifndef RTKLIB_H
#define RTKLIB_H
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef WIN32
#include <windows.h>
#include <winsock2.h>
#else
#include <pthread.h>
#include <sys/select.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* For Windows DLL */
#else
#define EXPORT
#endif

/* Constants -----------------------------------------------------------------*/

#define VER_RTKLIB "demo5" /* Library version */

#define PATCH_LEVEL "b34i" /* Patch level */

#define COPYRIGHT_RTKLIB "Copyright (C) 2007-2020 T.Takasu\nAll rights reserved."

#define PI 3.1415926535897932  /* Pi */
#define D2R (PI / 180.0)       /* Deg to rad */
#define R2D (180.0 / PI)       /* Rad to deg */
#define CLIGHT 299792458.0     /* Speed of light (m/s) */
#define SC2RAD 3.1415926535898 /* Semi-circle to radian (IS-GPS) */
#define AU 149597870691.0      /* 1 AU (m) */
#define AS2R (D2R / 3600.0)    /* Arc sec to radian */

#define OMGE 7.2921151467E-5 /* Earth angular velocity (IS-GPS) (rad/s) */

#define RE_WGS84 6378137.0             /* Earth semimajor axis (WGS84) (m) */
#define FE_WGS84 (1.0 / 298.257223563) /* Earth flattening (WGS84) */

#define HION 350000.0 /* Ionosphere height (m) */

#define MAXFREQ 6 /* Max NFREQ */

#define FREQL1 1.57542E9      /* L1/E1  frequency (Hz) */
#define FREQL2 1.22760E9      /* L2     frequency (Hz) */
#define FREQE5b 1.20714E9     /* E5b    frequency (Hz) */
#define FREQL5 1.17645E9      /* L5/E5a/B2a frequency (Hz) */
#define FREQL6 1.27875E9      /* E6/L6 frequency (Hz) */
#define FREQE5ab 1.191795E9   /* E5a+b  frequency (Hz) */
#define FREQs 2.492028E9      /* S      frequency (Hz) */
#define FREQ1_GLO 1.60200E9   /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO 0.56250E6   /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO 1.24600E9   /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO 0.43750E6   /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO 1.202025E9  /* GLONASS G3 frequency (Hz) */
#define FREQ1a_GLO 1.600995E9 /* GLONASS G1a frequency (Hz) */
#define FREQ2a_GLO 1.248060E9 /* GLONASS G2a frequency (Hz) */
#define FREQ1_CMP 1.561098E9  /* BDS B1I     frequency (Hz) */
#define FREQ2_CMP 1.20714E9   /* BDS B2I/B2b frequency (Hz) */
#define FREQ3_CMP 1.26852E9   /* BDS B3      frequency (Hz) */

#define EFACT_GPS 1.0 /* Error factor: GPS */
#define EFACT_GLO 1.5 /* Error factor: GLONASS */
#define EFACT_GAL 1.0 /* Error factor: Galileo */
#define EFACT_QZS 1.0 /* Error factor: QZSS */
#define EFACT_CMP 1.0 /* Error factor: BeiDou */
#define EFACT_IRN 1.5 /* Error factor: IRNSS */
#define EFACT_SBS 3.0 /* Error factor: SBAS */

#define SYS_NONE 0x00 /* Navigation system: none */
#define SYS_GPS 0x01  /* Navigation system: GPS */
#define SYS_SBS 0x02  /* Navigation system: SBAS */
#define SYS_GLO 0x04  /* Navigation system: GLONASS */
#define SYS_GAL 0x08  /* Navigation system: Galileo */
#define SYS_QZS 0x10  /* Navigation system: QZSS */
#define SYS_CMP 0x20  /* Navigation system: BeiDou */
#define SYS_IRN 0x40  /* Navigation system: IRNS */
#define SYS_LEO 0x80  /* Navigation system: LEO */
#define SYS_ALL 0xFF  /* Navigation system: all */

#define TSYS_GPS 0 /* Time system: GPS time */
#define TSYS_UTC 1 /* Time system: UTC */
#define TSYS_GLO 2 /* Time system: GLONASS time */
#define TSYS_GAL 3 /* Time system: Galileo time */
#define TSYS_QZS 4 /* Time system: QZSS time */
#define TSYS_CMP 5 /* Time system: BeiDou time */
#define TSYS_IRN 6 /* Time system: IRNSS time */

#ifndef NFREQ
#define NFREQ 3 /* Number of carrier frequencies */
#endif
#define NFREQGLO 2 /* Number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS 0 /* Number of extended obs codes */
#endif

#define SNR_UNIT 0.001 /* SNR unit (dBHz) */

#define MINPRNGPS 1                         /* Min satellite PRN number of GPS */
#define MAXPRNGPS 32                        /* Max satellite PRN number of GPS */
#define NSATGPS (MAXPRNGPS - MINPRNGPS + 1) /* Number of GPS satellites */
#define NSYSGPS 1

#ifdef ENAGLO
#define MINPRNGLO 1                         /* Min satellite slot number of GLONASS */
#define MAXPRNGLO 27                        /* Max satellite slot number of GLONASS */
#define NSATGLO (MAXPRNGLO - MINPRNGLO + 1) /* Number of GLONASS satellites */
#define NSYSGLO 1
#else
#define MINPRNGLO 0
#define MAXPRNGLO 0
#define NSATGLO 0
#define NSYSGLO 0
#endif
#ifdef ENAGAL
#define MINPRNGAL 1                         /* Min satellite PRN number of Galileo */
#define MAXPRNGAL 36                        /* Max satellite PRN number of Galileo */
#define NSATGAL (MAXPRNGAL - MINPRNGAL + 1) /* Number of Galileo satellites */
#define NSYSGAL 1
#else
#define MINPRNGAL 0
#define MAXPRNGAL 0
#define NSATGAL 0
#define NSYSGAL 0
#endif
#ifdef ENAQZS
#define MINPRNQZS 193                       /* Min satellite PRN number of QZSS */
#define MAXPRNQZS 202                       /* Max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                     /* Min satellite PRN number of QZSS L1S */
#define MAXPRNQZS_S 191                     /* Max satellite PRN number of QZSS L1S */
#define NSATQZS (MAXPRNQZS - MINPRNQZS + 1) /* Number of QZSS satellites */
#define NSYSQZS 1
#else
#define MINPRNQZS 0
#define MAXPRNQZS 0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS 0
#define NSYSQZS 0
#endif
#ifdef ENACMP
#define MINPRNCMP 1                         /* Min satellite sat number of BeiDou */
#define MAXPRNCMP 60                        /* Max satellite sat number of BeiDou */
#define NSATCMP (MAXPRNCMP - MINPRNCMP + 1) /* Number of BeiDou satellites */
#define NSYSCMP 1
#else
#define MINPRNCMP 0
#define MAXPRNCMP 0
#define NSATCMP 0
#define NSYSCMP 0
#endif
#ifdef ENAIRN
#define MINPRNIRN 1                         /* Min satellite sat number of IRNSS */
#define MAXPRNIRN 14                        /* Max satellite sat number of IRNSS */
#define NSATIRN (MAXPRNIRN - MINPRNIRN + 1) /* Number of IRNSS satellites */
#define NSYSIRN 1
#else
#define MINPRNIRN 0
#define MAXPRNIRN 0
#define NSATIRN 0
#define NSYSIRN 0
#endif
#ifdef ENALEO
#define MINPRNLEO 1                         /* Min satellite sat number of LEO */
#define MAXPRNLEO 10                        /* Max satellite sat number of LEO */
#define NSATLEO (MAXPRNLEO - MINPRNLEO + 1) /* Number of LEO satellites */
#define NSYSLEO 1
#else
#define MINPRNLEO 0
#define MAXPRNLEO 0
#define NSATLEO 0
#define NSYSLEO 0
#endif
#define NSYS \
  (NSYSGPS + NSYSGLO + NSYSGAL + NSYSQZS + NSYSCMP + NSYSIRN + NSYSLEO) /* Number of systems */

#define MINPRNSBS 120                       /* Min satellite PRN number of SBAS */
#define MAXPRNSBS 158                       /* Max satellite PRN number of SBAS */
#define NSATSBS (MAXPRNSBS - MINPRNSBS + 1) /* Number of SBAS satellites */

#define MAXSAT (NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATSBS + NSATLEO)
/* Max satellite number (1 to MAXSAT) */
#define MAXSTA 255

#ifndef MAXOBS
#define MAXOBS 96 /* Max number of obs in an epoch */
#endif
#define MAXRCV 64     /* Max receiver number (1 to MAXRCV) */
#define MAXOBSTYPE 64 /* Max number of obs type in RINEX */
#ifdef OBS_100HZ
#define DTTOL 0.005 /* Tolerance of time difference (s) */
#else
#define DTTOL 0.025 /* Tolerance of time difference (s) */
#endif
#define MAXDTOE 7200.0      /* Max time difference to GPS Toe (s) */
#define MAXDTOE_QZS 7200.0  /* Max time difference to QZSS Toe (s) */
#define MAXDTOE_GAL 14400.0 /* Max time difference to Galileo Toe (s) */
#define MAXDTOE_CMP 21600.0 /* Max time difference to BeiDou Toe (s) */
#define MAXDTOE_GLO 1800.0  /* Max time difference to GLONASS Toe (s) */
#define MAXDTOE_IRN 7200.0  /* Max time difference to IRNSS Toe (s) */
#define MAXDTOE_SBS 360.0   /* Max time difference to SBAS Toe (s) */
#define MAXDTOE_S 86400.0   /* Max time difference to ephem toe (s) for other */
#define MAXGDOP 300.0       /* Max GDOP */

#define INT_SWAP_TRAC 86400.0 /* Swap interval of trace file (s) */
#define INT_SWAP_STAT 86400.0 /* Swap interval of solution status file (s) */

#define MAXEXFILE 1024        /* Max number of expanded files */
#define MAXSBSAGEF 30.0       /* Max age of SBAS fast correction (s) */
#define MAXSBSAGEL 1800.0     /* Max age of SBAS long term corr (s) */
#define MAXSBSURA 8           /* Max URA of SBAS satellite */
#define MAXBAND 10            /* Max SBAS band of IGP */
#define MAXNIGP 201           /* Max number of IGP in SBAS band */
#define MAXNGEO 4             /* Max number of GEO satellites */
#define MAXCOMMENT 100        /* Max number of RINEX comments */
#define MAXSTRPATH 1024       /* Max length of stream path */
#define MAXSTRMSG 1024        /* Max length of stream message */
#define MAXSTRRTK 8           /* Max number of stream in RTK server */
#define MAXSBSMSG 32          /* Max number of SBAS msg in RTK server */
#define MAXSOLMSG 8191        /* Max length of solution message */
#define MAXRAWLEN 16384       /* Max length of receiver raw message */
#define MAXERRMSG 4096        /* Max length of error/warning message */
#define MAXANT 64             /* Max length of station name/antenna type */
#define MAXSOLBUF 256         /* Max number of solution buffer */
#define MAXOBSBUF 128         /* Max number of observation data buffer */
#define MAXNRPOS 16           /* Max number of reference positions */
#define MAXLEAPS 64           /* Max number of leap seconds table */
#define MAXGISLAYER 32        /* Max number of GIS data layers */
#define MAXRCVCMD 4096        /* Max length of receiver commands */
#define MAX_CODE_BIASES 3     /* Max # of different code biases per freq */
#define MAX_CODE_BIAS_FREQS 2 /* Max # of freqs supported for code biases  */

#define FNSIZE 1024 /* Size for file path names */

#define RNX2VER 2.10 /* RINEX ver.2 default output version */
#define RNX3VER 3.00 /* RINEX ver.3 default output version */

#define OBSTYPE_PR 0x01  /* Observation type: pseudorange */
#define OBSTYPE_CP 0x02  /* Observation type: carrier-phase */
#define OBSTYPE_DOP 0x04 /* Observation type: doppler-freq */
#define OBSTYPE_SNR 0x08 /* Observation type: SNR */
#define OBSTYPE_ALL 0xFF /* Observation type: all */

#define FREQTYPE_L1 0x01  /* Frequency type: L1/E1/B1 */
#define FREQTYPE_L2 0x02  /* Frequency type: L2/E5b/B2 */
#define FREQTYPE_L3 0x04  /* Frequency type: L5/E5a/L3 */
#define FREQTYPE_L4 0x08  /* Frequency type: L6/E6/B3 */
#define FREQTYPE_L5 0x10  /* Frequency type: E5ab */
#define FREQTYPE_ALL 0xFF /* Frequency type: all */

#define CODE_NONE 0 /* Obs code: none or unknown */
#define CODE_L1C 1  /* Obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P 2  /* Obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
#define CODE_L1W 3  /* Obs code: L1 Z-track (GPS) */
#define CODE_L1Y 4  /* Obs code: L1Y        (GPS) */
#define CODE_L1M 5  /* Obs code: L1M        (GPS) */
#define CODE_L1N 6  /* Obs code: L1codeless,B1codeless (GPS,BDS) */
#define CODE_L1S 7  /* Obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L 8  /* Obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E 9  /* (not used) */
#define CODE_L1A 10 /* Obs code: E1A,B1A    (GAL,BDS) */
#define CODE_L1B 11 /* Obs code: E1B        (GAL) */
#define CODE_L1X 12 /* Obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) */
#define CODE_L1Z 13 /* Obs code: E1A+B+C,L1S (GAL,QZS) */
#define CODE_L2C 14 /* Obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D 15 /* Obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S 16 /* Obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L 17 /* Obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X 18 /* Obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) */
#define CODE_L2P 19 /* Obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W 20 /* Obs code: L2 Z-track (GPS) */
#define CODE_L2Y 21 /* Obs code: L2Y        (GPS) */
#define CODE_L2M 22 /* Obs code: L2M        (GPS) */
#define CODE_L2N 23 /* Obs code: L2codeless (GPS) */
#define CODE_L5I 24 /* Obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) */
#define CODE_L5Q 25 /* Obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) */
#define CODE_L5X 26 /* Obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) */
#define CODE_L7I 27 /* Obs code: E5bI,B2bI  (GAL,BDS) */
#define CODE_L7Q 28 /* Obs code: E5bQ,B2bQ  (GAL,BDS) */
#define CODE_L7X 29 /* Obs code: E5bI+Q,B2bI+Q (GAL,BDS) */
#define CODE_L6A 30 /* Obs code: E6A,B3A    (GAL,BDS) */
#define CODE_L6B 31 /* Obs code: E6B        (GAL) */
#define CODE_L6C 32 /* Obs code: E6C        (GAL) */
#define CODE_L6X 33 /* Obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) */
#define CODE_L6Z 34 /* Obs code: E6A+B+C,L6D+E (GAL,QZS) */
#define CODE_L6S 35 /* Obs code: L6S        (QZS) */
#define CODE_L6L 36 /* Obs code: L6L        (QZS) */
#define CODE_L8I 37 /* Obs code: E5abI      (GAL) */
#define CODE_L8Q 38 /* Obs code: E5abQ      (GAL) */
#define CODE_L8X 39 /* Obs code: E5abI+Q,B2abD+P (GAL,BDS) */
#define CODE_L2I 40 /* Obs code: B1_2I      (BDS) */
#define CODE_L2Q 41 /* Obs code: B1_2Q      (BDS) */
#define CODE_L6I 42 /* Obs code: B3I        (BDS) */
#define CODE_L6Q 43 /* Obs code: B3Q        (BDS) */
#define CODE_L3I 44 /* Obs code: G3I        (GLO) */
#define CODE_L3Q 45 /* Obs code: G3Q        (GLO) */
#define CODE_L3X 46 /* Obs code: G3I+Q      (GLO) */
#define CODE_L1I 47 /* Obs code: B1I        (BDS) (obsolete) */
#define CODE_L1Q 48 /* Obs code: B1Q        (BDS) (obsolete) */
#define CODE_L5A 49 /* Obs code: L5A SPS    (IRN) */
#define CODE_L5B 50 /* Obs code: L5B RS(D)  (IRN) */
#define CODE_L5C 51 /* Obs code: L5C RS(P)  (IRN) */
#define CODE_L9A 52 /* Obs code: SA SPS     (IRN) */
#define CODE_L9B 53 /* Obs code: SB RS(D)   (IRN) */
#define CODE_L9C 54 /* Obs code: SC RS(P)   (IRN) */
#define CODE_L9X 55 /* Obs code: SB+C       (IRN) */
#define CODE_L1D 56 /* Obs code: B1D        (BDS) */
#define CODE_L5D 57 /* Obs code: L5D(L5S),B2aD (QZS,BDS) */
#define CODE_L5P 58 /* Obs code: L5P(L5S),B2aP (QZS,BDS) */
#define CODE_L5Z 59 /* Obs code: L5D+P(L5S) (QZS) */
#define CODE_L6E 60 /* Obs code: L6E        (QZS) */
#define CODE_L7D 61 /* Obs code: B2bD       (BDS) */
#define CODE_L7P 62 /* Obs code: B2bP       (BDS) */
#define CODE_L7Z 63 /* Obs code: B2bD+P     (BDS) */
#define CODE_L8D 64 /* Obs code: B2abD      (BDS) */
#define CODE_L8P 65 /* Obs code: B2abP      (BDS) */
#define CODE_L4A 66 /* Obs code: G1aL1OCd   (GLO) */
#define CODE_L4B 67 /* Obs code: G1aL1OCd   (GLO) */
#define CODE_L4X 68 /* Obs code: G1al1OCd+p (GLO) */
#define MAXCODE 68  /* Max number of obs code */

#define PMODE_SINGLE 0       /* Positioning mode: single */
#define PMODE_DGPS 1         /* Positioning mode: DGPS/DGNSS */
#define PMODE_KINEMA 2       /* Positioning mode: kinematic */
#define PMODE_STATIC 3       /* Positioning mode: static */
#define PMODE_STATIC_START 4 /* Positioning mode: static */
#define PMODE_MOVEB 5        /* Positioning mode: moving-base */
#define PMODE_FIXED 6        /* Positioning mode: fixed */
#define PMODE_PPP_KINEMA 7   /* Positioning mode: PPP-kinemaric */
#define PMODE_PPP_STATIC 8   /* Positioning mode: PPP-static */
#define PMODE_PPP_FIXED 9    /* Positioning mode: PPP-fixed */

#define SOLF_LLH 0  /* Solution format: lat/lon/height */
#define SOLF_XYZ 1  /* Solution format: x/y/z-ecef */
#define SOLF_ENU 2  /* Solution format: e/n/u-baseline */
#define SOLF_NMEA 3 /* Solution format: NMEA-183 */
#define SOLF_STAT 4 /* Solution format: solution status */
#define SOLF_GSIF 5 /* Solution format: GSI F1/F2 */

#define SOLQ_NONE 0   /* Solution status: no solution */
#define SOLQ_FIX 1    /* Solution status: fix */
#define SOLQ_FLOAT 2  /* Solution status: float */
#define SOLQ_SBAS 3   /* Solution status: SBAS */
#define SOLQ_DGPS 4   /* Solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5 /* Solution status: single */
#define SOLQ_PPP 6    /* Solution status: PPP */
#define SOLQ_DR 7     /* Solution status: dead reckoning */
#define MAXSOLQ 7     /* Max number of solution status */

#define SOLTYPE_FORWARD 0          /* Solution type: forward */
#define SOLTYPE_BACKWARD 1         /* Solution type: backward */
#define SOLTYPE_COMBINED 2         /* Solution type: combined */
#define SOLTYPE_COMBINED_NORESET 3 /* Solution type: combined no phase reset*/
#define SOLMODE_SINGLE_DIR 0       /* Single direction solution */
#define SOLMODE_COMBINED 1         /* Combined solution */

#define TIMES_GPST 0 /* Time system: GPS time */
#define TIMES_UTC 1  /* Time system: UTC */
#define TIMES_JST 2  /* Time system: jst */

#define IONOOPT_OFF 0  /* Ionosphere option: correction off */
#define IONOOPT_BRDC 1 /* Ionosphere option: broadcast model */
#define IONOOPT_SBAS 2 /* Ionosphere option: SBAS model */
#define IONOOPT_IFLC 3 /* Ionosphere option: L1/L2 or L1/L5 iono-free LC */
#define IONOOPT_EST 4  /* Ionosphere option: estimation */
#define IONOOPT_TEC 5  /* Ionosphere option: IONEX TEC model */
#define IONOOPT_QZS 6  /* Ionosphere option: QZSS broadcast model */
#define IONOOPT_STEC 8 /* Ionosphere option: SLANT TEC model */

#define TROPOPT_OFF 0  /* Troposphere option: correction off */
#define TROPOPT_SAAS 1 /* Troposphere option: Saastamoinen model */
#define TROPOPT_SBAS 2 /* Troposphere option: SBAS model */
#define TROPOPT_EST 3  /* Troposphere option: ZTD estimation */
#define TROPOPT_ESTG 4 /* Troposphere option: ZTD+grad estimation */

#define EPHOPT_BRDC 0   /* Ephemeris option: broadcast ephemeris */
#define EPHOPT_PREC 1   /* Ephemeris option: precise ephemeris */
#define EPHOPT_SBAS 2   /* Ephemeris option: broadcast + SBAS */
#define EPHOPT_SSRAPC 3 /* Ephemeris option: broadcast + SSR_APC */
#define EPHOPT_SSRCOM 4 /* Ephemeris option: broadcast + SSR_COM */

#define ARMODE_OFF 0     /* AR mode: off */
#define ARMODE_CONT 1    /* AR mode: continuous */
#define ARMODE_INST 2    /* AR mode: instantaneous */
#define ARMODE_FIXHOLD 3 /* AR mode: fix and hold */

#define GLO_ARMODE_OFF 0     /* GLO AR mode: off */
#define GLO_ARMODE_ON 1      /* GLO AR mode: on */
#define GLO_ARMODE_AUTOCAL 2 /* GLO AR mode: autocal */
#define GLO_ARMODE_FIXHOLD 3 /* GLO AR mode: fix and hold */

#define SBSOPT_LCORR 1 /* SBAS option: long term correction */
#define SBSOPT_FCORR 2 /* SBAS option: fast correction */
#define SBSOPT_ICORR 4 /* SBAS option: ionosphere correction */
#define SBSOPT_RANGE 8 /* SBAS option: ranging */

#define POSOPT_POS 0    /* Pos option: LLH/XYZ */
#define POSOPT_SINGLE 1 /* Pos option: average of single pos */
#define POSOPT_FILE 2   /* Pos option: read from pos file */
#define POSOPT_RINEX 3  /* Pos option: RINEX header pos */
#define POSOPT_RTCM 4   /* Pos option: RTCM/raw station pos */

#define STR_NONE 0     /* Stream type: none */
#define STR_SERIAL 1   /* Stream type: serial */
#define STR_FILE 2     /* Stream type: file */
#define STR_TCPSVR 3   /* Stream type: TCP server */
#define STR_TCPCLI 4   /* Stream type: TCP client */
#define STR_NTRIPSVR 5 /* Stream type: NTRIP server */
#define STR_NTRIPCLI 6 /* Stream type: NTRIP client */
#define STR_FTP 7      /* Stream type: FTP */
#define STR_HTTP 8     /* Stream type: HTTP */
#define STR_NTRIPCAS 9 /* Stream type: NTRIP caster */
#define STR_UDPSVR 10  /* Stream type: UDP server */
#define STR_UDPCLI 11  /* Stream type: UDP server */
#define STR_MEMBUF 12  /* Stream type: memory buffer */

#define STRFMT_RTCM2 0   /* Stream format: RTCM 2 */
#define STRFMT_RTCM3 1   /* Stream format: RTCM 3 */
#define STRFMT_OEM4 2    /* Stream format: NovAtel OEMV/4 */
#define STRFMT_CNAV 3    /* Stream format: ComNav */
#define STRFMT_UBX 4     /* Stream format: u-blox LEA-*T */
#define STRFMT_SBP 5     /* Stream format: Swift Navigation SBP */
#define STRFMT_CRES 6    /* Stream format: Hemisphere */
#define STRFMT_STQ 7     /* Stream format: SkyTraq S1315F */
#define STRFMT_JAVAD 8   /* Stream format: JAVAD GRIL/GREIS */
#define STRFMT_NVS 9     /* Stream format: NVS NVC08C */
#define STRFMT_BINEX 10  /* Stream format: BINEX */
#define STRFMT_RT17 11   /* Stream format: Trimble RT17 */
#define STRFMT_SEPT 12   /* Stream format: Septentrio */
#define STRFMT_TERSUS 13 /* Stream format: TERSUS */
#define STRFMT_RINEX 14  /* Stream format: RINEX */
#define STRFMT_SP3 15    /* Stream format: SP3 */
#define STRFMT_RNXCLK 16 /* Stream format: RINEX CLK */
#define STRFMT_SBAS 17   /* Stream format: SBAS messages */
#define STRFMT_NMEA 18   /* Stream format: NMEA 0183 */
#define MAXRCVFMT 13     /* Max number of receiver format */

#define STR_MODE_R 0x1  /* Stream mode: read */
#define STR_MODE_W 0x2  /* Stream mode: write */
#define STR_MODE_RW 0x3 /* Stream mode: read/write */

#define GEOID_EMBEDDED 0    /* Geoid model: embedded geoid */
#define GEOID_EGM96_M150 1  /* Geoid model: EGM96 15x15" */
#define GEOID_EGM2008_M25 2 /* Geoid model: EGM2008 2.5x2.5" */
#define GEOID_EGM2008_M10 3 /* Geoid model: EGM2008 1.0x1.0" */
#define GEOID_GSI2000_M15 4 /* Geoid model: GSI geoid 2000 1.0x1.5" */
#define GEOID_RAF09 5       /* Geoid model: IGN RAF09 for France 1.5"x2" */

#define COMMENTH "%"                   /* Comment line indicator for solution */
#define MSG_DISCONN "$_DISCONNECT\r\n" /* Disconnect message */

#define DLOPT_FORCE 0x01   /* Download option: force download existing */
#define DLOPT_KEEPCMP 0x02 /* Download option: keep compressed file */
#define DLOPT_HOLDERR 0x04 /* Download option: hold on error file */
#define DLOPT_HOLDLST 0x08 /* Download option: hold on listing file */

#define LLI_SLIP 0x01   /* LLI: cycle-slip */
#define LLI_HALFC 0x02  /* LLI: half-cycle not resolved */
#define LLI_BOCTRK 0x04 /* LLI: boc tracking of mboc signal */
#define LLI_HALFA 0x40  /* LLI: half-cycle added */
#define LLI_HALFS 0x80  /* LLI: half-cycle subtracted */

#define P2_5 0.03125                /* 2^-5 */
#define P2_6 0.015625               /* 2^-6 */
#define P2_11 4.882812500000000E-04 /* 2^-11 */
#define P2_15 3.051757812500000E-05 /* 2^-15 */
#define P2_17 7.629394531250000E-06 /* 2^-17 */
#define P2_19 1.907348632812500E-06 /* 2^-19 */
#define P2_20 9.536743164062500E-07 /* 2^-20 */
#define P2_21 4.768371582031250E-07 /* 2^-21 */
#define P2_23 1.192092895507810E-07 /* 2^-23 */
#define P2_24 5.960464477539063E-08 /* 2^-24 */
#define P2_27 7.450580596923828E-09 /* 2^-27 */
#define P2_29 1.862645149230957E-09 /* 2^-29 */
#define P2_30 9.313225746154785E-10 /* 2^-30 */
#define P2_31 4.656612873077393E-10 /* 2^-31 */
#define P2_32 2.328306436538696E-10 /* 2^-32 */
#define P2_33 1.164153218269348E-10 /* 2^-33 */
#define P2_35 2.910383045673370E-11 /* 2^-35 */
#define P2_38 3.637978807091710E-12 /* 2^-38 */
#define P2_39 1.818989403545856E-12 /* 2^-39 */
#define P2_40 9.094947017729280E-13 /* 2^-40 */
#define P2_43 1.136868377216160E-13 /* 2^-43 */
#define P2_48 3.552713678800501E-15 /* 2^-48 */
#define P2_50 8.881784197001252E-16 /* 2^-50 */
#define P2_55 2.775557561562891E-17 /* 2^-55 */

#ifdef WIN32
#define rtklib_thread_t HANDLE
#define rtklib_lock_t CRITICAL_SECTION
#define rtklib_initlock(f) InitializeCriticalSection(f)
#define rtklib_lock(f) EnterCriticalSection(f)
#define rtklib_unlock(f) LeaveCriticalSection(f)
#define RTKLIB_FILEPATHSEP '\\'
#else
#define rtklib_thread_t pthread_t
#define rtklib_lock_t pthread_mutex_t
#define rtklib_initlock(f) pthread_mutex_init(f, NULL)
#define rtklib_lock(f) pthread_mutex_lock(f)
#define rtklib_unlock(f) pthread_mutex_unlock(f)
#define RTKLIB_FILEPATHSEP '/'
#endif

/* Type definitions ----------------------------------------------------------*/

typedef struct { /* Time struct */
  time_t time;   /* Time (s) expressed by standard time_t */
  double sec;    /* Fraction of second under 1 s */
} gtime_t;

typedef struct {                /* Observation data record */
  gtime_t time;                 /* Receiver sampling time (GPST) */
  uint8_t sat, rcv;             /* Satellite/receiver number */
  uint16_t SNR[NFREQ + NEXOBS]; /* Signal strength (0.001 dBHz) */
  uint8_t LLI[NFREQ + NEXOBS];  /* Loss of lock indicator */
  uint8_t code[NFREQ + NEXOBS]; /* Code indicator (CODE_???) */
  double L[NFREQ + NEXOBS];     /* Observation data carrier-phase (cycle) */
  double P[NFREQ + NEXOBS];     /* Observation data pseudorange (m) */
  float D[NFREQ + NEXOBS];      /* Observation data doppler frequency (Hz) */

  int timevalid;                /* Time is valid (Valid GNSS fix) for time mark */
  gtime_t eventime;             /* Time of event (GPST) */
  uint8_t Lstd[NFREQ + NEXOBS]; /* Stdev of carrier phase (0.004 cycles)  */
  uint8_t Pstd[NFREQ + NEXOBS]; /* Stdev of pseudorange (0.01*2^(n+5) meters) */
  uint8_t freq;                 /* GLONASS frequency channel (0-13) */

} obsd_t;

typedef struct { /* Observation data */
  int n, nmax;   /* Number of observation data/allocated */
  int flag;      /* Epoch flag (0:ok,1:power failure,>1:event flag) */
  int rcvcount;  /* Count of rcv event */
  int tmcount;   /* Time mark count */
  obsd_t *data;  /* Observation data records */
} obs_t;

typedef struct {   /* Earth rotation parameter data type */
  double mjd;      /* Mjd (days) */
  double xp, yp;   /* Pole offset (rad) */
  double xpr, ypr; /* Pole offset rate (rad/day) */
  double ut1_utc;  /* Ut1-utc (s) */
  double lod;      /* Length of day (s/day) */
} erpd_t;

typedef struct { /* Earth rotation parameter type */
  int n, nmax;   /* Number and max number of data */
  erpd_t *data;  /* Earth rotation parameter data */
} erp_t;

typedef struct {         /* Antenna parameter type */
  int sat;               /* Satellite number (0:receiver) */
  char type[MAXANT];     /* Antenna type */
  char code[MAXANT];     /* Serial number or satellite code */
  gtime_t ts, te;        /* Valid time start and end */
  double off[NFREQ][3];  /* Phase center offset e/n/u or x/y/z (m) */
  double var[NFREQ][19]; /* Phase center variation (m) */
                         /* el=90,85,...,0 or nadir=0,1,2,3,... (deg) */
} pcv_t;

typedef struct { /* Antenna parameters type */
  int n, nmax;   /* Number of data/allocated */
  pcv_t *pcv;    /* Antenna parameters data */
} pcvs_t;

typedef struct { /* Almanac type */
  int sat;       /* Satellite number */
  int svh;       /* Sv health (0:ok) */
  int svconf;    /* As and sv config */
  int week;      /* GPS/QZS: GPS week, GAL: Galileo week */
  gtime_t toa;   /* Toa */
                 /* SV orbit parameters */
  double A, e, i0, OMG0, omg, M0, OMGd;
  double toas;   /* Toa (s) in week */
  double f0, f1; /* SV clock parameters (af0,af1) */
} alm_t;

typedef struct {         /* GPS/QZS/GAL broadcast ephemeris type */
  int sat;               /* Satellite number */
  int iode, iodc;        /* IODE,IODC */
  int sva;               /* SV accuracy (URA index) */
  int svh;               /* SV health (0:ok) */
  int week;              /* GPS/QZS: GPS week, GAL: Galileo week */
  int code;              /* GPS/QZS: code on L2 */
                         /* GAL: data source defined as RINEX 3.03 */
                         /* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
  int flag;              /* GPS/QZS: L2 P data flag */
                         /* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
  gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
                         /* SV orbit parameters */
  double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
  double crc, crs, cuc, cus, cic, cis;
  double toes;       /* Toe (s) in week */
  double fit;        /* Fit interval (h) */
  double f0, f1, f2; /* SV clock parameters (af0,af1,af2) */
  double tgd[6];     /* Group delay parameters */
                     /* GPS/QZS:tgd[0]=TGD */
                     /* GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b */
                     /* CMP:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp */
                     /*     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad */
  double Adot, ndot; /* Adot,ndot for CNAV */
} eph_t;

typedef struct {     /* GLONASS broadcast ephemeris type */
  int sat;           /* Satellite number */
  int iode;          /* IODE (0-6 bit of tb field) */
  int frq;           /* Satellite frequency number */
  int svh, sva, age; /* Satellite health, accuracy, age of operation */
  gtime_t toe;       /* Epoch of ephemerides (GPST) */
  gtime_t tof;       /* Message frame time (GPST) */
  double pos[3];     /* Satellite position (ECEF) (m) */
  double vel[3];     /* Satellite velocity (ECEF) (m/s) */
  double acc[3];     /* Satellite acceleration (ECEF) (m/s^2) */
  double taun, gamn; /* SV clock bias (s)/relative freq bias */
  double dtaun;      /* Delay between L1 and L2 (s) */
} geph_t;

typedef struct {         /* Precise ephemeris type */
  gtime_t time;          /* Time (GPST) */
  int index;             /* Ephemeris index for multiple files */
  double pos[MAXSAT][4]; /* Satellite position/clock (ECEF) (m|s) */
  float std[MAXSAT][4];  /* Satellite position/clock std (m|s) */
  double vel[MAXSAT][4]; /* Satellite velocity/clk-rate (m/s|s/s) */
  float vst[MAXSAT][4];  /* Satellite velocity/clk-rate std (m/s|s/s) */
  float cov[MAXSAT][3];  /* Satellite position covariance (m^2) */
  float vco[MAXSAT][3];  /* Satellite velocity covariance (m^2) */
} peph_t;

typedef struct {         /* Precise clock type */
  gtime_t time;          /* Time (GPST) */
  int index;             /* Clock index for multiple files */
  double clk[MAXSAT][1]; /* Satellite clock (s) */
  float std[MAXSAT][1];  /* Satellite clock std (s) */
} pclk_t;

typedef struct {   /* SBAS ephemeris type */
  int sat;         /* Satellite number */
  gtime_t t0;      /* Reference epoch time (GPST) */
  gtime_t tof;     /* Time of message frame (GPST) */
  int sva;         /* SV accuracy (URA index) */
  int svh;         /* SV health (0:ok) */
  double pos[3];   /* Satellite position (m) (ECEF) */
  double vel[3];   /* Satellite velocity (m/s) (ECEF) */
  double acc[3];   /* Satellite acceleration (m/s^2) (ECEF) */
  double af0, af1; /* Satellite clock-offset/drift (s,s/s) */
} seph_t;

typedef struct {  /* NORAD TLE data type */
  char name[32];  /* Common name */
  char alias[32]; /* Alias name */
  char satno[16]; /* Satellite catalog number */
  char satclass;  /* Classification */
  char desig[16]; /* International designator */
  gtime_t epoch;  /* Element set epoch (UTC) */
  double ndot;    /* 1st derivative of mean motion */
  double nddot;   /* 2st derivative of mean motion */
  double bstar;   /* B* drag term */
  int etype;      /* Element set type */
  int eleno;      /* Element number */
  double inc;     /* Orbit inclination (deg) */
  double OMG;     /* Right ascension of ascending node (deg) */
  double ecc;     /* Eccentricity */
  double omg;     /* Argument of perigee (deg) */
  double M;       /* Mean anomaly (deg) */
  double n;       /* Mean motion (rev/day) */
  int rev;        /* Revolution number at epoch */
} tled_t;

typedef struct { /* NORAD TLE (two line element) type */
  int n, nmax;   /* Number/max number of two line element data */
  tled_t *data;  /* NORAD TLE data */
} tle_t;

typedef struct {  /* TEC grid type */
  gtime_t time;   /* Epoch time (GPST) */
  int ndata[3];   /* TEC grid data size {nlat,nlon,nhgt} */
  double rb;      /* Earth radius (km) */
  double lats[3]; /* Latitude start/interval (deg) */
  double lons[3]; /* Longitude start/interval (deg) */
  double hgts[3]; /* Heights start/interval (km) */
  double *data;   /* TEC grid data (tecu) */
  float *rms;     /* RMS values (tecu) */
} tec_t;

typedef struct {    /* SBAS message type */
  int week, tow;    /* Reception time */
  uint8_t prn, rcv; /* SBAS satellite PRN,receiver number */
  uint8_t msg[29];  /* SBAS message (226bit) padded by 0 */
} sbsmsg_t;

typedef struct {  /* SBAS messages type */
  int n, nmax;    /* Number of SBAS messages/allocated */
  sbsmsg_t *msgs; /* SBAS messages */
} sbs_t;

typedef struct { /* SBAS fast correction type */
  gtime_t t0;    /* Time of applicability (TOF) */
  double prc;    /* Pseudorange correction (PRC) (m) */
  double rrc;    /* Range-rate correction (RRC) (m/s) */
  double dt;     /* Range-rate correction delta-time (s) */
  int iodf;      /* IODF (issue of date fast corr) */
  int16_t udre;  /* UDRE+1 */
  int16_t ai;    /* Degradation factor indicator */
} sbsfcorr_t;

typedef struct {     /* SBAS long term satellite error correction type */
  gtime_t t0;        /* Correction time */
  int iode;          /* IODE (issue of date ephemeris) */
  double dpos[3];    /* Delta position (m) (ECEF) */
  double dvel[3];    /* Delta velocity (m/s) (ECEF) */
  double daf0, daf1; /* Delta clock-offset/drift (s,s/s) */
} sbslcorr_t;

typedef struct {    /* SBAS satellite correction type */
  int sat;          /* Satellite number */
  sbsfcorr_t fcorr; /* Fast correction */
  sbslcorr_t lcorr; /* Long term correction */
} sbssatp_t;

typedef struct {         /* SBAS satellite corrections type */
  int iodp;              /* IODP (issue of date mask) */
  int nsat;              /* Number of satellites */
  int tlat;              /* System latency (s) */
  sbssatp_t sat[MAXSAT]; /* Satellite correction */
} sbssat_t;

typedef struct {    /* SBAS ionospheric correction type */
  gtime_t t0;       /* Correction time */
  int16_t lat, lon; /* Latitude/longitude (deg) */
  int16_t give;     /* GIVI+1 */
  float delay;      /* Vertical delay estimate (m) */
} sbsigp_t;

typedef struct {    /* IGP band type */
  int16_t x;        /* Longitude/latitude (deg) */
  const int16_t *y; /* Latitudes/longitudes (deg) */
  uint8_t bits;     /* IGP mask start bit */
  uint8_t bite;     /* IGP mask end bit */
} sbsigpband_t;

typedef struct {         /* SBAS ionospheric corrections type */
  int iodi;              /* IODI (issue of date ionos corr) */
  int nigp;              /* Number of igps */
  sbsigp_t igp[MAXNIGP]; /* Ionospheric correction */
} sbsion_t;

typedef struct { /* DGPS/GNSS correction type */
  gtime_t t0;    /* Correction time */
  double prc;    /* Pseudorange correction (PRC) (m) */
  double rrc;    /* Range rate correction (RRC) (m/s) */
  int iod;       /* Issue of data (IOD) */
  double udre;   /* UDRE */
} dgps_t;

typedef struct {            /* SSR correction type */
  gtime_t t0[6];            /* Epoch time (GPST) {eph,clk,hrclk,URA,bias,pbias} */
  double udi[6];            /* SSR update interval (s) */
  int iod[6];               /* Iod SSR {eph,clk,hrclk,URA,bias,pbias} */
  int iode;                 /* Issue of data */
  int iodcrc;               /* Issue of data crc for BeiDou/SBAS */
  int ura;                  /* URA indicator */
  int refd;                 /* Sat ref datum (0:ITRF,1:regional) */
  double deph[3];           /* Delta orbit {radial,along,cross} (m) */
  double ddeph[3];          /* Dot delta orbit {radial,along,cross} (m/s) */
  double dclk[3];           /* Delta clock {c0,c1,c2} (m,m/s,m/s^2) */
  double hrclk;             /* High-rate clock correction (m) */
  float cbias[MAXCODE];     /* Code biases (m) */
  double pbias[MAXCODE];    /* Phase biases (m) */
  float stdpb[MAXCODE];     /* Std-dev of phase biases (m) */
  double yaw_ang, yaw_rate; /* Yaw angle and yaw rate (deg,deg/s) */
  uint8_t update;           /* Update flag (0:no update,1:update) */
} ssr_t;

typedef struct {                   /* Navigation data type */
  int n[MAXSAT], nmax[MAXSAT];     /* Number of broadcast ephemeris */
  int ng[NSATGLO], ngmax[NSATGLO]; /* Number of GLONASS ephemeris */
  int ns[NSATSBS], nsmax[NSATSBS]; /* Number of SBAS ephemeris */
  int ne, nemax;                   /* Number of precise ephemeris */
  int nc, ncmax;                   /* Number of precise clock */
  int na, namax;                   /* Number of almanac data */
  int nt, ntmax;                   /* Number of TEC grid data */
  eph_t *eph[MAXSAT];              /* GPS/QZS/GAL/BDS/IRN ephemeris */
  geph_t *geph[NSATGLO];           /* GLONASS ephemeris */
  seph_t *seph[NSATSBS];           /* SBAS ephemeris */
  peph_t *peph;                    /* Precise ephemeris */
  pclk_t *pclk;                    /* Precise clock */
  alm_t *alm;                      /* Almanac data */
  tec_t *tec;                      /* TEC grid data */
  erp_t erp;                       /* Earth rotation parameters */
  double utc_gps[8]; /* GPS delta-UTC parameters {A0,A1,Tot,WNt,dt_LS,WN_LSF,DN,dt_LSF} */
  double utc_glo[8]; /* GLONASS UTC time parameters {tau_C,tau_GPS} */
  double utc_gal[8]; /* Galileo UTC parameters */
  double utc_qzs[8]; /* QZS UTC parameters */
  double utc_cmp[8]; /* BeiDou UTC parameters */
  double utc_irn[9]; /* IRNSS UTC parameters {A0,A1,Tot,...,dt_LSF,A2} */
  double utc_sbs[4]; /* SBAS UTC parameters */
  double ion_gps[8]; /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
  double ion_gal[4]; /* Galileo iono model parameters {ai0,ai1,ai2,0} */
  double ion_qzs[8]; /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
  double ion_cmp[8]; /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
  double ion_irn[8]; /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
  int glo_fcn[32];   /* GLONASS FCN + 8 */
  double cbias[MAXSAT][MAX_CODE_BIAS_FREQS]
              [MAX_CODE_BIASES]; /* Satellite DCB [0:P1-C1,1:P2-C2][code] (m) */
  double rbias[MAXRCV][MAX_CODE_BIAS_FREQS]
              [MAX_CODE_BIASES]; /* Receiver DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
  pcv_t pcvs[MAXSAT];            /* Satellite antenna pcv */
  sbssat_t sbssat;               /* SBAS satellite corrections */
  sbsion_t sbsion[MAXBAND + 1];  /* SBAS ionosphere corrections */
  dgps_t dgps[MAXSAT];           /* DGPS corrections */
  ssr_t ssr[MAXSAT];             /* SSR corrections */
} nav_t;

typedef struct {         /* Station parameter type */
  char name[MAXANT];     /* Marker name */
  char marker[MAXANT];   /* Marker number */
  char antdes[MAXANT];   /* Antenna descriptor */
  char antsno[MAXANT];   /* Antenna serial number */
  char rectype[MAXANT];  /* Receiver type descriptor */
  char recver[MAXANT];   /* Receiver firmware version */
  char recsno[MAXANT];   /* Receiver serial number */
  int antsetup;          /* Antenna setup id */
  int itrf;              /* ITRF realization year */
  int deltype;           /* Antenna delta type (0:enu,1:xyz) */
  double pos[3];         /* Station position (ECEF) (m) */
  double del[3];         /* Antenna position delta (e/n/u or x/y/z) (m) */
  double hgt;            /* Antenna height (m) */
  int glo_cp_align;      /* GLONASS code-phase alignment (0:no,1:yes) */
  double glo_cp_bias[4]; /* GLONASS code-phase biases {1C,1P,2C,2P} (m) */
} sta_t;

typedef struct {     /* Solution type */
  gtime_t time;      /* Time (GPST) */
  gtime_t eventime;  /* Time of event (GPST) */
  double rr[6];      /* Position/velocity (m|m/s) */
                     /* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
  float qr[6];       /* Position variance/covariance (m^2) */
                     /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
                     /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
  float qv[6];       /* Velocity variance/covariance (m^2/s^2) */
  double dtr[6];     /* Receiver clock bias to time systems (s) */
  uint8_t type;      /* Type (0:xyz-ecef,1:enu-baseline) */
  uint8_t stat;      /* Solution status (SOLQ_???) */
  uint8_t ns;        /* Number of valid satellites */
  float age;         /* Age of differential (s) */
  float ratio;       /* AR ratio factor for validation */
  float prev_ratio1; /* Previous initial AR ratio factor for validation */
  float prev_ratio2; /* Previous final AR ratio factor for validation */
  float thres;       /* AR ratio threshold for validation */
  int refstationid;  /* Ref station ID */
} sol_t;

typedef struct {               /* Solution buffer type */
  int n, nmax;                 /* Number of solution/max number of buffer */
  int cyclic;                  /* Cyclic buffer flag */
  int start, end;              /* Start/end index */
  gtime_t time;                /* Current solution time */
  sol_t *data;                 /* Solution data */
  double rb[3];                /* Reference position {x,y,z} (ECEF) (m) */
  uint8_t buff[MAXSOLMSG + 1]; /* Message buffer */
  int nb;                      /* Number of byte in message buffer */
} solbuf_t;

typedef struct {  /* Solution status type */
  gtime_t time;   /* Time (GPST) */
  uint8_t sat;    /* Satellite number */
  uint8_t frq;    /* Frequency (1:L1,2:L2,...) */
  float az, el;   /* Azimuth/elevation angle (rad) */
  float resp;     /* Pseudorange residual (m) */
  float resc;     /* Carrier-phase residual (m) */
  uint8_t flag;   /* Flags: (vsat<<5)+(slip<<3)+fix */
  uint16_t snr;   /* Signal strength (*SNR_UNIT dBHz) */
  uint16_t lock;  /* Lock counter */
  uint16_t outc;  /* Outage counter */
  uint16_t slipc; /* Slip counter */
  uint16_t rejc;  /* Reject counter */
} solstat_t;

typedef struct {   /* Solution status buffer type */
  int n, nmax;     /* Number of solution/max number of buffer */
  solstat_t *data; /* Solution status data */
} solstatbuf_t;

typedef struct {                          /* RTCM control struct type */
  int staid;                              /* Station id */
  int stah;                               /* Station health */
  int seqno;                              /* Sequence number for RTCM 2 or iods MSM */
  int outtype;                            /* Output message type */
  gtime_t time;                           /* Message time */
  gtime_t time_s;                         /* Message start time */
  obs_t obs;                              /* Observation data (uncorrected) */
  nav_t nav;                              /* Satellite ephemerides */
  sta_t sta;                              /* Station parameters */
  dgps_t *dgps;                           /* Output of dgps corrections */
  ssr_t ssr[MAXSAT];                      /* Output of SSR corrections */
  char msg[128];                          /* Special message */
  char msgtype[256];                      /* Last message type */
  char msmtype[7][128];                   /* MSM signal types */
  int obsflag;                            /* Obs data complete flag (1:ok,0:not complete) */
  int ephsat;                             /* Input ephemeris satellite number */
  int ephset;                             /* Input ephemeris set (0-1) */
  double cp[MAXSAT][NFREQ + NEXOBS];      /* Carrier-phase measurement */
  uint16_t lock[MAXSAT][NFREQ + NEXOBS];  /* Lock time */
  uint16_t loss[MAXSAT][NFREQ + NEXOBS];  /* Loss of lock count */
  gtime_t lltime[MAXSAT][NFREQ + NEXOBS]; /* Last lock time */
  int nbyte;                              /* Number of bytes in message buffer */
  int nbit;                               /* Number of bits in word buffer */
  int len;                                /* Message length (bytes) */
  uint8_t buff[1200];                     /* Message buffer */
  uint32_t word;                          /* Word buffer for RTCM 2 */
  uint32_t nmsg2[100];                    /* Message count of RTCM 2 (1-99:1-99,0:other) */
  uint32_t nmsg3[400]; /* Message count of RTCM 3 (1-299:1001-1299,300-329:4070-4099,0:ohter) */
  char opt[256];       /* RTCM dependent options */
} rtcm_t;

typedef struct {               /* RINEX control struct type */
  gtime_t time;                /* Message time */
  double ver;                  /* RINEX version */
  char type;                   /* RINEX file type ('O','N',...) */
  int sys;                     /* Navigation system */
  int tsys;                    /* Time system */
  char tobs[8][MAXOBSTYPE][4]; /* RINEX obs types */
  obs_t obs;                   /* Observation data */
  nav_t nav;                   /* Navigation data */
  sta_t sta;                   /* Station info */
  int ephsat;                  /* Input ephemeris satellite number */
  int ephset;                  /* Input ephemeris set (0-1) */
  char opt[256];               /* RINEX dependent options */
} rnxctr_t;

typedef struct {     /* Download URL type */
  char type[32];     /* Data type */
  char path[FNSIZE]; /* URL path */
  char dir[FNSIZE];  /* Local directory */
  double tint;       /* Time interval (s) */
} url_t;

typedef struct {       /* Option type */
  const char *name;    /* Option name */
  int format;          /* Option format (0:int,1:double,2:string,3:enum) */
  void *var;           /* Pointer to option variable */
  size_t vsize;        /* String option variable size. */
  const char *comment; /* Option comment/enum labels/unit */
} opt_t;

typedef struct {         /* SNR mask type */
  int ena[2];            /* Enable flag {rover,base} */
  double mask[NFREQ][9]; /* Mask (dBHz) at 5,10,...85 deg */
} snrmask_t;

typedef struct {        /* Processing options type */
  int mode;             /* Positioning mode (PMODE_???) */
  int soltype;          /* Solution type (0:forward,1:backward,2:combined) */
  int nf;               /* Number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
  int navsys;           /* Navigation system */
  double elmin;         /* Elevation mask angle (rad) */
  snrmask_t snrmask;    /* SNR mask */
  int sateph;           /* Satellite ephemeris/clock (EPHOPT_???) */
  int modear;           /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
  int glomodear;        /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
  int gpsmodear;        /* GPS AR mode, debug/learning only (0:off,1:on) */
  int bdsmodear;        /* BeiDou AR mode (0:off,1:on) */
  int arfilter;         /* AR filtering to reject bad sats (0:off,1:on) */
  int maxout;           /* Obs outage count to reset bias */
  int minlock;          /* Min lock count to fix ambiguity */
  int minfixsats;       /* Min sats to fix integer ambiguities */
  int minholdsats;      /* Min sats to hold integer ambiguities */
  int mindropsats;      /* Min sats to drop sats in AR */
  int minfix;           /* Min fix count to hold ambiguity */
  int armaxiter;        /* Max iteration to resolve ambiguity */
  int ionoopt;          /* Ionosphere option (IONOOPT_???) */
  int tropopt;          /* Troposphere option (TROPOPT_???) */
  int dynamics;         /* Dynamics model (0:none,1:velocity,2:accel) */
  int tidecorr;         /* Earth tide correction (0:off,1:solid,2:solid+otl+pole) */
  int niter;            /* Number of filter iteration */
  int codesmooth;       /* Code smoothing window size (0:none) */
  int intpref;          /* Interpolate reference obs (for post mission) */
  int sbascorr;         /* SBAS correction options */
  int sbassatsel;       /* SBAS satellite selection (0:all) */
  int rovpos;           /* Rover position for fixed mode */
  int refpos;           /* Base position for relative mode */
                        /* (0:pos in prcopt,  1:average of single pos, */
                        /*  2:read from file, 3:rinex header, 4:rtcm pos) */
  double eratio[NFREQ]; /* Code/phase error ratio */
  double err[8];        /* Observation error terms */
                        /* [reserved,constant,elevation,baseline,doppler,snr-max,snr, rcv_std] */
  double std[3];        /* Initial-state std [0]bias,[1]iono [2]trop */
  double prn[6];        /* Process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
  double sclkstab;      /* Satellite clock stability (sec/sec) */
  double thresar[8];    /* AR validation threshold */
  double elmaskar;      /* Elevation mask of AR for rising satellite (deg) */
  double elmaskhold;    /* Elevation mask to hold ambiguity (deg) */
  double thresslip;     /* Slip threshold of geometry-free phase (m) */
  double thresdop;      /* Slip threshold of doppler (m) */
  double varholdamb;    /* Variance for fix-and-hold pseudo measurements (cycle^2) */
  double gainholdamb;   /* Gain used for GLO and SBAS sats to adjust ambiguity */
  double maxtdiff;      /* Max difference of time (sec) */
  double maxinno[2];    /* Reject threshold of innovation for phase and code (m) */
  double baseline[2];   /* Baseline length constraint {const,sigma} (m) */
  double ru[3];         /* Rover position for fixed mode {x,y,z} (ECEF) (m) */
  double rb[3];         /* Base position for relative mode {x,y,z} (ECEF) (m) */
  char anttype[2][MAXANT]; /* Antenna types {rover,base} */
  double antdel[2][3];     /* Antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
  pcv_t pcvr[2];           /* Receiver antenna parameters {rov,base} */
  uint8_t exsats[MAXSAT];  /* Excluded satellites (1:excluded,2:included) */
  int maxaveep;            /* Max averaging epochs */
  int initrst;             /* Initialize by restart */
  int outsingle;           /* Output single by dgps/float/fix/ppp outage */
  char rnxopt[2][256];     /* RINEX options {rover,base} */
  int posopt[6];           /* Positioning options */
  int syncsol;             /* Solution sync mode (0:off,1:on) */
  double odisp[2][6 * 11]; /* Ocean tide loading parameters {rov,base} */
  int freqopt;             /* Disable L2-AR */
  char pppopt[256];        /* Ppp option */
} prcopt_t;

typedef struct {      /* Solution options type */
  int posf;           /* Solution format (SOLF_???) */
  int times;          /* Time system (TIMES_???) */
  int timef;          /* Time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) */
  int timeu;          /* Time digits under decimal point */
  int degf;           /* Latitude/longitude format (0:ddd.ddd,1:ddd mm ss) */
  int outhead;        /* Output header (0:no,1:yes) */
  int outopt;         /* Output processing options (0:no,1:yes) */
  int outvel;         /* Output velocity options (0:no,1:yes) */
  int datum;          /* Datum (0:WGS84,1:Tokyo) */
  int height;         /* Height (0:ellipsoidal,1:geodetic) */
  int geoid;          /* Geoid model (0:EGM96,1:JGD2000) */
  int solstatic;      /* Solution of static mode (0:all,1:single) */
  int sstat;          /* Solution statistics level (0:off,1:states,2:residuals) */
  int trace;          /* Debug trace level (0:off,1-5:debug) */
  double nmeaintv[2]; /* NMEA output interval (s) (<0:no,0:all) */
                      /* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
  char sep[64];       /* Field separator */
  char prog[64];      /* Program name */
  double maxsolstd;   /* Max std-dev for solution output (m) (0:all) */
} solopt_t;

typedef struct {            /* File options type */
  char satantp[MAXSTRPATH]; /* Satellite antenna parameters file */
  char rcvantp[MAXSTRPATH]; /* Receiver antenna parameters file */
  char stapos[MAXSTRPATH];  /* Station positions file */
  char geoid[MAXSTRPATH];   /* External geoid data file */
  char iono[MAXSTRPATH];    /* Ionosphere data file */
  char dcb[MAXSTRPATH];     /* Dcb data file */
  char eop[MAXSTRPATH];     /* Eop data file */
  char blq[MAXSTRPATH];     /* Ocean tide loading blq file */
  char tempdir[MAXSTRPATH]; /* FTP/HTTP temporary directory */
  char geexe[MAXSTRPATH];   /* Google earth exec file */
  char solstat[MAXSTRPATH]; /* Solution statistics file */
  char trace[MAXSTRPATH];   /* Debug trace file */
} filopt_t;

typedef struct {                /* RINEX options type */
  gtime_t ts, te;               /* Time start/end */
  double tint;                  /* Time interval (s) */
  double ttol;                  /* Time tolerance (s) */
  double tunit;                 /* Time unit for multiple-session (s) */
  int rnxver;                   /* RINEX version (x100) */
  int navsys;                   /* Navigation system */
  int obstype;                  /* Observation type */
  int freqtype;                 /* Frequency type */
  char mask[7][64];             /* Code mask {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
  char staid[32];               /* Station id for RINEX file name */
  char prog[32];                /* Program */
  char runby[32];               /* Run-by */
  char marker[64];              /* Marker name */
  char markerno[32];            /* Marker number */
  char markertype[32];          /* Marker type (ver.3) */
  char name[2][32];             /* Observer/agency */
  char rec[3][32];              /* Receiver #/type/vers */
  char ant[3][32];              /* Antenna #/type */
  double apppos[3];             /* Approx position x/y/z */
  double antdel[3];             /* Antenna delta h/e/n */
  double glo_cp_bias[4];        /* GLONASS code-phase biases (m) */
  char comment[MAXCOMMENT][64]; /* Comments */
  char rcvopt[256];             /* Receiver dependent options */
  uint8_t exsats[MAXSAT];       /* Excluded satellites */
  int glofcn[32];               /* GLONASS fcn+8 */
  int outiono;                  /* Output iono correction */
  int outtime;                  /* Output time system correction */
  int outleaps;                 /* Output leap seconds */
  int autopos;                  /* Auto approx position */
  int phshift;                  /* Phase shift correction */
  int halfcyc;                  /* Half cycle correction */
  int sep_nav;                  /* Separated nav files */
  gtime_t tstart;               /* First obs time */
  gtime_t tend;                 /* Last obs time */
  gtime_t trtcm;                /* Approx log start time for RTCM */
  char tobs[7][MAXOBSTYPE][4];  /* Obs types {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
  double shift[7][MAXOBSTYPE];  /* Phase shift (cyc) {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
  int nobs[7];                  /* Number of obs types {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
} rnxopt_t;

typedef struct {             /* Satellite status type */
  uint8_t sys;               /* Navigation system */
  uint8_t vs;                /* Valid satellite flag single */
  double azel[2];            /* Azimuth/elevation angles {az,el} (rad) */
  double resp[NFREQ];        /* Residuals of pseudorange (m) */
  double resc[NFREQ];        /* Residuals of carrier-phase (m) */
  double icbias[NFREQ];      /* GLONASS IC bias (cycles) */
  uint8_t vsat[NFREQ];       /* Valid satellite flag */
  uint16_t snr_rover[NFREQ]; /* Rover signal strength (0.25 dBHz) */
  uint16_t snr_base[NFREQ];  /* Base signal strength (0.25 dBHz) */
  uint8_t fix[NFREQ];        /* Ambiguity fix flag (1:float,2:fix,3:hold) */
  uint8_t slip[NFREQ];       /* Cycle-slip flag */
  uint8_t half[NFREQ];       /* Half-cycle valid flag */
  int lock[NFREQ];           /* Lock counter of phase */
  uint32_t outc[NFREQ];      /* Obs outage counter of phase */
  uint32_t slipc[NFREQ];     /* Cycle-slip counter */
  uint32_t rejc[NFREQ];      /* Reject counter */
  double gf[NFREQ - 1];      /* Geometry-free phase (m) */
  double mw[NFREQ - 1];      /* MW-LC (m) */
  double phw;                /* Phase windup (cycle) */
  gtime_t pt[2][NFREQ];      /* Previous carrier-phase time */
  double ph[2][NFREQ];       /* Previous carrier-phase observable (cycle) */
} ssat_t;

typedef struct {      /* Ambiguity control type */
  gtime_t epoch[4];   /* Last epoch */
  int n[4];           /* Number of epochs */
  double LC[4];       /* Linear combination average */
  double LCv[4];      /* Linear combination variance */
  int fixcnt;         /* Fix count */
  char flags[MAXSAT]; /* Fix flags */
} ambc_t;

typedef struct {       /* RTK control/result type */
  sol_t sol;           /* RTK solution */
  double rb[6];        /* Base position/velocity (ECEF) (m|m/s) */
  int nx, na;          /* Number of float states/fixed states */
  double tt;           /* Time difference between current and previous (s) */
  double *x, *P;       /* Float states and their covariance */
  double *xa, *Pa;     /* Fixed states and their covariance */
  int nfix;            /* Number of continuous fixes of ambiguity */
  int excsat;          /* Index of next satellite to be excluded for partial ambiguity resolution */
  int nb_ar;           /* Number of ambiguities used for AR last epoch */
  char holdamb;        /* Set if fix-and-hold has occurred at least once */
  ambc_t ambc[MAXSAT]; /* Ambiguity control */
  ssat_t ssat[MAXSAT]; /* Satellite status */
  int neb;             /* Bytes in error message buffer */
  char errbuf[MAXERRMSG]; /* Error message buffer */
  prcopt_t opt;           /* Processing options */
  int initial_mode;       /* Initial positioning mode */
  int epoch;              /* Epoch number */
} rtk_t;

typedef struct {                        /* Receiver raw data control type */
  gtime_t time;                         /* Message time */
  gtime_t tobs[MAXSAT][NFREQ + NEXOBS]; /* Observation data time */
  obs_t obs;                            /* Observation data */
  obs_t obuf;                           /* Observation data buffer */
  nav_t nav;                            /* Satellite ephemerides */
  sta_t sta;                            /* Station parameters */
  int ephsat;                           /* Update satellite of ephemeris (0:no satellite) */
  int ephset;                           /* Update set of ephemeris (0-1) */
  sbsmsg_t sbsmsg;                      /* SBAS message */
  char msgtype[256];                    /* Last message type */
  uint8_t subfrm[MAXSAT][380];          /* Subframe buffer */
  double lockt[MAXSAT][NFREQ + NEXOBS]; /* Lock time (s) */
  unsigned char lockflag[MAXSAT][NFREQ + NEXOBS]; /* Used for carrying forward cycle slip */
  double icpp[MAXSAT], off[MAXSAT], icpc;         /* Carrier params for ss2 */
  double prCA[MAXSAT], dpCA[MAXSAT];              /* L1/CA pseudorange/doppler for JAVAD */
  uint8_t halfc[MAXSAT][NFREQ + NEXOBS];          /* Half-cycle resolved */
  char freqn[MAXOBS];                             /* Frequency number for JAVAD */
  int nbyte;                                      /* Number of bytes in message buffer */
  int len;                                        /* Message length (bytes) */
  int iod;                                        /* Issue of data */
  int tod;                                        /* Time of day (ms) */
  int tbase;               /* Time base (0:gpst,1:utc(usno),2:glonass,3:utc(su) */
  int flag;                /* General purpose flag */
  int outtype;             /* Output message type */
  uint8_t buff[MAXRAWLEN]; /* Message buffer */
  char opt[256];           /* Receiver dependent options */
  int format;              /* Receiver stream format */
  int rcvtype;             /* Receiver type within format */
  void *rcv_data;          /* Receiver dependent data */
} raw_t;

typedef struct {         /* Stream type */
  int type;              /* Type (STR_???) */
  int mode;              /* Mode (STR_MODE_?) */
  int state;             /* State (-1:error,0:close,1:open) */
  uint32_t inb, inr;     /* Input bytes/rate */
  uint32_t outb, outr;   /* Output bytes/rate */
  uint32_t tick_i;       /* Input tick tick */
  uint32_t tick_o;       /* Output tick */
  uint32_t tact;         /* Active tick */
  uint32_t inbt, outbt;  /* Input/output bytes at tick */
  rtklib_lock_t lock;    /* Lock flag */
  void *port;            /* Type dependent port control struct */
  char path[MAXSTRPATH]; /* Stream path */
  char msg[MAXSTRMSG];   /* Stream message */
} stream_t;

typedef struct {     /* Stream converter type */
  int itype, otype;  /* Input and output stream type */
  int nmsg;          /* Number of output messages */
  int msgs[32];      /* Output message types */
  double tint[32];   /* Output message intervals (s) */
  uint32_t tick[32]; /* Cycle tick of output message */
  int ephsat[32];    /* Satellites of output ephemeris */
  int stasel;        /* Station info selection (0:remote,1:local) */
  rtcm_t rtcm;       /* RTCM input data buffer */
  raw_t raw;         /* Raw  input data buffer */
  rtcm_t out;        /* RTCM output data buffer */
} strconv_t;

typedef struct {                     /* Stream server type */
  int state;                         /* Server state (0:stop,1:running) */
  int cycle;                         /* Server cycle (ms) */
  int buffsize;                      /* Input/monitor buffer size (bytes) */
  int nmeacycle;                     /* NMEA request cycle (ms) (0:no) */
  int relayback;                     /* Relay back of output streams (0:no) */
  int nstr;                          /* Number of streams (1 input + (nstr-1) outputs */
  int npb;                           /* Data length in peek buffer (bytes) */
  char cmds_periodic[16][MAXRCVCMD]; /* Periodic commands */
  double nmeapos[3];                 /* NMEA request position (ECEF) (m) */
  uint8_t *buff;                     /* Input buffers */
  uint8_t *pbuf;                     /* Peek buffer */
  uint32_t tick;                     /* Start tick */
  stream_t stream[16];               /* Input/output streams */
  stream_t strlog[16];               /* Return log streams */
  strconv_t *conv[16];               /* Stream converter */
  rtklib_thread_t thread;            /* Server thread */
  rtklib_lock_t lock;                /* Lock flag */
} strsvr_t;

typedef struct {                    /* RTK server type */
  int state;                        /* Server state (0:stop,1:running) */
  int cycle;                        /* Processing cycle (ms) */
  int nmeacycle;                    /* NMEA request cycle (ms) (0:no req) */
  int nmeareq;                      /* NMEA request (0:no,1:nmeapos,2:single sol) */
  double nmeapos[3];                /* NMEA request position (ECEF) (m) */
  int buffsize;                     /* Input buffer size (bytes) */
  int format[3];                    /* Input format {rov,base,corr} */
  solopt_t solopt[2];               /* Output solution options {sol1,sol2} */
  int navsel;                       /* Ephemeris select (0:all,1:rover,2:base,3:corr) */
  int nsbs;                         /* Number of SBAS message */
  int nsol;                         /* Number of solution buffer */
  rtk_t rtk;                        /* RTK control/result struct */
  int nb[3];                        /* Bytes in input buffers {rov,base} */
  int nsb[2];                       /* Bytes in solution buffers */
  int npb[3];                       /* Bytes in input peek buffers */
  uint8_t *buff[3];                 /* Input buffers {rov,base,corr} */
  uint8_t *sbuf[2];                 /* Output buffers {sol1,sol2} */
  uint8_t *pbuf[3];                 /* Peek buffers {rov,base,corr} */
  sol_t solbuf[MAXSOLBUF];          /* Solution buffer */
  uint32_t nmsg[3][10];             /* Input message counts */
  raw_t raw[3];                     /* Receiver raw control {rov,base,corr} */
  rtcm_t rtcm[3];                   /* RTCM control {rov,base,corr} */
  gtime_t ftime[3];                 /* Download time {rov,base,corr} */
  char files[3][MAXSTRPATH];        /* Download paths {rov,base,corr} */
  obs_t obs[3][MAXOBSBUF];          /* Observation data {rov,base,corr} */
  nav_t nav;                        /* Navigation data */
  sbsmsg_t sbsmsg[MAXSBSMSG];       /* SBAS message buffer */
  stream_t stream[8];               /* Streams {rov,base,corr,sol1,sol2,logr,logb,logc} */
  stream_t *moni;                   /* Monitor stream */
  uint32_t tick;                    /* Start tick */
  rtklib_thread_t thread;           /* Server thread */
  int cputime;                      /* CPU time (ms) for a processing cycle */
  int prcout;                       /* Missing observation data count */
  int nave;                         /* Number of averaging base pos */
  double rb_ave[3];                 /* Averaging base pos */
  char cmds_periodic[3][MAXRCVCMD]; /* Periodic commands */
  char cmd_reset[MAXRCVCMD];        /* Reset command */
  double bl_reset;                  /* Baseline length to reset (km) */
  rtklib_lock_t lock;               /* Lock flag */
} rtksvr_t;

typedef struct { /* GIS data point type */
  double pos[3]; /* Point data {lat,lon,height} (rad,m) */
} gis_pnt_t;

typedef struct {   /* GIS data polyline type */
  int npnt;        /* Number of points */
  double bound[4]; /* Boundary {lat0,lat1,lon0,lon1} */
  double *pos;     /* Position data (3 x npnt) */
} gis_poly_t;

typedef struct {   /* GIS data polygon type */
  int npnt;        /* Number of points */
  double bound[4]; /* Boundary {lat0,lat1,lon0,lon1} */
  double *pos;     /* Position data (3 x npnt) */
} gis_polygon_t;

typedef struct gisd_tag { /* GIS data list type */
  int type;               /* Data type (1:point,2:polyline,3:polygon) */
  void *data;             /* Data body */
  struct gisd_tag *next;  /* Pointer to next */
} gisd_t;

typedef struct {               /* GIS type */
  char name[MAXGISLAYER][256]; /* Name */
  int flag[MAXGISLAYER];       /* Flag */
  gisd_t *data[MAXGISLAYER];   /* Gis data list */
  double bound[4];             /* Boundary {lat0,lat1,lon0,lon1} */
} gis_t;

typedef void fatalfunc_t(const char *); /* Fatal callback function type */

/* Global variables ----------------------------------------------------------*/
EXPORT extern const double chisqr[];             /* chi-sqr(n) table (alpha=0.001) */
EXPORT extern const prcopt_t prcopt_default;     /* Default positioning options */
EXPORT extern const solopt_t solopt_default;     /* Default solution output options */
EXPORT extern const sbsigpband_t igpband1[9][8]; /* SBAS IGP band 0-8 */
EXPORT extern const sbsigpband_t igpband2[2][5]; /* SBAS IGP band 9-10 */
EXPORT extern const char *formatstrs[];          /* Stream format strings */
EXPORT extern opt_t sysopts[];                   /* System options table */

/* Satellites, systems, codes functions --------------------------------------*/
EXPORT int satno(int sys, int prn);
EXPORT int satsys(int sat, int *prn);
EXPORT int satid2no(const char *id);
EXPORT void satno2id(int sat, char id[8]);
EXPORT uint8_t obs2code(const char *obs);
EXPORT char *code2obs(uint8_t code);
EXPORT double code2freq(int sys, uint8_t code, int fcn);
EXPORT double sat2freq(int sat, uint8_t code, const nav_t *nav);
EXPORT int code2idx(int sys, uint8_t code);
EXPORT bool satexclude(int sat, double var, int svh, const prcopt_t *opt);
EXPORT bool testsnr(int base, int freq, double el, double snr, const snrmask_t *mask);
EXPORT void setcodepri(int sys, int idx, const char *pri);
EXPORT int getcodepri(int sys, uint8_t code, const char *opt);

/* Satellite system to index number, or zero if no match. */
static inline int sys2no(int sys) {
#ifdef __GNUC__
  return __builtin_ffs(sys);
#else
  if (sys == 0) return 0;
  int i = 1;
  if (!(sys & 0x0f)) {
    i += 4;
    sys >>= 4;
  }
  if (!(sys & 0x03)) {
    i += 2;
    sys >>= 2;
  }
  if (!(sys & 0x01)) {
    i += 1;
  }
  return i;
#endif
}

/* Matrix and vector functions -----------------------------------------------*/
EXPORT double *mat(int n, int m);
EXPORT int *imat(int n, int m);
EXPORT double *zeros(int n, int m);
EXPORT double *eye(int n);
/* Dot product -----------------------------------------------------------------
 * Inner product of vectors of size 2
 * Args   : double *a,*b     I   vectors a and b
 * Return : a'*b
 *----------------------------------------------------------------------------*/
static inline double dot2(const double *a, const double *b) { return a[0] * b[0] + a[1] * b[1]; }

/* Dot product -----------------------------------------------------------------
 * Inner product of vectors of size 3
 * Args   : double *a,*b     I   vectors a and b
 * Return : a'*b
 *----------------------------------------------------------------------------*/
static inline double dot3(const double *a, const double *b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* Inner product ---------------------------------------------------------------
 * Inner product of vectors
 * Args   : double *a,*b     I   vector a,b (n x 1)
 *          int    n         I   size of vector a,b
 * Return : a'*b
 *----------------------------------------------------------------------------*/
static inline double dot(const double *a, const double *b, int n) {
  double c = 0.0;

  while (--n >= 0) c += a[n] * b[n];
  return c;
}

/* Euclid norm -----------------------------------------------------------------
 * Euclid norm of vector
 * Args   : double *a        I   vector a (n x 1)
 *          int    n         I   size of vector a
 * Return : || a ||
 *----------------------------------------------------------------------------*/
static inline double norm(const double *a, int n) { return sqrt(dot(a, a, n)); }
EXPORT void cross3(const double *a, const double *b, double *c);
EXPORT bool normv3(const double *a, double *b);
/* Copy matrix -----------------------------------------------------------------
 * Copy matrix
 * Args   : double *A        O   destination matrix A (n x m)
 *          double *B        I   source matrix B (n x m)
 *          int    n,m       I   number of rows and columns of matrix
 * Return : none
 *----------------------------------------------------------------------------*/
static inline void matcpy(double *A, const double *B, int n, int m) {
  memcpy(A, B, sizeof(double) * n * m);
}
EXPORT void matmul(const char *tr, int n, int k, int m, const double *A, const double *B,
                   double *C);
EXPORT void matmulp(const char *tr, int n, int k, int m, const double *A, const double *B,
                    double *C);
EXPORT void matmulm(const char *tr, int n, int k, int m, const double *A, const double *B,
                    double *C);
EXPORT int matinv(double *A, int n);
EXPORT int solve(const char *tr, const double *A, const double *Y, int n, int m, double *X);
EXPORT int lsq(const double *A, const double *y, int n, int m, double *x, double *Q);
EXPORT int filter_(double *x, const double *P, const double *H, const double *v, double *R, int n,
                   int m, double *Pp);
EXPORT int filter(double *x, double *P, const double *H, const double *v, double *R, int n, int m);
EXPORT int smoother(const double *xf, const double *Qf, const double *xb, const double *Qb, int n,
                    double *xs, double *Qs);
EXPORT void matprint(const double *A, int n, int m, int p, int q);
EXPORT void matfprint(const double *A, int n, int m, int p, int q, FILE *fp);

EXPORT void add_fatal(fatalfunc_t *func);

/* Time and string functions -------------------------------------------------*/
EXPORT double str2num(const char *s, int i, int n);
EXPORT int str2time(const char *s, int i, int n, gtime_t *t);
EXPORT char *time2str(gtime_t t, char str[40], int n);
EXPORT gtime_t epoch2time(const double *ep);
EXPORT void time2epoch(gtime_t t, double *ep);
EXPORT void time2epoch_n(gtime_t t, double *ep, int n);
EXPORT gtime_t gpst2time(int week, double sec);
EXPORT double time2gpst(gtime_t t, int *week);
EXPORT gtime_t gst2time(int week, double sec);
EXPORT double time2gst(gtime_t t, int *week);
EXPORT gtime_t bdt2time(int week, double sec);
EXPORT double time2bdt(gtime_t t, int *week);

#define RTKBOUNDSCHECK(buff, size, index) rtkboundscheck(__func__, __LINE__, buff, size, index);
EXPORT void rtkstrcpy(char *dst, size_t dsize, const char *src);
EXPORT void rtksubstrcpy(char *dst, size_t dsize, const char *src, size_t start);
EXPORT void rtkesubstrcpy(char *dst, size_t dsize, const char *src, size_t start, size_t end);
EXPORT void rtkboundscheck(const char *func, int line, const void *buff, size_t size, size_t index);
EXPORT void rtkstrcat(char *dst, size_t dsize, const char *src);
EXPORT void rtksubstrcat(char *dst, size_t dsize, const char *src, size_t start);
EXPORT void rtkesubstrcat(char *dst, size_t dsize, const char *src, size_t start, size_t end);
EXPORT void rtksnprintf(char *str, size_t size, const char *format, ...);
EXPORT void rtkcatprintf(char *str, size_t size, const char *format, ...);
EXPORT void rtksetstr(char *dst, size_t dsize, const char *src, size_t start, size_t end);

EXPORT int strchri(const char *s, size_t start, int c);
EXPORT int strrchri(const char *s, size_t start, int c);
EXPORT int strstri(const char *haystack, size_t start, const char *needle);

/* Add time --------------------------------------------------------------------
 * Add time to gtime_t struct
 * Args   : gtime_t t        I   gtime_t struct
 *          double sec       I   time to add (s)
 * Return : gtime_t struct (t+sec)
 *----------------------------------------------------------------------------*/
static inline gtime_t timeadd(gtime_t t, double sec) {
  double tt;

  t.sec += sec;
  tt = floor(t.sec);
  t.time += (int)tt;
  t.sec -= tt;
  return t;
}
/* Time difference -------------------------------------------------------------
 * Difference between gtime_t structs
 * Args   : gtime_t t1,t2    I   gtime_t structs
 * Return : time difference (t1-t2) (s)
 *----------------------------------------------------------------------------*/
static inline double timediff(gtime_t t1, gtime_t t2) {
  return (double)(t1.time - t2.time) + t1.sec - t2.sec;
}
EXPORT gtime_t gpst2utc(gtime_t t);
EXPORT gtime_t utc2gpst(gtime_t t);
EXPORT gtime_t gpst2bdt(gtime_t t);
EXPORT gtime_t bdt2gpst(gtime_t t);
EXPORT gtime_t timeget(void);
EXPORT void timeset(gtime_t t);
EXPORT void timereset(void);
EXPORT double time2doy(gtime_t t);
EXPORT double utc2gmst(gtime_t t, double ut1_utc);
EXPORT bool read_leaps(const char *file);

EXPORT int adjgpsweek(int week);
EXPORT uint32_t tickget(void);
EXPORT void sleepms(int ms);

EXPORT int reppath(const char *path, char *rpath, size_t size, gtime_t time, const char *rov,
                   const char *base);
EXPORT int reppaths(const char *path, char *rpaths[], size_t size, int nmax, gtime_t ts, gtime_t te,
                    const char *rov, const char *base);

/* Coordinates transformation ------------------------------------------------*/
EXPORT void ecef2pos(const double *r, double *pos);
EXPORT void pos2ecef(const double *pos, double *r);
EXPORT void ecef2enu(const double *pos, const double *r, double *e);
EXPORT void enu2ecef(const double *pos, const double *e, double *r);
EXPORT void covenu(const double *pos, const double *P, double *Q);
EXPORT void covecef(const double *pos, const double *Q, double *P);
EXPORT void xyz2enu(const double *pos, double *E);
EXPORT void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst);
EXPORT void deg2dms(double deg, double *dms, int ndec);
EXPORT double dms2deg(const double *dms);

/* Input and output functions ------------------------------------------------*/
EXPORT void readpos(const char *file, const char *rcv, double *pos);
EXPORT int sortobs(obs_t *obs);
EXPORT int navncnt(nav_t *nav);
EXPORT int navngcnt(nav_t *nav);
EXPORT int navnscnt(nav_t *nav);
EXPORT void uniqnav(nav_t *nav);
EXPORT bool screent(gtime_t time, gtime_t ts, gtime_t te, double tint);
EXPORT bool readnav(const char *file, nav_t *nav);
EXPORT bool savenav(const char *file, const nav_t *nav);
EXPORT void freeobs(obs_t *obs);
EXPORT void freenav(nav_t *nav, int opt);
EXPORT bool readblq(const char *file, const char *sta, double *odisp);
EXPORT bool readerp(const char *file, erp_t *erp);
EXPORT bool geterp(const erp_t *erp, gtime_t time, double *val);

/* Debug trace functions -----------------------------------------------------*/
#ifdef TRACE
#define trace(level, ...)                                         \
  do {                                                            \
    if (level <= gettracelevel()) trace_impl(level, __VA_ARGS__); \
  } while (0)
#define tracet(level, ...)                                         \
  do {                                                             \
    if (level <= gettracelevel()) tracet_impl(level, __VA_ARGS__); \
  } while (0)
#define tracemat(level, ...)                                         \
  do {                                                               \
    if (level <= gettracelevel()) tracemat_impl(level, __VA_ARGS__); \
  } while (0)
#define traceobs(level, ...)                                         \
  do {                                                               \
    if (level <= gettracelevel()) traceobs_impl(level, __VA_ARGS__); \
  } while (0)
#define tracenav(level, ...)                                         \
  do {                                                               \
    if (level <= gettracelevel()) tracenav_impl(level, __VA_ARGS__); \
  } while (0)
#define tracegnav(level, ...)                                         \
  do {                                                                \
    if (level <= gettracelevel()) tracegnav_impl(level, __VA_ARGS__); \
  } while (0)
#define tracehnav(level, ...)                                         \
  do {                                                                \
    if (level <= gettracelevel()) tracehnav_impl(level, __VA_ARGS__); \
  } while (0)
#define tracepeph(level, ...)                                         \
  do {                                                                \
    if (level <= gettracelevel()) tracepeph_impl(level, __VA_ARGS__); \
  } while (0)
#define tracepclk(level, ...)                                         \
  do {                                                                \
    if (level <= gettracelevel()) tracepclk_impl(level, __VA_ARGS__); \
  } while (0)
#define traceb(level, ...)                                         \
  do {                                                             \
    if (level <= gettracelevel()) traceb_impl(level, __VA_ARGS__); \
  } while (0)

EXPORT void traceopen(const char *file);
EXPORT void traceclose(void);
EXPORT void tracelevel(int level);
EXPORT int gettracelevel(void);

EXPORT void trace_impl(int level, const char *format, ...);
EXPORT void tracet_impl(int level, const char *format, ...);
EXPORT void tracemat_impl(int level, const double *A, int n, int m, int p, int q);
EXPORT void traceobs_impl(int level, const obsd_t *obs, int n);
EXPORT void tracenav_impl(int level, const nav_t *nav);
EXPORT void tracegnav_impl(int level, const nav_t *nav);
EXPORT void tracehnav_impl(int level, const nav_t *nav);
EXPORT void tracepeph_impl(int level, const nav_t *nav);
EXPORT void tracepclk_impl(int level, const nav_t *nav);
EXPORT void traceb_impl(int level, const uint8_t *p, size_t size, int n);

#else

#define traceopen(file) ((void)0)
#define traceclose() ((void)0)
#define tracelevel(level) ((void)0)
#define gettracelevel() 0

#define trace(level, ...) ((void)0)
#define tracet(level, ...) ((void)0)
#define tracemat(level, ...) ((void)0)
#define traceobs(level, ...) ((void)0)
#define tracenav(level, ...) ((void)0)
#define tracegnav(level, ...) ((void)0)
#define tracehnav(level, ...) ((void)0)
#define tracepeph(level, ...) ((void)0)
#define tracepclk(level, ...) ((void)0)
#define traceb(level, ...) ((void)0)

#endif /* TRACE */

/* Platform dependent functions ----------------------------------------------*/
EXPORT int execcmd(const char *cmd);
EXPORT int expath(const char *path, char *paths[], size_t size, int nmax);
EXPORT void createdir(const char *path);

/* Positioning models --------------------------------------------------------*/
EXPORT double satazel(const double *pos, const double *e, double *azel);
EXPORT double geodist(const double *rs, const double *rr, double *e);
EXPORT void dops(int ns, const double *azel, double elmin, double *dop);

/* Atmosphere models ---------------------------------------------------------*/
EXPORT double ionmodel(gtime_t t, const double *ion, const double *pos, const double *azel);
EXPORT double ionmapf(const double *pos, const double *azel);
EXPORT double ionppp(const double *pos, const double *azel, double re, double hion, double *pppos);
EXPORT double tropmodel(gtime_t time, const double *pos, const double *azel, double humi);
EXPORT double tropmapf(gtime_t time, const double *pos, const double *azel, double *mapfw);
EXPORT bool iontec(gtime_t time, const nav_t *nav, const double *pos, const double *azel, int opt,
                   double *delay, double *var);
EXPORT void readtec(const char *file, nav_t *nav, int opt);
EXPORT bool ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos, const double *azel,
                     int ionoopt, double *ion, double *var);
EXPORT bool tropcorr(gtime_t time, const nav_t *nav, const double *pos, const double *azel,
                     int tropopt, double *trp, double *var);
EXPORT int seliflc(int optnf, int sys);

/* Antenna models ------------------------------------------------------------*/
EXPORT bool readpcv(const char *file, pcvs_t *pcvs);
EXPORT pcv_t *searchpcv(int sat, const char *type, gtime_t time, const pcvs_t *pcvs);
EXPORT void antmodel(const pcv_t *pcv, const double *del, const double *azel, int opt,
                     double *dant);
EXPORT void antmodel_s(const pcv_t *pcv, double nadir, double *dant);

/* Earth tide models ---------------------------------------------------------*/
EXPORT void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun, double *rmoon, double *gmst);
EXPORT void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp, const double *odisp,
                     double *dr);

/* Geoid models --------------------------------------------------------------*/
EXPORT bool opengeoid(int model, const char *file);
EXPORT void closegeoid(void);
EXPORT double geoidh(const double *pos);

/* Datum transformation ------------------------------------------------------*/
EXPORT int loaddatump(const char *file);
EXPORT int tokyo2jgd(double *pos);
EXPORT int jgd2tokyo(double *pos);

/* RINEX functions -----------------------------------------------------------*/
EXPORT int readrnx(const char *file, int rcv, const char *opt, obs_t *obs, nav_t *nav, sta_t *sta);
EXPORT int readrnxt(const char *file, int rcv, gtime_t ts, gtime_t te, double tint, const char *opt,
                    obs_t *obs, nav_t *nav, sta_t *sta);
EXPORT int readrnxc(const char *file, nav_t *nav);
EXPORT bool outrnxobsh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxobsb(FILE *fp, const rnxopt_t *opt, const obsd_t *obs, int n, int epflag);
EXPORT bool outrnxnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxgnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxhnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxlnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxqnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxcnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxinavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav);
EXPORT bool outrnxnavb(FILE *fp, const rnxopt_t *opt, const eph_t *eph);
EXPORT bool outrnxgnavb(FILE *fp, const rnxopt_t *opt, const geph_t *geph);
EXPORT bool outrnxhnavb(FILE *fp, const rnxopt_t *opt, const seph_t *seph);
EXPORT int rtk_uncompress(const char *file, char *uncfile, size_t size);
EXPORT int convrnx(int format, rnxopt_t *opt, const char *file, char **ofile);
EXPORT bool init_rnxctr(rnxctr_t *rnx);
EXPORT void free_rnxctr(rnxctr_t *rnx);
EXPORT int open_rnxctr(rnxctr_t *rnx, FILE *fp);
EXPORT int input_rnxctr(rnxctr_t *rnx, FILE *fp);

/* Ephemeris and clock functions ---------------------------------------------*/
EXPORT double eph2clk(gtime_t time, const eph_t *eph);
EXPORT double geph2clk(gtime_t time, const geph_t *geph);
EXPORT double seph2clk(gtime_t time, const seph_t *seph);
EXPORT void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts, double *var);
EXPORT void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts, double *var);
EXPORT void seph2pos(gtime_t time, const seph_t *seph, double *rs, double *dts, double *var);
EXPORT bool peph2pos(gtime_t time, int sat, const nav_t *nav, int opt, double *rs, double *dts,
                     double *var);
EXPORT void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav, double *dant);
EXPORT bool satpos(gtime_t time, gtime_t teph, int sat, int ephopt, const nav_t *nav, double *rs,
                   double *dts, double *var, int *svh);
EXPORT void satposs(gtime_t time, const obsd_t *obs, int n, const nav_t *nav, int sateph,
                    double *rs, double *dts, double *var, int *svh);
EXPORT void setseleph(int sys, int sel);
EXPORT int getseleph(int sys);
EXPORT void readsp3(const char *file, nav_t *nav, int opt);
EXPORT bool readsap(const char *file, gtime_t time, nav_t *nav);
EXPORT bool readdcb(const char *file, nav_t *nav, const sta_t *sta);
EXPORT int code2bias_ix(const int sys, const int code);
/*EXPORT int readfcb(const char *file, nav_t *nav);*/
EXPORT void alm2pos(gtime_t time, const alm_t *alm, double *rs, double *dts);

EXPORT bool tle_read(const char *file, tle_t *tle);
EXPORT bool tle_name_read(const char *file, tle_t *tle);
EXPORT bool tle_pos(gtime_t time, const char *name, const char *satno, const char *desig,
                    const tle_t *tle, const erp_t *erp, double *rs);

/* Receiver raw data functions -----------------------------------------------*/
EXPORT uint32_t getbitu(const uint8_t *buff, size_t size, unsigned pos, unsigned len);
EXPORT int32_t getbits(const uint8_t *buff, size_t size, unsigned pos, unsigned len);
EXPORT void setbitu(uint8_t *buff, size_t size, unsigned pos, unsigned len, uint32_t data);
EXPORT void setbits(uint8_t *buff, size_t size, unsigned pos, unsigned len, int32_t data);
EXPORT uint32_t rtk_crc32(const uint8_t *buff, size_t size, unsigned len);
EXPORT uint32_t rtk_crc24q(const uint8_t *buff, size_t size, unsigned len);
EXPORT uint16_t rtk_crc16(const uint8_t *buff, size_t size, unsigned len);
EXPORT bool decode_word(uint32_t word, uint8_t data[4]);
EXPORT bool decode_frame(const uint8_t *buff, size_t size, eph_t *eph, alm_t *alm, double *ion,
                         double *utc);
EXPORT bool test_glostr(const uint8_t *buff, size_t size);
EXPORT bool decode_glostr(const uint8_t *buff, size_t size, geph_t *geph, double *utc);
EXPORT bool decode_bds_d1(const uint8_t *buff, size_t size, eph_t *eph, double *ion, double *utc);
EXPORT bool decode_bds_d2(const uint8_t *buff, size_t size, eph_t *eph, double *utc);
EXPORT bool decode_gal_inav(const uint8_t *buff, size_t size, eph_t *eph, double *ion, double *utc);
EXPORT bool decode_gal_fnav(const uint8_t *buff, size_t size, eph_t *eph, double *ion, double *utc);
EXPORT bool decode_irn_nav(const uint8_t *buff, size_t size, eph_t *eph, double *ion, double *utc);

EXPORT int init_raw(raw_t *raw, int format);
EXPORT void free_raw(raw_t *raw);
EXPORT int input_raw(raw_t *raw, int format, uint8_t data);
EXPORT int input_rawf(raw_t *raw, int format, FILE *fp);

EXPORT int init_rt17(raw_t *raw);
EXPORT int init_cmr(raw_t *raw);
EXPORT void free_rt17(raw_t *raw);
EXPORT void free_cmr(raw_t *raw);
EXPORT int update_cmr(raw_t *raw, rtksvr_t *svr, obs_t *obs);

EXPORT int input_oem4(raw_t *raw, uint8_t data);
EXPORT int input_cnav(raw_t *raw, uint8_t data);
EXPORT int input_ubx(raw_t *raw, uint8_t data);
EXPORT int input_sbp(raw_t *raw, uint8_t data);
EXPORT int input_cres(raw_t *raw, uint8_t data);
EXPORT int input_stq(raw_t *raw, uint8_t data);
EXPORT int input_javad(raw_t *raw, uint8_t data);
EXPORT int input_nvs(raw_t *raw, uint8_t data);
EXPORT int input_bnx(raw_t *raw, uint8_t data);
EXPORT int input_rt17(raw_t *raw, uint8_t data);
EXPORT int input_sbf(raw_t *raw, uint8_t data);
EXPORT int input_tersus(raw_t *raw, uint8_t data);
EXPORT int input_oem4f(raw_t *raw, FILE *fp);
EXPORT int input_cnavf(raw_t *raw, FILE *fp);
EXPORT int input_ubxf(raw_t *raw, FILE *fp);
EXPORT int input_sbpf(raw_t *raw, FILE *fp);
EXPORT int input_cresf(raw_t *raw, FILE *fp);
EXPORT int input_stqf(raw_t *raw, FILE *fp);
EXPORT int input_javadf(raw_t *raw, FILE *fp);
EXPORT int input_nvsf(raw_t *raw, FILE *fp);
EXPORT int input_bnxf(raw_t *raw, FILE *fp);
EXPORT int input_rt17f(raw_t *raw, FILE *fp);
EXPORT int input_sbff(raw_t *raw, FILE *fp);
EXPORT int input_tersusf(raw_t *raw, FILE *fp);

EXPORT int gen_ubx(const char *msg, uint8_t *buff, size_t size);
EXPORT int gen_stq(const char *msg, uint8_t *buff, size_t size);
EXPORT int gen_nvs(const char *msg, uint8_t *buff, size_t size);

/* RTCM functions ------------------------------------------------------------*/
EXPORT bool init_rtcm(rtcm_t *rtcm);
EXPORT void free_rtcm(rtcm_t *rtcm);
EXPORT int input_rtcm2(rtcm_t *rtcm, uint8_t data);
EXPORT int input_rtcm3(rtcm_t *rtcm, uint8_t data);
EXPORT int input_rtcm2f(rtcm_t *rtcm, FILE *fp);
EXPORT int input_rtcm3f(rtcm_t *rtcm, FILE *fp);
EXPORT bool gen_rtcm2(rtcm_t *rtcm, int type, int sync);
EXPORT bool gen_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync);

/* Solution functions --------------------------------------------------------*/
EXPORT bool initsolbuf(solbuf_t *solbuf, int cyclic, int nmax);
EXPORT void freesolbuf(solbuf_t *solbuf);
EXPORT void freesolstatbuf(solstatbuf_t *solstatbuf);
EXPORT sol_t *getsol(solbuf_t *solbuf, int index);
EXPORT bool addsol(solbuf_t *solbuf, const sol_t *sol);
EXPORT bool readsol(const char *files[], int nfile, solbuf_t *sol);
EXPORT bool readsolt(const char *files[], int nfile, gtime_t ts, gtime_t te, double tint, int qflag,
                     solbuf_t *sol);
EXPORT bool readsolstat(const char *files[], int nfile, solstatbuf_t *statbuf);
EXPORT bool readsolstatt(const char *files[], int nfile, gtime_t ts, gtime_t te, double tint,
                         solstatbuf_t *statbuf);
EXPORT int inputsol(uint8_t data, gtime_t ts, gtime_t te, double tint, int qflag,
                    const solopt_t *opt, solbuf_t *solbuf);

EXPORT void outprcopts(char *buff, size_t size, const prcopt_t *opt);
EXPORT void outsolheads(char *buff, size_t size, const solopt_t *opt);
EXPORT void outsols(char *buff, size_t size, const sol_t *sol, const double *rb,
                    const solopt_t *opt);
EXPORT void outsolexs(char *buff, size_t size, const sol_t *sol, const ssat_t *ssat,
                      const solopt_t *opt);
EXPORT void outprcopt(FILE *fp, const prcopt_t *opt);
EXPORT void outsolhead(FILE *fp, const solopt_t *opt);
EXPORT void outsol(FILE *fp, const sol_t *sol, const double *rb, const solopt_t *opt);
EXPORT void outsolex(FILE *fp, const sol_t *sol, const ssat_t *ssat, const solopt_t *opt);
EXPORT void outnmea_rmc(char *buff, size_t size, const sol_t *sol);
EXPORT void outnmea_gga(char *buff, size_t size, const sol_t *sol);
EXPORT void outnmea_gsa(char *buff, size_t size, const sol_t *sol, const ssat_t *ssat);
EXPORT void outnmea_gsv(char *buff, size_t size, const sol_t *sol, const ssat_t *ssat);

/* Google earth KML converter ------------------------------------------------*/
EXPORT int convkml(const char *infile, const char *outfile, gtime_t ts, gtime_t te, double tint,
                   int qflg, const double *offset, int tcolor, int pcolor, int outalt, int outtime);

/* GPX converter -------------------------------------------------------------*/
EXPORT int convgpx(const char *infile, const char *outfile, gtime_t ts, gtime_t te, double tint,
                   int qflg, const double *offset, int outtrk, int outpnt, int outalt, int outtime);

/* SBAS functions ------------------------------------------------------------*/
EXPORT int sbsreadmsg(const char *file, int sel, sbs_t *sbs);
EXPORT int sbsreadmsgt(const char *file, int sel, gtime_t ts, gtime_t te, sbs_t *sbs);
EXPORT void sbsoutmsg(FILE *fp, sbsmsg_t *sbsmsg);
EXPORT bool sbsdecodemsg(gtime_t time, int prn, const uint32_t *words, sbsmsg_t *sbsmsg);
EXPORT int sbsupdatecorr(const sbsmsg_t *msg, nav_t *nav);
EXPORT bool sbssatcorr(gtime_t time, int sat, const nav_t *nav, double *rs, double *dts,
                       double *var);
EXPORT bool sbsioncorr(gtime_t time, const nav_t *nav, const double *pos, const double *azel,
                       double *delay, double *var);
EXPORT double sbstropcorr(gtime_t time, const double *pos, const double *azel, double *var);

/* Options functions ---------------------------------------------------------*/
EXPORT opt_t *searchopt(const char *name, const opt_t *opts);
EXPORT bool str2opt(opt_t *opt, const char *str);
EXPORT void opt2str(const opt_t *opt, char *str, size_t size);
EXPORT void opt2buf(const opt_t *opt, char *buff, size_t size);
EXPORT bool loadopts(const char *file, opt_t *opts);
EXPORT bool saveopts(const char *file, const char *mode, const char *comment, const opt_t *opts);
EXPORT void resetsysopts(void);
EXPORT void getsysopts(prcopt_t *popt, solopt_t *sopt, filopt_t *fopt);
EXPORT void setsysopts(const prcopt_t *popt, const solopt_t *sopt, const filopt_t *fopt);

/* Stream data input and output functions ------------------------------------*/
EXPORT void strinitcom(void);
EXPORT void strinit(stream_t *stream);
EXPORT void strlock(stream_t *stream);
EXPORT void strunlock(stream_t *stream);
EXPORT bool stropen(stream_t *stream, int type, int mode, const char *path);
EXPORT void strclose(stream_t *stream);
EXPORT int strread(stream_t *stream, uint8_t *buff, int n);
EXPORT int strwrite(stream_t *stream, uint8_t *buff, int n);
EXPORT void strsync(stream_t *stream1, stream_t *stream2);
EXPORT int strstat(stream_t *stream, char *msg, size_t msize);
EXPORT int strstatx(stream_t *stream, char *msg, size_t msize);
EXPORT void strsum(stream_t *stream, int *inb, int *inr, int *outb, int *outr);
EXPORT void strsetopt(const int *opt);
EXPORT gtime_t strgettime(stream_t *stream);
EXPORT void strsendnmea(stream_t *stream, const sol_t *sol);
EXPORT void strsendcmd(stream_t *stream, const char *cmd);
EXPORT void strsettimeout(stream_t *stream, int toinact, int tirecon);
EXPORT void strsetdir(const char *dir);
EXPORT void strsetproxy(const char *addr);

/* Integer ambiguity resolution ----------------------------------------------*/
EXPORT int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);
EXPORT int lambda_reduction(int n, const double *Q, double *Z);
EXPORT int lambda_search(int n, int m, const double *a, const double *Q, double *F, double *s);

/* Standard positioning ------------------------------------------------------*/
EXPORT bool pntpos(const obsd_t *obs, int n, const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                   double *azel, ssat_t *ssat, char *msg, size_t msize);

/* Precise positioning -------------------------------------------------------*/
EXPORT void rtkinit(rtk_t *rtk, const prcopt_t *opt);
EXPORT void rtkfree(rtk_t *rtk);
EXPORT bool rtkpos(rtk_t *rtk, const obsd_t *obs, int nobs, const nav_t *nav);
EXPORT bool rtkopenstat(const char *file, int level);
EXPORT void rtkclosestat(void);
EXPORT void rtkoutstat(rtk_t *rtk, int level, char *buff, size_t size);

/* Precise point positioning -------------------------------------------------*/
EXPORT void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);
EXPORT int pppnx(const prcopt_t *opt);
EXPORT void pppoutstat(rtk_t *rtk, char *buff, size_t size);

EXPORT int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc, const nav_t *nav,
                  const double *azel, double *x, double *P);

/* Post-processing positioning -----------------------------------------------*/
EXPORT int postpos(gtime_t ts, gtime_t te, double ti, double tu, const prcopt_t *popt,
                   const solopt_t *sopt, const filopt_t *fopt, const char **infile, int n,
                   const char *outfile, const char *rov, const char *base);

/* Stream server functions ---------------------------------------------------*/
EXPORT void strsvrinit(strsvr_t *svr, int nout);
EXPORT bool strsvrstart(strsvr_t *svr, const int *opts, const int *strs, const char **paths,
                        const char **logs, strconv_t **conv, const char **cmds,
                        const char **cmds_periodic, const double *nmeapos);
EXPORT void strsvrstop(strsvr_t *svr, const char **cmds);
EXPORT void strsvrstat(strsvr_t *svr, int *stat, int *log_stat, int *byte, int *bps, char *msg,
                       size_t msize);
EXPORT strconv_t *strconvnew(int itype, int otype, const char *msgs, int staid, int stasel,
                             const char *opt);
EXPORT void strconvfree(strconv_t *conv);

/* Rtk server functions ------------------------------------------------------*/
EXPORT bool rtksvrinit(rtksvr_t *svr);
EXPORT void rtksvrfree(rtksvr_t *svr);
EXPORT bool rtksvrstart(rtksvr_t *svr, int cycle, int buffsize, const int *strs, const char **paths,
                        const int *formats, int navsel, const char **cmds,
                        const char **cmds_periodic, const char **rcvopts, int nmeacycle,
                        int nmeareq, const double *nmeapos, const prcopt_t *prcopt,
                        const solopt_t *solopt, stream_t *moni, char *errmsg, size_t msize);
EXPORT void rtksvrstop(rtksvr_t *svr, const char **cmds);
EXPORT bool rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path,
                          const solopt_t *solopt);
EXPORT void rtksvrclosestr(rtksvr_t *svr, int index);
EXPORT void rtksvrlock(rtksvr_t *svr);
EXPORT void rtksvrunlock(rtksvr_t *svr);
EXPORT int rtksvrostat(rtksvr_t *svr, int type, gtime_t *time, int *sat, double *az, double *el,
                       int **snr, int *vsat);
EXPORT void rtksvrsstat(rtksvr_t *svr, int *sstat, char *msg, size_t msize);
EXPORT bool rtksvrmark(rtksvr_t *svr, const char *name, const char *comment);

/* Downloader functions ------------------------------------------------------*/
EXPORT int dl_readurls(const char *file, const char **types, int ntype, url_t *urls, int nmax);
EXPORT int dl_readstas(const char *file, char **stas, size_t size, int nmax);
EXPORT int dl_exec(gtime_t ts, gtime_t te, double ti, int seqnos, int seqnoe, const url_t *urls,
                   int nurl, const char **stas, int nsta, const char *dir, const char *usr,
                   const char *pwd, const char *proxy, int opts, char *msg, size_t msize, FILE *fp);
EXPORT void dl_test(gtime_t ts, gtime_t te, double ti, const url_t *urls, int nurl,
                    const char **stas, int nsta, const char *dir, int ncol, int datefmt, FILE *fp);

/* GIS data functions --------------------------------------------------------*/
EXPORT int gis_read(const char *file, gis_t *gis, int layer);
EXPORT void gis_free(gis_t *gis);

/* Application defined functions ---------------------------------------------*/
extern int showmsg(const char *format, ...);
extern void settspan(gtime_t ts, gtime_t te);
extern void settime(gtime_t time);

#ifdef __cplusplus
}
#endif
#endif /* RTKLIB_H */
