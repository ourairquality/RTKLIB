/*------------------------------------------------------------------------------
 * rtkcmn.c : RTKLIB common functions
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DLAPACK   use LAPACK/BLAS
 *           -DMKL      use Intel MKL
 *           -DTRACE    enable debug trace
 *           -DWIN32    use WIN32 API
 *           -DNOCALLOC no use calloc for zero matrix
 *           -DIERS_MODEL use GMF instead of NMF
 *           -DDLL      built for shared library
 *           -DCPUTIME_IN_GPST cputime operated in GPST
 *
 * References :
 *     [1] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
 *         7 March, 2006
 *     [2] RTCA/DO-229C, Minimum operational performance standards for global
 *         positioning system/wide area augmentation system airborne equipment,
 *         November 28, 2001
 *     [3] M.Rothacher, R.Schmid, ANTEX: The Antenna Exchange Format Version 1.4,
 *         15 September, 2010
 *     [4] A.Gelb ed., Applied Optimal Estimation, The M.I.T Press, 1974
 *     [5] A.E.Niell, Global mapping functions for the atmosphere delay at radio
 *         wavelengths, Journal of geophysical research, 1996
 *     [6] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
 *         Version 3.00, November 28, 2007
 *     [7] J.Kouba, A Guide to using International GNSS Service (IGS) products,
 *         May 2009
 *     [8] China Satellite Navigation Office, BeiDou navigation satellite system
 *         signal in space interface control document, open service signal B1I
 *         (version 1.0), Dec 2012
 *     [9] J.Boehm, A.Niell, P.Tregoning and H.Shuh, Global Mapping Function
 *         (GMF): A new empirical mapping function base on numerical weather
 *         model data, Geophysical Research Letters, 33, L07304, 2006
 *     [10] GLONASS/GPS/Galileo/Compass/SBAS NV08C receiver series BINR interface
 *         protocol specification ver.1.3, August, 2012
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * History : 2007/01/12 1.0 new
 *           2007/03/06 1.1 input initial rover pos of pntpos()
 *                          update only effective states of filter()
 *                          fix bug of atan2() domain error
 *           2007/04/11 1.2 add function antmodel()
 *                          add gdop mask for pntpos()
 *                          change constant MAXDTOE value
 *           2007/05/25 1.3 add function execcmd(),expandpath()
 *           2008/06/21 1.4 add funciton sortobs(),uniqeph(),screent()
 *                          replace geodist() by sagnac correction way
 *           2008/10/29 1.5 fix bug of ionospheric mapping function
 *                          fix bug of seasonal variation term of tropmapf
 *           2008/12/27 1.6 add function tickget(), sleepms(), tracenav(),
 *                          xyz2enu(), satposv(), pntvel(), covecef()
 *           2009/03/12 1.7 fix bug on error-stop when localtime() returns NULL
 *           2009/03/13 1.8 fix bug on time adjustment for summer time
 *           2009/04/10 1.9 add function adjgpsweek(),getbits(),getbitu()
 *                          add function geph2pos()
 *           2009/06/08 1.10 add function seph2pos()
 *           2009/11/28 1.11 change function pntpos()
 *                           add function tracegnav(),tracepeph()
 *           2009/12/22 1.12 change default parameter of ionos std
 *                           valid under second for timeget()
 *           2010/07/28 1.13 fix bug in tropmapf()
 *                           added api:
 *                               obs2code(),code2obs(),cross3(),normv3(),
 *                               gst2time(),time2gst(),time_str(),timeset(),
 *                               deg2dms(),dms2deg(),searchpcv(),antmodel_s(),
 *                               tracehnav(),tracepclk(),reppath(),reppaths(),
 *                               createdir()
 *                           changed api:
 *                               readpcv(),
 *                           deleted api:
 *                               uniqeph()
 *           2010/08/20 1.14 omit to include mkl header files
 *                           fix bug on chi-sqr(n) table
 *           2010/12/11 1.15 added api:
 *                               freeobs(),freenav(),ionppp()
 *           2011/05/28 1.16 fix bug on half-hour offset by time2epoch()
 *                           added api:
 *                               uniqnav()
 *           2012/06/09 1.17 add a leap second after 2012-6-30
 *           2012/07/15 1.18 add api setbits(),setbitu(),utc2gmst()
 *                           fix bug on interpolation of antenna pcv
 *                           fix bug on str2num() for string with over 256 char
 *                           add api readblq(),satexclude(),setcodepri(),
 *                           getcodepri()
 *                           change api obs2code(),code2obs(),antmodel()
 *           2012/12/25 1.19 fix bug on satwavelen(),code2obs(),obs2code()
 *                           add api testsnr()
 *           2013/01/04 1.20 add api gpst2bdt(),bdt2gpst(),bdt2time(),time2bdt()
 *                           readblq(),readerp(),geterp(),crc16()
 *                           change api eci2ecef(),sunmoonpos()
 *           2013/03/26 1.21 tickget() uses clock_gettime() for Linux
 *           2013/05/08 1.22 fix bug on nutation coefficients for ast_args()
 *           2013/06/02 1.23 add #ifdef for undefined CLOCK_MONOTONIC_RAW
 *           2013/09/01 1.24 fix bug on interpolation of satellite antenna pcv
 *           2013/09/06 1.25 fix bug on extrapolation of erp
 *           2014/04/27 1.26 add SYS_LEO for satellite system
 *                           add BDS L1 code for RINEX 3.02 and RTCM 3.2
 *                           support BDS L1 in satwavelen()
 *           2014/05/29 1.27 fix bug on obs2code() to search obs code table
 *           2014/08/26 1.28 fix problem on output of uncompress() for tar file
 *                           add function to swap trace file with keywords
 *           2014/10/21 1.29 strtok() -> strtok_r() in expath() for thread-safe
 *                           add bdsmodear in procopt_default
 *           2015/03/19 1.30 fix bug on interpolation of erp values in geterp()
 *                           add leap second insertion before 2015/07/01 00:00
 *                           add api read_leaps()
 *           2015/05/31 1.31 delete api windupcorr()
 *           2015/08/08 1.32 add compile option CPUTIME_IN_GPST
 *                           add api add_fatal()
 *                           support usno leapsec.dat for api read_leaps()
 *           2016/01/23 1.33 enable septentrio
 *           2016/02/05 1.34 support GLONASS for savenav(), loadnav()
 *           2016/06/11 1.35 delete trace() in reppath() to avoid deadlock
 *           2016/07/01 1.36 support IRNSS
 *                           add leap second before 2017/1/1 00:00:00
 *           2016/07/29 1.37 rename api compress() -> rtk_uncompress()
 *                           rename api crc16()    -> rtk_crc16()
 *                           rename api crc24q()   -> rtk_crc24q()
 *                           rename api crc32()    -> rtk_crc32()
 *           2016/08/20 1.38 fix type incompatibility in win64 environment
 *                           change constant _POSIX_C_SOURCE 199309 -> 199506
 *           2016/08/21 1.39 fix bug on week overflow in time2gpst()/gpst2time()
 *           2016/09/05 1.40 fix bug on invalid nav data read in readnav()
 *           2016/09/17 1.41 suppress warnings
 *           2016/09/19 1.42 modify api deg2dms() to consider numerical error
 *           2017/04/11 1.43 delete EXPORT for global variables
 *           2018/10/10 1.44 modify api satexclude()
 *           2020/11/30 1.45 add API code2idx() to get freq-index
 *                           add API code2freq() to get carrier frequency
 *                           add API timereset() to reset current time
 *                           modify API obs2code(), code2obs() and setcodepri()
 *                           delete API satwavelen()
 *                           delete API csmooth()
 *                           delete global variable lam_carr[]
 *                           compensate L3,L4,... PCVs by L2 PCV if no PCV data
 *                            in input file by API readpcv()
 *                           add support hatanaka-compressed RINEX files with
 *                            extension ".crx" or ".CRX"
 *                           update stream format strings table
 *                           update obs code strings and priority table
 *                           use integer types in stdint.h
 *                           suppress warnings
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#ifndef WIN32
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#endif
#include "rtklib.h"

/* Constants -----------------------------------------------------------------*/

#define POLYCRC32 0xEDB88320u /* CRC32 polynomial */
#define POLYCRC24Q 0x1864CFBu /* CRC24Q polynomial */

#define SQR(x) ((x) * (x))
#define MAX_VAR_EPH SQR(300.0) /* Max variance eph to reject satellite (m^2) */

static const double gpst0[] = {1980, 1, 6, 0, 0, 0}; /* GPS time reference */
static const double gst0[] = {1999, 8, 22, 0, 0, 0}; /* Galileo system time reference */
static const double bdt0[] = {2006, 1, 1, 0, 0, 0};  /* BeiDou time reference */

static double leaps[MAXLEAPS + 1][7] = {/* Leap seconds (y,m,d,h,m,s,utc-gpst) */
                                        {2017, 1, 1, 0, 0, 0, -18},
                                        {2015, 7, 1, 0, 0, 0, -17},
                                        {2012, 7, 1, 0, 0, 0, -16},
                                        {2009, 1, 1, 0, 0, 0, -15},
                                        {2006, 1, 1, 0, 0, 0, -14},
                                        {1999, 1, 1, 0, 0, 0, -13},
                                        {1997, 7, 1, 0, 0, 0, -12},
                                        {1996, 1, 1, 0, 0, 0, -11},
                                        {1994, 7, 1, 0, 0, 0, -10},
                                        {1993, 7, 1, 0, 0, 0, -9},
                                        {1992, 7, 1, 0, 0, 0, -8},
                                        {1991, 1, 1, 0, 0, 0, -7},
                                        {1990, 1, 1, 0, 0, 0, -6},
                                        {1988, 1, 1, 0, 0, 0, -5},
                                        {1985, 7, 1, 0, 0, 0, -4},
                                        {1983, 7, 1, 0, 0, 0, -3},
                                        {1982, 7, 1, 0, 0, 0, -2},
                                        {1981, 7, 1, 0, 0, 0, -1},
                                        {0}};
const double chisqr[100] = {/* chi-sqr(n) (alpha=0.001) */
                            10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6, 31.3, 32.9,
                            34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3, 46.8, 48.3, 49.7, 51.2,
                            52.6, 54.1, 55.5, 56.9, 58.3, 59.7, 61.1, 62.5, 63.9, 65.2, 66.6, 68.0,
                            69.3, 70.7, 72.1, 73.4, 74.7, 76.0, 77.3, 78.6, 80.0, 81.3, 82.6, 84.0,
                            85.4, 86.7, 88.0, 89.3, 90.6, 91.9, 93.3, 94.7, 96.0, 97.4, 98.7, 100,
                            101,  102,  103,  104,  105,  107,  108,  109,  110,  112,  113,  114,
                            115,  116,  118,  119,  120,  122,  123,  125,  126,  127,  128,  129,
                            131,  132,  133,  134,  135,  137,  138,  139,  140,  142,  143,  144,
                            145,  147,  148,  149};
const prcopt_t prcopt_default = {
    /* Defaults processing options */
    PMODE_KINEMA,
    SOLTYPE_FORWARD, /* Mode,soltype */
    2,
    SYS_GPS | SYS_GLO | SYS_GAL, /* Nf, navsys */
    15.0 * D2R,
    {{0, 0}}, /* Elmin,snrmask */
    0,
    3,
    3,
    1,
    0,
    1, /* Sateph,modear,glomodear,gpsmodear,bdsmodear,arfilter */
    20,
    0,
    4,
    5,
    10,
    20, /* Maxout,minlock,minfixsats,minholdsats,mindropsats,minfix */
    1,
    1,
    1,
    1,
    0, /* Armaxiter,estion,esttrop,dynamics,tidecorr */
    1,
    0,
    0,
    0,
    0, /* Niter,codesmooth,intpref,sbascorr,sbassatsel */
    0,
    0,                                               /* Rovpos,refpos */
    {300.0, 300.0, 300.0},                           /* eratio[] */
    {100.0, 0.003, 0.003, 0.0, 1.0, 52.0, 0.0, 0.0}, /* err[-,base,el,bl,dop,snr_max,snr,rcverr] */
    {30.0, 0.03, 0.3},                               /* std[] */
    {1E-4, 1E-3, 1E-4, 1E-1, 1E-2, 0.0},             /* prn[] */
    5E-12,                                           /* Sclkstab */
    {3.0, 0.25, 0.0, 1E-9, 1E-5, 3.0, 3.0, 0.0},     /* Thresar */
    0.0,
    0.0,
    0.05,
    0, /* Elmaskar,elmaskhold,thresslip,thresdop, */
    0.1,
    0.01,
    30.0,        /* Varholdamb,gainholdamb,maxtdif */
    {5.0, 30.0}, /* Maxinno {phase,code} */
    {0},
    {0},
    {0},      /* Baseline,ru,rb */
    {"", ""}, /* Anttype */
    {{0}},
    {{0}},
    {0}, /* Antdel,pcv,exsats */
    1,
    1 /* Maxaveep,initrst */
};
const solopt_t solopt_default = {
    /* Defaults solution output options */
    SOLF_LLH,   TIMES_GPST, 1, 3,          /* Posf,times,timef,timeu */
    0,          1,          0, 0, 0, 0, 0, /* Degf,outhead,outopt,outvel,datum,height,geoid */
    0,          0,          0,             /* Solstatic,sstat,trace */
    {0.0, 0.0},                            /* Nmeaintv */
    " ",        ""                         /* Separator/program name */
};
const char *formatstrs[32] = {                        /* Stream format strings */
                              "RTCM 2",               /*  0 */
                              "RTCM 3",               /*  1 */
                              "NovAtel OEM7",         /*  2 */
                              "ComNav",               /*  3 */
                              "u-blox UBX",           /*  4 */
                              "Swift Navigation SBP", /*  5 */
                              "Hemisphere",           /*  6 */
                              "SkyTraq",              /*  7 */
                              "Javad GREIS",          /*  8 */
                              "NVS BINR",             /*  9 */
                              "BINEX",                /* 10 */
                              "Trimble RT17",         /* 11 */
                              "Septentrio SBF",       /* 12 */
                              "Tersus",               /* 13 */
                              "RINEX",                /* 14 */
                              "SP3",                  /* 15 */
                              "RINEX CLK",            /* 16 */
                              "SBAS",                 /* 17 */
                              "NMEA 0183",            /* 18 */
                              "TERSUS",               /* 19 */
                              NULL};

static char *obscodes[] = {
    /* Observation code strings */

    "",   "1C", "1P", "1W", "1Y", "1M", "1N", "1S", "1L", "1E", /*  0- 9 */
    "1A", "1B", "1X", "1Z", "2C", "2D", "2S", "2L", "2X", "2P", /* 10-19 */
    "2W", "2Y", "2M", "2N", "5I", "5Q", "5X", "7I", "7Q", "7X", /* 20-29 */
    "6A", "6B", "6C", "6X", "6Z", "6S", "6L", "8L", "8Q", "8X", /* 30-39 */
    "2I", "2Q", "6I", "6Q", "3I", "3Q", "3X", "1I", "1Q", "5A", /* 40-49 */
    "5B", "5C", "9A", "9B", "9C", "9X", "1D", "5D", "5P", "5Z", /* 50-59 */
    "6E", "7D", "7P", "7Z", "8D", "8P", "4A", "4B", "4X", ""    /* 60-69 */
};
static char codepris[7][MAXFREQ][16] = {
    /* Code priority for each freq-index */
    /* L1/E1/B1I L2/E5b/B2I L5/E5a/B3I E6/LEX/B2A E5(a+b)         */
    {"CPYWMNSLX", "CPYWMNDLSX", "IQX", "", "", ""},  /* GPS */
    {"CPABX", "CPABX", "IQX", "", "", ""},           /* GLO */
    {"CABXZ", "XIQ", "XIQ", "ABCXZ", "IQX", ""},     /* GAL */
    {"CLSXZ", "LSX", "IQXDPZ", "LSXEZ", "", ""},     /* QZS */
    {"C", "IQX", "", "", "", ""},                    /* SBS */
    {"IQXDPAN", "IQXDPZ", "IQXA", "DPX", "DPX", ""}, /* BDS */
    {"ABCX", "ABCX", "", "", "", ""}                 /* IRN */
};
static fatalfunc_t *fatalfunc = NULL; /* Fatal callback function */

/* Crc tables generated by util/gencrc ---------------------------------------*/
static const uint16_t tbl_CRC16[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B,
    0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738,
    0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
    0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
    0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB,
    0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
    0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D,
    0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
    0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};
static const uint32_t tbl_CRC24Q[] = {
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17, 0xA18139,
    0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272,
    0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B, 0xEE1ABD,
    0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F,
    0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631,
    0xB8FACA, 0xB4633C, 0x322FC7, 0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A,
    0xD0AC8C, 0x56E077, 0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5,
    0xF7614E, 0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
    0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC, 0xDCED5B,
    0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C, 0x7D6C62, 0xFB2099,
    0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0, 0x9FA736,
    0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C, 0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4,
    0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53,
    0xC596A8, 0xC90F5E, 0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791,
    0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448,
    0xAC38B3, 0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
    0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814,
    0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6,
    0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A, 0x9E1774, 0x185B8F, 0x14C279,
    0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863, 0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132,
    0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C,
    0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C,
    0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3,
    0x141A58, 0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
    0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1, 0x26359F,
    0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88, 0x87B4A6, 0x01F85D,
    0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED, 0x69AE1B,
    0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401, 0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9,
    0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};
/* Function prototypes -------------------------------------------------------*/
#ifdef MKL
#define LAPACK
#define dgemm_ dgemm
#define dgetrf_ dgetrf
#define dgetri_ dgetri
#define dgetrs_ dgetrs
#endif
#ifdef LAPACK
extern void dgemm_(char *, char *, int *, int *, int *, double *, double *, int *, double *, int *,
                   double *, double *, int *);
extern void dgetrf_(int *, int *, double *, int *, int *, int *);
extern void dgetri_(int *, double *, int *, int *, double *, int *, int *);
extern void dgetrs_(char *, int *, int *, double *, int *, int *, double *, int *, int *);
#endif

#ifdef IERS_MODEL
extern int gmf_(double *mjd, double *lat, double *lon, double *hgt, double *zd, double *gmfh,
                double *gmfw);
#endif

/* Fatal error ---------------------------------------------------------------*/
static void fatalerr(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  char msg[1024];
  vsnprintf(msg, sizeof(msg), format, ap);
  va_end(ap);
  if (fatalfunc)
    fatalfunc(msg);
  else
    fprintf(stderr, "%s", msg);
  exit(-9);
}
/* Add fatal callback function -------------------------------------------------
 * Add fatal callback function for mat(),zeros(),imat()
 * Args   : fatalfunc_t *func I  callback function
 * Return : none
 * Notes  : if malloc() failed in return : none
 *----------------------------------------------------------------------------*/
extern void add_fatal(fatalfunc_t *func) { fatalfunc = func; }
/* Satellite system+prn/slot number to satellite number ------------------------
 * Convert satellite system+prn/slot number to satellite number
 * Args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
 *          int    prn       I   satellite prn/slot number
 * Return : satellite number (0:error)
 *----------------------------------------------------------------------------*/
extern int satno(int sys, int prn) {
  if (prn <= 0) return 0;
  switch (sys) {
    case SYS_GPS:
      if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
      return prn - MINPRNGPS + 1;
    case SYS_GLO:
      if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
      return NSATGPS + prn - MINPRNGLO + 1;
    case SYS_GAL:
      if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
      return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
    case SYS_QZS:
      if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
      return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
    case SYS_CMP:
      if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
      return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
    case SYS_IRN:
      if (prn < MINPRNIRN || MAXPRNIRN < prn) return 0;
      return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + prn - MINPRNIRN + 1;
    case SYS_LEO:
      if (prn < MINPRNLEO || MAXPRNLEO < prn) return 0;
      return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + prn - MINPRNLEO + 1;
    case SYS_SBS:
      if (prn < MINPRNSBS || MAXPRNSBS < prn) return 0;
      return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATLEO + prn - MINPRNSBS +
             1;
  }
  return 0;
}
/* Satellite number to satellite system ----------------------------------------
 * Convert satellite number to satellite system
 * Args   : int    sat       I   satellite number (1-MAXSAT)
 *          int    *prn      IO  satellite prn/slot number (NULL: no output)
 * Return : satellite system (SYS_GPS,SYS_GLO,...)
 *----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn) {
  int sys = SYS_NONE;
  if (sat <= 0 || MAXSAT < sat)
    sat = 0;
  else if (sat <= NSATGPS) {
    sys = SYS_GPS;
    sat += MINPRNGPS - 1;
  } else if ((sat -= NSATGPS) <= NSATGLO) {
    sys = SYS_GLO;
    sat += MINPRNGLO - 1;
  } else if ((sat -= NSATGLO) <= NSATGAL) {
    sys = SYS_GAL;
    sat += MINPRNGAL - 1;
  } else if ((sat -= NSATGAL) <= NSATQZS) {
    sys = SYS_QZS;
    sat += MINPRNQZS - 1;
  } else if ((sat -= NSATQZS) <= NSATCMP) {
    sys = SYS_CMP;
    sat += MINPRNCMP - 1;
  } else if ((sat -= NSATCMP) <= NSATIRN) {
    sys = SYS_IRN;
    sat += MINPRNIRN - 1;
  } else if ((sat -= NSATIRN) <= NSATLEO) {
    sys = SYS_LEO;
    sat += MINPRNLEO - 1;
  } else if ((sat -= NSATLEO) <= NSATSBS) {
    sys = SYS_SBS;
    sat += MINPRNSBS - 1;
  } else
    sat = 0;
  if (prn) *prn = sat;
  return sys;
}
/* Satellite id to satellite number --------------------------------------------
 * Convert satellite id to satellite number
 * Args   : char   *id       I   satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
 * Return : satellite number (0: error)
 * Notes  : 120-142 and 193-199 are also recognized as SBAS and QZSS
 *----------------------------------------------------------------------------*/
extern int satid2no(const char *id) {
  int prn;
  if (sscanf(id, "%d", &prn) == 1) {
    int sys;
    if (MINPRNGPS <= prn && prn <= MAXPRNGPS)
      sys = SYS_GPS;
    else if (MINPRNSBS <= prn && prn <= MAXPRNSBS)
      sys = SYS_SBS;
    else if (MINPRNQZS <= prn && prn <= MAXPRNQZS)
      sys = SYS_QZS;
    else
      return 0;
    return satno(sys, prn);
  }
  char code;
  if (sscanf(id, "%c%d", &code, &prn) < 2) return 0;

  int sys;
  switch (code) {
    case 'G':
      sys = SYS_GPS;
      prn += MINPRNGPS - 1;
      break;
    case 'R':
      sys = SYS_GLO;
      prn += MINPRNGLO - 1;
      break;
    case 'E':
      sys = SYS_GAL;
      prn += MINPRNGAL - 1;
      break;
    case 'J':
      sys = SYS_QZS;
      prn += MINPRNQZS - 1;
      break;
    case 'C':
      sys = SYS_CMP;
      prn += MINPRNCMP - 1;
      break;
    case 'I':
      sys = SYS_IRN;
      prn += MINPRNIRN - 1;
      break;
    case 'L':
      sys = SYS_LEO;
      prn += MINPRNLEO - 1;
      break;
    case 'S':
      sys = SYS_SBS;
      prn += 100;
      break;
    default:
      return 0;
  }
  return satno(sys, prn);
}
/* Satellite number to satellite id --------------------------------------------
 * Convert satellite number to satellite id
 * Args   : int    sat       I   satellite number
 *          char   *id       O   satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or nnn)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void satno2id(int sat, char id[8]) {
  int prn;
  switch (satsys(sat, &prn)) {
    case SYS_GPS:
      rtksnprintf(id, 8, "G%02d", prn - MINPRNGPS + 1);
      return;
    case SYS_GLO:
      rtksnprintf(id, 8, "R%02d", prn - MINPRNGLO + 1);
      return;
    case SYS_GAL:
      rtksnprintf(id, 8, "E%02d", prn - MINPRNGAL + 1);
      return;
    case SYS_QZS:
      rtksnprintf(id, 8, "J%02d", prn - MINPRNQZS + 1);
      return;
    case SYS_CMP:
      rtksnprintf(id, 8, "C%02d", prn - MINPRNCMP + 1);
      return;
    case SYS_IRN:
      rtksnprintf(id, 8, "I%02d", prn - MINPRNIRN + 1);
      return;
    case SYS_LEO:
      rtksnprintf(id, 8, "L%02d", prn - MINPRNLEO + 1);
      return;
    case SYS_SBS:
      rtksnprintf(id, 8, "%03d", prn);
      return;
  }
  rtkstrcpy(id, 8, "");
}
/* Test excluded satellite -----------------------------------------------------
 * Test excluded satellite
 * Args   : int    sat       I   satellite number
 *          double var       I   variance of ephemeris (m^2)
 *          int    svh       I   sv health flag
 *          prcopt_t *opt    I   processing options (NULL: not used)
 * Return : status (true:excluded,false:not excluded)
 *----------------------------------------------------------------------------*/
extern bool satexclude(int sat, double var, int svh, const prcopt_t *opt) {
  if (svh < 0) return true; /* Ephemeris unavailable */

  int sys = satsys(sat, NULL);
  if (opt) {
    if (opt->exsats[sat - 1] == 1) return true;  /* Excluded satellite */
    if (opt->exsats[sat - 1] == 2) return false; /* Included satellite */
    if (!(sys & opt->navsys)) return true;       /* Unselected sat sys */
  }
  if (sys == SYS_QZS) svh &= 0xFE; /* Mask QZSS LEX health */
  if (svh) {
    trace(3, "unhealthy satellite: sat=%3d svh=%02X\n", sat, svh);
    return true;
  }
  if (var > MAX_VAR_EPH) {
    trace(3, "invalid ura satellite: sat=%3d ura=%.2f\n", sat, sqrt(var));
    return true;
  }
  return false;
}
/* Test SNR mask ---------------------------------------------------------------
 * Test SNR mask
 * Args   : int    base      I   rover or base-station (0:rover,1:base station)
 *          int    idx       I   frequency index (0:L1,1:L2,2:L3,...)
 *          double el        I   elevation angle (rad)
 *          double snr       I   C/N0 (dBHz)
 *          snrmask_t *mask  I   SNR mask
 * Return : status (true:masked,false:unmasked)
 *----------------------------------------------------------------------------*/
extern bool testsnr(int base, int idx, double el, double snr, const snrmask_t *mask) {
  if (!mask->ena[base] || idx < 0 || idx >= NFREQ) return false;

  double a = (el * R2D + 5.0) / 10.0;
  int i = (int)floor(a);
  a -= i;

  double minsnr;
  if (i < 1)
    minsnr = mask->mask[idx][0];
  else if (i > 8)
    minsnr = mask->mask[idx][8];
  else
    minsnr = (1.0 - a) * mask->mask[idx][i - 1] + a * mask->mask[idx][i];

  return snr < minsnr;
}
/* Obs type string to obs code -------------------------------------------------
 * Convert obs code type string to obs code
 * Args   : char   *str   I      obs code string ("1C","1P","1Y",...)
 * Return : obs code (CODE_???)
 * Notes  : obs codes are based on RINEX 3.04
 *----------------------------------------------------------------------------*/
extern enum code obs2code(const char *obs) {
  for (int i = 1; *obscodes[i]; i++) {
    if (strcmp(obscodes[i], obs)) continue;
    return (uint8_t)i;
  }
  return CODE_NONE;
}
/* Obs code to obs code string -------------------------------------------------
 * Convert obs code to obs code string
 * Args   : enum code code     I   obs code (CODE_???)
 * Return : obs code string ("1C","1P","1P",...)
 * Notes  : obs codes are based on RINEX 3.04
 *----------------------------------------------------------------------------*/
extern char *code2obs(enum code code) {
  if (code <= CODE_NONE || MAXCODE < code) return "";
  return obscodes[code];
}
/* GPS obs code to frequency -------------------------------------------------*/
static int code2freq_GPS(enum code code, double *freq) {
  const char *obs = code2obs(code);

  switch (obs[0]) {
    case '1':
      *freq = FREQL1;
      return 0; /* L1 */
    case '2':
      *freq = FREQL2;
      return 1; /* L2 */
    case '5':
      *freq = FREQL5;
      return 2; /* L5 */
  }
  return -1;
}
/* GLONASS obs code to frequency ---------------------------------------------*/
static int code2freq_GLO(enum code code, int fcn, double *freq) {
  const char *obs = code2obs(code);

  if (fcn < -7 || fcn > 6) return -1;

  switch (obs[0]) {
    case '1':
      *freq = FREQ1_GLO + DFRQ1_GLO * fcn;
      return 0; /* G1 */
    case '2':
      *freq = FREQ2_GLO + DFRQ2_GLO * fcn;
      return 1; /* G2 */
    case '3':
      *freq = FREQ3_GLO;
      return 2; /* G3 */
    case '4':
      *freq = FREQ1a_GLO;
      return 0; /* G1a */
    case '6':
      *freq = FREQ2a_GLO;
      return 1; /* G2a */
  }
  return -1;
}
/* Galileo obs code to frequency ---------------------------------------------*/
static int code2freq_GAL(enum code code, double *freq) {
  const char *obs = code2obs(code);

  switch (obs[0]) {
    case '1':
      *freq = FREQL1;
      return 0; /* E1 */
    case '7':
      *freq = FREQE5b;
      return 1; /* E5b */
    case '5':
      *freq = FREQL5;
      return 2; /* E5a */
    case '6':
      *freq = FREQL6;
      return 3; /* E6 */
    case '8':
      *freq = FREQE5ab;
      return 4; /* E5ab */
  }
  return -1;
}
/* QZSS obs code to frequency ------------------------------------------------*/
static int code2freq_QZS(enum code code, double *freq) {
  const char *obs = code2obs(code);

  switch (obs[0]) {
    case '1':
      *freq = FREQL1;
      return 0; /* L1 */
    case '2':
      *freq = FREQL2;
      return 1; /* L2 */
    case '5':
      *freq = FREQL5;
      return 2; /* L5 */
    case '6':
      *freq = FREQL6;
      return 3; /* L6 */
  }
  return -1;
}
/* SBAS obs code to frequency ------------------------------------------------*/
static int code2freq_SBS(enum code code, double *freq) {
  const char *obs = code2obs(code);

  switch (obs[0]) {
    case '1':
      *freq = FREQL1;
      return 0; /* L1 */
    case '5':
      *freq = FREQL5;
      return 1; /* L5 */
  }
  return -1;
}
/* BDS obs code to frequency -------------------------------------------------*/
static int code2freq_BDS(enum code code, double *freq) {
  const char *obs = code2obs(code);

  switch (obs[0]) {
    case '1':
      *freq = FREQL1;
      return 0; /* B1C */
    case '2':
      *freq = FREQ1_CMP;
      return 0; /* B1I */
    case '7':
      *freq = FREQ2_CMP;
      return 1; /* B2I/B2b */
    case '6':
      *freq = FREQ3_CMP;
      return 2; /* B3 */
    case '5':
      *freq = FREQL5;
      return 3; /* B2a */
    case '8':
      *freq = FREQE5ab;
      return 4; /* B2ab */
  }
  return -1;
}
/* NavIC obs code to frequency -----------------------------------------------*/
static int code2freq_IRN(enum code code, double *freq) {
  const char *obs = code2obs(code);

  switch (obs[0]) {
    case '5':
      *freq = FREQL5;
      return 0; /* L5 */
    case '9':
      *freq = FREQs;
      return 1; /* S */
  }
  return -1;
}
/* System and obs code to frequency index --------------------------------------
 * Convert system and obs code to frequency index
 * Args   : int       sys       I   satellite system (SYS_???)
 *          enum code code      I   obs code (CODE_???)
 * Return : frequency index (-1: error)
 *                       0     1     2     3     4
 *           --------------------------------------
 *            GPS       L1    L2    L5     -     -
 *            GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
 *            Galileo   E1    E5b   E5a   E6   E5ab
 *            QZSS      L1    L2    L5    L6     -
 *            SBAS      L1     -    L5     -     -
 *            BDS       B1    B2    B3   B2a   B2ab (B1=B1I,B1C,B2=B2I,B2b)
 *            NavIC     L5     S     -     -     -
 *----------------------------------------------------------------------------*/
extern int code2idx(int sys, enum code code) {
  double freq;

  switch (sys) {
    case SYS_GPS:
      return code2freq_GPS(code, &freq);
    case SYS_GLO:
      return code2freq_GLO(code, 0, &freq);
    case SYS_GAL:
      return code2freq_GAL(code, &freq);
    case SYS_QZS:
      return code2freq_QZS(code, &freq);
    case SYS_SBS:
      return code2freq_SBS(code, &freq);
    case SYS_CMP:
      return code2freq_BDS(code, &freq);
    case SYS_IRN:
      return code2freq_IRN(code, &freq);
  }
  return -1;
}
/* System and obs code to frequency --------------------------------------------
 * Convert system and obs code to carrier frequency
 * Args   : int       sys       I   satellite system (SYS_???)
 *          enum code code      I   obs code (CODE_???)
 *          int       fcn       I   frequency channel number for GLONASS
 * Return : carrier frequency (Hz) (0.0: error)
 *----------------------------------------------------------------------------*/
extern double code2freq(int sys, enum code code, int fcn) {
  double freq = 0.0;

  switch (sys) {
    case SYS_GPS:
      (void)code2freq_GPS(code, &freq);
      break;
    case SYS_GLO:
      (void)code2freq_GLO(code, fcn, &freq);
      break;
    case SYS_GAL:
      (void)code2freq_GAL(code, &freq);
      break;
    case SYS_QZS:
      (void)code2freq_QZS(code, &freq);
      break;
    case SYS_SBS:
      (void)code2freq_SBS(code, &freq);
      break;
    case SYS_CMP:
      (void)code2freq_BDS(code, &freq);
      break;
    case SYS_IRN:
      (void)code2freq_IRN(code, &freq);
      break;
  }
  return freq;
}
/* Satellite and obs code to frequency -----------------------------------------
 * Convert satellite and obs code to carrier frequency
 * Args   : int       sat       I   satellite number
 *          enum code code      I   obs code (CODE_???)
 *          nav_t     *nav_t    I   navigation data for GLONASS (NULL: not used)
 * Return : carrier frequency (Hz) (0.0: error)
 *----------------------------------------------------------------------------*/
extern double sat2freq(int sat, enum code code, const nav_t *nav) {
  int prn;
  int sys = satsys(sat, &prn);

  int fcn = 0;
  if (sys == SYS_GLO) {
    if (!nav) return 0.0;
    /* First non-empty entry */
    int i;
    for (i = 0; i < nav->ng[prn - 1]; i++) {
      if (nav->geph[prn - 1][i].sat == sat) break;
    }
    if (i < nav->ng[prn - 1]) {
      fcn = nav->geph[prn - 1][i].frq;
    } else if (nav->glo_fcn[prn - 1] > 0) {
      fcn = nav->glo_fcn[prn - 1] - 8;
    } else
      return 0.0;
  }
  return code2freq(sys, code, fcn);
}
/* Set code priority -----------------------------------------------------------
 * Set code priority for multiple codes in a frequency
 * Args   : int    sys       I   system (or of SYS_???)
 *          int    idx       I   frequency index (0- )
 *          char   *pri      I   priority of codes (series of code characters)
 *                               (higher priority precedes lower)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void setcodepri(int sys, int idx, const char *pri) {
  trace(3, "setcodepri:sys=%d idx=%d pri=%s\n", sys, idx, pri);

  if (idx < 0 || idx >= MAXFREQ) return;
  if (sys & SYS_GPS) rtkstrcpy(codepris[0][idx], sizeof(codepris[0][0]), pri);
  if (sys & SYS_GLO) rtkstrcpy(codepris[1][idx], sizeof(codepris[0][0]), pri);
  if (sys & SYS_GAL) rtkstrcpy(codepris[2][idx], sizeof(codepris[0][0]), pri);
  if (sys & SYS_QZS) rtkstrcpy(codepris[3][idx], sizeof(codepris[0][0]), pri);
  if (sys & SYS_SBS) rtkstrcpy(codepris[4][idx], sizeof(codepris[0][0]), pri);
  if (sys & SYS_CMP) rtkstrcpy(codepris[5][idx], sizeof(codepris[0][0]), pri);
  if (sys & SYS_IRN) rtkstrcpy(codepris[6][idx], sizeof(codepris[0][0]), pri);
}
/* Get code priority -----------------------------------------------------------
 * Get code priority for multiple codes in a frequency
 * Args   : int       sys       I   system (SYS_???)
 *          enum code code      I   obs code (CODE_???)
 *          char      *opt      I   code options (NULL:no option)
 * Return : priority (15:highest-1:lowest,0:error)
 *----------------------------------------------------------------------------*/
extern int getcodepri(int sys, enum code code, const char *opt) {
  int i;
  const char *optstr;
  switch (sys) {
    case SYS_GPS:
      i = 0;
      optstr = "-GL%2s";
      break;
    case SYS_GLO:
      i = 1;
      optstr = "-RL%2s";
      break;
    case SYS_GAL:
      i = 2;
      optstr = "-EL%2s";
      break;
    case SYS_QZS:
      i = 3;
      optstr = "-JL%2s";
      break;
    case SYS_SBS:
      i = 4;
      optstr = "-SL%2s";
      break;
    case SYS_CMP:
      i = 5;
      optstr = "-CL%2s";
      break;
    case SYS_IRN:
      i = 6;
      optstr = "-IL%2s";
      break;
    default:
      return 0;
  }
  int j = code2idx(sys, code);
  if (j < 0) return 0;
  const char *obs = code2obs(code);

  /* Parse code options */
  const char *p;
  for (p = opt; p && (p = strchr(p, '-')); p++) {
    char str[8] = "";
    if (sscanf(p, optstr, str) < 1 || str[0] != obs[0]) continue;
    return str[1] == obs[1] ? 15 : 0;
  }
  /* Search code priority */
  p = strchr(codepris[i][j], obs[1]);
  return p ? 14 - (int)(p - codepris[i][j]) : 0;
}
/* Extract unsigned/signed bits ------------------------------------------------
 * Extract unsigned/signed bits from byte data
 * Args   : uint8_t   *buff    I   byte data
 *          unsigned   pos     I   bit position from start of data (bits)
 *          unsigned   len     I   bit length (bits) (len<=32)
 * Return : extracted unsigned/signed bits
 *----------------------------------------------------------------------------*/
extern uint32_t getbitu(const uint8_t *buff, size_t size, unsigned pos, unsigned len) {
  if (len > 32) trace(2, "getbitu: len=%u out of range\n", len);
  uint32_t bits = 0;
  if (len == 0) return bits;
  RTKBOUNDSCHECK(buff, size, (pos + len - 1) / 8);
  for (unsigned i = pos; i < pos + len; i++)
    bits = (bits << 1) | ((buff[i / 8] >> (7 - i % 8)) & 1u);
  return bits;
}
extern int32_t getbits(const uint8_t *buff, size_t size, unsigned pos, unsigned len) {
  uint32_t bits = getbitu(buff, size, pos, len);
  if (len == 0) {
    /* Would need at least one bit for a sign, so emit a warning. */
    trace(2, "getbits: len=%u out of range\n", len);
    return 0;
  }
  if (len >= 32) {
    if (len > 32) trace(2, "getbits: len=%u out of range\n", len);
    return (int32_t)bits;
  }
  /* Check the sign bit */
  if (!(bits & (1u << (len - 1)))) return (int32_t)bits;
  return (int32_t)(bits | (~0u << len)); /* Extend sign */
}
/* Set unsigned/signed bits ----------------------------------------------------
 * Set unsigned/signed bits to byte data
 * Args   : uint8_t    *buff IO byte data
 *          unsigned   pos   I   bit position from start of data (bits)
 *          unsigned   len   I   bit length (bits) (len<=32)
 *          [u]int32_t data  I   unsigned/signed data
 * Return : none
 *----------------------------------------------------------------------------*/
extern void setbitu(uint8_t *buff, size_t size, unsigned pos, unsigned len, uint32_t data) {
  if (len == 0 || 32 < len) {
    trace(0, "Warning setbitu len %u out of range for data %x\n", len, data);
    return;
  }
  RTKBOUNDSCHECK(buff, size, (pos + len - 1) / 8);
  uint32_t mask = 1u << (len - 1);
  for (unsigned i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask)
      buff[i / 8] |= 1u << (7 - i % 8);
    else
      buff[i / 8] &= ~(1u << (7 - i % 8));
  }
}
extern void setbits(uint8_t *buff, size_t size, unsigned pos, unsigned len, int32_t data) {
  if (len == 0 || 32 < len) {
    trace(0, "Warning setbits len %u out of range for data %x\n", len, data);
    return;
  }

  uint32_t limit = 1u << (len - 1);
  if (len < 32) {
    /* Clamp the data in the case it overflows the len */
    if (data >= 0) {
      if ((uint32_t)data >= limit) {
        trace(0, "Warning setbits overflow for data %x len %u\n", data, len);
        /* Clamp */
        data = limit - 1;
        trace(0, "  clamped to %x\n", data);
      }
    } else if ((uint32_t)(-data) > limit) {
      trace(0, "Warning setbits underflow for data %x len %u\n", data, len);
      /* Clamp */
      data = -limit;
      trace(0, "  clamped to %x\n", data);
    }
  }

  uint32_t udata = data;
  if (data < 0)
    udata |= limit; /* Set sign bit */
  else
    udata &= ~limit; /* Clear sign bit */
  setbitu(buff, size, pos, len, udata);
}

/* Crc-32 parity ---------------------------------------------------------------
 * Compute crc-32 parity for novatel raw
 * Args   : uint8_t   *buff    I   data
 *          unsigned  len      I   data length (bytes)
 * Return : crc-32 parity
 * Notes  : see NovAtel OEMV firmware manual 1.7 32-bit CRC
 *----------------------------------------------------------------------------*/
extern uint32_t rtk_crc32(const uint8_t *buff, size_t size, unsigned len) {
  trace(4, "rtk_crc32: len=%u\n", len);
  if (len > 0) RTKBOUNDSCHECK(buff, size, len - 1);
  uint32_t crc = 0;
  for (unsigned i = 0; i < len; i++) {
    crc ^= buff[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ POLYCRC32;
      else
        crc >>= 1;
    }
  }
  return crc;
}
/* Crc-24q parity --------------------------------------------------------------
 * Compute crc-24q parity for SBAS, RTCM3
 * Args   : uint8_t  *buff    I   data
 *          unsigned  len     I   data length (bytes)
 * Return : crc-24Q parity
 * Notes  : see reference [2] A.4.3.3 Parity
 *----------------------------------------------------------------------------*/
extern uint32_t rtk_crc24q(const uint8_t *buff, size_t size, unsigned len) {
  trace(4, "rtk_crc24q: len=%u\n", len);
  if (len > 0) RTKBOUNDSCHECK(buff, size, len - 1);

  uint32_t crc = 0;
  for (unsigned i = 0; i < len; i++)
    crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
  return crc;
}
/* Crc-16 parity ---------------------------------------------------------------
 * Compute crc-16 parity for binex, nvs
 * Args   : uint8_t  *buff    I   data
 *          unsigned  len       I   data length (bytes)
 * Return : crc-16 parity
 * Notes  : see reference [10] A.3.
 *----------------------------------------------------------------------------*/
extern uint16_t rtk_crc16(const uint8_t *buff, size_t size, unsigned len) {
  trace(4, "rtk_crc16: len=%d\n", len);
  if (len > 0) RTKBOUNDSCHECK(buff, size, len - 1);

  uint16_t crc = 0;
  for (unsigned i = 0; i < len; i++) crc = (crc << 8) ^ tbl_CRC16[((crc >> 8) ^ buff[i]) & 0xFF];
  return crc;
}
/* Decode navigation data word -------------------------------------------------
 * Check party and decode navigation data word
 * Args   : uint32_t word    I   navigation data word (2+30bit)
 *                               (previous word D29*-30* + current word D1-30)
 *          uint8_t *data    O   decoded navigation data without parity
 *                               (8bitx3)
 * Return : status (true:ok,false:parity error)
 * Notes  : see reference [1] 20.3.5.2 user parity algorithm
 *----------------------------------------------------------------------------*/
extern bool decode_word(uint32_t word, uint8_t data[4]) {
  trace(5, "decodeword: word=%08x\n", word);

  if (word & 0x40000000) word ^= 0x3FFFFFC0;

  const uint32_t hamming[] = {0xBB1F3480, 0x5D8F9A40, 0xAEC7CD00,
                              0x5763E680, 0x6BB1F340, 0x8B7A89C0};
  uint32_t parity = 0;
  for (int i = 0; i < 6; i++) {
    parity <<= 1;
    for (uint32_t w = (word & hamming[i]) >> 6; w; w >>= 1) parity ^= w & 1;
  }
  if (parity != (word & 0x3F)) return false;

  for (int i = 0; i < 3; i++) data[i] = (uint8_t)(word >> (22 - i * 8));
  return true;
}
/* New matrix ------------------------------------------------------------------
 * Allocate memory of matrix
 * Args   : int    n,m       I   number of rows and columns of matrix
 * Return : matrix pointer (if n<=0 or m<=0, return NULL)
 *----------------------------------------------------------------------------*/
extern double *mat(int n, int m) {
  if (n <= 0 || m <= 0) return NULL;
  double *p = (double *)malloc(sizeof(double) * n * m);
  if (!p) {
    fatalerr("matrix memory allocation error: n=%d,m=%d\n", n, m);
  }
  return p;
}
/* New integer matrix ----------------------------------------------------------
 * Allocate memory of integer matrix
 * Args   : int    n,m       I   number of rows and columns of matrix
 * Return : matrix pointer (if n<=0 or m<=0, return NULL)
 *----------------------------------------------------------------------------*/
extern int *imat(int n, int m) {
  if (n <= 0 || m <= 0) return NULL;
  int *p = (int *)malloc(sizeof(int) * n * m);
  if (!p) {
    fatalerr("integer matrix memory allocation error: n=%d,m=%d\n", n, m);
  }
  return p;
}
/* Zero matrix -----------------------------------------------------------------
 * Generate new zero matrix
 * Args   : int    n,m       I   number of rows and columns of matrix
 * Return : matrix pointer (if n<=0 or m<=0, return NULL)
 *----------------------------------------------------------------------------*/
extern double *zeros(int n, int m) {
  if (n <= 0 || m <= 0) return NULL;

#if NOCALLOC
  double *p = mat(n, m);
  if (p)
    for (n = n * m - 1; n >= 0; n--) p[n] = 0.0;
#else
  double *p = (double *)calloc(sizeof(double), n * m);
  if (!p) {
    fatalerr("matrix memory allocation error: n=%d,m=%d\n", n, m);
  }
#endif
  return p;
}
/* Identity matrix -------------------------------------------------------------
 * Generate new identity matrix
 * Args   : int    n         I   number of rows and columns of matrix
 * Return : matrix pointer (if n<=0, return NULL)
 *----------------------------------------------------------------------------*/
extern double *eye(int n) {
  double *p = zeros(n, n);
  if (p)
    for (int i = 0; i < n; i++) p[i + i * n] = 1.0;
  return p;
}

/* Outer product of 3d vectors -------------------------------------------------
 * Outer product of 3d vectors
 * Args   : double *a,*b     I   vector a,b (3 x 1)
 *          double *c        O   outer product (a x b) (3 x 1)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void cross3(const double *a, const double *b, double *c) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}
/* Normalize 3d vector ---------------------------------------------------------
 * Normalize 3d vector
 * Args   : double *a        I   vector a (3 x 1)
 *          double *b        O   normalized vector (3 x 1) || b || = 1
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool normv3(const double *a, double *b) {
  double r = norm(a, 3);
  if (r <= 0.0) return false;
  b[0] = a[0] / r;
  b[1] = a[1] / r;
  b[2] = a[2] / r;
  return true;
}
/* Matrix routines -----------------------------------------------------------*/

#ifdef LAPACK /* With LAPACK/BLAS or MKL */

/* Multiply matrix (wrapper of blas dgemm) -------------------------------------
 * Multiply matrix by matrix (C=A*B)
 * Args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
 *          int    n,k,m     I  size of (transposed) matrix A,B
 *          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
 *          double *C        O matrix C (n x k)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void matmul(const char *tr, int n, int k, int m, const double *A, const double *B,
                   double *C) {
  int lda = tr[0] == 'T' ? m : n, ldb = tr[1] == 'T' ? k : m;
  const double alpha = 1, beta = 0;

  dgemm_((char *)tr, (char *)tr + 1, &n, &k, &m, &alpha, (double *)A, &lda, (double *)B, &ldb,
         &beta, C, &n);
}
/* Multiply matrix (wrapper of blas dgemm) -------------------------------------
 * Multiply matrix by matrix (C=C+A*B)
 *----------------------------------------------------------------------------*/
extern void matmulp(const char *tr, int n, int k, int m, const double *A, const double *B,
                    double *C) {
  int lda = tr[0] == 'T' ? m : n, ldb = tr[1] == 'T' ? k : m;
  const double alpha = 1, beta = 1;

  dgemm_((char *)tr, (char *)tr + 1, &n, &k, &m, &alpha, (double *)A, &lda, (double *)B, &ldb,
         &beta, C, &n);
}
/* Multiply matrix (wrapper of blas dgemm) -------------------------------------
 * Multiply matrix by matrix (C=C-A*B)
 *----------------------------------------------------------------------------*/
extern void matmulm(const char *tr, int n, int k, int m, const double *A, const double *B,
                    double *C) {
  int lda = tr[0] == 'T' ? m : n, ldb = tr[1] == 'T' ? k : m;
  const double alpha = -1, beta = 1;

  dgemm_((char *)tr, (char *)tr + 1, &n, &k, &m, &alpha, (double *)A, &lda, (double *)B, &ldb,
         &beta, C, &n);
}
/* Inverse of matrix -----------------------------------------------------------
 * Inverse of matrix (A=A^-1)
 * Args   : double *A        IO  matrix (n x n)
 *          int    n         I   size of matrix A
 * Return : status (0:ok,0>:error)
 *----------------------------------------------------------------------------*/
extern int matinv(double *A, int n) {
  double *work = mat(lwork, 1);
  int *ipiv = imat(n, 1);
  dgetrf_(&n, &n, A, &n, ipiv, &info);
  int info, lwork = n * 16;
  if (!info) dgetri_(&n, A, &n, ipiv, work, &lwork, &info);
  free(ipiv);
  free(work);
  return info;
}
/* Solve linear equation -------------------------------------------------------
 * Solve linear equation (X=A\Y or X=A'\Y)
 * Args   : char   *tr       I   transpose flag ("N":normal,"T":transpose)
 *          double *A        I   input matrix A (n x n)
 *          double *Y        I   input matrix Y (n x m)
 *          int    n,m       I   size of matrix A,Y
 *          double *X        O   X=A\Y or X=A'\Y (n x m)
 * Return : status (0:ok,0>:error)
 * Notes  : matrix stored by column-major order (fortran convention)
 *          X can be same as Y
 *----------------------------------------------------------------------------*/
extern int solve(const char *tr, const double *A, const double *Y, int n, int m, double *X) {
  double *B = mat(n, n);
  matcpy(B, A, n, n);
  matcpy(X, Y, n, m);
  int info;
  int *ipiv = imat(n, 1);
  dgetrf_(&n, &n, B, &n, ipiv, &info);
  if (!info) dgetrs_((char *)tr, &n, &m, B, &n, ipiv, X, &n, &info);
  free(ipiv);
  free(B);
  return info;
}

#else /* Without LAPACK/BLAS or MKL */

/* Multiply matrix -----------------------------------------------------------*/
extern void matmul(const char *tr, int n, int k, int m, const double *A, const double *B,
                   double *C) {
  int f = (tr[0] != 'N') * 2 + (tr[1] != 'N');

  switch (f) {
    case 0: /* NN */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m];
          C[i + j * n] = d;
        }
      }
      break;
    case 1: /* NT */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k];
          C[i + j * n] = d;
        }
      }
      break;
    case 2: /* TN */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m];
          C[i + j * n] = d;
        }
      }
      break;
    case 3: /* TT */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k];
          C[i + j * n] = d;
        }
      }
      break;
  }
}
extern void matmulp(const char *tr, int n, int k, int m, const double *A, const double *B,
                    double *C) {
  int f = (tr[0] != 'N') * 2 + (tr[1] != 'N');

  switch (f) {
    case 0: /* NN */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m];
          C[i + j * n] += d;
        }
      }
      break;
    case 1: /* NT */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k];
          C[i + j * n] += d;
        }
      }
      break;
    case 2: /* TN */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m];
          C[i + j * n] += d;
        }
      }
      break;
    case 3: /* TT */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k];
          C[i + j * n] += d;
        }
      }
      break;
  }
}
extern void matmulm(const char *tr, int n, int k, int m, const double *A, const double *B,
                    double *C) {
  int f = (tr[0] != 'N') * 2 + (tr[1] != 'N');

  switch (f) {
    case 0: /* NN */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m];
          C[i + j * n] -= d;
        }
      }
      break;
    case 1: /* NT */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k];
          C[i + j * n] -= d;
        }
      }
      break;
    case 2: /* TN */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m];
          C[i + j * n] -= d;
        }
      }
      break;
    case 3: /* TT */
      for (int j = 0; j < k; j++) {
        for (int i = 0; i < n; i++) {
          double d = 0.0;
          for (int x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k];
          C[i + j * n] -= d;
        }
      }
      break;
  }
}
/* LU decomposition ----------------------------------------------------------*/
static int ludcmp(double *A, int n, int *indx, double *d) {
  double *vv = mat(n, 1);
  *d = 1.0;
  for (int i = 0; i < n; i++) {
    double big = 0.0;
    for (int j = 0; j < n; j++) {
      double tmp = fabs(A[i + j * n]);
      if (tmp > big) big = tmp;
    }
    if (big > 0.0)
      vv[i] = 1.0 / big;
    else {
      free(vv);
      return -1;
    }
  }
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < j; i++) {
      double s = A[i + j * n];
      for (int k = 0; k < i; k++) s -= A[i + k * n] * A[k + j * n];
      A[i + j * n] = s;
    }
    int imax = 0;
    double big = 0.0;
    for (int i = j; i < n; i++) {
      double s = A[i + j * n];
      for (int k = 0; k < j; k++) s -= A[i + k * n] * A[k + j * n];
      A[i + j * n] = s;
      double tmp = vv[i] * fabs(s);
      if (tmp >= big) {
        big = tmp;
        imax = i;
      }
    }
    if (j != imax) {
      for (int k = 0; k < n; k++) {
        double tmp = A[imax + k * n];
        A[imax + k * n] = A[j + k * n];
        A[j + k * n] = tmp;
      }
      *d = -(*d);
      vv[imax] = vv[j];
    }
    indx[j] = imax;
    if (A[j + j * n] == 0.0) {
      free(vv);
      return -1;
    }
    if (j != n - 1) {
      double tmp = 1.0 / A[j + j * n];
      for (int i = j + 1; i < n; i++) A[i + j * n] *= tmp;
    }
  }
  free(vv);
  return 0;
}
/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const double *A, int n, const int *indx, double *b) {
  for (int i = 0, ii = -1; i < n; i++) {
    int ip = indx[i];
    double s = b[ip];
    b[ip] = b[i];
    if (ii >= 0)
      for (int j = ii; j < i; j++) s -= A[i + j * n] * b[j];
    else if (s)
      ii = i;
    b[i] = s;
  }
  for (int i = n - 1; i >= 0; i--) {
    double s = b[i];
    for (int j = i + 1; j < n; j++) s -= A[i + j * n] * b[j];
    b[i] = s / A[i + i * n];
  }
}
/* Inverse of matrix ---------------------------------------------------------*/
extern int matinv(double *A, int n) {
  double *B = mat(n, n);
  matcpy(B, A, n, n);
  double d;
  int *indx = imat(n, 1);
  if (ludcmp(B, n, indx, &d)) {
    free(indx);
    free(B);
    return -1;
  }
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < n; i++) A[i + j * n] = 0.0;
    A[j + j * n] = 1.0;
    lubksb(B, n, indx, A + j * n);
  }
  free(indx);
  free(B);
  return 0;
}
/* Solve linear equation -----------------------------------------------------*/
extern int solve(const char *tr, const double *A, const double *Y, int n, int m, double *X) {
  double *B = mat(n, n);
  matcpy(B, A, n, n);
  int info = matinv(B, n);
  if (!info) matmul(tr[0] == 'N' ? "NN" : "TN", n, m, n, B, Y, X);
  free(B);
  return info;
}
#endif

/* End of matrix routines ----------------------------------------------------*/

/* Least square estimation -----------------------------------------------------
 * Least square estimation by solving normal equation (x=(A*A')^-1*A*y)
 * Args   : double *A        I   transpose of (weighted) design matrix (n x m)
 *          double *y        I   (weighted) measurements (m x 1)
 *          int    n,m       I   number of parameters and measurements (n<=m)
 *          double *x        O   estimated parameters (n x 1)
 *          double *Q        O   estimated parameters covariance matrix (n x n)
 * Return : status (0:ok,0>:error)
 * Notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
 *          matrix stored by column-major order (fortran convention)
 *----------------------------------------------------------------------------*/
extern int lsq(const double *A, const double *y, int n, int m, double *x, double *Q) {
  if (m < n) return -1;
  double *Ay = mat(n, 1);
  matmul("NN", n, 1, m, A, y, Ay); /* Ay=A*y */
  matmul("NT", n, n, m, A, A, Q);  /* Q=A*A' */
  int info = matinv(Q, n);
  if (!info) matmul("NN", n, 1, n, Q, Ay, x); /* x=Q^-1*Ay */
  free(Ay);
  return info;
}
/* Kalman filter ---------------------------------------------------------------
 * Kalman filter state update as follows:
 *
 *   K=P*H*(H'*P*H+R)^-1, xp=x+K*v, Pp=(I-K*H')*P
 *
 * Args   : double *x        IO  states vector (n x 1)
 *          double *P        I   covariance matrix of states (n x n)
 *          double *H        I   transpose of design matrix (n x m)
 *          double *v        I   innovation (measurement - model) (m x 1)
 *          double *R        IX  covariance matrix of measurement error (m x m)
 *          int    n,m       I   number of states and measurements
 *          double *Pp       O   covariance matrix of states after update (n x n)
 * Return : status (0:ok,<0:error)
 * Notes  : matrix stored by column-major order (fortran convention)
 *          if state x[i]==0.0, not updates state x[i]/P[i+i*n]
 *          The x array is not modified on error.
 *          The R input matrix is destructively modified, even on error.
 *----------------------------------------------------------------------------*/
extern int filter_(double *x, const double *P, const double *H, const double *v, double *R, int n,
                   int m, double *Pp) {
  double *PH = mat(n, m), *K = mat(n, m), *I = eye(n);
  int info;

  matmul("NN", n, m, n, P, H, PH); /* P*H */
  /* R is destructively modified to store Q */
  matmulp("TN", m, m, n, H, PH, R);  /* Q=H'*P*H+R */
  if (!(info = matinv(R, m))) {      /* Q^-1 */
    matmul("NN", n, m, m, PH, R, K); /* K=P*H*Q^-1 */
    matmulp("NN", n, 1, m, K, v, x); /* xp=x+K*v */
    matmulm("NT", n, n, m, K, H, I); /* (I-K*H') */
    matmul("NN", n, n, n, I, P, Pp); /* Pp=(I-K*H')*P */
  }
  free(PH);
  free(K);
  free(I);
  return info;
}
extern int filter(double *x, double *P, const double *H, const double *v, double *R, int n, int m) {
  /* Create list of non-zero states */
  int *ix = imat(n, 1);
  int k = 0;
  for (int i = 0; i < n; i++)
    if (x[i] != 0.0 && P[i + i * n] > 0.0) ix[k++] = i;
  double *x_ = mat(k, 1), *P_ = mat(k, k), *Pp_ = mat(k, k), *H_ = mat(k, m);
  /* Compress array by removing zero elements to save computation time */
  for (int i = 0; i < k; i++) x_[i] = x[ix[i]];
  for (int j = 0; j < k; j++)
    for (int i = 0; i < k; i++) P_[i + j * k] = P[ix[i] + ix[j] * n];
  for (int j = 0; j < m; j++)
    for (int i = 0; i < k; i++) H_[i + j * k] = H[ix[i] + j * n];
  /* Do kalman filter state update on compressed arrays */
  int info = filter_(x_, P_, H_, v, R, k, m, Pp_);
  if (!info) {
    /* Copy values from compressed arrays back to full arrays */
    for (int i = 0; i < k; i++) x[ix[i]] = x_[i];
    for (int j = 0; j < k; j++)
      for (int i = 0; i < k; i++) P[ix[i] + ix[j] * n] = Pp_[i + j * k];
  }
  free(ix);
  free(x_);
  free(P_);
  free(Pp_);
  free(H_);
  return info;
}
/* Smoother --------------------------------------------------------------------
 * Combine forward and backward filters by fixed-interval smoother as follows:
 *
 *   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
 *
 * Args   : double *xf       I   forward solutions (n x 1)
 * Args   : double *Qf       I   forward solutions covariance matrix (n x n)
 *          double *xb       I   backward solutions (n x 1)
 *          double *Qb       I   backward solutions covariance matrix (n x n)
 *          int    n         I   number of solutions
 *          double *xs       O   smoothed solutions (n x 1)
 *          double *Qs       O   smoothed solutions covariance matrix (n x n)
 * Return : status (0:ok,0>:error)
 * Notes  : see reference [4] 5.2
 *          matrix stored by column-major order (fortran convention)
 *----------------------------------------------------------------------------*/
extern int smoother(const double *xf, const double *Qf, const double *xb, const double *Qb, int n,
                    double *xs, double *Qs) {
  double *invQf = mat(n, n), *invQb = mat(n, n), *xx = mat(n, 1);

  matcpy(invQf, Qf, n, n);
  matcpy(invQb, Qb, n, n);
  int info = -1;
  if (!matinv(invQf, n) && !matinv(invQb, n)) {
    for (int i = 0; i < n * n; i++) Qs[i] = invQf[i] + invQb[i];
    info = matinv(Qs, n);
    if (!info) {
      matmul("NN", n, 1, n, invQf, xf, xx);
      matmulp("NN", n, 1, n, invQb, xb, xx);
      matmul("NN", n, 1, n, Qs, xx, xs);
    }
  }
  free(invQf);
  free(invQb);
  free(xx);
  return info;
}
/* Print matrix ----------------------------------------------------------------
 * Print matrix to stdout
 * Args   : double *A        I   matrix A (n x m)
 *          int    n,m       I   number of rows and columns of A
 *          int    p,q       I   total columns, columns under decimal point
 *         (FILE  *fp        I   output file pointer)
 * Return : none
 * Notes  : matrix stored by column-major order (fortran convention)
 *----------------------------------------------------------------------------*/
extern void matfprint(const double A[], int n, int m, int p, int q, FILE *fp) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) fprintf(fp, " %*.*f", p, q, A[i + j * n]);
    fprintf(fp, "\n");
  }
}
extern void matprint(const double A[], int n, int m, int p, int q) {
  matfprint(A, n, m, p, q, stdout);
}

/* Check that the index is within the buffer size, generating an error
 * If not. See the macro RTKBOUNDSCHECK(). */
extern void rtkboundscheck(const char *func, int line, const void *buff, size_t size,
                           size_t index) {
  if (index > size) {
    fatalerr("rtk out of bound in %s line %d for buffer %p of size %zd at index %zd\n", func, line,
             buff, size, index);
  }
}

/* Copies a nul terminated source string into the destination, checking that the
 * Destination is large enough. */
extern void rtkstrcpy(char *dst, size_t dsize, const char *src) {
  size_t len = strlen(src);
  if (len + 1 > dsize) {
    fatalerr("rtkstrcpy: destination size %zd too small for source length %zd\n", dsize, len);
  }
  memcpy(dst, src, len);
  dst[len] = '\0';
}

/* Copies a nul terminated source string, starting at index 'start',
 * Into the destination, checking that the destination is large
 * enough. If the start greater than the source string length then an
 * Error is generated.
 *
 * The destination is nul terminated, and an error is generated if
 * There is not enough room in the destination buffer for the source
 * Sub-string and a nul termination.
 */
extern void rtksubstrcpy(char *dst, size_t dsize, const char *src, size_t start) {
  size_t end = strlen(src);
  if (start > end) {
    fatalerr("rtksubstrcpy: source start %zd < source end %zd in '%s'\n", start, end, src);
  }
  size_t len = end - start;
  if (len + 1 > dsize) {
    fatalerr("rtksubstrcpy: destination size %zd too small for substring length %zd\n", dsize, len);
  }
  memcpy(dst, src + start, len);
  dst[len] = '\0';
}

/* Copy the source sub-string, starting at index 'start' inclusive and
 * Ending at index 'end' exclusive. The end index is expected to be
 * Less than or equal to the source string length, and an error is
 * Generated if a nul if encountered. The start index is expected to
 * Be less than or equal to the end index, otherwise an error is
 * generated.
 *
 * The destination is nul terminated, and an error is generated if
 * There is not enough room in the destination buffer for the source
 * Sub-string and a nul termination.
 */
extern void rtkesubstrcpy(char *dst, size_t dsize, const char *src, size_t start, size_t end) {
  if (start > end) {
    fatalerr("rtkesubstrcpy: source start %zd > end %zd in '%s'\n", start, end, src);
  }
  /* Extra check, that the string did not terminate before the start index */
  size_t slen = strlen(src);
  if (start > slen) {
    fatalerr("rtkesubstrcpy: source start %zd > source length %zd in '%s'n", start, slen, src);
  }
  size_t len = end - start;
  if (len + 1 > dsize) {
    fatalerr("rtkesubstrcpy: destination size %zd too small for substring length %zd\n", dsize,
             len);
  }
  for (size_t i = start, j = 0; i < end; i++, j++) {
    char c = src[i];
    if (c == '\0') {
      fatalerr("rtkesubstrcpy: source end %zd out of range at %zd\n", end, i);
    }
    dst[j] = src[i];
  }
  dst[len] = '\0';
}

/* As for rtkesubstrcpy, but trim trailing space. Useful for strings
 * From fixed width formatted fields, such as in RINEX files. */
extern void rtksetstr(char *dst, size_t dsize, const char *src, size_t start, size_t end) {
  if (start > end) {
    fatalerr("rtksetstr: source start %zd > end %zd in '%s'\n", start, end, src);
  }

  /* Find the trimmed end */
  size_t tend;
  for (tend = start; tend < end; tend++) {
    if (!src[tend]) break;
  }
  /* Trim trailing spaces */
  for (; tend > start; tend--) {
    if (src[tend - 1] != ' ') break;
  }

  rtkesubstrcpy(dst, dsize, src, start, tend);
}

extern void rtkstrcat(char *dst, size_t dsize, const char *src) {
  size_t dlen = strlen(dst);
  size_t slen = strlen(src);
  if (dlen + slen + 1 > dsize) {
    fatalerr(
        "rtkstrcat: destination size %zd too small for destination length %zd plus source length "
        "%zd\n",
        dsize, dlen, slen);
  }
  memcpy(dst + dlen, src, slen);
  dst[dlen + slen] = '\0';
}

extern void rtksubstrcat(char *dst, size_t dsize, const char *src, size_t start) {
  size_t end = strlen(src);
  if (start > end) {
    fatalerr("rtksubstrcat: source start %zd < source end %zd in '%s'\n", start, end, src);
  }
  size_t dlen = strlen(dst);
  size_t slen = end - start;
  if (dlen + slen + 1 > dsize) {
    fatalerr(
        "rtksubstrcat: destination size %zd too small for destination length %zd plus substring "
        "length "
        "%zd\n",
        dsize, dlen, slen);
  }
  memcpy(dst + dlen, src + start, slen);
  dst[dlen + slen] = '\0';
}

extern void rtkesubstrcat(char *dst, size_t dsize, const char *src, size_t start, size_t end) {
  if (start > end) {
    fatalerr("rtkesubstrcat: source start %zd > end %zd in '%s'\n", start, end, src);
  }
  /* Extra check, that the string did not terminate before the start index */
  size_t slen = strlen(src);
  if (start > slen) {
    fatalerr("rtkesubstrcat: source start %zd > source length %zd in '%s'n", start, slen, src);
  }
  size_t dlen = strlen(dst);
  size_t len = end - start;
  if (dlen + len + 1 > dsize) {
    fatalerr(
        "rtkesubstrcpy: destination size %zd too small for  destination length %zd plus substring "
        "length %zd\n",
        dsize, dlen, len);
  }
  for (size_t i = start, j = dlen; i < end; i++, j++) {
    char c = src[i];
    if (c == '\0') {
      fatalerr("rtkesubstrcat: source end %zd out of range at %zd\n", end, i);
    }
    dst[j] = src[i];
  }
  dst[dlen + len] = '\0';
}

extern void rtksnprintf(char *str, size_t size, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  int len = vsnprintf(str, size, format, ap);
  va_end(ap);
  if (len < 0) {
    fatalerr("rtksnprintf: format error in '%s'\n", format);
  }
  if ((size_t)len >= size) {
    fatalerr("rtksnprintf: output needed %d overflows buffer of %zd for format '%s'\n", len, size,
             format);
  }
}

extern void rtkcatprintf(char *str, size_t size, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  int len = strlen(str);
  len = vsnprintf(str + len, size - len, format, ap);
  va_end(ap);
  if (len < 0) {
    fatalerr("rtkcatprintf: format error in '%s'\n", format);
  }
  if ((size_t)len >= size) {
    fatalerr("rtkcatprintf: output needed %d overflows buffer of %zd for format '%s'\n", len, size,
             format);
  }
}

/* Variants of standard C library string functions that return an index
 * Rather than a pointer. For use were an index is needed directly. */
extern int strchri(const char *s, size_t start, int c) {
  size_t len = strlen(s);
  if (start > len)
    fatalerr("strchri start=%zd outside string length %d for string '%s'\n", start, len, s);
  char *p = strchr(s + start, c);
  if (p) return p - s;
  return -1;
}
extern int strrchri(const char *s, size_t start, int c) {
  size_t len = strlen(s);
  if (start > len)
    fatalerr("strrchri start=%zd outside string length %d for string '%s'\n", start, len, s);
  char *p = strrchr(s + start, c);
  if (p) return p - s;
  return -1;
}
extern int strstri(const char *haystack, size_t start, const char *needle) {
  size_t len = strlen(haystack);
  if (start > len)
    fatalerr("strstri start=%zd outside string length %d for string '%s'\n", start, len, haystack);
  char *p = strstr(haystack + start, needle);
  if (p) return p - haystack;
  return -1;
}

/* String to number ------------------------------------------------------------
 * Convert substring in string to number
 * Args   : char   *s        I   string ("... nnn.nnn ...")
 *          int    i,n       I   substring position and width
 * Return : converted number (0.0:error)
 *----------------------------------------------------------------------------*/
extern double str2num(const char *s, int i, int n) {
  char str[256];
  if (i < 0 || (int)sizeof(str) - 1 < n) return 0.0;
  /* Special case i==0, skipping the strlen check.
   * Note: Could usefully use strnlen(s,i) here */
  if (i > 0 && (int)strlen(s) < i) return 0.0;

  char *p = str;
  for (s += i; --n >= 0; s++) {
    char c = *s;
    if (!c) break;
    *p++ = ((c | 0x20) == 'd') ? 'E' : c;
  }
  *p = '\0';
  return strtod(str, NULL);
}
/* String to time --------------------------------------------------------------
 * Convert substring in string to gtime_t struct
 * Args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
 *          int    i,n       I   substring position and width
 *          gtime_t *t       O   gtime_t struct
 * Return : status (0:ok,0>:error)
 *----------------------------------------------------------------------------*/
extern int str2time(const char *s, int i, int n, gtime_t *t) {
  char str[256];
  if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < i) return -1;
  char *p = str;
  for (s += i; *s && --n >= 0;) *p++ = *s++;
  *p = '\0';
  double ep[6];
  if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6)
    return -1;
  if (ep[0] < 100.0) ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;
  *t = epoch2time(ep);
  return 0;
}
/* Convert calendar day/time to time -------------------------------------------
 * Convert calendar day/time to gtime_t struct
 * Args   : double *ep       I   day/time {year,month,day,hour,min,sec}
 * Return : gtime_t struct
 * Notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
 *----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep) {
  const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
  gtime_t time = {0};
  int year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

  if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

  /* Leap year if year%4==0 in 1901-2099 */
  int days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 +
             (year % 4 == 0 && mon >= 3 ? 1 : 0);
  int sec = (int)floor(ep[5]);
  time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
  time.sec = ep[5] - sec;
  return time;
}
/* Time to calendar day/time ---------------------------------------------------
 * Convert gtime_t struct to calendar day/time
 * Args   : gtime_t t        I   gtime_t struct
 *          double *ep       O   day/time {year,month,day,hour,min,sec}
 * Return : none
 * Notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
 *----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep) {
  const int mday[] = {/* # of days in a month */
                      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30,
                      31, 30, 31, 31, 30, 31, 30, 31, 31, 29, 31, 30, 31, 30, 31, 31,
                      30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  /* Leap year if year%4==0 in 1901-2099 */
  int days = (int)(t.time / 86400);
  int sec = (int)(t.time - (time_t)days * 86400);
  int mon, day = days % 1461;
  for (mon = 0; mon < 48; mon++) {
    if (day >= mday[mon])
      day -= mday[mon];
    else
      break;
  }
  ep[0] = 1970 + days / 1461 * 4 + mon / 12;
  ep[1] = mon % 12 + 1;
  ep[2] = day + 1;
  ep[3] = sec / 3600;
  ep[4] = sec % 3600 / 60;
  ep[5] = sec % 60 + t.sec;
}
/* Same as above but output limited to n decimals for formatted output */
extern void time2epoch_n(gtime_t t, double *ep, int n) {
  if (n < 0)
    n = 0;
  else if (n > 12)
    n = 12;
  if (1.0 - t.sec < 0.5 / pow(10.0, n)) {
    t.time++;
    t.sec = 0.0;
  };
  time2epoch(t, ep);
}
/* GPS time to time ------------------------------------------------------------
 * Convert week and tow in GPS time to gtime_t struct
 * Args   : int    week      I   week number in GPS time
 *          double sec       I   time of week in GPS time (s)
 * Return : gtime_t struct
 *----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec) {
  gtime_t t = epoch2time(gpst0);

  if (sec < -1E9 || 1E9 < sec) sec = 0.0;
  t.time += (time_t)86400 * 7 * week + (int)sec;
  t.sec = sec - (int)sec;
  return t;
}
/* Time to GPS time ------------------------------------------------------------
 * Convert gtime_t struct to week and tow in GPS time
 * Args   : gtime_t t        I   gtime_t struct
 *          int    *week     IO  week number in GPS time (NULL: no output)
 * Return : time of week in GPS time (s)
 *----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week) {
  gtime_t t0 = epoch2time(gpst0);
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));

  if (week) *week = w;
  return (double)(sec - (double)w * 86400 * 7) + t.sec;
}
/* Galileo system time to time -------------------------------------------------
 * Convert week and tow in Galileo system time (GST) to gtime_t struct
 * Args   : int    week      I   week number in GST
 *          double sec       I   time of week in GST (s)
 * Return : gtime_t struct
 *----------------------------------------------------------------------------*/
extern gtime_t gst2time(int week, double sec) {
  if (sec < -1E9 || 1E9 < sec) sec = 0.0;
  gtime_t t = epoch2time(gst0);
  t.time += (time_t)86400 * 7 * week + (int)sec;
  t.sec = sec - (int)sec;
  return t;
}
/* Time to Galileo system time -------------------------------------------------
 * Convert gtime_t struct to week and tow in Galileo system time (GST)
 * Args   : gtime_t t        I   gtime_t struct
 *          int    *week     IO  week number in GST (NULL: no output)
 * Return : time of week in GST (s)
 *----------------------------------------------------------------------------*/
extern double time2gst(gtime_t t, int *week) {
  gtime_t t0 = epoch2time(gst0);
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));

  if (week) *week = w;
  return (double)(sec - (double)w * 86400 * 7) + t.sec;
}
/* BeiDou time (BDT) to time ---------------------------------------------------
 * Convert week and tow in BeiDou time (BDT) to gtime_t struct
 * Args   : int    week      I   week number in BDT
 *          double sec       I   time of week in BDT (s)
 * Return : gtime_t struct
 *----------------------------------------------------------------------------*/
extern gtime_t bdt2time(int week, double sec) {
  if (sec < -1E9 || 1E9 < sec) sec = 0.0;

  gtime_t t = epoch2time(bdt0);
  t.time += (time_t)86400 * 7 * week + (int)sec;
  t.sec = sec - (int)sec;
  return t;
}
/* Time to beidouo time (BDT) --------------------------------------------------
 * Convert gtime_t struct to week and tow in BeiDou time (BDT)
 * Args   : gtime_t t        I   gtime_t struct
 *          int    *week     IO  week number in BDT (NULL: no output)
 * Return : time of week in BDT (s)
 *----------------------------------------------------------------------------*/
extern double time2bdt(gtime_t t, int *week) {
  gtime_t t0 = epoch2time(bdt0);
  time_t sec = t.time - t0.time;
  int w = (int)(sec / (86400 * 7));

  if (week) *week = w;
  return (double)(sec - (double)w * 86400 * 7) + t.sec;
}
/* Get current time in UTC -----------------------------------------------------
 * Get current time in UTC
 * Args   : none
 * Return : current time in UTC
 *----------------------------------------------------------------------------*/
static double timeoffset_ = 0.0; /* Time offset (s) */

extern gtime_t timeget(void) {
  double ep[6] = {0};
#ifdef WIN32
  SYSTEMTIME ts;

  GetSystemTime(&ts); /* UTC */
  ep[0] = ts.wYear;
  ep[1] = ts.wMonth;
  ep[2] = ts.wDay;
  ep[3] = ts.wHour;
  ep[4] = ts.wMinute;
  ep[5] = ts.wSecond + ts.wMilliseconds * 1E-3;
#else
  struct timeval tv;
  struct tm *tt;
  if (!gettimeofday(&tv, NULL) && (tt = gmtime(&tv.tv_sec))) {
    ep[0] = tt->tm_year + 1900;
    ep[1] = tt->tm_mon + 1;
    ep[2] = tt->tm_mday;
    ep[3] = tt->tm_hour;
    ep[4] = tt->tm_min;
    ep[5] = tt->tm_sec + tv.tv_usec * 1E-6;
  }
#endif
  gtime_t time = epoch2time(ep);

#ifdef CPUTIME_IN_GPST /* Cputime operated in GPST */
  time = gpst2utc(time);
#endif
  return timeadd(time, timeoffset_);
}
/* Set current time in UTC -----------------------------------------------------
 * Set current time in UTC
 * Args   : gtime_t          I   current time in UTC
 * Return : none
 * Notes  : just set time offset between cpu time and current time
 *          the time offset is reflected to only timeget()
 *          not reentrant
 *----------------------------------------------------------------------------*/
extern void timeset(gtime_t t) { timeoffset_ += timediff(t, timeget()); }
/* Reset current time ----------------------------------------------------------
 * Reset current time
 * Args   : none
 * Return : none
 *----------------------------------------------------------------------------*/
extern void timereset(void) { timeoffset_ = 0.0; }
/* Read leap seconds table by text -------------------------------------------*/
static int read_leaps_text(FILE *fp) {
  rewind(fp);

  char buff[256];
  int n = 0;
  while (fgets(buff, sizeof(buff), fp) && n < MAXLEAPS) {
    char *p = strchr(buff, '#');
    if (p) *p = '\0';
    int ep[6], ls;
    if (sscanf(buff, "%d %d %d %d %d %d %d", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5, &ls) < 7)
      continue;
    for (int i = 0; i < 6; i++) leaps[n][i] = ep[i];
    leaps[n++][6] = ls;
  }
  return n;
}
/* Read leap seconds table by usno -------------------------------------------*/
static int read_leaps_usno(FILE *fp) {
  static const char *months[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                 "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
  rewind(fp);

  char ls[MAXLEAPS][7] = {{0}};
  char buff[256];
  int n = 0;
  while (fgets(buff, sizeof(buff), fp) && n < MAXLEAPS) {
    char month[32] = {'\0'};
    double jd, tai_utc;
    int y, d;
    if (sscanf(buff, "%d %31s %d =JD %lf TAI-UTC= %lf", &y, month, &d, &jd, &tai_utc) < 5) continue;
    if (y < 1980) continue;
    int m;
    for (m = 1; m <= 12; m++)
      if (!strcmp(months[m - 1], month)) break;
    if (m >= 13) continue;
    ls[n][0] = y;
    ls[n][1] = m;
    ls[n][2] = d;
    ls[n++][6] = (char)(19.0 - tai_utc);
  }
  for (int i = 0; i < n; i++)
    for (int j = 0; j < 7; j++) {
      leaps[i][j] = ls[n - i - 1][j];
    }
  return n;
}
/* Read leap seconds table -----------------------------------------------------
 * Read leap seconds table
 * Args   : char    *file    I   leap seconds table file
 * Return : status (true:ok,false:error)
 * Notes  : The leap second table should be as follows or leapsec.dat provided
 *          by USNO.
 *          (1) The records in the table file consist of the following fields:
 *              year month day hour min sec UTC-GPST(s)
 *          (2) The date and time indicate the start UTC time for the UTC-GPST
 *          (3) The date and time should be descending order.
 *----------------------------------------------------------------------------*/
extern bool read_leaps(const char *file) {
  FILE *fp = fopen(file, "r");
  if (!fp) return false;

  /* Read leap seconds table by text or usno */
  int n = read_leaps_text(fp);
  if (!n && !(n = read_leaps_usno(fp))) {
    fclose(fp);
    return false;
  }
  for (int i = 0; i < 7; i++) leaps[n][i] = 0.0;
  fclose(fp);
  return true;
}
/* GPStime to UTC --------------------------------------------------------------
 * Convert GPStime to UTC considering leap seconds
 * Args   : gtime_t t        I   time expressed in GPStime
 * Return : time expressed in UTC
 * Notes  : ignore slight time offset under 100 ns
 *----------------------------------------------------------------------------*/
extern gtime_t gpst2utc(gtime_t t) {
  for (int i = 0; leaps[i][0] > 0; i++) {
    gtime_t tu = timeadd(t, leaps[i][6]);
    if (timediff(tu, epoch2time(leaps[i])) >= 0.0) return tu;
  }
  return t;
}
/* UTC to GPStime --------------------------------------------------------------
 * Convert UTC to GPStime considering leap seconds
 * Args   : gtime_t t        I   time expressed in UTC
 * Return : time expressed in GPStime
 * Notes  : ignore slight time offset under 100 ns
 *----------------------------------------------------------------------------*/
extern gtime_t utc2gpst(gtime_t t) {
  for (int i = 0; leaps[i][0] > 0; i++) {
    if (timediff(t, epoch2time(leaps[i])) >= 0.0) return timeadd(t, -leaps[i][6]);
  }
  return t;
}
/* GPStime to BDT --------------------------------------------------------------
 * Convert GPStime to BDT (BeiDou navigation satellite system time)
 * Args   : gtime_t t        I   time expressed in GPStime
 * Return : time expressed in BDT
 * Notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
 *          no leap seconds in BDT
 *          ignore slight time offset under 100 ns
 *----------------------------------------------------------------------------*/
extern gtime_t gpst2bdt(gtime_t t) { return timeadd(t, -14.0); }
/* BDT to GPStime --------------------------------------------------------------
 * Convert BDT (BeiDou navigation satellite system time) to GPStime
 * Args   : gtime_t t        I   time expressed in BDT
 * Return : time expressed in GPStime
 * Notes  : see gpst2bdt()
 *----------------------------------------------------------------------------*/
extern gtime_t bdt2gpst(gtime_t t) { return timeadd(t, 14.0); }
/* Time to day and sec -------------------------------------------------------*/
static double time2sec(gtime_t time, gtime_t *day) {
  double ep[6], sec;
  time2epoch(time, ep);
  sec = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
  ep[3] = ep[4] = ep[5] = 0.0;
  *day = epoch2time(ep);
  return sec;
}
/* UTC to GMST -----------------------------------------------------------------
 * Convert UTC to GMST (Greenwich mean sidereal time)
 * Args   : gtime_t t        I   time expressed in UTC
 *          double ut1_utc   I   UT1-UTC (s)
 * Return : GMST (rad)
 *----------------------------------------------------------------------------*/
extern double utc2gmst(gtime_t t, double ut1_utc) {
  const double ep2000[] = {2000, 1, 1, 12, 0, 0};

  gtime_t tut = timeadd(t, ut1_utc);
  gtime_t tut0;
  double ut = time2sec(tut, &tut0);
  double t1 = timediff(tut0, epoch2time(ep2000)) / 86400.0 / 36525.0;
  double t2 = t1 * t1;
  double t3 = t2 * t1;
  double gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
  double gmst = gmst0 + 1.002737909350795 * ut;

  return fmod(gmst, 86400.0) * PI / 43200.0; /* 0 <= gmst <= 2*PI */
}
/* Time to string --------------------------------------------------------------
 * Convert gtime_t struct to string
 * Args   : gtime_t t        I   gtime_t struct
 *          char   [40]      O   string ("yyyy/mm/dd hh:mm:ss.ssss")
 *          int    n         I   number of decimals
 * Return : time string
 *----------------------------------------------------------------------------*/
extern char *time2str(gtime_t t, char s[40], int n) {
  if (n < 0)
    n = 0;
  else if (n > 12)
    n = 12;
  if (1.0 - t.sec < 0.5 / pow(10.0, n)) {
    t.time++;
    t.sec = 0.0;
  };
  double ep[6];
  time2epoch(t, ep);
  rtksnprintf(s, 40, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f", ep[0], ep[1], ep[2], ep[3], ep[4],
              n <= 0 ? 2 : n + 3, n <= 0 ? 0 : n, ep[5]);
  return s;
}
/* Time to day of year ---------------------------------------------------------
 * Convert time to day of year
 * Args   : gtime_t t        I   gtime_t struct
 * Return : day of year (days)
 *----------------------------------------------------------------------------*/
extern double time2doy(gtime_t t) {
  double ep[6];
  time2epoch(t, ep);
  ep[1] = ep[2] = 1.0;
  ep[3] = ep[4] = ep[5] = 0.0;
  return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}
/* Adjust GPS week number ------------------------------------------------------
 * Adjust GPS week number using cpu time
 * Args   : int   week       I   not-adjusted GPS week number (0-1023)
 * Return : adjusted GPS week number
 *----------------------------------------------------------------------------*/
extern int adjgpsweek(int week) {
  int w;
  (void)time2gpst(utc2gpst(timeget()), &w);
  if (w < 1560) w = 1560; /* Use 2009/12/1 if time is earlier than 2009/12/1 */
  return week + (w - week + 1) / 1024 * 1024;
}
/* Get tick time ---------------------------------------------------------------
 * Get current tick in ms
 * Args   : none
 * Return : current tick in ms
 *----------------------------------------------------------------------------*/
extern uint32_t tickget(void) {
#ifdef WIN32
  return (uint32_t)timeGetTime();
#else
  struct timespec tp = {0};
  struct timeval tv = {0};

#ifdef CLOCK_MONOTONIC_RAW
  /* Linux kernel > 2.6.28 */
  if (!clock_gettime(CLOCK_MONOTONIC_RAW, &tp)) {
    return tp.tv_sec * 1000u + tp.tv_nsec / 1000000u;
  } else {
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000u + tv.tv_usec / 1000u;
  }
#else
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000u + tv.tv_usec / 1000u;
#endif
#endif /* WIN32 */
}
/* Sleep ms --------------------------------------------------------------------
 * Sleep ms
 * Args   : int   ms         I   milliseconds to sleep (<0:no sleep)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void sleepms(int ms) {
#ifdef WIN32
  if (ms < 5)
    Sleep(1);
  else
    Sleep(ms);
#else
  struct timespec ts;
  if (ms <= 0) return;
  ts.tv_sec = (time_t)(ms / 1000);
  ts.tv_nsec = (long)(ms % 1000 * 1000000);
  nanosleep(&ts, NULL);
#endif
}
/* Convert degree to deg-min-sec -----------------------------------------------
 * Convert degree to degree-minute-second
 * Args   : double deg       I   degree
 *          double *dms      O   degree-minute-second {deg,min,sec}
 *          int    ndec      I   number of decimals of second
 * Return : none
 *----------------------------------------------------------------------------*/
extern void deg2dms(double deg, double *dms, int ndec) {
  double a = fabs(deg);
  dms[0] = floor(a);
  a = (a - dms[0]) * 60.0;
  dms[1] = floor(a);
  a = (a - dms[1]) * 60.0;
  double unit = pow(0.1, ndec);
  dms[2] = floor(a / unit + 0.5) * unit;
  if (dms[2] >= 60.0) {
    dms[2] = 0.0;
    dms[1] += 1.0;
    if (dms[1] >= 60.0) {
      dms[1] = 0.0;
      dms[0] += 1.0;
    }
  }
  double sign = deg < 0.0 ? -1.0 : 1.0;
  dms[0] *= sign;
}
/* Convert deg-min-sec to degree -----------------------------------------------
 * Convert degree-minute-second to degree
 * Args   : double *dms      I   degree-minute-second {deg,min,sec}
 * Return : degree
 *----------------------------------------------------------------------------*/
extern double dms2deg(const double *dms) {
  double sign = dms[0] < 0.0 ? -1.0 : 1.0;
  return sign * (fabs(dms[0]) + dms[1] / 60.0 + dms[2] / 3600.0);
}
/* Transform ECEF to geodetic position -----------------------------------------
 * Transform ECEF position to geodetic position
 * Args   : double *r        I   ECEF position {x,y,z} (m)
 *          double *pos      O   geodetic position {lat,lon,h} (rad,m)
 * Return : none
 * Notes  : WGS84, ellipsoidal height
 *----------------------------------------------------------------------------*/
extern void ecef2pos(const double *r, double *pos) {
  double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot2(r, r), v = RE_WGS84;
  double z = r[2];
  for (double zk = 0.0; fabs(z - zk) >= 1E-8;) {
    zk = z;
    double sinp = z / sqrt(r2 + z * z);
    v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
    z = r[2] + v * e2 * sinp;
  }
  pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
  pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
  pos[2] = sqrt(r2 + z * z) - v;
}
/* Transform geodetic to ECEF position -----------------------------------------
 * Transform geodetic position to ECEF position
 * Args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
 *          double *r        O   ECEF position {x,y,z} (m)
 * Return : none
 * Notes  : WGS84, ellipsoidal height
 *----------------------------------------------------------------------------*/
extern void pos2ecef(const double *pos, double *r) {
  double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
  double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

  r[0] = (v + pos[2]) * cosp * cosl;
  r[1] = (v + pos[2]) * cosp * sinl;
  r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}
/* ECEF to local coordinate transformation matrix ------------------------------
 * Compute ECEF to local coordinate transformation matrix
 * Args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *E        O   ECEF to local coord transformation matrix (3x3)
 * Return : none
 * Notes  : matrix stored by column-major order (fortran convention)
 *----------------------------------------------------------------------------*/
extern void xyz2enu(const double *pos, double *E) {
  double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

  E[0] = -sinl;
  E[3] = cosl;
  E[6] = 0.0;
  E[1] = -sinp * cosl;
  E[4] = -sinp * sinl;
  E[7] = cosp;
  E[2] = cosp * cosl;
  E[5] = cosp * sinl;
  E[8] = sinp;
}
/* Transform ECEF vector to local tangential coordinate ------------------------
 * Transform ECEF vector to local tangential coordinate
 * Args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *r        I   vector in ECEF coordinate {x,y,z}
 *          double *e        O   vector in local tangential coordinate {e,n,u}
 * Return : none
 *----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e) {
  double E[9];
  xyz2enu(pos, E);
  matmul("NN", 3, 1, 3, E, r, e);
}
/* Transform local vector to ECEF coordinate -----------------------------------
 * Transform local tangential coordinate vector to ECEF
 * Args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *e        I   vector in local tangential coordinate {e,n,u}
 *          double *r        O   vector in ECEF coordinate {x,y,z}
 * Return : none
 *----------------------------------------------------------------------------*/
extern void enu2ecef(const double *pos, const double *e, double *r) {
  double E[9];
  xyz2enu(pos, E);
  matmul("TN", 3, 1, 3, E, e, r);
}
/* Transform covariance to local tangential coordinate -------------------------
 * Transform ECEF covariance to local tangential coordinate
 * Args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *P        I   covariance in ECEF coordinate
 *          double *Q        O   covariance in local tangential coordinate
 * Return : none
 *----------------------------------------------------------------------------*/
extern void covenu(const double *pos, const double *P, double *Q) {
  double E[9];
  xyz2enu(pos, E);
  double EP[9];
  matmul("NN", 3, 3, 3, E, P, EP);
  matmul("NT", 3, 3, 3, EP, E, Q);
}
/* Transform local ENU coordinate covariance to xyz-ecef -----------------------
 * Transform local ENU covariance to xyz-ecef coordinate
 * Args   : double *pos      I   geodetic position {lat,lon} (rad)
 *          double *Q        I   covariance in local ENU coordinate
 *          double *P        O   covariance in xyz-ecef coordinate
 * Return : none
 *----------------------------------------------------------------------------*/
extern void covecef(const double *pos, const double *Q, double *P) {
  double E[9];
  xyz2enu(pos, E);
  double EQ[9];
  matmul("TN", 3, 3, 3, E, Q, EQ);
  matmul("NN", 3, 3, 3, EQ, E, P);
}
/* Coordinate rotation matrix ------------------------------------------------*/
#define Rx(t, X)                             \
  do {                                       \
    (X)[0] = 1.0;                            \
    (X)[1] = (X)[2] = (X)[3] = (X)[6] = 0.0; \
    (X)[4] = (X)[8] = cos(t);                \
    (X)[7] = sin(t);                         \
    (X)[5] = -(X)[7];                        \
  } while (0)

#define Ry(t, X)                             \
  do {                                       \
    (X)[4] = 1.0;                            \
    (X)[1] = (X)[3] = (X)[5] = (X)[7] = 0.0; \
    (X)[0] = (X)[8] = cos(t);                \
    (X)[2] = sin(t);                         \
    (X)[6] = -(X)[2];                        \
  } while (0)

#define Rz(t, X)                             \
  do {                                       \
    (X)[8] = 1.0;                            \
    (X)[2] = (X)[5] = (X)[6] = (X)[7] = 0.0; \
    (X)[0] = (X)[4] = cos(t);                \
    (X)[3] = sin(t);                         \
    (X)[1] = -(X)[3];                        \
  } while (0)

/* Astronomical arguments: f={l,l',F,D,OMG} (rad) ----------------------------*/
static void ast_args(double t, double *f) {
  static const double fc[][5] = {/* Coefficients for IAU 1980 nutation */
                                 {134.96340251, 1717915923.2178, 31.8792, 0.051635, -0.00024470},
                                 {357.52910918, 129596581.0481, -0.5532, 0.000136, -0.00001149},
                                 {93.27209062, 1739527262.8478, -12.7512, -0.001037, 0.00000417},
                                 {297.85019547, 1602961601.2090, -6.3706, 0.006593, -0.00003169},
                                 {125.04455501, -6962890.2665, 7.4722, 0.007702, -0.00005939}};
  double tt[4];
  tt[0] = t;
  for (int i = 1; i < 4; i++) tt[i] = tt[i - 1] * t;

  for (int i = 0; i < 5; i++) {
    f[i] = fc[i][0] * 3600.0;
    for (int j = 0; j < 4; j++) f[i] += fc[i][j + 1] * tt[j];
    f[i] = fmod(f[i] * AS2R, 2.0 * PI);
  }
}
/* IAU 1980 nutation ---------------------------------------------------------*/
static void nut_iau1980(double t, const double *f, double *dpsi, double *deps) {
  static const double nut[106][10] = {{0, 0, 0, 0, 1, -6798.4, -171996, -174.2, 92025, 8.9},
                                      {0, 0, 2, -2, 2, 182.6, -13187, -1.6, 5736, -3.1},
                                      {0, 0, 2, 0, 2, 13.7, -2274, -0.2, 977, -0.5},
                                      {0, 0, 0, 0, 2, -3399.2, 2062, 0.2, -895, 0.5},
                                      {0, -1, 0, 0, 0, -365.3, -1426, 3.4, 54, -0.1},
                                      {1, 0, 0, 0, 0, 27.6, 712, 0.1, -7, 0.0},
                                      {0, 1, 2, -2, 2, 121.7, -517, 1.2, 224, -0.6},
                                      {0, 0, 2, 0, 1, 13.6, -386, -0.4, 200, 0.0},
                                      {1, 0, 2, 0, 2, 9.1, -301, 0.0, 129, -0.1},
                                      {0, -1, 2, -2, 2, 365.2, 217, -0.5, -95, 0.3},
                                      {-1, 0, 0, 2, 0, 31.8, 158, 0.0, -1, 0.0},
                                      {0, 0, 2, -2, 1, 177.8, 129, 0.1, -70, 0.0},
                                      {-1, 0, 2, 0, 2, 27.1, 123, 0.0, -53, 0.0},
                                      {1, 0, 0, 0, 1, 27.7, 63, 0.1, -33, 0.0},
                                      {0, 0, 0, 2, 0, 14.8, 63, 0.0, -2, 0.0},
                                      {-1, 0, 2, 2, 2, 9.6, -59, 0.0, 26, 0.0},
                                      {-1, 0, 0, 0, 1, -27.4, -58, -0.1, 32, 0.0},
                                      {1, 0, 2, 0, 1, 9.1, -51, 0.0, 27, 0.0},
                                      {-2, 0, 0, 2, 0, -205.9, -48, 0.0, 1, 0.0},
                                      {-2, 0, 2, 0, 1, 1305.5, 46, 0.0, -24, 0.0},
                                      {0, 0, 2, 2, 2, 7.1, -38, 0.0, 16, 0.0},
                                      {2, 0, 2, 0, 2, 6.9, -31, 0.0, 13, 0.0},
                                      {2, 0, 0, 0, 0, 13.8, 29, 0.0, -1, 0.0},
                                      {1, 0, 2, -2, 2, 23.9, 29, 0.0, -12, 0.0},
                                      {0, 0, 2, 0, 0, 13.6, 26, 0.0, -1, 0.0},
                                      {0, 0, 2, -2, 0, 173.3, -22, 0.0, 0, 0.0},
                                      {-1, 0, 2, 0, 1, 27.0, 21, 0.0, -10, 0.0},
                                      {0, 2, 0, 0, 0, 182.6, 17, -0.1, 0, 0.0},
                                      {0, 2, 2, -2, 2, 91.3, -16, 0.1, 7, 0.0},
                                      {-1, 0, 0, 2, 1, 32.0, 16, 0.0, -8, 0.0},
                                      {0, 1, 0, 0, 1, 386.0, -15, 0.0, 9, 0.0},
                                      {1, 0, 0, -2, 1, -31.7, -13, 0.0, 7, 0.0},
                                      {0, -1, 0, 0, 1, -346.6, -12, 0.0, 6, 0.0},
                                      {2, 0, -2, 0, 0, -1095.2, 11, 0.0, 0, 0.0},
                                      {-1, 0, 2, 2, 1, 9.5, -10, 0.0, 5, 0.0},
                                      {1, 0, 2, 2, 2, 5.6, -8, 0.0, 3, 0.0},
                                      {0, -1, 2, 0, 2, 14.2, -7, 0.0, 3, 0.0},
                                      {0, 0, 2, 2, 1, 7.1, -7, 0.0, 3, 0.0},
                                      {1, 1, 0, -2, 0, -34.8, -7, 0.0, 0, 0.0},
                                      {0, 1, 2, 0, 2, 13.2, 7, 0.0, -3, 0.0},
                                      {-2, 0, 0, 2, 1, -199.8, -6, 0.0, 3, 0.0},
                                      {0, 0, 0, 2, 1, 14.8, -6, 0.0, 3, 0.0},
                                      {2, 0, 2, -2, 2, 12.8, 6, 0.0, -3, 0.0},
                                      {1, 0, 0, 2, 0, 9.6, 6, 0.0, 0, 0.0},
                                      {1, 0, 2, -2, 1, 23.9, 6, 0.0, -3, 0.0},
                                      {0, 0, 0, -2, 1, -14.7, -5, 0.0, 3, 0.0},
                                      {0, -1, 2, -2, 1, 346.6, -5, 0.0, 3, 0.0},
                                      {2, 0, 2, 0, 1, 6.9, -5, 0.0, 3, 0.0},
                                      {1, -1, 0, 0, 0, 29.8, 5, 0.0, 0, 0.0},
                                      {1, 0, 0, -1, 0, 411.8, -4, 0.0, 0, 0.0},
                                      {0, 0, 0, 1, 0, 29.5, -4, 0.0, 0, 0.0},
                                      {0, 1, 0, -2, 0, -15.4, -4, 0.0, 0, 0.0},
                                      {1, 0, -2, 0, 0, -26.9, 4, 0.0, 0, 0.0},
                                      {2, 0, 0, -2, 1, 212.3, 4, 0.0, -2, 0.0},
                                      {0, 1, 2, -2, 1, 119.6, 4, 0.0, -2, 0.0},
                                      {1, 1, 0, 0, 0, 25.6, -3, 0.0, 0, 0.0},
                                      {1, -1, 0, -1, 0, -3232.9, -3, 0.0, 0, 0.0},
                                      {-1, -1, 2, 2, 2, 9.8, -3, 0.0, 1, 0.0},
                                      {0, -1, 2, 2, 2, 7.2, -3, 0.0, 1, 0.0},
                                      {1, -1, 2, 0, 2, 9.4, -3, 0.0, 1, 0.0},
                                      {3, 0, 2, 0, 2, 5.5, -3, 0.0, 1, 0.0},
                                      {-2, 0, 2, 0, 2, 1615.7, -3, 0.0, 1, 0.0},
                                      {1, 0, 2, 0, 0, 9.1, 3, 0.0, 0, 0.0},
                                      {-1, 0, 2, 4, 2, 5.8, -2, 0.0, 1, 0.0},
                                      {1, 0, 0, 0, 2, 27.8, -2, 0.0, 1, 0.0},
                                      {-1, 0, 2, -2, 1, -32.6, -2, 0.0, 1, 0.0},
                                      {0, -2, 2, -2, 1, 6786.3, -2, 0.0, 1, 0.0},
                                      {-2, 0, 0, 0, 1, -13.7, -2, 0.0, 1, 0.0},
                                      {2, 0, 0, 0, 1, 13.8, 2, 0.0, -1, 0.0},
                                      {3, 0, 0, 0, 0, 9.2, 2, 0.0, 0, 0.0},
                                      {1, 1, 2, 0, 2, 8.9, 2, 0.0, -1, 0.0},
                                      {0, 0, 2, 1, 2, 9.3, 2, 0.0, -1, 0.0},
                                      {1, 0, 0, 2, 1, 9.6, -1, 0.0, 0, 0.0},
                                      {1, 0, 2, 2, 1, 5.6, -1, 0.0, 1, 0.0},
                                      {1, 1, 0, -2, 1, -34.7, -1, 0.0, 0, 0.0},
                                      {0, 1, 0, 2, 0, 14.2, -1, 0.0, 0, 0.0},
                                      {0, 1, 2, -2, 0, 117.5, -1, 0.0, 0, 0.0},
                                      {0, 1, -2, 2, 0, -329.8, -1, 0.0, 0, 0.0},
                                      {1, 0, -2, 2, 0, 23.8, -1, 0.0, 0, 0.0},
                                      {1, 0, -2, -2, 0, -9.5, -1, 0.0, 0, 0.0},
                                      {1, 0, 2, -2, 0, 32.8, -1, 0.0, 0, 0.0},
                                      {1, 0, 0, -4, 0, -10.1, -1, 0.0, 0, 0.0},
                                      {2, 0, 0, -4, 0, -15.9, -1, 0.0, 0, 0.0},
                                      {0, 0, 2, 4, 2, 4.8, -1, 0.0, 0, 0.0},
                                      {0, 0, 2, -1, 2, 25.4, -1, 0.0, 0, 0.0},
                                      {-2, 0, 2, 4, 2, 7.3, -1, 0.0, 1, 0.0},
                                      {2, 0, 2, 2, 2, 4.7, -1, 0.0, 0, 0.0},
                                      {0, -1, 2, 0, 1, 14.2, -1, 0.0, 0, 0.0},
                                      {0, 0, -2, 0, 1, -13.6, -1, 0.0, 0, 0.0},
                                      {0, 0, 4, -2, 2, 12.7, 1, 0.0, 0, 0.0},
                                      {0, 1, 0, 0, 2, 409.2, 1, 0.0, 0, 0.0},
                                      {1, 1, 2, -2, 2, 22.5, 1, 0.0, -1, 0.0},
                                      {3, 0, 2, -2, 2, 8.7, 1, 0.0, 0, 0.0},
                                      {-2, 0, 2, 2, 2, 14.6, 1, 0.0, -1, 0.0},
                                      {-1, 0, 0, 0, 2, -27.3, 1, 0.0, -1, 0.0},
                                      {0, 0, -2, 2, 1, -169.0, 1, 0.0, 0, 0.0},
                                      {0, 1, 2, 0, 1, 13.1, 1, 0.0, 0, 0.0},
                                      {-1, 0, 4, 0, 2, 9.1, 1, 0.0, 0, 0.0},
                                      {2, 1, 0, -2, 0, 131.7, 1, 0.0, 0, 0.0},
                                      {2, 0, 0, 2, 0, 7.1, 1, 0.0, 0, 0.0},
                                      {2, 0, 2, -2, 1, 12.8, 1, 0.0, -1, 0.0},
                                      {2, 0, -2, 0, 1, -943.2, 1, 0.0, 0, 0.0},
                                      {1, -1, 0, -2, 0, -29.3, 1, 0.0, 0, 0.0},
                                      {-1, 0, 0, 1, 1, -388.3, 1, 0.0, 0, 0.0},
                                      {-1, -1, 0, 2, 1, 35.0, 1, 0.0, 0, 0.0},
                                      {0, 1, 0, 1, 0, 27.3, 1, 0.0, 0, 0.0}};

  *dpsi = *deps = 0.0;

  for (int i = 0; i < 106; i++) {
    double ang = 0.0;
    for (int j = 0; j < 5; j++) ang += nut[i][j] * f[j];
    *dpsi += (nut[i][6] + nut[i][7] * t) * sin(ang);
    *deps += (nut[i][8] + nut[i][9] * t) * cos(ang);
  }
  *dpsi *= 1E-4 * AS2R; /* 0.1 mas -> rad */
  *deps *= 1E-4 * AS2R;
}
/* ECI to ECEF transformation matrix -------------------------------------------
 * Compute ECI to ECEF transformation matrix
 * Args   : gtime_t tutc     I   time in UTC
 *          double *erpv     I   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 *          double *U        O   ECI to ECEF transformation matrix (3 x 3)
 *          double *gmst     IO  greenwich mean sidereal time (rad)
 *                               (NULL: no output)
 * Return : none
 * Note   : see ref [3] chap 5
 *          not thread-safe
 *----------------------------------------------------------------------------*/
extern void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst) {
  const double ep2000[] = {2000, 1, 1, 12, 0, 0};
  static gtime_t tutc_;
  static double U_[9], gmst_;

  char tstr[40];
  trace(4, "eci2ecef: tutc=%s\n", time2str(tutc, tstr, 3));

  /* TODO is the cache useful? Is the tolerance good? */
  if (fabs(timediff(tutc, tutc_)) < 0.01) { /* Read cache */
    for (int i = 0; i < 9; i++) U[i] = U_[i];
    if (gmst) *gmst = gmst_;
    return;
  }
  tutc_ = tutc;

  /* Terrestrial time */
  gtime_t tgps = utc2gpst(tutc_);
  double t = (timediff(tgps, epoch2time(ep2000)) + 19.0 + 32.184) / 86400.0 / 36525.0;
  double t2 = t * t;
  double t3 = t2 * t;

  /* Astronomical arguments */
  double f[5];
  ast_args(t, f);

  /* IAU 1976 precession */
  double ze = (2306.2181 * t + 0.30188 * t2 + 0.017998 * t3) * AS2R;
  double th = (2004.3109 * t - 0.42665 * t2 - 0.041833 * t3) * AS2R;
  double z = (2306.2181 * t + 1.09468 * t2 + 0.018203 * t3) * AS2R;
  double eps = (84381.448 - 46.8150 * t - 0.00059 * t2 + 0.001813 * t3) * AS2R;
  double R1[9], R2[9], R3[9];
  Rz(-z, R1);
  Ry(th, R2);
  Rz(-ze, R3);
  double R[9];
  matmul("NN", 3, 3, 3, R1, R2, R);
  double P[9];
  matmul("NN", 3, 3, 3, R, R3, P); /* P=Rz(-z)*Ry(th)*Rz(-ze) */

  /* IAU 1980 nutation */
  double dpsi, deps;
  nut_iau1980(t, f, &dpsi, &deps);
  Rx(-eps - deps, R1);
  Rz(-dpsi, R2);
  Rx(eps, R3);
  matmul("NN", 3, 3, 3, R1, R2, R);
  double N[9];
  matmul("NN", 3, 3, 3, R, R3, N); /* N=Rx(-eps)*Rz(-dspi)*Rx(eps) */

  /* Greenwich aparent sidereal time (rad) */
  gmst_ = utc2gmst(tutc_, erpv[2]);
  double gast = gmst_ + dpsi * cos(eps);
  gast += (0.00264 * sin(f[4]) + 0.000063 * sin(2.0 * f[4])) * AS2R;

  /* ECI to ECEF transformation matrix */
  Ry(-erpv[0], R1);
  Rx(-erpv[1], R2);
  Rz(gast, R3);
  double W[9];
  matmul("NN", 3, 3, 3, R1, R2, W);
  matmul("NN", 3, 3, 3, W, R3, R); /* W=Ry(-xp)*Rx(-yp) */
  double NP[9];
  matmul("NN", 3, 3, 3, N, P, NP);
  matmul("NN", 3, 3, 3, R, NP, U_); /* U=W*Rz(gast)*N*P */

  for (int i = 0; i < 9; i++) U[i] = U_[i];
  if (gmst) *gmst = gmst_;

  trace(5, "gmst=%.12f gast=%.12f\n", gmst_, gast);
  trace(5, "P=\n");
  tracemat(5, P, 3, 3, 15, 12);
  trace(5, "N=\n");
  tracemat(5, N, 3, 3, 15, 12);
  trace(5, "W=\n");
  tracemat(5, W, 3, 3, 15, 12);
  trace(5, "U=\n");
  tracemat(5, U, 3, 3, 15, 12);
}
/* Decode antenna parameter field --------------------------------------------*/
static int decodef(char *p, int n, double *v) {
  for (int i = 0; i < n; i++) v[i] = 0.0;

  int i;
  char *q;
  p = strtok_r(p, " ", &q);
  for (i = 0; p && i < n; p = strtok_r(NULL, " ", &q)) {
    v[i++] = atof(p) * 1E-3;
  }
  return i;
}
/* Add antenna parameter -----------------------------------------------------*/
static void addpcv(const pcv_t *pcv, pcvs_t *pcvs) {
  if (pcvs->nmax <= pcvs->n) {
    pcvs->nmax += 256;
    pcv_t *pcvs_pcv = (pcv_t *)realloc(pcvs->pcv, sizeof(pcv_t) * pcvs->nmax);
    if (!pcvs_pcv) {
      trace(1, "addpcv: memory allocation error\n");
      free(pcvs->pcv);
      pcvs->pcv = NULL;
      pcvs->n = pcvs->nmax = 0;
      return;
    }
    pcvs->pcv = pcvs_pcv;
  }
  pcvs->pcv[pcvs->n++] = *pcv;
}
/* Read ngs antenna parameter file -------------------------------------------*/
static bool readngspcv(const char *file, pcvs_t *pcvs) {
  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "ngs pcv file open error: %s\n", file);
    return false;
  }
  pcv_t pcv;
  static const pcv_t pcv0 = {0};
  int n = 0;
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    if (strlen(buff) >= 62 && buff[61] == '|') continue;

    if (buff[0] != ' ') n = 0; /* Start line */
    if (++n == 1) {
      pcv = pcv0;
      rtksetstr(pcv.type, sizeof(pcv.type), buff, 0, 61);
    } else if (n == 2) {
      double neu[3];
      if (decodef(buff, 3, neu) < 3) continue;
      pcv.off[0][0] = neu[1];
      pcv.off[0][1] = neu[0];
      pcv.off[0][2] = neu[2];
    } else if (n == 3)
      decodef(buff, 10, pcv.var[0]);
    else if (n == 4)
      decodef(buff, 9, pcv.var[0] + 10);
    else if (n == 5) {
      double neu[3];
      if (decodef(buff, 3, neu) < 3) continue;
      pcv.off[1][0] = neu[1];
      pcv.off[1][1] = neu[0];
      pcv.off[1][2] = neu[2];
    } else if (n == 6)
      decodef(buff, 10, pcv.var[1]);
    else if (n == 7) {
      decodef(buff, 9, pcv.var[1] + 10);
      addpcv(&pcv, pcvs);
    }
  }
  fclose(fp);

  return true;
}
/* Read antex file -----------------------------------------------------------*/
static bool readantex(const char *file, pcvs_t *pcvs) {
  trace(3, "readantex: file=%s\n", file);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "antex pcv file open error: %s\n", file);
    return false;
  }
  static const pcv_t pcv0 = {0};
  pcv_t pcv;
  int freq = 0, state = 0, freqs[] = {1, 2, 5, 0};
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    if (strlen(buff) < 60 || strstr(buff + 60, "COMMENT")) continue;

    if (strstr(buff + 60, "START OF ANTENNA")) {
      pcv = pcv0;
      state = 1;
    }
    if (strstr(buff + 60, "END OF ANTENNA")) {
      addpcv(&pcv, pcvs);
      state = 0;
    }
    if (!state) continue;

    if (strstr(buff + 60, "TYPE / SERIAL NO")) {
      rtksetstr(pcv.type, sizeof(pcv.type), buff, 0, 20);
      rtksetstr(pcv.code, sizeof(pcv.code), buff, 20, 40);
      if (strlen(pcv.code) == 3) {
        pcv.sat = satid2no(pcv.code);
      }
    } else if (strstr(buff + 60, "VALID FROM")) {
      if (!str2time(buff, 0, 43, &pcv.ts)) continue;
    } else if (strstr(buff + 60, "VALID UNTIL")) {
      if (!str2time(buff, 0, 43, &pcv.te)) continue;
    } else if (strstr(buff + 60, "START OF FREQUENCY")) {
      if (!pcv.sat && buff[3] != 'G') continue; /* Only read rec ant for GPS */
      int f;
      if (sscanf(buff + 4, "%d", &f) < 1) continue;
      int i;
      for (i = 0; freqs[i]; i++)
        if (freqs[i] == f) break;
      if (freqs[i]) freq = i + 1;
      /* For Galileo E5b: save to E2, not E7  */
      if (satsys(pcv.sat, NULL) == SYS_GAL && f == 7) freq = 2;
    } else if (strstr(buff + 60, "END OF FREQUENCY")) {
      freq = 0;
    } else if (strstr(buff + 60, "NORTH / EAST / UP")) {
      if (freq < 1 || NFREQ < freq) continue;
      double neu[3];
      if (decodef(buff, 3, neu) < 3) continue;
      pcv.off[freq - 1][0] = neu[pcv.sat ? 0 : 1]; /* x or e */
      pcv.off[freq - 1][1] = neu[pcv.sat ? 1 : 0]; /* y or n */
      pcv.off[freq - 1][2] = neu[2];               /* z or u */
    } else if (strstr(buff, "NOAZI")) {
      if (freq < 1 || NFREQ < freq) continue;
      int i = decodef(buff + 8, 19, pcv.var[freq - 1]);
      if (i <= 0) continue;
      for (; i < 19; i++) pcv.var[freq - 1][i] = pcv.var[freq - 1][i - 1];
    }
  }
  fclose(fp);

  return true;
}
/* Read antenna parameters -----------------------------------------------------
 * Read antenna parameters
 * Args   : char   *file       I   antenna parameter file (antex)
 *          pcvs_t *pcvs       IO  antenna parameters
 * Return : status (true:ok,false:file open error)
 * Notes  : file with the extension .atx or .ATX is recognized as antex
 *          file except for antex is recognized ngs antenna parameters
 *          see reference [3]
 *          only support non-azimuth-depedent parameters
 *----------------------------------------------------------------------------*/
extern bool readpcv(const char *file, pcvs_t *pcvs) {
  trace(3, "readpcv: file=%s\n", file);

  const char *ext = strrchr(file, '.');
  if (!ext) ext = "";

  bool stat;
  if (!strcmp(ext, ".atx") || !strcmp(ext, ".ATX")) {
    stat = readantex(file, pcvs);
  } else {
    stat = readngspcv(file, pcvs);
  }
  for (int i = 0; i < pcvs->n; i++) {
    pcv_t *pcv = pcvs->pcv + i;
    trace(4, "sat=%2d type=%20s code=%s off=%8.4f %8.4f %8.4f  %8.4f %8.4f %8.4f\n", pcv->sat,
          pcv->type, pcv->code, pcv->off[0][0], pcv->off[0][1], pcv->off[0][2], pcv->off[1][0],
          pcv->off[1][1], pcv->off[1][2]);
  }
  return stat;
}
/* Search antenna parameter ----------------------------------------------------
 * Read satellite antenna phase center position
 * Args   : int    sat         I   satellite number (0: receiver antenna)
 *          char   *type       I   antenna type for receiver antenna
 *          gtime_t time       I   time to search parameters
 *          pcvs_t *pcvs       IO  antenna parameters
 * Return : antenna parameter (NULL: no antenna)
 *----------------------------------------------------------------------------*/
extern pcv_t *searchpcv(int sat, const char *type, gtime_t time, const pcvs_t *pcvs) {
  trace(4, "searchpcv: sat=%2d type=%s\n", sat, type);

  if (sat) { /* Search satellite antenna */
    for (int i = 0; i < pcvs->n; i++) {
      pcv_t *pcv = pcvs->pcv + i;
      if (pcv->sat != sat) continue;
      if (pcv->ts.time != 0 && timediff(pcv->ts, time) > 0.0) continue;
      if (pcv->te.time != 0 && timediff(pcv->te, time) < 0.0) continue;
      return pcv;
    }
  } else {
    char buff[MAXANT];
    rtkstrcpy(buff, sizeof(buff), type);
    int n = 0;
    char *types[2], *p, *q;
    for (p = strtok_r(buff, " ", &q); p && n < 2; p = strtok_r(NULL, " ", &q)) types[n++] = p;
    if (n <= 0) return NULL;

    /* Search receiver antenna with radome at first */
    for (int i = 0; i < pcvs->n; i++) {
      pcv_t *pcv = pcvs->pcv + i;
      int j;
      for (j = 0; j < n; j++)
        if (!strstr(pcv->type, types[j])) break;
      if (j >= n) return pcv;
    }
    /* Search receiver antenna without radome */
    for (int i = 0; i < pcvs->n; i++) {
      pcv_t *pcv = pcvs->pcv + i;
      if (strstr(pcv->type, types[0]) != pcv->type) continue;

      trace(2, "pcv without radome is used type=%s\n", type);
      return pcv;
    }
  }
  return NULL;
}
/* Read station positions ------------------------------------------------------
 * Read positions from station position file
 * Args   : char  *file      I   station position file containing
 *                               lat(deg) lon(deg) height(m) name in a line
 *          char  *rcvs      I   station name
 *          double *pos      O   station position {lat,lon,h} (rad/m)
 *                               (all 0 if search error)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void readpos(const char *file, const char *rcv, double *pos) {
  trace(3, "readpos: file=%s\n", file);

  static double poss[2048][3];
  static char stas[2048][16];

  FILE *fp = fopen(file, "r");
  if (!fp) {
    fprintf(stderr, "reference position file open error : %s\n", file);
    return;
  }
  char buff[256];
  int np = 0;
  while (np < 2048 && fgets(buff, sizeof(buff), fp)) {
    if (buff[0] == '%' || buff[0] == '#') continue;
    char str[256];
    if (sscanf(buff, "%lf %lf %lf %255s", &poss[np][0], &poss[np][1], &poss[np][2], str) < 4)
      continue;
    rtksnprintf(stas[np++], sizeof(stas[0]), "%.15s", str);
  }
  fclose(fp);
  int len = (int)strlen(rcv);
  for (int i = 0; i < np; i++) {
    if (strncmp(stas[i], rcv, len)) continue;
    for (int j = 0; j < 3; j++) pos[j] = poss[i][j];
    pos[0] *= D2R;
    pos[1] *= D2R;
    return;
  }
  pos[0] = pos[1] = pos[2] = 0.0;
}
/* Read blq record -----------------------------------------------------------*/
static bool readblqrecord(FILE *fp, double *odisp) {
  char buff[256];
  int n = 0;
  while (fgets(buff, sizeof(buff), fp)) {
    if (!strncmp(buff, "$$", 2)) continue;
    double v[11];
    if (sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", v, v + 1, v + 2, v + 3, v + 4,
               v + 5, v + 6, v + 7, v + 8, v + 9, v + 10) < 11)
      continue;
    for (int i = 0; i < 11; i++) odisp[n + i * 6] = v[i];
    if (++n == 6) return true;
  }
  return false;
}
/* Read blq ocean tide loading parameters --------------------------------------
 * Read blq ocean tide loading parameters
 * Args   : char   *file       I   BLQ ocean tide loading parameter file
 *          char   *sta        I   station name
 *          double *odisp      O   ocean tide loading parameters
 * Return : status (true:ok,false:file open error)
 *----------------------------------------------------------------------------*/
extern bool readblq(const char *file, const char *sta, double *odisp) {
  /* Station name to upper case */
  char staname[17] = "";
  if (sscanf(sta, "%16s", staname) < 1) return 0;
  for (char *p = staname; (*p = (char)toupper((int)(*p))); p++)
    ;

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "blq file open error: file=%s\n", file);
    return false;
  }
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    if (!strncmp(buff, "$$", 2) || strlen(buff) < 2) continue;

    char name[17];
    if (sscanf(buff + 2, "%16s", name) < 1) continue;
    for (char *p = name; (*p = (char)toupper((int)(*p))); p++)
      ;
    if (strcmp(name, staname)) continue;

    /* Read blq record */
    if (readblqrecord(fp, odisp)) {
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  trace(2, "no otl parameters: sta=%s file=%s\n", sta, file);
  return false;
}
/* Read earth rotation parameters ----------------------------------------------
 * Read earth rotation parameters
 * Args   : char   *file       I   IGS ERP file (IGS ERP ver.2)
 *          erp_t  *erp        O   earth rotation parameters
 * Return : status (true:ok,false:file open error)
 *----------------------------------------------------------------------------*/
extern bool readerp(const char *file, erp_t *erp) {
  trace(3, "readerp: file=%s\n", file);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "erp file open error: file=%s\n", file);
    return false;
  }
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    double v[14] = {0};
    if (sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", v, v + 1, v + 2,
               v + 3, v + 4, v + 5, v + 6, v + 7, v + 8, v + 9, v + 10, v + 11, v + 12,
               v + 13) < 5) {
      continue;
    }
    if (erp->n >= erp->nmax) {
      erp->nmax = erp->nmax <= 0 ? 128 : erp->nmax * 2;
      erpd_t *erp_data = (erpd_t *)realloc(erp->data, sizeof(erpd_t) * erp->nmax);
      if (!erp_data) {
        free(erp->data);
        erp->data = NULL;
        erp->n = erp->nmax = 0;
        fclose(fp);
        return false;
      }
      erp->data = erp_data;
    }
    erp->data[erp->n].mjd = v[0];
    erp->data[erp->n].xp = v[1] * 1E-6 * AS2R;
    erp->data[erp->n].yp = v[2] * 1E-6 * AS2R;
    erp->data[erp->n].ut1_utc = v[3] * 1E-7;
    erp->data[erp->n].lod = v[4] * 1E-7;
    erp->data[erp->n].xpr = v[12] * 1E-6 * AS2R;
    erp->data[erp->n++].ypr = v[13] * 1E-6 * AS2R;
  }
  fclose(fp);
  return true;
}
/* Get earth rotation parameter values -----------------------------------------
 * Get earth rotation parameter values
 * Args   : erp_t  *erp        I   earth rotation parameters
 *          gtime_t time       I   time (GPST)
 *          double *erpv       O   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool geterp(const erp_t *erp, gtime_t time, double *erpv) {
  const double ep[] = {2000, 1, 1, 12, 0, 0};

  trace(4, "geterp:\n");

  if (erp->n <= 0) return false;

  double mjd = 51544.5 + (timediff(gpst2utc(time), epoch2time(ep))) / 86400.0;

  if (mjd <= erp->data[0].mjd) {
    double day = mjd - erp->data[0].mjd;
    erpv[0] = erp->data[0].xp + erp->data[0].xpr * day;
    erpv[1] = erp->data[0].yp + erp->data[0].ypr * day;
    erpv[2] = erp->data[0].ut1_utc - erp->data[0].lod * day;
    erpv[3] = erp->data[0].lod;
    return true;
  }
  if (mjd >= erp->data[erp->n - 1].mjd) {
    double day = mjd - erp->data[erp->n - 1].mjd;
    erpv[0] = erp->data[erp->n - 1].xp + erp->data[erp->n - 1].xpr * day;
    erpv[1] = erp->data[erp->n - 1].yp + erp->data[erp->n - 1].ypr * day;
    erpv[2] = erp->data[erp->n - 1].ut1_utc - erp->data[erp->n - 1].lod * day;
    erpv[3] = erp->data[erp->n - 1].lod;
    return true;
  }
  int j = 0;
  for (int k = erp->n - 1; j < k - 1;) {
    int i = (j + k) / 2;
    if (mjd < erp->data[i].mjd)
      k = i;
    else
      j = i;
  }
  double a;
  if (erp->data[j].mjd == erp->data[j + 1].mjd) {
    a = 0.5;
  } else {
    a = (mjd - erp->data[j].mjd) / (erp->data[j + 1].mjd - erp->data[j].mjd);
  }
  erpv[0] = (1.0 - a) * erp->data[j].xp + a * erp->data[j + 1].xp;
  erpv[1] = (1.0 - a) * erp->data[j].yp + a * erp->data[j + 1].yp;
  erpv[2] = (1.0 - a) * erp->data[j].ut1_utc + a * erp->data[j + 1].ut1_utc;
  erpv[3] = (1.0 - a) * erp->data[j].lod + a * erp->data[j + 1].lod;
  return true;
}
extern int navncnt(nav_t *nav) {
  int n = 0;
  for (int i = 0; i < MAXSAT; i++) n += nav->n[i];
  return n;
}
extern int navngcnt(nav_t *nav) {
  int n = 0;
  for (int i = 0; i < NSATGLO; i++) n += nav->ng[i];
  return n;
}
extern int navnscnt(nav_t *nav) {
  int n = 0;
  for (int i = 0; i < NSATSBS; i++) n += nav->ns[i];
  return n;
}
/* Compare ephemeris ---------------------------------------------------------*/
static int cmpeph(const void *p1, const void *p2) {
  eph_t *q1 = (eph_t *)p1, *q2 = (eph_t *)p2;
  return q1->ttr.time != q2->ttr.time
             ? (int)(q1->ttr.time - q2->ttr.time)
             : (q1->toe.time != q2->toe.time ? (int)(q1->toe.time - q2->toe.time)
                                             : q1->sat - q2->sat);
}
/* Sort and unique ephemeris -------------------------------------------------*/
static void uniqeph(nav_t *nav) {
  int ns = 0, ne = 0;

  for (int k = 0; k < MAXSAT; k++) {
    if (nav->n[k] <= 0) continue;
    ns += nav->n[k];

    qsort(nav->eph[k], nav->n[k], sizeof(eph_t), cmpeph);

    int j = 0;
    for (int i = 1; i < nav->n[k]; i++) {
      if (nav->eph[k][i].toe.time != nav->eph[k][j].toe.time ||
          nav->eph[k][i].iode != nav->eph[k][j].iode)
        nav->eph[k][++j] = nav->eph[k][i];
    }
    nav->n[k] = j + 1;
    ne += j + 1;

    eph_t *nav_eph;
    if (!(nav_eph = (eph_t *)realloc(nav->eph[k], sizeof(eph_t) * nav->n[k]))) {
      trace(1, "uniqeph malloc error n=%d\n", nav->n[k]);
      free(nav->eph[k]);
      nav->eph[k] = NULL;
      nav->n[k] = nav->nmax[k] = 0;
      return;
    }
    nav->eph[k] = nav_eph;
    nav->nmax[k] = nav->n[k];
  }

  trace(4, "uniqeph: n=%d %d\n", ns, ne);
}
/* Compare GLONASS ephemeris -------------------------------------------------*/
static int cmpgeph(const void *p1, const void *p2) {
  geph_t *q1 = (geph_t *)p1, *q2 = (geph_t *)p2;
  return q1->tof.time != q2->tof.time
             ? (int)(q1->tof.time - q2->tof.time)
             : (q1->toe.time != q2->toe.time ? (int)(q1->toe.time - q2->toe.time)
                                             : q1->sat - q2->sat);
}
/* Sort and unique GLONASS ephemeris -----------------------------------------*/
static void uniqgeph(nav_t *nav) {
  int ns = 0, ne = 0;

  trace(3, "uniqgeph\n");

  for (int k = 0; k < NSATGLO; k++) {
    if (nav->ng[k] <= 0) continue;
    ns += nav->ng[k];

    qsort(nav->geph[k], nav->ng[k], sizeof(geph_t), cmpgeph);

    int j = 0;
    for (int i = 0; i < nav->ng[k]; i++) {
      if (nav->geph[k][i].toe.time != nav->geph[k][j].toe.time ||
          nav->geph[k][i].svh != nav->geph[k][j].svh) {
        nav->geph[k][++j] = nav->geph[k][i];
      }
    }
    nav->ng[k] = j + 1;
    ne += j + 1;

    geph_t *nav_geph;
    if (!(nav_geph = (geph_t *)realloc(nav->geph[k], sizeof(geph_t) * nav->ng[k]))) {
      trace(1, "uniqgeph malloc error ng=%d\n", nav->ng[k]);
      free(nav->geph[k]);
      nav->geph[k] = NULL;
      nav->ng[k] = nav->ngmax[k] = 0;
      return;
    }
    nav->geph[k] = nav_geph;
    nav->ngmax[k] = nav->ng[k];
  }

  trace(4, "uniqgeph: ng=%d %d\n", ns, ne);
}
/* Compare SBAS ephemeris ----------------------------------------------------*/
static int cmpseph(const void *p1, const void *p2) {
  seph_t *q1 = (seph_t *)p1, *q2 = (seph_t *)p2;
  return q1->tof.time != q2->tof.time
             ? (int)(q1->tof.time - q2->tof.time)
             : (q1->t0.time != q2->t0.time ? (int)(q1->t0.time - q2->t0.time) : q1->sat - q2->sat);
}
/* Sort and unique SBAS ephemeris --------------------------------------------*/
static void uniqseph(nav_t *nav) {
  int ns = 0, ne = 0;

  for (int k = 0; k < NSATSBS; k++) {
    if (nav->ns[k] <= 0) continue;
    ns += nav->ns[k];

    qsort(nav->seph[k], nav->ns[k], sizeof(seph_t), cmpseph);

    int j = 0;
    for (int i = 0; i < nav->ns[k]; i++) {
      if (nav->seph[k][i].t0.time != nav->seph[k][j].t0.time) {
        nav->seph[k][++j] = nav->seph[k][i];
      }
    }
    nav->ns[k] = j + 1;
    ne += j + 1;

    seph_t *nav_seph;
    if (!(nav_seph = (seph_t *)realloc(nav->seph[k], sizeof(seph_t) * nav->ns[k]))) {
      trace(1, "uniqseph malloc error ns=%d\n", nav->ns[k]);
      free(nav->seph[k]);
      nav->seph[k] = NULL;
      nav->ns[k] = nav->nsmax[k] = 0;
      return;
    }
    nav->seph[k] = nav_seph;
    nav->nsmax[k] = nav->ns[k];
  }

  trace(4, "uniqseph: ns=%d %d\n", ns, ne);
}
/* Unique ephemerides ----------------------------------------------------------
 * Unique ephemerides in navigation data and update carrier wave length
 * Args   : nav_t *nav    IO     navigation data
 * Return : number of epochs
 *----------------------------------------------------------------------------*/
extern void uniqnav(nav_t *nav) {
  trace(3, "uniqnav\n");

  /* Unique ephemeris */
  uniqeph(nav);
  uniqgeph(nav);
  uniqseph(nav);
}
/* Compare observation data --------------------------------------------------*/
static int cmpobs(const void *p1, const void *p2) {
  obsd_t *q1 = (obsd_t *)p1, *q2 = (obsd_t *)p2;
  double tt = timediff(q1->time, q2->time);
  if (fabs(tt) > DTTOL) return tt < 0 ? -1 : 1;
  if (q1->rcv != q2->rcv) return (int)q1->rcv - (int)q2->rcv;
  return (int)q1->sat - (int)q2->sat;
}
/* Sort and unique observation data --------------------------------------------
 * Sort and unique observation data by time, rcv, sat
 * Args   : obs_t *obs    IO     observation data
 * Return : number of epochs
 *----------------------------------------------------------------------------*/
extern int sortobs(obs_t *obs) {
  trace(3, "sortobs: nobs=%d\n", obs->n);

  if (obs->n <= 0) return 0;

  qsort(obs->data, obs->n, sizeof(obsd_t), cmpobs);

  /* Delete duplicated data */
  int j = 0;
  for (int i = 0; i < obs->n; i++) {
    if (obs->data[i].sat != obs->data[j].sat || obs->data[i].rcv != obs->data[j].rcv ||
        timediff(obs->data[i].time, obs->data[j].time) != 0.0) {
      obs->data[++j] = obs->data[i];
    }
  }
  obs->n = j + 1;

  int n = 0;
  for (int i = 0, k; i < obs->n; i = k, n++) {
    for (k = i + 1; k < obs->n; k++) {
      if (timediff(obs->data[k].time, obs->data[i].time) > DTTOL) break;
    }
  }
  return n;
}
/* Screen by time --------------------------------------------------------------
 * Screening by time start, time end, and time interval
 * Args   : gtime_t time  I      time
 *          gtime_t ts    I      time start (ts.time==0:no screening by ts)
 *          gtime_t te    I      time end   (te.time==0:no screening by te)
 *          double  tint  I      time interval (s) (0.0:no screen by tint)
 * Return : true:on condition, false:not on condition
 *----------------------------------------------------------------------------*/
extern bool screent(gtime_t time, gtime_t ts, gtime_t te, double tint) {
  return (tint <= 0.0 || fmod(time2gpst(time, NULL) + DTTOL, tint) <= DTTOL * 2.0) &&
         (ts.time == 0 || timediff(time, ts) >= -DTTOL) &&
         (te.time == 0 || timediff(time, te) < DTTOL);
}
/* Read/save navigation data ---------------------------------------------------
 * Save or load navigation data
 * Args   : char    file  I      file path
 *          nav_t   nav   O/I    navigation data
 * Return : status (true:ok,false:no file)
 *----------------------------------------------------------------------------*/
extern bool readnav(const char *file, nav_t *nav) {
  trace(3, "loadnav: file=%s\n", file);

  FILE *fp = fopen(file, "r");
  if (!fp) return false;

  eph_t eph0 = {0};
  geph_t geph0 = {0};

  char buff[4096];
  while (fgets(buff, sizeof(buff), fp)) {
    if (!strncmp(buff, "IONUTC", 6)) {
      for (int i = 0; i < 8; i++) nav->ion_gps[i] = 0.0;
      for (int i = 0; i < 8; i++) nav->utc_gps[i] = 0.0;
      (void)sscanf(buff, "IONUTC,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &nav->ion_gps[0], &nav->ion_gps[1], &nav->ion_gps[2], &nav->ion_gps[3],
                   &nav->ion_gps[4], &nav->ion_gps[5], &nav->ion_gps[6], &nav->ion_gps[7],
                   &nav->utc_gps[0], &nav->utc_gps[1], &nav->utc_gps[2], &nav->utc_gps[3],
                   &nav->utc_gps[4]);
      continue;
    }
    char *p = strchr(buff, ',');
    if (p)
      *p = '\0';
    else
      continue;
    int sat = satid2no(buff);
    if (!sat) continue;
    int prn;
    if (satsys(sat, &prn) == SYS_GLO) {
      nav->geph[prn - 1][0] = geph0;
      nav->geph[prn - 1][0].sat = sat;
      long toe_time = 0, tof_time = 0;
      (void)sscanf(p + 1,
                   "%d,%d,%d,%d,%d,%ld,%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
                   "%lf,%lf,%lf,%lf",
                   &nav->geph[prn - 1][0].iode, &nav->geph[prn - 1][0].frq,
                   &nav->geph[prn - 1][0].svh, &nav->geph[prn - 1][0].sva,
                   &nav->geph[prn - 1][0].age, &toe_time, &tof_time, &nav->geph[prn - 1][0].pos[0],
                   &nav->geph[prn - 1][0].pos[1], &nav->geph[prn - 1][0].pos[2],
                   &nav->geph[prn - 1][0].vel[0], &nav->geph[prn - 1][0].vel[1],
                   &nav->geph[prn - 1][0].vel[2], &nav->geph[prn - 1][0].acc[0],
                   &nav->geph[prn - 1][0].acc[1], &nav->geph[prn - 1][0].acc[2],
                   &nav->geph[prn - 1][0].taun, &nav->geph[prn - 1][0].gamn,
                   &nav->geph[prn - 1][0].dtaun);
      nav->geph[prn - 1][0].toe.time = toe_time;
      nav->geph[prn - 1][0].tof.time = tof_time;
    } else {
      nav->eph[sat - 1][0] = eph0;
      nav->eph[sat - 1][0].sat = sat;
      long toe_time = 0, toc_time = 0, ttr_time = 0;
      (void)sscanf(p + 1,
                   "%d,%d,%d,%d,%ld,%ld,%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
                   "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d",
                   &nav->eph[sat - 1][0].iode, &nav->eph[sat - 1][0].iodc,
                   &nav->eph[sat - 1][0].sva, &nav->eph[sat - 1][0].svh, &toe_time, &toc_time,
                   &ttr_time, &nav->eph[sat - 1][0].A, &nav->eph[sat - 1][0].e,
                   &nav->eph[sat - 1][0].i0, &nav->eph[sat - 1][0].OMG0, &nav->eph[sat - 1][0].omg,
                   &nav->eph[sat - 1][0].M0, &nav->eph[sat - 1][0].deln, &nav->eph[sat - 1][0].OMGd,
                   &nav->eph[sat - 1][0].idot, &nav->eph[sat - 1][0].crc, &nav->eph[sat - 1][0].crs,
                   &nav->eph[sat - 1][0].cuc, &nav->eph[sat - 1][0].cus, &nav->eph[sat - 1][0].cic,
                   &nav->eph[sat - 1][0].cis, &nav->eph[sat - 1][0].toes, &nav->eph[sat - 1][0].fit,
                   &nav->eph[sat - 1][0].f0, &nav->eph[sat - 1][0].f1, &nav->eph[sat - 1][0].f2,
                   &nav->eph[sat - 1][0].tgd[0], &nav->eph[sat - 1][0].code,
                   &nav->eph[sat - 1][0].flag);
      nav->eph[sat - 1][0].toe.time = toe_time;
      nav->eph[sat - 1][0].toc.time = toc_time;
      nav->eph[sat - 1][0].ttr.time = ttr_time;
    }
  }
  fclose(fp);
  return true;
}
extern bool savenav(const char *file, const nav_t *nav) {
  trace(3, "savenav: file=%s\n", file);

  FILE *fp = fopen(file, "w");
  if (!fp) return false;

  for (int i = 0; i < MAXSAT; i++) {
    if (nav->eph[i][0].ttr.time == 0) continue;
    char id[8];
    satno2id(nav->eph[i][0].sat, id);
    fprintf(fp,
            "%s,%d,%d,%d,%d,%d,%d,%d,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,"
            "%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,"
            "%.14E,%.14E,%.14E,%.14E,%.14E,%d,%d\n",
            id, nav->eph[i][0].iode, nav->eph[i][0].iodc, nav->eph[i][0].sva, nav->eph[i][0].svh,
            (int)nav->eph[i][0].toe.time, (int)nav->eph[i][0].toc.time,
            (int)nav->eph[i][0].ttr.time, nav->eph[i][0].A, nav->eph[i][0].e, nav->eph[i][0].i0,
            nav->eph[i][0].OMG0, nav->eph[i][0].omg, nav->eph[i][0].M0, nav->eph[i][0].deln,
            nav->eph[i][0].OMGd, nav->eph[i][0].idot, nav->eph[i][0].crc, nav->eph[i][0].crs,
            nav->eph[i][0].cuc, nav->eph[i][0].cus, nav->eph[i][0].cic, nav->eph[i][0].cis,
            nav->eph[i][0].toes, nav->eph[i][0].fit, nav->eph[i][0].f0, nav->eph[i][0].f1,
            nav->eph[i][0].f2, nav->eph[i][0].tgd[0], nav->eph[i][0].code, nav->eph[i][0].flag);
  }
  for (int i = 0; i < MAXPRNGLO; i++) {
    if (nav->geph[i][0].tof.time == 0) continue;
    char id[8];
    satno2id(nav->geph[i][0].sat, id);
    fprintf(fp,
            "%s,%d,%d,%d,%d,%d,%d,%d,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,"
            "%.14E,%.14E,%.14E,%.14E,%.14E,%.14E\n",
            id, nav->geph[i][0].iode, nav->geph[i][0].frq, nav->geph[i][0].svh, nav->geph[i][0].sva,
            nav->geph[i][0].age, (int)nav->geph[i][0].toe.time, (int)nav->geph[i][0].tof.time,
            nav->geph[i][0].pos[0], nav->geph[i][0].pos[1], nav->geph[i][0].pos[2],
            nav->geph[i][0].vel[0], nav->geph[i][0].vel[1], nav->geph[i][0].vel[2],
            nav->geph[i][0].acc[0], nav->geph[i][0].acc[1], nav->geph[i][0].acc[2],
            nav->geph[i][0].taun, nav->geph[i][0].gamn, nav->geph[i][0].dtaun);
  }
  /*fprintf(fp,"IONUTC,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,"
             "%.14E,%.14E,%.14E,%.0f",
          nav->ion_gps[0],nav->ion_gps[1],nav->ion_gps[2],nav->ion_gps[3],
          nav->ion_gps[4],nav->ion_gps[5],nav->ion_gps[6],nav->ion_gps[7],
          nav->utc_gps[0],nav->utc_gps[1],nav->utc_gps[2],nav->utc_gps[3],
          nav->utc_gps[4]);*/

  fclose(fp);
  return true;
}
/* Free observation data -------------------------------------------------------
 * Free memory for observation data
 * Args   : obs_t *obs    IO     observation data
 * Return : none
 *----------------------------------------------------------------------------*/
extern void freeobs(obs_t *obs) {
  free(obs->data);
  obs->data = NULL;
  obs->n = obs->nmax = 0;
}
/* Free navigation data --------------------------------------------------------
 * Free memory for navigation data
 * Args   : nav_t *nav    IO     navigation data
 *          int   opt     I      option (or of followings)
 *                               (0x01: GPS/QZS ephemeris, 0x02: GLONASS ephemeris,
 *                                0x04: SBAS ephemeris,    0x08: precise ephemeris,
 *                                0x10: precise clock      0x20: almanac,
 *                                0x40: TEC data)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void freenav(nav_t *nav, int opt) {
  if (opt & 0x01) {
    for (int i = 0; i < MAXSAT; i++) {
      free(nav->eph[i]);
      nav->eph[i] = NULL;
      nav->n[i] = nav->nmax[i] = 0;
    }
  }
  if (opt & 0x02) {
    for (int i = 0; i < NSATGLO; i++) {
      free(nav->geph[i]);
      nav->geph[i] = NULL;
      nav->ng[i] = nav->ngmax[i] = 0;
    }
  }
  if (opt & 0x04) {
    for (int i = 0; i < NSATSBS; i++) {
      free(nav->seph[i]);
      nav->seph[i] = NULL;
      nav->ns[i] = nav->nsmax[i] = 0;
    }
  }
  if (opt & 0x08) {
    free(nav->peph);
    nav->peph = NULL;
    nav->ne = nav->nemax = 0;
  }
  if (opt & 0x10) {
    free(nav->pclk);
    nav->pclk = NULL;
    nav->nc = nav->ncmax = 0;
  }
  if (opt & 0x20) {
    free(nav->alm);
    nav->alm = NULL;
    nav->na = nav->namax = 0;
  }
  if (opt & 0x40) {
    free(nav->tec);
    nav->tec = NULL;
    nav->nt = nav->ntmax = 0;
  }
}

/* Execute command -------------------------------------------------------------
 * Execute command line by operating system shell
 * Args   : char   *cmd      I   command line
 * Return : execution status (0:ok,0>:error)
 *----------------------------------------------------------------------------*/
extern int execcmd(const char *cmd) {
#ifdef WIN32
  trace(3, "execcmd: cmd=%s\n", cmd);

  STARTUPINFO si = {0};
  si.cb = sizeof(si);
  char cmds[1024];
  rtksnprintf(cmds, sizeof(cmds), "cmd /c %s", cmd);
  PROCESS_INFORMATION info;
  if (!CreateProcess(NULL, (LPTSTR)cmds, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si,
                     &info))
    return -1;
  WaitForSingleObject(info.hProcess, INFINITE);
  DWORD stat;
  if (!GetExitCodeProcess(info.hProcess, &stat)) stat = -1;
  CloseHandle(info.hProcess);
  CloseHandle(info.hThread);
  return (int)stat;
#else
  trace(3, "execcmd: cmd=%s\n", cmd);

  return system(cmd);
#endif
}
/* Expand file path ------------------------------------------------------------
 * Expand file path with wild-card (*) in file
 * Args   : char   *path     I   file path to expand (captal insensitive)
 *          char   *paths    O   expanded file paths
 *          size_t size      I   expanded file path buffer size
 *          int    nmax      I   max number of expanded file paths
 * Return : number of expanded file paths
 * Notes  : the order of expanded files is alphabetical order
 *----------------------------------------------------------------------------*/
extern int expath(const char *path, char *paths[], size_t size, int nmax) {
#ifdef WIN32
  trace(3, "expath  : path=%s nmax=%d\n", path, nmax);

  char *p = strrchr(path, '\\');
  char dir[FNSIZE] = "";
  if (p) {
    size_t n = p - path + 1;
    rtkesubstrcpy(dir, sizeof(dir), path, 0, n);
  }
  WIN32_FIND_DATA file;
  HANDLE h = FindFirstFile((LPCTSTR)path, &file);
  if (h == INVALID_HANDLE_VALUE) {
    rtkstrcpy(paths[0], size, path);
    return 1;
  }
  rtksnprintf(paths[n++], size, "%s%s", dir, file.cFileName);
  int n = 0;
  while (FindNextFile(h, &file) && n < nmax) {
    if (file.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
    rtksnprintf(paths[n++], size, "%s%s", dir, file.cFileName);
  }
  FindClose(h);
#else
  trace(3, "expath  : path=%s nmax=%d\n", path, nmax);

  const char *file = path;
  char dir[FNSIZE] = "";
  char *p = strrchr(path, '/');
  if (p || (p = strrchr(path, '\\'))) {
    file = p + 1;
    size_t n = p - path + 1;
    rtkesubstrcpy(dir, sizeof(dir), path, 0, n);
  }
  DIR *dp = opendir(*dir ? dir : ".");
  if (!dp) return 0;
  int n = 0;
  struct dirent *d;
  char s1[FNSIZE], s2[FNSIZE];
  while ((d = readdir(dp))) {
    if (*(d->d_name) == '.') continue;
    rtksnprintf(s1, sizeof(s1), "^%s$", d->d_name);
    rtksnprintf(s2, sizeof(s2), "^%s$", file);
    for (p = s1; *p; p++) *p = (char)tolower((int)*p);
    for (p = s2; *p; p++) *p = (char)tolower((int)*p);

    char *q, *r;
    for (p = s1, q = strtok_r(s2, "*", &r); q; q = strtok_r(NULL, "*", &r)) {
      p = strstr(p, q);
      if (p)
        p += strlen(q);
      else
        break;
    }
    if (p && n < nmax) rtksnprintf(paths[n++], size, "%s%s", dir, d->d_name);
  }
  closedir(dp);
#endif
  /* Sort paths in alphabetical order */
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (strcmp(paths[i], paths[j]) > 0) {
        char tmp[FNSIZE];
        rtkstrcpy(tmp, sizeof(tmp), paths[i]);
        rtkstrcpy(paths[i], size, paths[j]);
        rtkstrcpy(paths[j], size, tmp);
      }
    }
  }
  for (int i = 0; i < n; i++) trace(3, "expath  : file=%s\n", paths[i]);

  return n;
}
/* Generate local directory recursively --------------------------------------*/
static bool mkdir_r(const char *dir) {
#ifdef WIN32
  if (!*dir || !strcmp(dir + 1, ":\\")) return true;

  char pdir[FNSIZE];
  rtksnprintf(pdir, sizeof(pdir), "%.1023s", dir);
  char *p = strrchr(pdir, RTKLIB_FILEPATHSEP);
  if (p) {
    *p = '\0';
    WIN32_FIND_DATA data;
    HANDLE h = FindFirstFile(pdir, &data);
    if (h == INVALID_HANDLE_VALUE) {
      if (!mkdir_r(pdir)) return false;
    } else
      FindClose(h);
  }
  if (CreateDirectory(dir, NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
    return true;
  }
#else
  if (!*dir) return true;

  char pdir[FNSIZE];
  rtksnprintf(pdir, sizeof(pdir), "%.1023s", dir);
  char *p = strrchr(pdir, RTKLIB_FILEPATHSEP);
  if (p) {
    *p = '\0';
    FILE *fp = fopen(pdir, "r");
    if (!fp) {
      if (!mkdir_r(pdir)) return false;
    } else
      fclose(fp);
  }
  if (!mkdir(dir, 0777) || errno == EEXIST) return true;
#endif
  trace(2, "directory generation error: dir=%s\n", dir);
  return false;
}
/* Create directory ------------------------------------------------------------
 * Create directory if not exists
 * Args   : char   *path     I   file path to be saved
 * Return : none
 * Notes  : recursively.
 *----------------------------------------------------------------------------*/
extern void createdir(const char *path) {
  tracet(3, "createdir: path=%s\n", path);

  char buff[FNSIZE];
  rtkstrcpy(buff, sizeof(buff), path);
  char *p = strrchr(buff, RTKLIB_FILEPATHSEP);
  if (!p) return;
  *p = '\0';

  mkdir_r(buff);
}
/* Replace string ------------------------------------------------------------*/
static int repstr(char *str, size_t size, const char *pat, const char *rep) {
  size_t len = strlen(pat);
  char buff[FNSIZE] = {'\0'};
  size_t i = 0, j = 0;
  while (str[i]) {
    char *q = strstr(str + i, pat);
    if (!q) break;
    size_t end = q - str;
    rtkesubstrcpy(buff + j, sizeof(buff) - j, str, i, end);
    j = strlen(buff);
    rtksnprintf(buff + j, sizeof(buff) - j, "%s", rep);
    j = strlen(buff);
    i = end + len;
  }
  if (i == 0) return 0;
  rtksubstrcpy(buff + j, sizeof(buff) - j, str, i);
  rtkstrcpy(str, size, buff);
  return 1;
}
/* Replace keywords in file path -----------------------------------------------
 * Replace keywords in file path with date, time, rover and base station id
 * Args   : char   *path     I   file path (see below)
 *          char   *rpath    O   file path in which keywords replaced (see below)
 *          size_t size      I   file path in which keywords replaced buffer size
 *          gtime_t time     I   time (GPST)  (time.time==0: not replaced)
 *          char   *rov      I   rover id string        ("": not replaced)
 *          char   *base     I   base station id string ("": not replaced)
 * Return : status (1:keywords replaced, 0:no valid keyword in the path,
 *                  -1:no valid time)
 * Notes  : the following keywords in path are replaced by date, time and name
 *              %Y -> yyyy : year (4 digits) (1900-2099)
 *              %y -> yy   : year (2 digits) (00-99)
 *              %m -> mm   : month           (01-12)
 *              %d -> dd   : day of month    (01-31)
 *              %h -> hh   : hours           (00-23)
 *              %M -> mm   : minutes         (00-59)
 *              %S -> ss   : seconds         (00-59)
 *              %n -> ddd  : day of year     (001-366)
 *              %W -> wwww : GPS week        (0001-9999)
 *              %D -> d    : day of GPS week (0-6)
 *              %H -> h    : hour code       (a=0,b=1,c=2,...,x=23)
 *              %ha-> hh   : 3 hours         (00,03,06,...,21)
 *              %hb-> hh   : 6 hours         (00,06,12,18)
 *              %hc-> hh   : 12 hours        (00,12)
 *              %t -> mm   : 15 minutes      (00,15,30,45)
 *              %r -> rrrr : rover id
 *              %b -> bbbb : base station id
 *----------------------------------------------------------------------------*/
extern int reppath(const char *path, char *rpath, size_t size, gtime_t time, const char *rov,
                   const char *base) {
  rtkstrcpy(rpath, size, path);

  if (!strstr(rpath, "%")) return 0;

  int stat = 0;
  if (*rov) stat |= repstr(rpath, size, "%r", rov);
  if (*base) stat |= repstr(rpath, size, "%b", base);
  if (time.time != 0) {
    double ep[6];
    time2epoch(time, ep);
    double ep0[6] = {2000, 1, 1, 0, 0, 0};
    ep0[0] = ep[0];
    int week;
    int dow = (int)floor(time2gpst(time, &week) / 86400.0);
    int doy = (int)floor(timediff(time, epoch2time(ep0)) / 86400.0) + 1;
    char rep[64];
    rtksnprintf(rep, sizeof(rep), "%02d", ((int)ep[3] / 3) * 3);
    stat |= repstr(rpath, size, "%ha", rep);
    rtksnprintf(rep, sizeof(rep), "%02d", ((int)ep[3] / 6) * 6);
    stat |= repstr(rpath, size, "%hb", rep);
    rtksnprintf(rep, sizeof(rep), "%02d", ((int)ep[3] / 12) * 12);
    stat |= repstr(rpath, size, "%hc", rep);
    rtksnprintf(rep, sizeof(rep), "%04.0f", ep[0]);
    stat |= repstr(rpath, size, "%Y", rep);
    rtksnprintf(rep, sizeof(rep), "%02.0f", fmod(ep[0], 100.0));
    stat |= repstr(rpath, size, "%y", rep);
    rtksnprintf(rep, sizeof(rep), "%02.0f", ep[1]);
    stat |= repstr(rpath, size, "%m", rep);
    rtksnprintf(rep, sizeof(rep), "%02.0f", ep[2]);
    stat |= repstr(rpath, size, "%d", rep);
    rtksnprintf(rep, sizeof(rep), "%02.0f", ep[3]);
    stat |= repstr(rpath, size, "%h", rep);
    rtksnprintf(rep, sizeof(rep), "%02.0f", ep[4]);
    stat |= repstr(rpath, size, "%M", rep);
    rtksnprintf(rep, sizeof(rep), "%02.0f", floor(ep[5]));
    stat |= repstr(rpath, size, "%S", rep);
    rtksnprintf(rep, sizeof(rep), "%03d", doy);
    stat |= repstr(rpath, size, "%n", rep);
    rtksnprintf(rep, sizeof(rep), "%04d", week);
    stat |= repstr(rpath, size, "%W", rep);
    rtksnprintf(rep, sizeof(rep), "%d", dow);
    stat |= repstr(rpath, size, "%D", rep);
    rtksnprintf(rep, sizeof(rep), "%c", 'a' + (int)ep[3]);
    stat |= repstr(rpath, size, "%H", rep);
    rtksnprintf(rep, sizeof(rep), "%02d", ((int)ep[4] / 15) * 15);
    stat |= repstr(rpath, size, "%t", rep);
  } else if (strstr(rpath, "%ha") || strstr(rpath, "%hb") || strstr(rpath, "%hc") ||
             strstr(rpath, "%Y") || strstr(rpath, "%y") || strstr(rpath, "%m") ||
             strstr(rpath, "%d") || strstr(rpath, "%h") || strstr(rpath, "%M") ||
             strstr(rpath, "%S") || strstr(rpath, "%n") || strstr(rpath, "%W") ||
             strstr(rpath, "%D") || strstr(rpath, "%H") || strstr(rpath, "%t")) {
    return -1; /* No valid time */
  }
  return stat;
}
/* Replace keywords in file path and generate multiple paths -------------------
 * Replace keywords in file path with date, time, rover and base station id
 * Generate multiple keywords-replaced paths
 * Args   : char   *path     I   file path (see below)
 *          char   *rpath[]  O   file paths in which keywords replaced
 *          size_t size      I   file paths in which keywords replaced buffer size
 *          int    nmax      I   max number of output file paths
 *          gtime_t ts       I   time start (GPST)
 *          gtime_t te       I   time end   (GPST)
 *          char   *rov      I   rover id string        ("": not replaced)
 *          char   *base     I   base station id string ("": not replaced)
 * Return : number of replaced file paths
 * Notes  : see reppath() for replacements of keywords.
 *          minimum interval of time replaced is 900s.
 *----------------------------------------------------------------------------*/
extern int reppaths(const char *path, char *rpath[], size_t size, int nmax, gtime_t ts, gtime_t te,
                    const char *rov, const char *base) {
  trace(3, "reppaths: path =%s nmax=%d rov=%s base=%s\n", path, nmax, rov, base);

  if (ts.time == 0 || te.time == 0 || timediff(ts, te) > 0.0) return 0;

  double tint = 86400.0;
  if (strstr(path, "%S") || strstr(path, "%M") || strstr(path, "%t"))
    tint = 900.0;
  else if (strstr(path, "%h") || strstr(path, "%H"))
    tint = 3600.0;

  int week;
  double tow = time2gpst(ts, &week);
  gtime_t time = gpst2time(week, floor(tow / tint) * tint);

  int n = 0;
  while (timediff(time, te) <= 0.0 && n < nmax) {
    reppath(path, rpath[n], size, time, rov, base);
    if (n == 0 || strcmp(rpath[n], rpath[n - 1])) n++;
    time = timeadd(time, tint);
  }
  for (int i = 0; i < n; i++) trace(3, "reppaths: rpath=%s\n", rpath[i]);
  return n;
}
/* Geometric distance ----------------------------------------------------------
 * Compute geometric distance and receiver-to-satellite unit vector
 * Args   : double *rs       I   satellite position (ECEF at transmission) (m)
 *          double *rr       I   receiver position (ECEF at reception) (m)
 *          double *e        O   line-of-sight vector (ECEF)
 * Return : geometric distance (m) (0>:error/no satellite position)
 * Notes  : distance includes sagnac effect correction
 *----------------------------------------------------------------------------*/
extern double geodist(const double *rs, const double *rr, double *e) {
  if (norm(rs, 3) < RE_WGS84) return -1.0;
  for (int i = 0; i < 3; i++) e[i] = rs[i] - rr[i];
  double r = norm(e, 3);
  for (int i = 0; i < 3; i++) e[i] /= r;
  return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
}
/* Satellite azimuth/elevation angle -------------------------------------------
 * Compute satellite azimuth/elevation angle
 * Args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
 *          double *e        I   receiver-to-satellilte unit vevtor (ECEF)
 *          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
 *                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
 * Return : elevation angle (rad)
 *----------------------------------------------------------------------------*/
extern double satazel(const double *pos, const double *e, double *azel) {
  double az = 0.0, el = PI / 2.0;
  if (pos[2] > -RE_WGS84) {
    double enu[3];
    ecef2enu(pos, e, enu);
    az = dot2(enu, enu) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
    if (az < 0.0) az += 2 * PI;
    el = asin(enu[2]);
  }
  if (azel) {
    azel[0] = az;
    azel[1] = el;
  }
  return el;
}
/* Compute dops ----------------------------------------------------------------
 * Compute DOP (dilution of precision)
 * Args   : int    ns        I   number of satellites
 *          double *azel     I   satellite azimuth/elevation angle (rad)
 *          double elmin     I   elevation cutoff angle (rad)
 *          double *dop      O   DOPs {GDOP,PDOP,HDOP,VDOP}
 * Return : none
 * Notes  : dop[0]-[3] return 0 in case of dop computation error
 *----------------------------------------------------------------------------*/
#define SQRT(x) ((x) < 0.0 || (x) != (x) ? 0.0 : sqrt(x))

extern void dops(int ns, const double *azel, double elmin, double *dop) {
  for (int i = 0; i < 4; i++) dop[i] = 0.0;
  int n = 0;
  double H[4 * MAXSAT];
  for (int i = 0; i < ns && i < MAXSAT; i++) {
    if (azel[1 + i * 2] < elmin || azel[1 + i * 2] <= 0.0) continue;
    double cosel = cos(azel[1 + i * 2]);
    double sinel = sin(azel[1 + i * 2]);
    H[4 * n] = cosel * sin(azel[i * 2]);
    H[1 + 4 * n] = cosel * cos(azel[i * 2]);
    H[2 + 4 * n] = sinel;
    H[3 + 4 * n++] = 1.0;
  }
  if (n < 4) return;

  double Q[16];
  matmul("NT", 4, 4, n, H, H, Q);
  if (!matinv(Q, 4)) {
    dop[0] = SQRT(Q[0] + Q[5] + Q[10] + Q[15]); /* GDOP */
    dop[1] = SQRT(Q[0] + Q[5] + Q[10]);         /* PDOP */
    dop[2] = SQRT(Q[0] + Q[5]);                 /* HDOP */
    dop[3] = SQRT(Q[10]);                       /* VDOP */
  }
}
/* Ionosphere model ------------------------------------------------------------
 * Compute ionospheric delay by broadcast ionosphere model (klobuchar model)
 * Args   : gtime_t t        I   time (GPST)
 *          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
 *          double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 * Return : ionospheric delay (L1) (m)
 *----------------------------------------------------------------------------*/
extern double ionmodel(gtime_t t, const double *ion, const double *pos, const double *azel) {
  const double ion_default[] = {/* 2004/1/1 */
                                0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
                                0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};
  if (pos[2] < -1E3 || azel[1] <= 0) return 0.0;
  if (norm(ion, 8) <= 0.0) ion = ion_default;

  /* Earth centered angle (semi-circle) */
  double psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;

  /* Subionospheric latitude/longitude (semi-circle) */
  double phi = pos[0] / PI + psi * cos(azel[0]);
  if (phi > 0.416)
    phi = 0.416;
  else if (phi < -0.416)
    phi = -0.416;
  double lam = pos[1] / PI + psi * sin(azel[0]) / cos(phi * PI);

  /* Geomagnetic latitude (semi-circle) */
  phi += 0.064 * cos((lam - 1.617) * PI);

  /* Local time (s) */
  int week;
  double tt = 43200.0 * lam + time2gpst(t, &week);
  tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

  /* Slant factor */
  double f = 1.0 + 16.0 * pow(0.53 - azel[1] / PI, 3.0);

  /* Ionospheric delay */
  double amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
  double per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
  amp = amp < 0.0 ? 0.0 : amp;
  per = per < 72000.0 ? 72000.0 : per;
  double x = 2.0 * PI * (tt - 50400.0) / per;

  return CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
}
/* Ionosphere mapping function -------------------------------------------------
 * Compute ionospheric delay mapping function by single layer model
 * Args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 * Return : ionospheric mapping function
 *----------------------------------------------------------------------------*/
extern double ionmapf(const double *pos, const double *azel) {
  if (pos[2] >= HION) return 1.0;
  return 1.0 / cos(asin((RE_WGS84 + pos[2]) / (RE_WGS84 + HION) * sin(PI / 2.0 - azel[1])));
}
/* Ionospheric pierce point position -------------------------------------------
 * Compute ionospheric pierce point (IPP) position and slant factor
 * Args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          double re        I   earth radius (km)
 *          double hion      I   altitude of ionosphere (km)
 *          double *posp     O   pierce point position {lat,lon,h} (rad,m)
 * Return : slant factor, the single-layer mapping function.
 * Notes  : see ref [2], only valid on the earth surface
 *          fixing bug on ref [2] A.4.4.10.1 A-22,23
 *----------------------------------------------------------------------------*/
extern double ionppp(const double *pos, const double *azel, double re, double hion, double *posp) {
  /* The radius at the receiver station. */
  double r = re + pos[2];
  /* asin(rp) is the zenith angle at the IPP. */
  double rp = r / (re + hion) * cos(azel[1]);
  /* The angle at the center of the earth. */
  double ap = PI / 2.0 - azel[1] - asin(rp);
  double sinap = sin(ap);
  double cosaz = cos(azel[0]);
  posp[0] = asin(sin(pos[0]) * cos(ap) + cos(pos[0]) * sinap * cosaz);

  double tanap = tan(ap);
  if ((pos[0] > 70.0 * D2R && tanap * cosaz > tan(PI / 2.0 - pos[0])) ||
      (pos[0] < -70.0 * D2R && -tanap * cosaz > tan(PI / 2.0 + pos[0]))) {
    posp[1] = pos[1] + PI - asin(sinap * sin(azel[0]) / cos(posp[0]));
  } else {
    posp[1] = pos[1] + asin(sinap * sin(azel[0]) / cos(posp[0]));
  }
  /* Equivalent to 1/cos(asin(rp)) */
  return 1.0 / sqrt(1.0 - rp * rp);
}
/* Select iono-free linear combination (L1/L2 or L1/L5) ----------------------*/
extern int seliflc(int optnf, int sys) {
  /* Use L1/L5 for Galileo if L5 is enabled */
  return ((optnf == 2 || sys != SYS_GAL) ? 1 : 2);
}
/* Troposphere model -----------------------------------------------------------
 * Compute tropospheric delay by standard atmosphere and saastamoinen model
 * Args   : gtime_t time     I   time
 *          double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          double humi      I   relative humidity
 * Return : tropospheric delay (m)
 *----------------------------------------------------------------------------*/
extern double tropmodel(gtime_t time, const double *pos, const double *azel, double humi) {
  const double temp0 = 15.0; /* Temperature at sea level */
  if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0) return 0.0;

  /* Standard atmosphere */
  double hgt = pos[2] < 0.0 ? 0.0 : pos[2];

  double pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
  double temp = temp0 - 6.5E-3 * hgt + 273.16;
  double e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

  /* Saastamoinen model */
  double z = PI / 2.0 - azel[1];
  double trph =
      0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3) / cos(z);
  double trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
  return trph + trpw;
}
#ifndef IERS_MODEL

static double interpc(const double coef[], double lat) {
  int i = (int)(lat / 15.0);
  if (i < 1)
    return coef[0];
  else if (i > 4)
    return coef[4];
  return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
}
static double mapf(double el, double a, double b, double c) {
  double sinel = sin(el);
  return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
}
static double nmf(gtime_t time, const double pos[], const double azel[], double *mapfw) {
  /* Ref [5] table 3 */
  /* Hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
  const double coef[][5] = {{1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
                            {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
                            {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},

                            {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
                            {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
                            {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},

                            {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
                            {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
                            {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}};
  const double aht[] = {2.53E-5, 5.49E-3, 1.14E-3}; /* Height correction */

  double el = azel[1];
  if (el <= 0.0) {
    if (mapfw) *mapfw = 0.0;
    return 0.0;
  }
  /* Year from doy 28, added half a year for southern latitudes */
  double lat = pos[0] * R2D;
  double y = (time2doy(time) - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);

  double cosy = cos(2.0 * PI * y);

  lat = fabs(lat);
  double ah[3], aw[3];
  for (int i = 0; i < 3; i++) {
    ah[i] = interpc(coef[i], lat) - interpc(coef[i + 3], lat) * cosy;
    aw[i] = interpc(coef[i + 6], lat);
  }
  /* Ellipsoidal height is used instead of height above sea level */
  double hgt = pos[2];
  double dm = (1.0 / sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;

  if (mapfw) *mapfw = mapf(el, aw[0], aw[1], aw[2]);

  return mapf(el, ah[0], ah[1], ah[2]) + dm;
}
#endif /* !IERS_MODEL */

/* Troposphere mapping function ------------------------------------------------
 * Compute tropospheric mapping function by NMF
 * Args   : gtime_t t        I   time
 *          double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          double *mapfw    IO  wet mapping function (NULL: not output)
 * Return : dry mapping function
 * Note   : see ref [5] (NMF) and [9] (GMF)
 *          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
 *          paper is obtained from:
 *          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
 *----------------------------------------------------------------------------*/
extern double tropmapf(gtime_t time, const double pos[], const double azel[], double *mapfw) {
  trace(4, "tropmapf: pos=%10.6f %11.6f %6.1f azel=%5.1f %4.1f\n", pos[0] * R2D, pos[1] * R2D,
        pos[2], azel[0] * R2D, azel[1] * R2D);

  if (pos[2] < -1000.0 || pos[2] > 20000.0) {
    if (mapfw) *mapfw = 0.0;
    return 0.0;
  }
#ifdef IERS_MODEL
  const double ep[] = {2000, 1, 1, 12, 0, 0};
  double mjd = 51544.5 + (timediff(time, epoch2time(ep))) / 86400.0;
  double lat = pos[0], lon = pos[1];
  double hgt = pos[2] - geoidh(pos); /* Height in m (mean sea level) */
  double zd = PI / 2.0 - azel[1];

  /* Call GMF */
  double gmfh, gmfw;
  gmf_(&mjd, &lat, &lon, &hgt, &zd, &gmfh, &gmfw);

  if (mapfw) *mapfw = gmfw;
  return gmfh;
#else
  return nmf(time, pos, azel, mapfw); /* NMF */
#endif
}
/* Interpolate antenna phase center variation --------------------------------*/
static double interpvar(double ang, const double *var) {
  double a = ang / 5.0; /* ang=0-90 */
  int i = (int)a;
  if (i < 0)
    return var[0];
  else if (i >= 18)
    return var[18];
  return var[i] * (1.0 - a + i) + var[i + 1] * (a - i);
}
/* Receiver antenna model ------------------------------------------------------
 * Compute antenna offset by antenna phase center parameters
 * Args   : pcv_t *pcv       I   antenna phase center parameters
 *          double *del      I   antenna delta {e,n,u} (m)
 *          double *azel     I   azimuth/elevation for receiver {az,el} (rad)
 *          int     opt      I   option (0:only offset,1:offset+pcv)
 *          double *dant     O   range offsets for each frequency (m)
 * Return : none
 * Notes  : current version does not support azimuth dependent terms
 *----------------------------------------------------------------------------*/
extern void antmodel(const pcv_t *pcv, const double *del, const double *azel, int opt,
                     double *dant) {
  trace(4, "antmodel: azel=%6.1f %4.1f opt=%d\n", azel[0] * R2D, azel[1] * R2D, opt);

  double e[3], cosel = cos(azel[1]);
  e[0] = sin(azel[0]) * cosel;
  e[1] = cos(azel[0]) * cosel;
  e[2] = sin(azel[1]);

  double off[3];
  for (int i = 0; i < NFREQ; i++) {
    for (int j = 0; j < 3; j++) off[j] = pcv->off[i][j] + del[j];

    dant[i] = -dot3(off, e) + (opt ? interpvar(90.0 - azel[1] * R2D, pcv->var[i]) : 0.0);
  }
  trace(4, "antmodel: dant=%6.3f %6.3f\n", dant[0], dant[1]);
}
/* Satellite antenna model -----------------------------------------------------
 * Compute satellite antenna phase center parameters
 * Args   : pcv_t *pcv       I   antenna phase center parameters
 *          double nadir     I   nadir angle for satellite (rad)
 *          double *dant     O   range offsets for each frequency (m)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void antmodel_s(const pcv_t *pcv, double nadir, double *dant) {
  trace(4, "antmodel_s: nadir=%6.1f\n", nadir * R2D);

  for (int i = 0; i < NFREQ; i++) {
    dant[i] = interpvar(nadir * R2D * 5.0, pcv->var[i]);
  }
  trace(4, "antmodel_s: dant=%6.3f %6.3f\n", dant[0], dant[1]);
}
/* Sun and moon position in ECI (ref [4] 5.1.1, 5.2.1) -----------------------*/
static void sunmoonpos_eci(gtime_t tut, double *rsun, double *rmoon) {
  const double ep2000[] = {2000, 1, 1, 12, 0, 0};

  char tstr[40];
  trace(4, "sunmoonpos_eci: tut=%s\n", time2str(tut, tstr, 3));

  double t = timediff(tut, epoch2time(ep2000)) / 86400.0 / 36525.0;

  /* Astronomical arguments */
  double f[5];
  ast_args(t, f);

  /* Obliquity of the ecliptic */
  double eps = 23.439291 - 0.0130042 * t;
  double sine = sin(eps * D2R);
  double cose = cos(eps * D2R);

  /* Sun position in ECI */
  if (rsun) {
    double Ms = 357.5277233 + 35999.05034 * t;
    double ls =
        280.460 + 36000.770 * t + 1.914666471 * sin(Ms * D2R) + 0.019994643 * sin(2.0 * Ms * D2R);
    double rs =
        AU * (1.000140612 - 0.016708617 * cos(Ms * D2R) - 0.000139589 * cos(2.0 * Ms * D2R));
    double sinl = sin(ls * D2R);
    double cosl = cos(ls * D2R);
    rsun[0] = rs * cosl;
    rsun[1] = rs * cose * sinl;
    rsun[2] = rs * sine * sinl;

    trace(5, "rsun =%.3f %.3f %.3f\n", rsun[0], rsun[1], rsun[2]);
  }
  /* Moon position in ECI */
  if (rmoon) {
    double lm = 218.32 + 481267.883 * t + 6.29 * sin(f[0]) - 1.27 * sin(f[0] - 2.0 * f[3]) +
                0.66 * sin(2.0 * f[3]) + 0.21 * sin(2.0 * f[0]) - 0.19 * sin(f[1]) -
                0.11 * sin(2.0 * f[2]);
    double pm = 5.13 * sin(f[2]) + 0.28 * sin(f[0] + f[2]) - 0.28 * sin(f[2] - f[0]) -
                0.17 * sin(f[2] - 2.0 * f[3]);
    double rm = RE_WGS84 / sin((0.9508 + 0.0518 * cos(f[0]) + 0.0095 * cos(f[0] - 2.0 * f[3]) +
                                0.0078 * cos(2.0 * f[3]) + 0.0028 * cos(2.0 * f[0])) *
                               D2R);
    double sinl = sin(lm * D2R);
    double cosl = cos(lm * D2R);
    double sinp = sin(pm * D2R);
    double cosp = cos(pm * D2R);
    rmoon[0] = rm * cosp * cosl;
    rmoon[1] = rm * (cose * cosp * sinl - sine * sinp);
    rmoon[2] = rm * (sine * cosp * sinl + cose * sinp);

    trace(5, "rmoon=%.3f %.3f %.3f\n", rmoon[0], rmoon[1], rmoon[2]);
  }
}
/* Sun and moon position -------------------------------------------------------
 * Get sun and moon position in ECEF
 * Args   : gtime_t tut      I   time in UT1
 *          double *erpv     I   erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 *          double *rsun     IO  sun position in ECEF  (m) (NULL: not output)
 *          double *rmoon    IO  moon position in ECEF (m) (NULL: not output)
 *          double *gmst     O   GMST (rad)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun, double *rmoon,
                       double *gmst) {
  char tstr[40];
  trace(4, "sunmoonpos: tutc=%s\n", time2str(tutc, tstr, 3));

  gtime_t tut = timeadd(tutc, erpv[2]); /* UTC -> UT1 */

  /* Sun and moon position in ECI */
  double rs[3], rm[3];
  sunmoonpos_eci(tut, rsun ? rs : NULL, rmoon ? rm : NULL);

  /* ECI to ECEF transformation matrix */
  double U[9], gmst_;
  eci2ecef(tutc, erpv, U, &gmst_);

  /* Sun and moon position in ECEF */
  if (rsun) matmul("NN", 3, 1, 3, U, rs, rsun);
  if (rmoon) matmul("NN", 3, 1, 3, U, rm, rmoon);
  if (gmst) *gmst = gmst_;
}
/* Uncompress file -------------------------------------------------------------
 * Uncompress (uncompress/unzip/uncompact hatanaka-compression/tar) file
 * Args   : char   *file     I   input file
 *          char   *uncfile  O   uncompressed file
 * Return : status (-1:error,0:not compressed file,1:uncompress completed)
 * Note   : creates uncompressed file in temporary directory
 *          gzip, tar and crx2rnx commands have to be installed in commands path
 *----------------------------------------------------------------------------*/
extern int rtk_uncompress(const char *file, char *uncfile, size_t usize) {
  trace(3, "rtk_uncompress: file=%s\n", file);

  char tmpfile[FNSIZE] = "";
  rtkstrcpy(tmpfile, sizeof(tmpfile), file);
  char *p = strrchr(tmpfile, '.');
  if (!p) return 0;

  /* Uncompress by gzip */
  int stat = 0;
  if (!strcmp(p, ".z") || !strcmp(p, ".Z") || !strcmp(p, ".gz") || !strcmp(p, ".GZ") ||
      !strcmp(p, ".zip") || !strcmp(p, ".ZIP")) {
    rtkstrcpy(uncfile, usize, tmpfile);
    uncfile[p - tmpfile] = '\0';
    char cmd[64 + 2048] = "";
    rtksnprintf(cmd, sizeof(cmd), "gzip -f -d -c \"%s\" > \"%s\"", tmpfile, uncfile);

    if (execcmd(cmd)) {
      remove(uncfile);
      return -1;
    }
    rtkstrcpy(tmpfile, sizeof(tmpfile), uncfile);
    stat = 1;
  }
  /* Extract tar file */
  p = strrchr(tmpfile, '.');
  if (p && !strcmp(p, ".tar")) {
    rtkstrcpy(uncfile, usize, tmpfile);
    uncfile[p - tmpfile] = '\0';
    char buff[FNSIZE];
    rtkstrcpy(buff, sizeof(buff), tmpfile);
    char *fname = buff;
    char *dir = "";
#ifdef WIN32
    p = strrchr(buff, '\\');
    if (p) {
      *p = '\0';
      dir = fname;
      fname = p + 1;
    }
    char cmd[64 + 2048] = "";
    rtksnprintf(cmd, sizeof(cmd), "set PATH=%%CD%%;%%PATH%% & cd /D \"%s\" & tar -xf \"%s\"", dir,
                fname);
#else
    p = strrchr(buff, '/');
    if (p) {
      *p = '\0';
      dir = fname;
    }
    char cmd[64 + 2048] = "";
    rtksnprintf(cmd, sizeof(cmd), "tar -C \"%s\" -xf \"%s\"", dir, tmpfile);
#endif
    if (execcmd(cmd)) {
      if (stat) remove(tmpfile);
      return -1;
    }
    if (stat) remove(tmpfile);
    stat = 1;
  }
  /* Extract hatanaka-compressed file by cnx2rnx */
  else if ((p = strrchr(tmpfile, '.')) &&
           ((strlen(p) > 3 && (*(p + 3) == 'd' || *(p + 3) == 'D')) || !strcmp(p, ".crx") ||
            !strcmp(p, ".CRX"))) {
    rtkstrcpy(uncfile, usize, tmpfile);
    uncfile[p - tmpfile + 3] = *(p + 3) == 'D' ? 'O' : 'o';
    char cmd[64 + 2048] = "";
    rtksnprintf(cmd, sizeof(cmd), "crx2rnx < \"%s\" > \"%s\"", tmpfile, uncfile);

    if (execcmd(cmd)) {
      remove(uncfile);
      if (stat) remove(tmpfile);
      return -1;
    }
    if (stat) remove(tmpfile);
    stat = 1;
  }
  trace(3, "rtk_uncompress: stat=%d\n", stat);
  return stat;
}
/* Dummy application functions for shared library ----------------------------*/
#if defined(WIN_DLL) || defined(DLL)
extern int showmsg(const char *format, ...) { return 0; }
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}
#endif
