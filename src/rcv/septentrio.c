/*------------------------------------------------------------------------------
* septentrio.c : Septentrio Binary Format decoder (All Septentrio receivers)
*
*          Copyright (C) 2013 by Fabrizio Tappero.
*          Copyright (C) 2015 by Jens Reimann
*
* reference :
*     [1] Septentrio, SBF Reference Guide, Version 130722r38600, 07/2013
*
* note: - QZSS and Compass/Beidou is deactivated. The code is not tested. Use -DTESTING to activate.
*
* version : $Revision: 1.4 $ $Date: 2016/01/29 15:05:00 $
*
* history : 2013/07/17  1.0  begin writing
*           2013/10/24  1.1  GPS L1 working
*           2013/11/02  1.2  modified by TTAKASU
*           2015/01/26  1.3  fix some problems by Jens Reimann
*           2016/02/04  1.4  by Jens Reimann
*                           - added more sanity checks
*                           - added galileon raw decoding
*                           - added usage of decoded SBAS messages for testing
*                           - add QZSS and Compass/Beidou navigation messages
*                           - fixed code and Doppler for 2nd and following frequency
*                           - fixed bug in glonass ephemeris
*                           - fixed decoding of galileo ephemeris
*                           - fixed lost lock indicator
*                           - fixed sbas decoding
*                           - cleanups
*           2016/03/03  1.5 - fixed TOW in SBAS messages
*           2016/03/12  1.6 - respect code priorities
*                           - fixed bug in carrier phase calculation of type2 data
*                           - unify frequency determination
*                           - improve lock handling
*                           - various bug fixes
*           2016/05/25  1.7  rtk_crc24q() -> crc24q() by T.T
*           2016/07/29  1.8  crc24q() -> rtk_crc24q() by T.T
*           2017/04/11  1.9  (char *) -> (signed char *) by T.T
*           2017/09/01  1.10 suppress warnings
*           2024/01/12  1.11 update with new code from Tomoji TAKASU
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#include <math.h>
#include <stdint.h>

extern const sbsigpband_t igpband1[][8]; /* SBAS IGP band 0-8 */
extern const sbsigpband_t igpband2[][5]; /* SBAS IGP band 9-10 */

/* SBF definitions Version 2.9.1 */
#define SBF_SYNC1   0x24        /* SBF message header sync field 1 (correspond to $) */
#define SBF_SYNC2   0x40        /* SBF message header sync field 2 (correspont to @)*/
#define SBF_MAXSIG      36      /* SBF max signal number */

#define ID_MEASEPOCH       4027 /* SBF message id: range measurememts */
#define ID_MEASEPOCHEXTRA  4000 /* SBF message id: range measurememts extra info */
#define ID_MEASEPOCH_END   5922 /* SBF message id: end of SBF range measurememts */

#define ID_GPSRAWCA     4017    /* SBF message id: GPS raw navigation page or frame */
#define ID_GPSRAWL2C    4018    /* SBF message id: GPS raw navigation page or frame */
#define ID_GPSRAWL5     4019    /* SBF message id: GPS raw navigation page or frame */
#define ID_GEORAWL1     4020    /* SBF message id: SBAS raw navigation page or frame */
#define ID_GEORAWL5     4021    /* SBF message id: SBAS raw navigation page or frame */
#define ID_GALRAWFNAV   4022    /* SBF message id: Galileo raw navigation page or frame */
#define ID_GALRAWINAV   4023    /* SBF message id: Galileo raw navigation page or frame */
#define ID_GALRAWCNAV   4024    /* SBF message id: Galileo raw navigation page or frame */
#define ID_GLORAWCA     4026    /* SBF message id: GLONASS raw navigation page or frame */
#define ID_CMPRAW       4047    /* SBF message id: Compass raw navigation page or frame */
#define ID_QZSSL1CA     4066    /* SBF message id: QZSS raw navigation page or frame */
#define ID_QZSSL2C      4067    /* SBF message id: QZSS raw navigation page or frame */
#define ID_QZSSL5       4068    /* SBF message id: QZSS raw navigation page or frame */
#define ID_IRNSSRAW     4093    /* SBF message id: IRNSS raw navigation page or frame */

#define ID_GEONAV                   5896 /* SBF message id:  SBAS navigation message */
#define ID_GEOALM                   5897 /* SBF message id:  SBAS satellite almanac */
#define ID_GEOSERVICELEVEL          5917 /* SBF message id:  SBAS Service Message */
#define ID_GEONETWORKTIME           5918 /* SBF message id:  SBAS Network Time/UTC offset parameters */
#define ID_GEOMT00                  5925 /* SBF message id:  SBAS: Don't use for safety application */
#define ID_GEOPRNMASK               5926 /* SBF message id:  PRN Mask assignments */
#define ID_GEOFASTCORR              5927 /* SBF message id:  Fast Corrections */
#define ID_GEOINTEGRITY             5928 /* SBF message id:  Integrity information */
#define ID_GEOFASTCORRDEGR          5929 /* SBF message id:  fast correction degradation factor */
#define ID_GEODEGRFACTORS           5930 /* SBF message id:  Degration factors */
#define ID_GEOIGPMASK               5931 /* SBF message id:  Ionospheric grid point mask */
#define ID_GEOLONGTERMCOR           5932 /* SBF message id:  Long term satellite error corrections */
#define ID_GEOIONODELAY             5933 /* SBF message id:  Inospheric delay correction */
#define ID_GEOCLOCKEPHCOVMATRIX     5934 /* SBF message id:  Clock-Ephemeris Covariance Matrix l*/


#define ID_GPSNAV   5891        /* SBF message id: GPS navigation data */
#define ID_GPSALM   5892        /* SBF message id: GPS almanac */
#define ID_GPSION   5893        /* SBF message id: GPS ionosphere data, Klobuchar coefficients */
#define ID_GPSUTC   5894        /* SBF message id: GPS UTC data */

#define ID_GLONAV   4004        /* SBF message id: GLONASS navigation data */
#define ID_GLOALM   4005        /* SBF message id: GLONASS almanac */
#define ID_GLOTIME  4036        /* SBF message id: GLONASS time data */

#define ID_GALNAV   4002        /* SBF message id: Galileo navigation data */
#define ID_GALALM   4003        /* SBF message id: Galileo almanac */
#define ID_GALION   4030        /* SBF message id: Galileo ionosphere data, Klobuchar coefficients */
#define ID_GALUTC   4031        /* SBF message id: Galileo UTC data */

#define ID_CMPNAV   4081        /* SBF message id: Compass navigation data */
#define ID_QZSSNAV  4095        /* SBF message id: QZSS navigation data */

#define ID_GALGSTGPS  4032      /* SBF message id: Galileo GPS time offset */

#define ID_PVTCART    4006      /* SBF message id: Rx Position Velocity and Time data in Cartesian coordinates in m */
#define ID_PVTGEOD    4007      /* SBF message id: Rx Position Velocity and Time data in Geodetic coordinates */
#define ID_DOP        4001      /* SBF message id: Dilution of Precision data */
#define ID_PVTSATCART 4008      /* SBF message id: Satellite Position Velocity and Time data */

#define ID_ENDOFPVT     5921    /* SBF message id: End of any PVT block */

#define ID_RXTIME       5914    /* SBF message id: Receiver time data */

#define ID_DIFFCORRIN   5919    /* SBF message id: incoming RTCM2 or RTCM3 or CMR message */

#define ID_BASESTATION  5949    /* SBF message id: Base station position */

#define ID_CHNSTATUS   4013     /* SBF message id: Status of the receiver channels */
#define ID_RXSTATUS    4014     /* SBF message id: Status of the receiver */
#define ID_RXSETUP     5902     /* SBF message id: Status of the receiver */
#define ID_COMMENT     5936     /* SBF message id: Status of the receiver */

#define ID_SATVISIBILITY  4012  /* SBF message id: Ssatellites visibility */
#define ID_BBSMPS         4040  /* SBF message id: series of successive Rx baseband samples */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}
static signed int     I4(unsigned char *p) {signed int     u; memcpy(&u,p,4); return u;}
static short          I2(unsigned char *p) {short          i; memcpy(&i,p,2); return i;}

/* checksum lookup table -----------------------------------------------------*/
static const unsigned short CRC_16CCIT_LookUp[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

/* SBF checksum calculation --------------------------------------------------*/
static unsigned short sbf_checksum(unsigned char *buff, int len)
{
    int i;
    unsigned short crc = 0;
    for (i=0; i<len; i++) {
        crc = (crc << 8) ^ CRC_16CCIT_LookUp[ (crc >> 8) ^ buff[i] ];
    }
    return crc;
}

/* svid to satellite number ([1] 4.1.9) --------------------------------------*/
static int svid2sat(int svid)
{
    if (svid<= 37) return satno(SYS_GPS,svid);
    if (svid<= 61) return satno(SYS_GLO,svid-37);
    if (svid<= 62) return 0; /* glonass unknown slot */
    if (svid<= 68) return satno(SYS_GLO,svid-38);
    if (svid<= 70) return 0;
    if (svid<=106) return satno(SYS_GAL,svid-70);
    if (svid<=119) return 0;
    if (svid<=140) return satno(SYS_SBS,svid);
    if (svid<=180) return satno(SYS_CMP,svid-140);
    if (svid<=187) return satno(SYS_QZS,svid-180+192);
    if (svid<=190) return 0;
    if (svid<=197) return satno(SYS_IRN,svid-190);
    if (svid<=215) return satno(SYS_SBS,svid-57);
    if (svid<=222) return satno(SYS_IRN,svid-208);
    if (svid<=245) return satno(SYS_CMP,svid-182);
    return 0; /* error */
}
/* signal number table ([1] 4.1.10) ------------------------------------------*/
static uint8_t sig_tbl[SBF_MAXSIG+1][2]={ /* system, obs-code */
    {SYS_GPS, CODE_L1C}, /*  0: GPS L1C/A */
    {SYS_GPS, CODE_L1W}, /*  1: GPS L1P */
    {SYS_GPS, CODE_L2W}, /*  2: GPS L2P */
    {SYS_GPS, CODE_L2L}, /*  3: GPS L2C */
    {SYS_GPS, CODE_L5Q}, /*  4: GPS L5 */
    {SYS_GPS, CODE_L1L}, /*  5: GPS L1C */
    {SYS_QZS, CODE_L1C}, /*  6: QZS L1C/A */
    {SYS_QZS, CODE_L2L}, /*  7: QZS L2C */
    {SYS_GLO, CODE_L1C}, /*  8: GLO L1C/A */
    {SYS_GLO, CODE_L1P}, /*  9: GLO L1P */
    {SYS_GLO, CODE_L2P}, /* 10: GLO L2P */
    {SYS_GLO, CODE_L2C}, /* 11: GLO L2C/A */
    {SYS_GLO, CODE_L3Q}, /* 12: GLO L3 */
    {SYS_CMP, CODE_L1P}, /* 13: BDS B1C */
    {SYS_CMP, CODE_L5P}, /* 14: BDS B2a */
    {SYS_IRN, CODE_L5A}, /* 15: IRN L5 */
    {      0,        0}, /* 16: reserved */
    {SYS_GAL, CODE_L1C}, /* 17: GAL E1(L1BC) */
    {      0,        0}, /* 18: reserved */
    {SYS_GAL, CODE_L6C}, /* 19: GAL E6(E6BC) */
    {SYS_GAL, CODE_L5Q}, /* 20: GAL E5a */
    {SYS_GAL, CODE_L7Q}, /* 21: GAL E5b */
    {SYS_GAL, CODE_L8Q}, /* 22: GAL E5 AltBoc */
    {      0,        0}, /* 23: LBand */
    {SYS_SBS, CODE_L1C}, /* 24: SBS L1C/A */
    {SYS_SBS, CODE_L5I}, /* 25: SBS L5 */
    {SYS_QZS, CODE_L5Q}, /* 26: QZS L5 */
    {SYS_QZS, CODE_L6L}, /* 27: QZS L6 */
    {SYS_CMP, CODE_L2I}, /* 28: BDS B1I */
    {SYS_CMP, CODE_L7I}, /* 29: BDS B2I */
    {SYS_CMP, CODE_L6I}, /* 30: BDS B3I */
    {      0,        0}, /* 31: reserved */
    {SYS_QZS, CODE_L1L}, /* 32: QZS L1C */
    {SYS_QZS, CODE_L1Z}, /* 33: QZS L1S */
    {SYS_CMP, CODE_L7D}, /* 34: BDS B2b */
    {      0,        0}, /* 35: reserved */
    {SYS_IRN, CODE_L9A}  /* 36: IRN S */
};
/* signal number to freq-index and code --------------------------------------*/
static int sig2idx(int sat, int sig, const char *opt, uint8_t *code)
{
    int idx,sys=satsys(sat,NULL),nex=NEXOBS;

    if (sig<0||sig>SBF_MAXSIG||sig_tbl[sig][0]!=sys) return -1;
    *code=sig_tbl[sig][1];
    idx=code2idx(sys,*code);

    /* resolve code priority in a freq-index */
    if (sys==SYS_GPS) {
        if (strstr(opt,"-GL1W")&&idx==0) return (*code==CODE_L1W)?0:-1;
        if (strstr(opt,"-GL1L")&&idx==0) return (*code==CODE_L1L)?0:-1;
        if (strstr(opt,"-GL2L")&&idx==1) return (*code==CODE_L2L)?1:-1;
        if (*code==CODE_L1W) return (nex<1)?-1:NFREQ;
        if (*code==CODE_L2L) return (nex<2)?-1:NFREQ+1;
        if (*code==CODE_L1L) return (nex<3)?-1:NFREQ+2;
    }
    else if (sys==SYS_GLO) {
        if (strstr(opt,"-RL1P")&&idx==0) return (*code==CODE_L1P)?0:-1;
        if (strstr(opt,"-RL2C")&&idx==1) return (*code==CODE_L2C)?1:-1;
        if (*code==CODE_L1P) return (nex<1)?-1:NFREQ;
        if (*code==CODE_L2C) return (nex<2)?-1:NFREQ+1;
    }
    else if (sys==SYS_QZS) {
        if (strstr(opt,"-JL1L")&&idx==0) return (*code==CODE_L1L)?0:-1;
        if (strstr(opt,"-JL1Z")&&idx==0) return (*code==CODE_L1Z)?0:-1;
        if (*code==CODE_L1L) return (nex<1)?-1:NFREQ;
        if (*code==CODE_L1Z) return (nex<2)?-1:NFREQ+1;
    }
    else if (sys==SYS_CMP) {
        if (strstr(opt,"-CL1P")&&idx==0) return (*code==CODE_L1P)?0:-1;
        if (*code==CODE_L1P) return (nex<1)?-1:NFREQ;
    }
    return (idx<NFREQ)?idx:-1;
}
/* initialize obs data fields ------------------------------------------------*/
static void init_obsd(gtime_t time, int sat, obsd_t *data)
{
    int i;

    data->time=time;
    data->sat=(uint8_t)sat;

    for (i=0;i<NFREQ+NEXOBS;i++) {
        data->L[i]=data->P[i]=0.0;
        data->D[i]=0.0f;
        data->SNR[i]=(uint16_t)0;
        data->LLI[i]=(uint8_t)0;
        data->code[i]=CODE_NONE;
    }
}

/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;

    if (utc[3]>=256.0) return;
    time2gpst(time,&week);
    utc[3]+=week/256*256;
    if      (utc[3]<week-128) utc[3]+=256.0;
    else if (utc[3]>week+128) utc[3]-=256.0;
}
/* adjust daily rollover of time ---------------------------------------------*/
static gtime_t adjday(gtime_t time, double tod)
{
    double ep[6],tod_p;
    time2epoch(time,ep);
    tod_p=ep[3]*3600.0+ep[4]*60.0+ep[5];
    if      (tod<tod_p-43200.0) tod+=86400.0;
    else if (tod>tod_p+43200.0) tod-=86400.0;
    ep[3]=ep[4]=ep[5]=0.0;
    return timeadd(epoch2time(ep),tod);
}

/* decode SBF measurements message (observables) -----------------------------*/
/*
 * this is the most importan block in the SBF format. It it contains all code
 * pseudoranges and carrier phase measurements of all received satellites.
 * This block is made of one Type1 sub-block per santellite followed by, if any,
 * a certain number of Type2 sub-blocks. SB2Num defines how many Type2
 * sub-blocks there are inside its Type1 sub-block.
 * Type1 subplock contains code pseudorange and carrier phase range of the first
 * decoded sygnal defined by signType1, this is typically L1 signal.
 * Any following Type2 sub-block (if there are any) contains signType2 signal
 * information, typically L2 signal. Inside Type2 sub-blocks, information is
 * expressed as difference from the data in signType1 sub-block. This makes the
 * format a little more compact.
 *
*/
static int decode_measepoch(raw_t *raw)
{
    uint8_t *p=raw->buff+14,code;
    double P1,P2,L1,L2,D1,D2,S1,S2,freq1,freq2;
    int i,j,idx,n,n1,n2,len1,len2,sig,ant,svid,info,sat,sys,lock,fcn,LLI;
    int ant_sel=0; /* antenna selection (0:main) */

    if      (strstr(raw->opt,"-AUX1")) ant_sel=1;
    else if (strstr(raw->opt,"-AUX2")) ant_sel=2;

    if (raw->len<20) {
        trace(2,"sbf measepoch length error: len=%d\n",raw->len);
        return -1;
    }
    n1  =U1(p);
    len1=U1(p+1);
    len2=U1(p+2);

    if (U1(p+3)&0x80) {
        trace(2,"sbf measepoch scrambled\n");
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," nsat=%d",n1);
    }
    for (i=n=0,p+=6;i<n1&&n<MAXOBS&&p+20<=raw->buff+raw->len;i++) {
        svid=U1(p+2);
        ant =U1(p+1)>>5;
        sig =U1(p+1)&0x1f;
        info=U1(p+18);
        n2  =U1(p+19);
        fcn =0;
        if (sig==31) sig+=(info>>3)*32;
        else if (sig>=8&&sig<=11) fcn=(info>>3)-8;

        if (ant!=ant_sel) {
            trace(3,"sbf measepoch ant error: svid=%d ant=%d\n",svid,ant);
            p+=len1+len2*n2;
            continue;
        }
        if (!(sat=svid2sat(svid))) {
            trace(3,"sbf measepoch svid error: svid=%d\n",svid);
            p+=len1+len2*n2;
            continue;
        }
        if ((idx=sig2idx(sat,sig,raw->opt,&code))<0) {
            trace(2,"sbf measepoch sig error: sat=%d sig=%d\n",sat,sig);
            p+=len1+len2*n2;
            continue;
        }
        init_obsd(raw->time,sat,raw->obs.data+n);
        P1=D1=0.0;
        sys=satsys(sat,NULL);
        freq1=code2freq(sys,code,fcn);

        if ((U1(p+3)&0x1f)!=0||U4(p+4)!=0) {
            P1=(U1(p+3)&0x0f)*4294967.296+U4(p+4)*0.001;
            raw->obs.data[n].P[idx]=P1;
        }
        if (I4(p+8)!=-2147483648) {
            D1=I4(p+8)*0.0001;
            raw->obs.data[n].D[idx]=(float)D1;
        }
        lock=U2(p+16);
        if (P1!=0.0&&freq1>0.0&&lock!=65535&&(I1(p+14)!=-128||U2(p+12)!=0)) {
            L1=I1(p+14)*65.536+U2(p+12)*0.001;
            raw->obs.data[n].L[idx]=P1*freq1/CLIGHT+L1;
            LLI=(lock<raw->lockt[sat-1][idx]?1:0)+((info&(1<<2))?2:0);
            raw->obs.data[n].LLI[idx]=(uint8_t)LLI;
            raw->lockt[sat-1][idx]=lock;
        }
        if (U1(p+15)!=255) {
            S1=U1(p+15)*0.25+((sig==1||sig==2)?0.0:10.0);
            raw->obs.data[n].SNR[idx]=(uint16_t)(S1/SNR_UNIT+0.5);
        }
        raw->obs.data[n].code[idx]=code;

        for (j=0,p+=len1;j<n2&&p+12<=raw->buff+raw->len;j++,p+=len2) {
            sig =U1(p)&0x1f;
            ant =U1(p)>>5;
            info=U1(p+5);
            if (sig==31) sig+=(info>>3)*32;

            if (ant!=ant_sel) {
                trace(3,"sbf measepoch ant error: sat=%d ant=%d\n",sat,ant);
                continue;
            }
            if ((idx=sig2idx(sat,sig,raw->opt,&code))<0) {
                trace(3,"sbf measepoch sig error: sat=%d sig=%d\n",sat,sig);
                continue;
            }
            P2=0.0;
            freq2=code2freq(sys,code,fcn);

            if (P1!=0.0&&(getbits(p+3,5,3)!=-4||U2(p+6)!=0)) {
                P2=P1+getbits(p+3,5,3)*65.536+U2(p+6)*0.001;
                raw->obs.data[n].P[idx]=P2;
            }
            if (P2!=0.0&&freq2>0.0&&(I1(p+4)!=-128||U2(p+8)!=0)) {
                L2=I1(p+4)*65.536+U2(p+8)*0.001;
                raw->obs.data[n].L[idx]=P2*freq2/CLIGHT+L2;
            }
            if (D1!=0.0&&freq1>0.0&&freq2>0.0&&
                (getbits(p+3,0,5)!=-16||U2(p+10)!=0)) {
                D2=getbits(p+3,0,5)*6.5536+U2(p+10)*0.0001;
                raw->obs.data[n].D[idx]=(float)(D1*freq2/freq1)+D2;
            }
            lock=U1(p+1);
            if (lock!=255) {
                LLI=(lock<raw->lockt[sat-1][idx]?1:0)+((info&(1<<2))?2:0);
                raw->obs.data[n].LLI[idx]=(uint8_t)LLI;
                raw->lockt[sat-1][idx]=lock;
            }
            if (U1(p+2)!=255) {
                S2=U1(p+2)*0.25+((sig==1||sig==2)?0.0:10.0);
                raw->obs.data[n].SNR[idx]=(uint16_t)(S2/SNR_UNIT+0.5);
            }
            raw->obs.data[n].code[idx]=code;
        }
        n++;
    }
    raw->obs.n=n;
    return 1;
}
/* decode SBF nav message for GPS (navigation data) --------------------------*/
static int decode_gpsnav(raw_t *raw){

    uint8_t *puiTmp = (raw->buff)+6;
    eph_t eph={0};
    double toc;
    uint8_t prn, sat;
    uint16_t week;

    trace(4,"SBF decode_gpsnav: len=%d\n",raw->len);

    if ((raw->len)<120) {
        trace(2,"SBF decode_gpsnav frame length error: len=%d\n",raw->len);
        return -1;
    }

    prn = U1(puiTmp+8);
    sat = satno(SYS_GPS,prn);

    if (sat == 0) return -1;

    if (!((prn>=1)&&(prn<=37))){
        trace(2,"SBF decode_gpsnav prn error: sat=%d\n",prn);
        return -1;
    }

    eph.crs    = R4(puiTmp +  42);
    eph.deln   = R4(puiTmp +  46) * PI;
    eph.M0     = R8(puiTmp +  50) * PI;
    eph.cuc    = R4(puiTmp +  58);
    eph.e      = R8(puiTmp +  62);
    eph.cus    = R4(puiTmp +  70);
    eph.A      = pow(R8(puiTmp +  74), 2);
    eph.toes   = U4(puiTmp +  82);
    eph.cic    = R4(puiTmp +  86);
    eph.OMG0   = R8(puiTmp +  90) * PI;
    eph.cis    = R4(puiTmp +  98);
    eph.i0     = R8(puiTmp + 102) * PI;
    eph.crc    = R4(puiTmp + 110);
    eph.omg    = R8(puiTmp + 114) * PI;
    eph.OMGd   = R4(puiTmp + 122) * PI;
    eph.idot   = R4(puiTmp + 126) * PI;
    eph.tgd[0] = R4(puiTmp +  22);
    toc        = U4(puiTmp +  26);
    eph.f2     = R4(puiTmp +  30);
    eph.f1     = R4(puiTmp +  34);
    eph.f0     = R4(puiTmp +  38);
    eph.sva    = U1(puiTmp +  13); /* URA */
    eph.iodc   = U2(puiTmp +  16);
    eph.iode   = U1(puiTmp +  18);
    eph.code   = U1(puiTmp +  12);
    eph.flag   = U1(puiTmp +  15);
    eph.fit    = U1(puiTmp +  20)?0:4;
    week       = U2(puiTmp +  10); /* WN */

    if (week>=4096) {
        trace(2,"SBF gps ephemeris week error: sat=%2d week=%d\n",sat,week);
        return -1;
    }

    eph.week=adjgpsweek(week);
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GPS Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if ((eph.iode==raw->nav.eph[sat-1][0].iode) &&
            (eph.iodc==raw->nav.eph[sat-1][0].iodc)) return 0;
    }

    eph.sat=sat;
    raw->nav.eph[sat-1][0]=eph;
    raw->ephsat=sat;
    return 2;
}
/* decode SBF nav message for Galileo (navigation data) --------------------------*/
static int decode_galnav(raw_t *raw){

    uint8_t *puiTmp = (raw->buff)+6;
    eph_t eph={0};
    double toc;
    int prn, sat;
    uint16_t week_oe, week_oc;
    uint32_t tow;

    trace(4,"SBF decode_galnav: len=%d\n",raw->len);

    if ((raw->len)<152) {
        trace(2,"SBF decode_galnav frame length error: len=%d\n",raw->len);
        return -1;
    }

    prn = U1(puiTmp+8)-70;
    sat = satno(SYS_GAL,prn);

    if (sat == 0) return -1;

    if (!((prn>=1)&&(prn<=36))){
        trace(2,"SBF decode_galnav prn error: sat=%d\n",prn);
        return -1;
    }

    tow        = U4(puiTmp +  2);
    eph.week   = U2(puiTmp +  6); /* GAL week number */
    eph.code   = U1(puiTmp +  9); /* 2:INAV,16:FNAV */
    eph.A      = pow(R8(puiTmp +  10), 2);
    eph.M0     = R8(puiTmp +  18) * PI;
    eph.e      = R8(puiTmp +  26);
    eph.i0     = R8(puiTmp +  34) * PI;
    eph.omg    = R8(puiTmp +  42) * PI;
    eph.OMG0   = R8(puiTmp +  50) * PI;
    eph.OMGd   = R4(puiTmp +  58) * PI;
    eph.idot   = R4(puiTmp +  62) * PI;
    eph.deln   = R4(puiTmp +  66) * PI;
    eph.cuc    = R4(puiTmp +  70);
    eph.cus    = R4(puiTmp +  74);
    eph.crc    = R4(puiTmp +  78);
    eph.crs    = R4(puiTmp +  82);
    eph.cic    = R4(puiTmp +  86);
    eph.cis    = R4(puiTmp +  90);
    eph.toes   = U4(puiTmp +  94);
    toc        = U4(puiTmp +  98);
    eph.f2     = R4(puiTmp + 102);
    eph.f1     = R4(puiTmp + 106);
    eph.f0     = R8(puiTmp + 110);
    week_oe    = U2(puiTmp + 118); /* WNt_oc */
    week_oc    = U2(puiTmp + 120);
    eph.iode   = U2(puiTmp + 122);
    eph.iodc   = 0;
    if (eph.code==2) /* INAV */
    {
        eph.sva    = U1(puiTmp + 128);
        eph.svh    = (U2(puiTmp + 124)& 0x00ff)^0x0011;
    } else { /* FNAV */
        eph.sva    = U1(puiTmp + 127);
        eph.svh    = (U2(puiTmp + 124)& 0x0f0f)^0x0101;
    }

    eph.tgd[0] = R4(puiTmp + 130);
    eph.tgd[1] = R4(puiTmp + 134);
    eph.fit    = 0;

    week_oe=adjgpsweek(week_oe);
    week_oc=adjgpsweek(week_oc);
    eph.toe=gpst2time(week_oe,eph.toes);
    eph.toc=gpst2time(week_oc,toc);
    eph.ttr=gpst2time(eph.week,tow/1000);

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF Galileo Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1][0].iode) return 0;
    }

    if (sat == 0) return -1;

    eph.sat=sat;
    raw->nav.eph[sat-1][0]=eph;
    raw->ephsat=sat;
    return 2;
}

/* decode SBF nav message for glonass (navigation data) ----------------------*/
static int decode_glonav(raw_t *raw){

    uint8_t *puiTmp = (raw->buff)+6;
    geph_t eph={0};
    int prn, sat;
    uint16_t week;

    trace(4,"SBF decode_glonav: len=%d\n",raw->len);

    if ((raw->len)<96) {
        trace(2,"SBF decode_glonav frame length error: len=%d\n",raw->len);
        return -1;
    }
    prn = U1(puiTmp+8)-37;
    sat = satno(SYS_GLO,prn);

    if (sat == 0) return -1;

    if (!((prn>=1)&&(prn<=24))){
        trace(2,"SBF decode_glonav prn error: sat=%d\n",prn);
        return -1;
    }

    eph.frq    = U1(puiTmp +  9) - 8;
    eph.pos[0] = R8(puiTmp +  10) * 1000;
    eph.pos[1] = R8(puiTmp +  18) * 1000;
    eph.pos[2] = R8(puiTmp +  26) * 1000;
    eph.vel[0] = R4(puiTmp +  34) * 1000;
    eph.vel[1] = R4(puiTmp +  38) * 1000;
    eph.vel[2] = R4(puiTmp +  42) * 1000;
    eph.acc[0] = R4(puiTmp +  46) * 1000;
    eph.acc[1] = R4(puiTmp +  50) * 1000;
    eph.acc[2] = R4(puiTmp +  54) * 1000;
    eph.gamn   = R4(puiTmp +  58);
    eph.taun   = R4(puiTmp +  62);
    eph.dtaun  = R4(puiTmp +  66);
    week       = U2(puiTmp +  74); /* WN_toe modulo 1024 */
    week       = adjgpsweek(week);

    eph.toe    = gpst2time(week, U4(puiTmp +  70));
    eph.tof    = raw->time;
    eph.age    = U1(puiTmp +  78);
    eph.svh    = U1(puiTmp +  79);
    eph.iode   = U2(puiTmp +  80);
    eph.sva    = U2(puiTmp +  88);

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GLONASS Decoded Navigation Data (PRN=%d, Frequency Number=%d IODE=%d, AGE=%d )", prn, eph.frq, eph.iode, eph.age);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.geph[prn-1][0].iode) return 0;
    }

    eph.sat=sat;
    raw->nav.geph[prn-1][0]=eph;
    raw->ephsat=sat;
    raw->nav.glo_fcn[prn-1] = eph.frq + 8; /* savbe frequency number */

    return 2;
}
/* decode SBF nav message for sbas (navigation data) ----------------------*/
static int decode_sbasnav(raw_t *raw){

    uint8_t *puiTmp = (raw->buff)+6;
    seph_t eph={0};
    int prn, sat;
    uint16_t week;
    uint32_t tod,tow;

    trace(4,"SBF decode_sbasnav: len=%d\n",raw->len);

    if ((raw->len)<104) {
        trace(2,"SBF decode_sbasnav frame length error: len=%d\n",raw->len);
        return -1;
    }
    prn = U1(puiTmp+8);
    sat = satno(SYS_SBS,prn);

    if (!((prn>=120)&&(prn<=140))){
        trace(2,"SBF decode_sbasnav prn error: sat=%d\n",prn);
        return -1;
    }

    if (sat == 0) return -1;

    week       = U2(puiTmp +   6);
    tow        = U4(puiTmp +  2)/1000;
    tod        = U4(puiTmp +  14);
    eph.tof    = gpst2time(adjgpsweek(week),tow);
    eph.t0     = adjday(eph.tof,tod);
    eph.sva    = U2(puiTmp +  12);
    eph.svh    = eph.sva==15?1:0;
    eph.pos[0] = R8(puiTmp +  18);
    eph.pos[1] = R8(puiTmp +  26);
    eph.pos[2] = R8(puiTmp +  34);
    eph.vel[0] = R8(puiTmp +  42);
    eph.vel[1] = R8(puiTmp +  50);
    eph.vel[2] = R8(puiTmp +  58);
    eph.acc[0] = R8(puiTmp +  66);
    eph.acc[1] = R8(puiTmp +  74);
    eph.acc[2] = R8(puiTmp +  82);
    eph.af0    = R4(puiTmp +  90);
    eph.af1    = R4(puiTmp +  94);

    /* debug */
    trace(2,"sat=%2d, week=%d, tow=%f\n",sat,week,U4(puiTmp +  2)/1000);

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF SBAS Decoded Navigation Data (PRN=%d, TOW=%d, SVA=%d )", prn, tow, eph.sva);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if (fabs(timediff(eph.t0,raw->nav.seph[prn-120][0].t0))<1.0&&
            eph.sva==raw->nav.seph[prn-120][0].sva)
            return 0;
    }

    eph.sat=sat;
    raw->nav.seph[prn-120][0]=eph;
    raw->ephsat=eph.sat;
    return 2;
}

#if 0 /* UNUSED */

/* decode SBF nav message for Compass/Beidou (navigation data) --------------------------*/
static int decode_cmpnav(raw_t *raw){

    uint8_t *puiTmp = (raw->buff)+6;
    eph_t eph={0};
    double toc;
    int prn, sat;
    uint16_t week_oc,week_oe;

    trace(4,"SBF decode_cmpnav: len=%d\n",raw->len);

    if ((raw->len)<140) {
        trace(2,"SBF decode_cmpnav frame length error: len=%d\n",raw->len);
        return -1;
    }

    prn = U1(puiTmp+8)-140;
    sat = satno(SYS_CMP,prn);

    if (sat == 0) return -1;

    if (!((prn>=1)&&(prn<=32))){
        trace(2,"SBF decode_cmpnav prn error: sat=%d\n",prn);
        return -1;
    }

    eph.code   = 0;
    eph.sva    = U1(puiTmp + 12);
    eph.svh    = U1(puiTmp + 13);
    eph.iodc   = U1(puiTmp + 14);
    eph.iode   = U1(puiTmp + 15);
    eph.tgd[0] = R4(puiTmp + 18);
    eph.tgd[1] = R4(puiTmp + 22);
    toc        = U4(puiTmp + 26);
    eph.f2     = R4(puiTmp + 30);
    eph.f1     = R4(puiTmp + 34);
    eph.f0     = R4(puiTmp + 38);
    eph.crs    = R4(puiTmp + 42);
    eph.deln   = R4(puiTmp + 46) * PI;
    eph.M0     = R8(puiTmp + 50) * PI;
    eph.cuc    = R4(puiTmp + 58);
    eph.e      = R8(puiTmp + 62);
    eph.cus    = R4(puiTmp + 70);
    eph.A      = pow(R8(puiTmp +  74), 2);
    eph.toes   = U4(puiTmp + 82);
    eph.cic    = R4(puiTmp + 86);
    eph.OMG0   = R8(puiTmp + 90) * PI;
    eph.cis    = R4(puiTmp + 98);
    eph.i0     = R8(puiTmp +102) * PI;
    eph.crc    = R4(puiTmp +110);
    eph.omg    = R8(puiTmp +114) * PI;
    eph.OMGd   = R4(puiTmp +122) * PI;
    eph.idot   = R4(puiTmp +126) * PI;
    week_oc    = U2(puiTmp +130); /* WNt_oc */
    week_oe    = U2(puiTmp +132); /* WNt_oe l*/
    eph.fit    = 0;

    eph.week=adjgpsweek(week_oc);
    eph.toe=bdt2time(adjgpsweek(week_oe),eph.toes);
    eph.toc=bdt2time(eph.week,toc);
    eph.ttr=raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF Compass Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode) return 0;
    }

    if (sat == 0) return -1;

    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}

/* decode SBF nav message for QZSS (navigation data) --------------------------*/
static int decode_qzssnav(raw_t *raw){

    uint8_t *puiTmp = (raw->buff)+6;
    eph_t eph={0};
    double toc;
    int prn, sat;
    uint16_t week_oc,week_oe;

    trace(4,"SBF decode_qzssnav: len=%d\n",raw->len);

    if ((raw->len)<140) {
        trace(2,"SBF decode_qzssnav frame length error: len=%d\n",raw->len);
        return -1;
    }

    prn = U1(puiTmp+8)-180;
    sat = satno(SYS_QZS,prn);

    if (sat == 0) return -1;

    if (!((prn>=1)&&(prn<=7))){
        trace(2,"SBF decode_qzssnav prn error: sat=%d\n",prn);
        return -1;
    }

    eph.code   = U1(puiTmp + 12);
    eph.sva    = U1(puiTmp + 13);
    eph.svh    = U1(puiTmp + 14);
    eph.iodc   = U2(puiTmp + 16);
    eph.iode   = U1(puiTmp + 18);
    eph.fit    = U1(puiTmp + 20);
    eph.tgd[0] = R4(puiTmp + 22);
    toc        = U4(puiTmp + 26);
    eph.f2     = R4(puiTmp + 30);
    eph.f1     = R4(puiTmp + 34);
    eph.f0     = R4(puiTmp + 38);
    eph.crs    = R4(puiTmp + 42);
    eph.deln   = R4(puiTmp + 46) * PI;
    eph.M0     = R8(puiTmp + 50) * PI;
    eph.cuc    = R4(puiTmp + 58);
    eph.e      = R8(puiTmp + 62);
    eph.cus    = R4(puiTmp + 70);
    eph.A      = pow(R8(puiTmp +  74), 2);
    eph.toes   = U4(puiTmp + 82);
    eph.cic    = R4(puiTmp + 86);
    eph.OMG0   = R8(puiTmp + 90) * PI;
    eph.cis    = R4(puiTmp + 98);
    eph.i0     = R8(puiTmp +102) * PI;
    eph.crc    = R4(puiTmp +110);
    eph.omg    = R8(puiTmp +114) * PI;
    eph.OMGd   = R4(puiTmp +122) * PI;
    eph.idot   = R4(puiTmp +126) * PI;
    week_oc    = U2(puiTmp +130); /* WNt_oc */
    week_oe    = U2(puiTmp +132); /* WNt_oe l*/

    eph.week=adjgpsweek(week_oc);
    eph.toe=gpst2time(adjgpsweek(week_oe),eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF QZSS Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode) return 0;
    }

    if (sat == 0) return -1;

    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}

#endif /* UNUSED */

/* decode ION/UTC parameters -------------------------------------------------*/
static int decode_ionutc(raw_t *raw, int sat)
{
    double ion[8],utc[8];
    int sys=satsys(sat,NULL);

    if (!decode_frame(raw->subfrm[sat-1],NULL,NULL,ion,utc)) return 0;

    adj_utcweek(raw->time,utc);
    if (sys==SYS_QZS) {
        matcpy(raw->nav.ion_qzs,ion,8,1);
        matcpy(raw->nav.utc_qzs,utc,8,1);
    }
    else {
        matcpy(raw->nav.ion_gps,ion,8,1);
        matcpy(raw->nav.utc_gps,utc,8,1);
    }
    return 1;
}

/* decode SBF raw nav message (raw navigation data) --------------------------*/
static int decode_rawnav(raw_t *raw, int sys){

    /* NOTE. This function works quite well but it somestimes fails in line:
     * if (resp>5 || resp<=0){
     * To debug the problem an understanding of the whole RTK code is needed
     */

    uint8_t *p=(raw->buff)+6,id;
    eph_t eph={0};
    int sat, svid, prn, ret;
    uint8_t _buf[30]={0};
    int i = 0;

    trace(3,"SBF decode_gpsrawcanav: len=%d\n",raw->len);

    if (raw->len<60) {
        trace(2,"SBF decode_gpsrawcanav block length error: len=%d\n",raw->len);
        return -1;
    }

    /* get GPS satellite number */
    svid = U1(p+8);
    if (!(sat = svid2sat(svid))||satsys(sat, &prn) != sys) {
        trace(2,"sbf rawca svid error: sys=%d svid=%d\n", sys, svid);
        return -1;
    }

    if (!U1(p+9)) {
        trace(3, "sbf rawca parity/crc error: sys=%d prn=%d\n", sys, prn);
        return 0;
    }

    if (raw->outtype) {
        if (sys==SYS_GPS)
            sprintf(raw->msgtype,"SBF GPS Raw Navigation Data (PRN=%d)", prn);
        if (sys==SYS_QZS)
            sprintf(raw->msgtype,"SBF QZSS Raw Navigation Data (PRN=%d)", prn);
    }

    /* clean up subframe from Septentrio. This is a little bit of work because
     * Septentrio Rx add some parity bits to this message.
     * We have to throw away the reserved bits as well as the parity bits.
     */

     /*   | 2bits |         24bits        |  6bits  |       <- SBF 32-bit word
         ------------------------------------------
                  | byte1 | bite2 | byte3 |                 <- sat nav message
     */
     for (i=0,p+=14;i<10;i++,p+=4) { /* 24 x 10 bits w/o parity */
         setbitu(_buf, 24*i, 24, U4(p)>>6);
     }

     /* Now that we have a classic subframe we call the generic function */
     id=getbitu(_buf, 43, 3); /* get subframe id */
     if ((id<1)||(id>5)) {
         trace(2, "sbf rawca subframe id error: sys=%d prn=%d id=%d\n", sys, prn, id);
         return -1;
     }

     memcpy(raw->subfrm[sat-1]+(id-1)*30,_buf,30);

     if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL)==1&&
         decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL)==2&&
         decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL)==3) {

             if (!strstr(raw->opt,"-EPHALL")) {
                 if (eph.iode==raw->nav.eph[sat-1][0].iode&&
                     eph.iodc==raw->nav.eph[sat-1][0].iodc&&
                     timediff(eph.toe,raw->nav.eph[sat-1][0].toe)==0.0&&
                     timediff(eph.toc,raw->nav.eph[sat-1][0].toc)==0.0) return 0;
             }
             eph.sat=sat;
             raw->nav.eph[sat-1][0]=eph;
             raw->ephsat=sat;
             raw->ephset=0;
             return 2;
     }
     if (id==4||id==5) {
         ret=decode_ionutc(raw,sat);
         memset(raw->subfrm[sat-1]+id*30,0,30);
         return ret;
     }

     trace(4,"SBF, decode_gpsrawcanav: sat=%2d\n",sat);
     return 0;
}
/* decode SBF raw nav message (raw navigation data) --------------------------*/
static int decode_georaw(raw_t *raw){
    uint8_t *p=raw->buff+14,buff[32];
    int i,svid,sat,prn;

    if (raw->len<52) {
        trace(2,"sbf georawl1 length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_SBS) {
        trace(2,"sbf georawl1 svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf georawl1 parity/crc error: prn=%d err=%d\n",prn,U1(p+2));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    raw->sbsmsg.tow=(int)time2gpst(raw->time,&raw->sbsmsg.week);
    raw->sbsmsg.prn=prn;

    for (i=0;i<8;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i));
    }
    memcpy(raw->sbsmsg.msg,buff,29); /* 226 bits w/o CRC */
    raw->sbsmsg.msg[28]&=0xC0;
    return 3;
}
/* decode SBF Galileo F/NAV navigation page ----------------------------------*/
static int decode_galrawfnav(raw_t *raw)
{
    eph_t eph={0};
    double ion[4]={0},utc[8]={0};
    uint8_t *p=raw->buff+14,buff[32];
    int i,svid,src,sat,prn,type;

    if (strstr(raw->opt,"-GALINAV")) return 0;

    if (raw->len<52) {
        trace(2,"sbf galrawfnav length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    src =U1(p+3)&0x1f;

    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GAL) {
        trace(2,"sbf galrawfnav svid error: svid=%d src=%d\n",svid,src);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf galrawfnav parity/crc error: prn=%d src=%d\n",prn,src);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d src=%d",prn,src);
    }
    if (src!=20&&src!=22) { /* E5a or E5 AltBOC */
        trace(2,"sbf galrawfnav source error: prn=%d src=%d\n",prn,src);
        return -1;
    }
    for (i=0;i<8;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i)); /* 244 bits page */
    }
    type=getbitu(buff,0,6); /* page type */

    if (type==63) return 0; /* dummy page */
    if (type<1||type>6) {
        trace(2,"sbf galrawfnav page type error: prn=%d type=%d\n",prn,type);
        return -1;
    }
    /* save 244 bits page (31 bytes * 6 page) */
    memcpy(raw->subfrm[sat-1]+128+(type-1)*31,buff,31);

    if (type!=4) return 0;
    if (!decode_gal_fnav(raw->subfrm[sat-1]+128,&eph,ion,utc)) return 0;

    if (eph.sat!=sat) {
        trace(2,"sbf galrawfnav satellite error: sat=%d %d\n",sat,eph.sat);
        return -1;
    }
    eph.code|=(1<<1); /* data source: E5a */

    adj_utcweek(raw->time,utc);
    matcpy(raw->nav.ion_gal,ion,4,1);
    matcpy(raw->nav.utc_gal,utc,8,1);

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1][1].iode&&
            timediff(eph.toe,raw->nav.eph[sat-1][1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1][1].toc)==0.0) return 0;
    }
    raw->nav.eph[sat-1][1]=eph;
    raw->ephsat=sat;
    raw->ephset=1; /* 1:F/NAV */
    return 2;
}

/* decode SBF raw nav message (raw navigation data) for galileo I/NAV---------*/
static int decode_galrawinav(raw_t *raw){
    eph_t eph={0};
    double ion[4]={0},utc[8]={0};
    uint8_t *p=raw->buff+14,buff[32],type,part1,part2,page1,page2;
    int i,j,svid,src,sat,prn;

    if (strstr(raw->opt,"-GALFNAV")) return 0;

    if (raw->len<52) {
        trace(2,"sbf galrawinav length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    src =U1(p+3)&0x1f;

    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GAL) {
        trace(2,"sbf galrawinav svid error: svid=%d src=%d\n",svid,src);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf galrawinav parity/crc error: prn=%d src=%d\n",prn,src);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d src=%d",prn,src);
    }
    if (src!=17&&src!=21&&src!=22) { /* E1, E5b or E5 AltBOC */
        trace(2,"sbf galrawinav source error: prn=%d src=%d\n",prn,src);
        return -1;
    }
    for (i=0,p+=6;i<8;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p)); /* 114(even) + 120(odd) bits */
    }
    part1=getbitu(buff,  0,1);
    page1=getbitu(buff,  1,1);
    part2=getbitu(buff,114,1);
    page2=getbitu(buff,115,1);

    if (part1!=0||part2!=1) {
        trace(3,"sbf galrawinav part error: prn=%d even/odd=%d %d\n",prn,part1,
              part2);
        return -1;
    }
    if (page1==1||page2==1) return 0; /* alert page */

    type=getbitu(buff,2,6); /* word type */

    if (type>6) return 0;

    /* save 128 (112:even+16:odd) bits word (16 bytes * 7 word) */
    for (i=0,j=2;i<14;i++,j+=8) {
        raw->subfrm[sat-1][type*16+i]=getbitu(buff,j,8);
    }
    for (i=14,j=116;i<16;i++,j+=8) {
        raw->subfrm[sat-1][type*16+i]=getbitu(buff,j,8);
    }
    if (type!=5) return 0;
    if (!decode_gal_inav(raw->subfrm[sat-1],&eph,ion,utc)) return 0;

    if (eph.sat!=sat) {
        trace(2,"sbf galrawinav satellite error: sat=%d %d\n",sat,eph.sat);
        return -1;
    }
    eph.code|=(src==17)?(1<<0):(1<<2); /* data source: E1 or E5b */

    adj_utcweek(raw->time,utc);
    matcpy(raw->nav.ion_gal,ion,4,1);
    matcpy(raw->nav.utc_gal,utc,8,1);

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1][0].iode&&
            timediff(eph.toe,raw->nav.eph[sat-1][0].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1][0].toc)==0.0) return 0;
    }
    raw->nav.eph[sat-1][0]=eph;
    raw->ephsat=sat;
    raw->ephset=0; /* 0:I/NAV */
    return 2;
}

/* decode SBF raw nav message (raw navigation data) for glonass---------------*/
static int decode_glorawcanav(raw_t *raw){
    geph_t geph={0};
    gtime_t *time;
    double utc[8]={0};
    uint8_t *p=raw->buff+14,buff[12];
    int i,svid,sat,prn,m;

    if (raw->len<32) {
        trace(2,"sbf glorawca length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GLO) {
        trace(3,"sbf glorawca svid error: svid=%d\n",svid);
        return (svid==62)?0:-1; /* svid=62: slot unknown */
    }
    if (!U1(p+1)) {
        trace(3,"sbf glorawca parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0;i<3;i++) {
        setbitu(buff,32*i,32,U4(p+6+4*i)); /* 85 bits */
    }
    m=getbitu(buff,1,4);
    if (m<1||m>15) {
        trace(2,"sbf glorawca string number error: prn=%d m=%d\n",prn,m);
        return -1;
    }
    time=(gtime_t *)(raw->subfrm[sat-1]+150);
    if (fabs(timediff(raw->time,*time))>30.0) {
        memset(raw->subfrm[sat-1],0,40);
        memcpy(time,&raw->time,sizeof(gtime_t));
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    if (m!=4) return 0;

    geph.tof=raw->time;
    if (!decode_glostr(raw->subfrm[sat-1],&geph,utc)) return 0;

    matcpy(raw->nav.utc_glo,utc,8,1);

    if (geph.sat!=sat) {
        trace(2,"sbf glorawca satellite error: sat=%d %d\n",sat,geph.sat);
        return -1;
    }
    geph.frq=(int)U1(p+4)-8;

    if (!strstr(raw->opt,"-EPHALL")) {
        if (geph.iode==raw->nav.geph[prn-1][0].iode&&
            timediff(geph.toe,raw->nav.geph[prn-1][0].toe)==0.0) return 0;
    }
    raw->nav.geph[prn-1][0]=geph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}


/* decode SBF raw nav message (raw navigation data) for COMPASS ---------*/
static int decode_cmpraw(raw_t *raw){
    eph_t eph={0};
    double ion[8],utc[8];
    uint8_t *p=raw->buff+14,buff[40];
    int i,id,svid,sat,prn,pgn;

    if (raw->len<52) {
        trace(2,"sbf bdsraw length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_CMP) {
        trace(2,"sbf bdsraw svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf bdsraw parity/crc error: prn=%d\n",prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<10;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
    }
    id=getbitu(buff,15,3); /* subframe ID */
    if (id<1||id>5) {
        trace(2,"sbf bdsraw id error: prn=%d id=%d\n",prn,id);
        return -1;
    }
    if (prn>=6&&prn<=58) { /* IGSO/MEO */
        memcpy(raw->subfrm[sat-1]+(id-1)*38,buff,38);

        if (id==3) {
            if (!decode_bds_d1(raw->subfrm[sat-1],&eph,NULL,NULL)) return 0;
        }
        else if (id==5) {
            if (!decode_bds_d1(raw->subfrm[sat-1],NULL,ion,utc)) return 0;
            matcpy(raw->nav.ion_cmp,ion,8,1);
            matcpy(raw->nav.utc_cmp,utc,8,1);
            return 9;
        }
        else return 0;
    }
    else { /* GEO */
        pgn=getbitu(buff,42,4); /* page number */

        if (id==1&&pgn>=1&&pgn<=10) {
            memcpy(raw->subfrm[sat-1]+(pgn-1)*38,buff,38);
            if (pgn!=10) return 0;
            if (!decode_bds_d2(raw->subfrm[sat-1],&eph,NULL)) return 0;
        }
        else if (id==1&&pgn==102) {
            memcpy(raw->subfrm[sat-1]+10*38,buff,38);
            if (!decode_bds_d2(raw->subfrm[sat-1],NULL,utc)) return 0;
            matcpy(raw->nav.utc_cmp,utc,8,1);
            return 9;
        }
        else return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(eph.toe,raw->nav.eph[sat-1][0].toe)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1][0]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode SBF NavIC/IRNSS subframe -------------------------------------------*/
static int decode_navicraw(raw_t *raw)
{
    eph_t eph={0};
    double ion[8],utc[9];
    uint8_t *p=raw->buff+14,buff[40];
    int i,id,svid,sat,prn,ret=0;

    if (raw->len<52) {
        trace(2,"sbf navicraw length error: len=%d\n",raw->len);
        return -1;
    }
    svid=U1(p);
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_IRN) {
        trace(2,"sbf navicraw svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3,"sbf navicraw parity/crc error: prn=%d err=%d\n",prn,U1(p+2));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype)," prn=%d",prn);
    }
    for (i=0,p+=6;i<10;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
    }
    id=getbitu(buff,27,2); /* subframe ID (0-3) */

    memcpy(raw->subfrm[sat-1]+id*37,buff,37);

    if (id==1) { /* subframe 2 */
        if (!decode_irn_nav(raw->subfrm[sat-1],&eph,NULL,NULL)) return 0;

        if (!strstr(raw->opt,"-EPHALL")) {
            if (eph.iode==raw->nav.eph[sat-1][0].iode&&
                timediff(eph.toe,raw->nav.eph[sat-1][0].toe)==0.0) {
                return 0;
            }
        }
        eph.sat=sat;
        raw->nav.eph[sat-1][0]=eph;
        raw->ephsat=sat;
        raw->ephset=0;
        return 2;
    }
    else if (id==2||id==3) { /* subframe 3 or 4 */
        if (decode_irn_nav(raw->subfrm[sat-1],NULL,ion,NULL)) {
            matcpy(raw->nav.ion_irn,ion,8,1);
            ret=9;
        }
        if (decode_irn_nav(raw->subfrm[sat-1],NULL,NULL,utc)) {
            adj_utcweek(raw->time,utc);
            matcpy(raw->nav.utc_irn,utc,9,1);
            ret=9;
        }
        memset(raw->subfrm[sat-1]+id*37,0,37);
        return ret;
    }
    return 0;
}

/* decode SBF gloutc --------------------------------------------------------*/
static int decode_gloutc(raw_t *raw)
{
    uint8_t *p=(raw->buff)+8;                 /* points at TOW location */

    trace(4,"SBF decode_gloutc: len=%d\n", raw->len);

    if (raw->len<40)
    {
        trace(1,"SBF decode_gloutc: Block too short\n");
        return -1;
    }

    /* GPS delta-UTC parameters */
    raw->nav.utc_glo[0] = R8(p + 16);                                 /*  tau_c */
    raw->nav.utc_glo[1] = U4(p + 24);                                 /*  B1 */
    raw->nav.utc_glo[2] = R4(p + 28);                                 /*  B2 */
    raw->nav.utc_glo[3] = R4(p + 12);                                 /*  tau_GPS */

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GLONASS UTC Offsets");
    }

    return 9;
}
/* decode SBF gpsion --------------------------------------------------------*/
static int decode_gpsion(raw_t *raw){
    uint8_t *p=(raw->buff)+8;            /* points at TOW location */

    trace(4,"SBF decode_gpsion: len=%d\n", raw->len);

    if (raw->len<48)
    {
        trace(1,"SBF decode_gpsion: Block too short\n");
        return -1;
    }

    raw->nav.ion_gps[0] = R4(p + 8);
    raw->nav.ion_gps[1] = R4(p + 12);
    raw->nav.ion_gps[2] = R4(p + 16);
    raw->nav.ion_gps[3] = R4(p + 20);
    raw->nav.ion_gps[4] = R4(p + 24);
    raw->nav.ion_gps[5] = R4(p + 28);
    raw->nav.ion_gps[6] = R4(p + 32);
    raw->nav.ion_gps[7] = R4(p + 36);

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GPS Ionospheric Data");
    }

    return 9;
}
/* decode SBF galion --------------------------------------------------------*/
static int decode_galion(raw_t *raw){
    uint8_t *p=(raw->buff)+6;            /* points at TOW location */

    trace(4,"SBF decode_galion: len=%d\n", raw->len);

    if (raw->len<29)
    {
        trace(1,"SBF decode_galion: Block too short\n");
        return -1;
    }

    raw->nav.ion_gal[0] = R4(p + 10);
    raw->nav.ion_gal[1] = R4(p + 14);
    raw->nav.ion_gal[2] = R4(p + 18);
    raw->nav.ion_gal[3] = 0;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF Galileon Ionospheric Data");
    }

    return 9;
}

/* decode SBF gpsutc --------------------------------------------------------*/
static int decode_gpsutc(raw_t *raw)
{
    uint8_t *p=(raw->buff)+8;                 /* points at TOW location */
    uint8_t leaps;

    trace(4,"SBF decode_gpsutc: len=%d\n", raw->len);

    if (raw->len<37)
    {
        trace(1,"SBF decode_gpsutc: Block too short\n");
        return -1;
    }

    /* GPS delta-UTC parameters */
    raw->nav.utc_gps[1] = R4(p + 8);                                  /*   A1 */
    raw->nav.utc_gps[0] = R8(p + 12);                                 /*   A0 */
    raw->nav.utc_gps[2] = U4(p + 20);                                 /*  tot */
    /* raw->nav.utc_gps[3] = U1(p + 24); */                           /*  WNt */
    raw->nav.utc_gps[3] = adjgpsweek(U2(p + 4));                      /*   WN */
    leaps = I1(p + 25);                                               /* Dtls */

    /*NOTE. it is kind of strange that I have to use U1(p+4) and not U1(p+24)
            in fact if I take U1(p+24) I do not seem to ge the correct W in
            the header of RINEX nav file, line DELTA-UTC: A0,A1,T,W
    */

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GPS UTC Offsets");
    }

    return 9;
}
/* decode SBF gpsalm --------------------------------------------------------*/
static int decode_gpsalm(raw_t *raw)
{
    uint8_t *p=(raw->buff)+8;                 /* points at TOW location */
    alm_t alm;

    trace(4,"SBF decode_gpsalm: len=%d\n", raw->len);

    if (raw->len<60)
    {
        trace(1,"SBF decode_gpsalm: Block too short\n");
        return -1;
    }

    alm.sat =   satno(SYS_GPS,U1(p + 6));
    alm.e     = R4(p + 8);
    alm.toas  = U4(p + 12);
    alm.i0    = R4(p + 16);
    alm.OMGd  = R4(p + 20);
    alm.A     = pow(R4(p + 24),2);
    alm.OMG0  = R4(p + 28);
    alm.omg   = R4(p + 32);
    alm.M0    = R4(p + 36);
    alm.f1    = R4(p + 40);
    alm.f0    = R4(p + 44);
    alm.week  = U1(p + 48);
    alm.toa   = gpst2time(alm.week,alm.toas);
    alm.svconf= U1(p + 49);
    alm.svh   = U1(p + 50);

    if (alm.sat == 0) return -1;

    raw->nav.alm[alm.sat-1]=alm;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GPS Almanach (PRN=%d)", U1(p + 6));
    }

    return 9;
}
/* decode SBF galutc --------------------------------------------------------*/
static int decode_galutc(raw_t *raw)
{
    uint8_t *p=(raw->buff)+8;                 /* points at TOW location */
    uint8_t leaps;

    trace(4,"SBF decode_galutc: len=%d\n", raw->len);

    if (raw->len<36)
    {
        trace(1,"SBF decode_galutc: Block too short\n");
        return -1;
    }

    /* GPS delta-UTC parameters */
    raw->nav.utc_gal[1] = R4(p + 8);                                  /*   A1 */
    raw->nav.utc_gal[0] = R8(p + 12);                                 /*   A0 */
    raw->nav.utc_gal[2] = U4(p + 20);                                 /*  tot */
    raw->nav.utc_gal[3] = adjgpsweek(U2(p + 4));                      /*   WN */
    leaps = I1(p + 25);                                               /* Dtls */

    /*NOTE. it is kind of strange that I have to use U1(p+4) and not U1(p+24)
            in fact if I take U1(p+24) I do not seem to ge the correct W in
            the header of RINEX nav file, line DELTA-UTC: A0,A1,T,W
    */

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF Galileo UTC Offsets");
    }

    return 9;
}
/* decode SBF galalm --------------------------------------------------------*/
static int decode_galalm(raw_t *raw)
{
    uint8_t *p=(raw->buff)+8;                 /* points at TOW location */
    alm_t alm;

    trace(4,"SBF decode_galalm: len=%d\n", raw->len);

    if (raw->len<62)
    {
        trace(1,"SBF decode_galalm: Block too short\n");
        return -1;
    }

    alm.sat =   satno(SYS_GAL,U1(p + 49) - 70);
    alm.e     = R4(p + 8);
    alm.toas  = U4(p + 12);
    alm.i0    = R4(p + 16) + 0.3;
    alm.OMGd  = R4(p + 20);
    alm.A     = pow(R4(p + 24),2);
    alm.OMG0  = R4(p + 28);
    alm.omg   = R4(p + 32);
    alm.M0    = R4(p + 36);
    alm.f1    = R4(p + 40);
    alm.f0    = R4(p + 44);
    alm.week  = U1(p + 48);
    alm.toa   = gpst2time(alm.week,alm.toas);
    alm.svconf= 0;
    alm.svh   = 0;

    if (alm.sat == 0) return -1;
    raw->nav.alm[alm.sat-1]=alm;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF Galileon Almanach (PRN=%d)", U1(p + 49) - 70);
    }

    return 9;
}

#if 0 /* UNUSED */

/* type 2-5,0: fast corrections ---------------------------------------*/
static int decode_sbsfast(raw_t *raw)
{
    int i,j;
    int prn,sat;
    uint8_t sbLength,sbCount,iodf,type;
    double prc_old,dt;
    gtime_t t0_old;
    uint32_t tow;
    uint16_t week;
    uint8_t *p=(raw->buff)+6;

    trace(4,"SBF decode_sbsfast: len=%d\n", raw->len);

    if (raw->len<20)
    {
        trace(1,"SBF decode_sbsfast: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 140) return -1;

    sat=satno(SYS_SBS,prn);
    if (sat == 0) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF SBAS Ionosphere Fast Correction from PRN=%d", prn);
    }

    tow=U4(p+2)/1000;
    week=U2(p+6);
    week=adjgpsweek(week);

    type=U1(p+9);
    iodf=U1(p+11);
    sbCount=U1(p+12);
    sbLength=U1(p+13);

    if (raw->nav.sbssat.iodp!=U1(p+10)) return 0;
    if (type>5) return -1;

    for (i=0;i<sbCount;i++) {
        if ((j=13*((type==0?2:type)-2)+i)>=raw->nav.sbssat.nsat) break;
        t0_old =raw->nav.sbssat.sat[j].fcorr.t0;
        prc_old =raw->nav.sbssat.sat[j].fcorr.prc;

        raw->nav.sbssat.sat[j].fcorr.t0=gpst2time(week,tow);
        raw->nav.sbssat.sat[j].fcorr.udre=U1(p+14+i*sbLength+1);
        raw->nav.sbssat.sat[j].fcorr.prc=R4(p+14+i*sbLength+4);

        dt=timediff(raw->nav.sbssat.sat[j].fcorr.t0,t0_old);
        if (t0_old.time==0||dt<=0.0||18.0<dt||raw->nav.sbssat.sat[j].fcorr.ai==0) {
            raw->nav.sbssat.sat[j].fcorr.rrc=0.0;
            raw->nav.sbssat.sat[j].fcorr.dt=0.0;
        }
        else {
            raw->nav.sbssat.sat[j].fcorr.rrc=(raw->nav.sbssat.sat[j].fcorr.prc-prc_old)/dt;
            raw->nav.sbssat.sat[j].fcorr.dt=dt;
        }
        raw->nav.sbssat.sat[j].fcorr.iodf=iodf;
    }
    trace(5,"SBF decode_sbsfast: type=%d iodf=%d\n",U1(p+9),iodf);
    return 0;
}

/* decode type 1: prn masks --------------------------------------------------*/
static int decode_sbsprnmask(raw_t *raw)
{
    int i,n,sat,prn;
    uint8_t *p=(raw->buff)+6;


    trace(4,"SBF decode_sbsprnmask:\n");

    if (raw->len<18)
    {
        trace(1,"SBF decode_sbsprnmask: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    raw->nav.sbssat.nsat=U1(p+10);

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF SBAS PRN Mask from PRN=%d", prn);
    }

    for (n=0;n<raw->nav.sbssat.nsat&&n<MAXSAT;n++) {
       i=U1(p+11+n);
       if      (i<= 37) sat=satno(SYS_GPS,i);    /*   0- 37: gps */
       else if (i<= 61) sat=satno(SYS_GLO,i-37); /*  38- 61: glonass */
       else if (i<=119) sat=0;                   /*  62-119: future gnss */
       else if (i<=138) sat=satno(SYS_SBS,i);    /* 120-138: geo/waas */
       else if (i<=182) sat=0;                   /* 139-182: reserved */
       else if (i<=192) sat=satno(SYS_SBS,i+10); /* 183-192: qzss ref [2] */
       else if (i<=202) sat=satno(SYS_QZS,i);    /* 193-202: qzss ref [2] */
       else             sat=0;                   /* 203-   : reserved */
       raw->nav.sbssat.sat[n].sat=sat;
    }
    raw->nav.sbssat.iodp=U1(p+9);

    trace(5,"SBF decode_sbsprnmask: nprn=%d iodp=%d\n",n,raw->nav.sbssat.iodp);
    return 0;
}

/* decode type 6: integrity info ---------------------------------------------*/
static int decode_sbsintegriy(raw_t *raw)
{
    int i,prn;
    uint8_t *p=(raw->buff)+6;
    uint8_t iodf[4],udre;


    trace(4,"decode_sbsintegriy:\n");

    if (raw->len<71)
    {
        trace(1,"SBF decode_sbsintegriy: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF SABS Integrity Data from PRN=%d", prn);
    }

    for (i=0;i<4;i++) {
        iodf[i]=U1(p+10+i);
    }
    for (i=0;i<raw->nav.sbssat.nsat&&i<MAXSAT;i++) {
        if (raw->nav.sbssat.sat[i].fcorr.iodf!=iodf[i/13]) continue;
        udre=U1(p+14+i);
        raw->nav.sbssat.sat[i].fcorr.udre=udre;
    }
    trace(5,"SBF decode_sbsintegriy: iodf=%d %d %d %d\n",iodf[0],iodf[1],iodf[2],iodf[3]);
    return 0;
}

/* decode type 7: fast correction degradation factor -------------------------*/
static int decode_sbsfastcorrdegr(raw_t *raw)
{
    int i,prn;
    uint8_t *p=(raw->buff)+6;


    trace(4,"SBF decode_sbsfastcorrdegr:\n");

    if (raw->len<68)
    {
        trace(1,"SBF decode_sbsfastcorrdegr: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF SBAS Fast Correction Degradation Factor from PRN=%d", prn);
    }

    if (raw->nav.sbssat.iodp!=U1(p+9)) return 0;

    raw->nav.sbssat.tlat=U1(p+10);

    for (i=0;i<raw->nav.sbssat.nsat&&i<MAXSAT;i++) {
        raw->nav.sbssat.sat[i].fcorr.ai=U1(p+11+i);
    }
    return 0;
}

/* decode type 26: ionospheric delay corrections -----------------------------*/
static int decode_sbsionodelay(raw_t *raw)
{
    int i,j,give,prn;
    int band;
    uint8_t *p=(raw->buff)+6, sbLength, count;
    uint16_t week;
    uint32_t tow;

    trace(4,"SBF decode_sbsionodelay:\n");

    if (raw->len<20)
    {
        trace(1,"SBF decode_sbsionodelay: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF SBAS Ionospheric Delay Correction from PRN=%d", prn);
    }

    band=U1(p+9);

    if (band>MAXBAND||raw->nav.sbsion[band].iodi!=U1(p+10)) return 0;

    tow=U4(p+2)/1000;
    week=U2(p+6);
    week=adjgpsweek(week);

    sbLength=U1(p+12);
    count=U1(p+11);

    if (count!=15)
    {
        trace(1,"SBF decode_sbsionodelay: wrong number of IDC blocks: %d\n",count);
        return -1;
    }

    for (i=0;i<count;i++) {
        j=U1(p+14+i*sbLength);
        give=U1(p+14+i*sbLength+1);

        raw->nav.sbsion[band].igp[j].t0=gpst2time(week,tow);
        raw->nav.sbsion[band].igp[j].delay=R4(p+14+i*sbLength+4);
        raw->nav.sbsion[band].igp[j].give=give;

        if (raw->nav.sbsion[band].igp[j].give>=16) {
            raw->nav.sbsion[band].igp[j].give=0;
        }
    }
    trace(5,"decode_sbsionodelay: band=%d\n",band);
    return 0;
}

/* decode type 18: ionospheric grid point masks ------------------------------*/
static int decode_sbsigpmask(raw_t *raw) /* TODO: verify this function */
{
    const sbsigpband_t *b;
    int i,j,n,m,prn;
    uint8_t band;

    uint8_t *p=(raw->buff)+6;

    trace(4,"SBF decode_sbsigpmask:\n");

    if (raw->len<20)
    {
        trace(1,"SBF decode_sbsigpmask: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Ionospheric Grid Points from PRN=%d", prn);
    }

    band=U1(p+10);

    if      (band<= 8) {b=igpband1[band  ]; m=8;}
    else if (9<=band&&band<=10) {b=igpband2[band-9]; m=5;}
    else return 0;

    raw->nav.sbsion[band].iodi=U1(p+11);
    raw->nav.sbsion[band].nigp=U1(p+12);

    for (n=0;n<raw->nav.sbsion[band].nigp;n++)
    {
        i=U1(p+13+n);
        for (j=0;j<m;j++) {
            if (i<b[j].bits||b[j].bite<i) continue;
            raw->nav.sbsion[band].igp[n].lat=band<=8?b[j].y[i-b[j].bits]:b[j].x;
            raw->nav.sbsion[band].igp[n++].lon=band<=8?b[j].x:b[j].y[i-b[j].bits];
            break;
        }
    }

    trace(5,"decode_sbsigpmask: band=%d nigp=%d\n",band,n);
    return 0;
}
/* decode long term correction ------------------------------------------*/
static int decode_sbslongcorrh(raw_t* raw)
{
    int prn,i;
    uint8_t *p=(raw->buff)+6;
    uint8_t count,sbLength,no;
    uint32_t tow;
    uint16_t week;
    int64_t t;

    trace(4,"SBF decode_sbslongcorrh:\n");

    if (raw->len<20)
    {
        trace(1,"SBF decode_sbslongcorrh: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn=U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Long Term Corrections from PRN=%d", prn);
    }

    tow=U4(p+2)/1000;
    week=U2(p+6);
    week=adjgpsweek(week);

    count=U1(p+9);
    sbLength=U1(p+10);

    if (count >4) return -1;

    for (i=0;i<count;i++)
    {
        no=U1(p+14+i*sbLength+1);
        raw->nav.sbssat.sat[no-1].lcorr.iode=U1(p+14+i*sbLength+3);
        raw->nav.sbssat.sat[no-1].lcorr.dpos[0]=R4(p+14+i*sbLength+ 4);
        raw->nav.sbssat.sat[no-1].lcorr.dpos[1]=R4(p+14+i*sbLength+ 8);
        raw->nav.sbssat.sat[no-1].lcorr.dpos[2]=R4(p+14+i*sbLength+12);
        if (U1(p+14+i*sbLength)==1)
        {
            raw->nav.sbssat.sat[no-1].lcorr.dvel[i]=R4(p+14+i*sbLength+16);
            raw->nav.sbssat.sat[no-1].lcorr.dvel[i]=R4(p+14+i*sbLength+20);
            raw->nav.sbssat.sat[no-1].lcorr.dvel[i]=R4(p+14+i*sbLength+24);

            raw->nav.sbssat.sat[no-1].lcorr.daf1=R4(p+14+i*sbLength+32);
        } else
        {
            raw->nav.sbssat.sat[no-1].lcorr.dvel[0]=raw->nav.sbssat.sat[no-1].lcorr.dvel[1]=raw->nav.sbssat.sat[no-1].lcorr.dvel[2]=0.0;
            raw->nav.sbssat.sat[no-1].lcorr.daf1=0;
        };
        raw->nav.sbssat.sat[no-1].lcorr.daf0=R4(p+14+i*sbLength+28);

        t=(int)U4(p+14+i*sbLength+32)-(int)tow%86400;
        if      (t<=-43200) t+=86400;
        else if (t>  43200) t-=86400;
        raw->nav.sbssat.sat[no-1].lcorr.t0=gpst2time(week,tow+t);
    };

    return 0;
}

#endif /* UNUSED */

/* decode SBF raw message --------------------------------------------------*/
static int decode_sbf(raw_t *raw)
{
    unsigned short crc;
    uint32_t week,tow;
    char tstr[40];

    /* read the SBF block ID and revision */
    int type = U2(raw->buff+4) & 0x1fff << 0;
    int revision = U2(raw->buff+4) >> 13;
    (void)revision;  /* unused */

    trace(3,"decode_sbf: type=%04x len=%d\n", type, raw->len);

    /* read the SBF block CRC */
    crc = U2(raw->buff+2);

    /* checksum skipping first 4 bytes */
    if (sbf_checksum(raw->buff+4, raw->len-4) !=  crc){
        trace(2,"sbf checksum error: type=%04x len=%d\n", type, raw->len);
        return -1;
    }

    if (raw->len < 14) {
        trace(2,"sbf length error: type=%d len=%d\n", type, raw->len);
        return -1;
    }
    tow = U4(raw->buff+8);
    week = U2(raw->buff+12);
    if (tow == 4294967295u || week == 65535u) {
        trace(2,"sbf tow/week error: type=%d len=%d\n", type, raw->len);
        return -1;
    }
    raw->time=gpst2time(week, tow*0.001);

    if (raw->outtype) {
        time2str(raw->time, tstr, 2);
        sprintf(raw->msgtype,"SBF %4d (%4d): %s",type,raw->len,tstr);
    }

    switch (type) {
        case ID_MEASEPOCH:      return decode_measepoch(raw);
        case ID_MEASEPOCHEXTRA:
            if (raw->outtype) {
                sprintf(raw->msgtype, "SBF Measurement Data Extra");
            }
            return 0;
        case ID_MEASEPOCH_END:
            if (raw->outtype) {
                sprintf(raw->msgtype, "SBF Measurement Epoch End");
            }
            return 0;

        case ID_GPSNAV:         return decode_gpsnav(raw);
        case ID_GPSION:         return decode_gpsion(raw);
        case ID_GPSUTC:         return decode_gpsutc(raw);
        case ID_GPSALM:         return decode_gpsalm(raw);
        case ID_GPSRAWCA:
        case ID_GPSRAWL2C:
        case ID_GPSRAWL5:       return decode_rawnav(raw, SYS_GPS);

        case ID_GEONAV:         return decode_sbasnav(raw);
        case ID_GEORAWL1:
        case ID_GEORAWL5:       return decode_georaw(raw);

#ifdef ENAGLO
        case ID_GLONAV:         return decode_glonav(raw);
        case ID_GLORAWCA:       return decode_glorawcanav(raw);
        case ID_GLOTIME:        return decode_gloutc(raw);
#endif

#ifdef ENAGAL
        case ID_GALNAV:         return decode_galnav(raw);
        case ID_GALION:         return decode_galion(raw);
        case ID_GALUTC:         return decode_galutc(raw);
        case ID_GALALM:         return decode_galalm(raw);
        case ID_GALRAWINAV:     return decode_galrawinav(raw);
        case ID_GALRAWFNAV:     return decode_galrawfnav(raw);
#endif

#ifdef ENACMP
        case ID_CMPRAW:         return decode_cmpraw(raw);
#endif

#ifdef ENAQZS
    case ID_QZSSL1CA:
    case ID_QZSSL2C:
    case ID_QZSSL5:             return decode_rawnav(raw, SYS_QZS);
#endif

#ifdef ENAIRN
    case ID_IRNSSRAW:           return decode_navicraw  (raw);
#endif
#ifdef TESTING /* not tested */
#ifdef ENAQZS
        case ID_QZSSL1CA:
        case ID_QZSSL2C:
        case ID_QZSSL5:         return decode_rawnav(raw, SYS_QZS);
        case ID_QZSS_NAV:       return decode_qzssnav(raw);
#endif

#ifdef ENACMP
        case ID_CMPRAW:         return decode_cmpraw(raw);
        case ID_CMPNAV:         return decode_cmpnav(raw);
#endif
#endif

#if 0 /* not yet supported by RTKLIB */
        case ID_GEOMT00:        return decode_sbsfast(raw);
        case ID_GEOPRNMASK:     return decode_sbsprnmask(raw);
        case ID_GEOFASTCORR:    return decode_sbsfast(raw);
        case ID_GEOINTEGRITY:   return decode_sbsintegriy(raw);
        case ID_GEOFASTCORRDEGR:return decode_sbsfastcorrdegr(raw);
        case ID_GEOIGPMASK:     return decode_sbsigpmask(raw);
        case ID_GEOLONGTERMCOR: return decode_sbslongcorrh(raw);
        case ID_GEOIONODELAY:   return decode_sbsionodelay(raw);
#endif /* UNUSED */

#if 0 /* UNUSED */
        case ID_GALRAWFNAV:     return decode_galrawfnav(raw); /* not yet supported in RTKLIB */
        case ID_GLOALM:         return decode_glosalm(raw); /* not yet supported in RTKLIB */

        case ID_PVTGEOD:        return decode_pvtgeod(raw);
        case ID_RXSETUP:        return decode_rxsetup(raw);
        case ID_COMMENT:        return decode_comment(raw);
#endif /* UNUSED */
        default:
#if 0 /* debug output */
            if (raw->outtype) {
                sprintf(raw->msgtype,"SBF 0x%04X (%4d):",type, raw->len);
            }
#endif
            trace(3,"decode_sbf: unused frame type=%04x len=%d\n",type,raw->len);
        /* there are many more SBF blocks to be extracted */
    }
    return 0;
}

/* sync to the beginning of a block ------------------------------------------*/
static int sync_sbf(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]== SBF_SYNC1 && buff[1]==SBF_SYNC2;
}
/* input sbf raw data from stream ----------------------------------------------
* get to the next sbf raw block from stream
* args   : raw_t  *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*          to specify input options for sbf, set raw->opt to the following
*          option strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -AUX1   : select antenna Aux1  (default: main)
*          -AUX2   : select antenna Aux2  (default: main)
*          -GL1W   : select 1W for GPS L1 (default: 1C)
*          -GL1L   : select 1L for GPS L1 (default: 1C)
*          -GL2L   : select 2L for GPS L2 (default: 2W)
*          -RL1P   : select 1P for GLO G1 (default: 1C)
*          -RL2C   : select 2C for GLO G2 (default: 2P)
*          -JL1L   : select 1L for QZS L1 (default: 1C)
*          -JL1Z   : select 1Z for QZS L1 (default: 1C)
*          -CL1P   : select 1P for BDS B1 (default: 2I)
*          -GALINAV: select I/NAV for Galileo ephemeris (default: all)
*          -GALFNAV: select F/NAV for Galileo ephemeris (default: all)
*-----------------------------------------------------------------------------*/
extern int input_sbf(raw_t *raw, unsigned char data)
{
    trace(5,"input_sbf: data=%02x\n",data);

    if (raw->nbyte==0) {
        if (sync_sbf(raw->buff,data)) raw->nbyte=2;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;

    if (raw->nbyte<8) return 0;

    if ((raw->len=U2(raw->buff+6))>MAXRAWLEN) {
        trace(2,"sbf length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (raw->nbyte<raw->len) return 0;
    raw->nbyte=0;

    return decode_sbf(raw);
}
/* sbf raw block finder --------------------------------------------------------
* get to the next sbf raw block from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_sbff(raw_t *raw, FILE *fp)
{
    int i,data;
    trace(4,"input_sbff:\n");

    /* go to the beginning of the first block */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_sbf(raw->buff,(unsigned char)data)) break;
            if (i>=MAXRAWLEN) return 0;
        }
    }

    /* load block header content (8 bytes) in raw->buff */
    /* since we already read the first two, we just read the next 6 bytes */
    if (fread(raw->buff+2,1,6,fp)<6) return -2;
    raw->nbyte=8;

    /* decode the length of the block and store it in len*/
    if ((raw->len=U2(raw->buff+6))>MAXRAWLEN) {
        trace(2,"sbf length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }

    /* let's store in raw->buff the whole block of length len */
    /* 8 bytes have been already read, we read raw->len-8 more */
    if (fread(raw->buff+8,1,raw->len-8,fp)<(size_t)(raw->len-8)) return -2;
    raw->nbyte=0;           /* this indicates where we point inside raw->buff */

    /* decode SBF block */
    return decode_sbf(raw);
}

