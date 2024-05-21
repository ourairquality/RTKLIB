/*------------------------------------------------------------------------------
 * rtcm.c : RTCM functions
 *
 *          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
 *
 * References :
 *     [1] RTCM Recommended Standards for Differential GNSS (Global Navigation
 *         Satellite Systems) Service version 2.3, August 20, 2001
 *     [7] RTCM Standard 10403.1 - Amendment 5, Differential GNSS (Global
 *         Navigation Satellite Systems) Services - version 3, July 1, 2011
 *     [10] RTCM Paper 059-2011-SC104-635 (draft Galileo and QZSS SSR messages)
 *     [15] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
 *          Systems) Services - version 3, with amendment 1/2, November 7, 2013
 *     [16] Proposal of new RTCM SSR Messages (ssr_1_gal_qzss_sbas_dbs_v05)
 *          2014/04/17
 *     [17] RTCM Standard 10403.3, Differential GNSS (Global Navigation Satellite
 *          Systems) Services - version 3, with amendment 1, April 28, 2020
 *     [18] IGS State Space Representation (SSR) Format version 1.00, October 5,
 *          2020
 *
 * Version : $Revision:$ $Date:$
 * History : 2009/04/10 1.0  new
 *           2009/06/29 1.1  support type 1009-1012 to get synchronous-gnss-flag
 *           2009/12/04 1.2  support type 1010,1012,1020
 *           2010/07/15 1.3  support type 1057-1068 for SSR corrections
 *                           support type 1007,1008,1033 for antenna info
 *           2010/09/08 1.4  fix problem of ephemeris and SSR sequence upset
 *                           (2.4.0_p8)
 *           2012/05/11 1.5  comply with RTCM 3 final SSR format (RTCM 3
 *                           Amendment 5) (ref [7]) (2.4.1_p6)
 *           2012/05/14 1.6  separate rtcm2.c, rtcm3.c
 *                           add options to select used codes for MSM
 *           2013/04/27 1.7  comply with RTCM 3.2 with amendment 1/2 (ref[15])
 *           2013/12/06 1.8  support SBAS/BeiDou SSR messages (ref[16])
 *           2018/01/29 1.9  support RTCM 3.3 (ref[17])
 *                           crc24q() -> rtk_crc24q()
 *           2018/10/10 1.10 fix bug on initializing RTCM struct
 *                           add RTCM option -GALINAV, -GALFNAV
 *           2018/11/05 1.11 add notes for api gen_rtcm3()
 *           2020/11/30 1.12 modify API gen_rtcm3()
 *                           support NavIC/IRNSS MSM and ephemeris (ref [17])
 *                           allocate double size of ephemeris buffer to support
 *                            multiple ephemeris sets in init_rtcm()
 *                           delete references [2]-[6],[8],[9],[11]-[14]
 *                           update reference [17]
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Function prototypes -------------------------------------------------------*/
extern int decode_rtcm2(rtcm_t *rtcm);
extern int decode_rtcm3(rtcm_t *rtcm);
extern bool encode_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync);

/* Constants -----------------------------------------------------------------*/

#define RTCM2PREAMB 0x66 /* RTCM ver.2 frame preamble */
#define RTCM3PREAMB 0xD3 /* RTCM ver.3 frame preamble */

/* Initialize RTCM control -----------------------------------------------------
 * Initialize RTCM control struct and reallocate memory for observation and
 * Ephemeris buffer in RTCM control struct
 * Args   : rtcm_t *raw   IO     RTCM control struct
 * Return : status (true:ok,false:memory allocation error)
 *----------------------------------------------------------------------------*/
extern bool init_rtcm(rtcm_t *rtcm) {
  trace(3, "init_rtcm:\n");

  gtime_t time0 = {0};
  obsd_t data0 = {{0}};
  eph_t eph0 = {0, -1, -1};
  geph_t geph0 = {0, -1};
  ssr_t ssr0 = {{{0}}};

  rtcm->staid = rtcm->stah = rtcm->seqno = rtcm->outtype = 0;
  rtcm->time = rtcm->time_s = time0;
  rtcm->sta.name[0] = rtcm->sta.marker[0] = '\0';
  rtcm->sta.antdes[0] = rtcm->sta.antsno[0] = '\0';
  rtcm->sta.rectype[0] = rtcm->sta.recver[0] = rtcm->sta.recsno[0] = '\0';
  rtcm->sta.antsetup = rtcm->sta.itrf = rtcm->sta.deltype = 0;
  for (int i = 0; i < 3; i++) {
    rtcm->sta.pos[i] = rtcm->sta.del[i] = 0.0L;
  }
  rtcm->sta.hgt = 0.0L;
  rtcm->dgps = NULL;
  for (int i = 0; i < MAXSAT; i++) {
    rtcm->ssr[i] = ssr0;
  }
  rtcm->msg[0] = rtcm->msgtype[0] = rtcm->opt[0] = '\0';
  for (int i = 0; i < 6; i++) rtcm->msmtype[i][0] = '\0';
  rtcm->obsflag = rtcm->ephsat = 0;
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      rtcm->cp[i][j] = 0.0L;
      rtcm->lock[i][j] = rtcm->loss[i][j] = 0;
      rtcm->lltime[i][j] = time0;
    }
  rtcm->nbyte = rtcm->nbit = rtcm->len = 0;
  rtcm->word = 0;
  for (int i = 0; i < 100; i++) rtcm->nmsg2[i] = 0;
  for (int i = 0; i < 400; i++) rtcm->nmsg3[i] = 0;

  rtcm->obs.data = NULL;
  for (int i = 0; i < MAXSAT; i++) rtcm->nav.eph[i] = NULL;
  for (int i = 0; i < NSATGLO; i++) rtcm->nav.geph[i] = NULL;

  /* Reallocate memory for observation and ephemeris buffer */
  if (!(rtcm->obs.data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS))) {
    free_rtcm(rtcm);
    return false;
  }
  rtcm->obs.n = 0;
  for (int i = 0; i < MAXOBS; i++) rtcm->obs.data[i] = data0;

  for (int i = 0; i < MAXSAT; i++) {
    if (!(rtcm->nav.eph[i] = (eph_t *)malloc(sizeof(eph_t) * 2))) {
      free_rtcm(rtcm);
      return false;
    }
    rtcm->nav.eph[i][0] = eph0;
    rtcm->nav.eph[i][1] = eph0;
    rtcm->nav.n[i] = rtcm->nav.nmax[i] = 2;
  }
  for (int i = 0; i < NSATGLO; i++) {
    if (!(rtcm->nav.geph[i] = (geph_t *)malloc(sizeof(geph_t)))) {
      free_rtcm(rtcm);
      return false;
    }
    rtcm->nav.geph[i][0] = geph0;
    rtcm->nav.ng[i] = rtcm->nav.ngmax[i] = 1;
  }
  return true;
}
/* Free RTCM control -----------------------------------------------------------
 * Free observation and ephemeris buffer in RTCM control struct
 * Args   : rtcm_t *raw   IO     RTCM control struct
 * Return : none
 *----------------------------------------------------------------------------*/
extern void free_rtcm(rtcm_t *rtcm) {
  trace(3, "free_rtcm:\n");

  /* Free memory for observation and ephemeris buffer */
  free(rtcm->obs.data);
  rtcm->obs.data = NULL;
  rtcm->obs.n = 0;
  for (int i = 0; i < MAXSAT; i++) {
    free(rtcm->nav.eph[i]);
    rtcm->nav.eph[i] = NULL;
    rtcm->nav.n[i] = rtcm->nav.nmax[i] = 0;
  }
  for (int i = 0; i < NSATGLO; i++) {
    free(rtcm->nav.geph[i]);
    rtcm->nav.geph[i] = NULL;
    rtcm->nav.ng[i] = rtcm->nav.ngmax[i] = 0;
  }
}
/* Input RTCM 2 message from stream --------------------------------------------
 * Fetch next RTCM 2 message and input a message from byte stream
 * Args   : rtcm_t *rtcm IO   RTCM control struct
 *          uint8_t data     I   stream data (1 byte)
 * Return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 5: input station pos/ant parameters,
 *                  6: input time parameter, 7: input dgps corrections,
 *                  9: input special message)
 * Notes  : before firstly calling the function, time in RTCM control struct has
 *          to be set to the approximate time within 1/2 hour in order to resolve
 *          ambiguity of time in RTCM messages.
 *          supported msgs RTCM ver.2: 1,3,9,14,16,17,18,19,22
 *          refer [1] for RTCM ver.2
 *----------------------------------------------------------------------------*/
extern int input_rtcm2(rtcm_t *rtcm, uint8_t data) {
  trace(5, "input_rtcm2: data=%02x\n", data);

  if ((data & 0xC0) != 0x40) return 0; /* Ignore if upper 2bit != 01 */

  for (int i = 0; i < 6; i++, data >>= 1) { /* Decode 6-of-8 form */
    rtcm->word = (rtcm->word << 1) + (data & 1);

    /* Synchronize frame */
    if (rtcm->nbyte == 0) {
      uint8_t preamb = (uint8_t)(rtcm->word >> 22);
      if (rtcm->word & 0x40000000) preamb ^= 0xFF; /* Decode preamble */
      if (preamb != RTCM2PREAMB) continue;

      /* Check parity */
      if (!decode_word(rtcm->word, rtcm->buff)) continue;
      rtcm->nbyte = 3;
      rtcm->nbit = 0;
      continue;
    }
    if (++rtcm->nbit < 30)
      continue;
    else
      rtcm->nbit = 0;

    /* Check parity */
    if (!decode_word(rtcm->word, rtcm->buff + rtcm->nbyte)) {
      trace(2, "rtcm2 partity error: i=%d word=%08x\n", i, rtcm->word);
      rtcm->nbyte = 0;
      rtcm->word &= 0x3;
      continue;
    }
    rtcm->nbyte += 3;
    if (rtcm->nbyte == 6) rtcm->len = (rtcm->buff[5] >> 3) * 3 + 6;
    if (rtcm->nbyte < rtcm->len) continue;
    rtcm->nbyte = 0;
    rtcm->word &= 0x3;

    /* Decode RTCM2 message */
    return decode_rtcm2(rtcm);
  }
  return 0;
}
static uint32_t rtcm_getbitu(const rtcm_t *rtcm, unsigned pos, unsigned len) {
  return getbitu(rtcm->buff, sizeof(rtcm->buff), pos, len);
}
/* Input RTCM 3 message from stream --------------------------------------------
 * Fetch next RTCM 3 message and input a message from byte stream
 * Args   : rtcm_t *rtcm IO   RTCM control struct
 *          uint8_t data     I   stream data (1 byte)
 * Return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 5: input station pos/ant parameters,
 *                  10: input SSR messages)
 * Notes  : before firstly calling the function, time in RTCM control struct has
 *          to be set to the approximate time within 1/2 week in order to resolve
 *          ambiguity of time in RTCM messages.
 *
 *          to specify input options, set rtcm->opt to the following option
 *          strings separated by spaces.
 *
 *          -EPHALL  : input all ephemerides (default: only new)
 *          -STA=nnn : input only message with STAID=nnn (default: all)
 *          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
 *          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
 *          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
 *          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
 *          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
 *          -ILss    : select signal ss for IRN MSM (ss=5A,9A,...)
 *          -GALINAV : select I/NAV for Galileo ephemeris (default: all)
 *          -GALFNAV : select F/NAV for Galileo ephemeris (default: all)
 *
 *          supported RTCM 3 messages (ref [7][10][15][16][17][18])
 *
 *            TYPE       :  GPS   GLONASS Galileo  QZSS     BDS    SBAS    NavIC
 *         ---------------------------------------------------------------------
 *          OBS COMP L1  : 1001~   1009~     -       -       -       -       -
 *              FULL L1  : 1002    1010      -       -       -       -       -
 *              COMP L1L2: 1003~   1011~     -       -       -       -       -
 *              FULL L1L2: 1004    1012      -       -       -       -       -
 *
 *          NAV          : 1019    1020    1045**  1044    1042      -     1041
 *                           -       -     1046**    -       63*     -       -
 *
 *          MSM 1        : 1071~   1081~   1091~   1111~   1121~   1101~   1131~
 *              2        : 1072~   1082~   1092~   1112~   1122~   1102~   1132~
 *              3        : 1073~   1083~   1093~   1113~   1123~   1103~   1133~
 *              4        : 1074    1084    1094    1114    1124    1104    1134
 *              5        : 1075    1085    1095    1115    1125    1105    1135
 *              6        : 1076    1086    1096    1116    1126    1106    1136
 *              7        : 1077    1087    1097    1117    1127    1107    1137
 *
 *          SSR ORBIT    : 1057    1063    1240*   1246*   1258*     -       -
 *              CLOCK    : 1058    1064    1241*   1247*   1259*     -       -
 *              CODE BIAS: 1059    1065    1242*   1248*   1260*     -       -
 *              OBT/CLK  : 1060    1066    1243*   1249*   1261*     -       -
 *              URA      : 1061    1067    1244*   1250*   1262*     -       -
 *              HR-CLOCK : 1062    1068    1245*   1251*   1263*     -       -
 *              PHAS BIAS:   11*     -       12*     13*     14*     -       -
 *
 *          ANT/RCV INFO : 1007    1008    1033
 *          STA POSITION : 1005    1006
 *
 *          PROPRIETARY  : 4076 (IGS)
 *         ---------------------------------------------------------------------
 *                            (* draft, ** 1045:F/NAV,1046:I/NAV, ~ only encode)
 *
 *          for MSM observation data with multiple signals for a frequency,
 *          a signal is selected according to internal priority. to select
 *          a specified signal, use the input options.
 *
 *          RTCM 3 message format:
 *            +----------+--------+-----------+--------------------+----------+
 *            | preamble | 000000 |  length   |    data message    |  parity  |
 *            +----------+--------+-----------+--------------------+----------+
 *            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
 *
 *----------------------------------------------------------------------------*/
extern int input_rtcm3(rtcm_t *rtcm, uint8_t data) {
  trace(5, "input_rtcm3: data=%02x\n", data);

  /* Synchronize frame */
  if (rtcm->nbyte == 0) {
    if (data != RTCM3PREAMB) return 0;
    rtcm->buff[rtcm->nbyte++] = data;
    return 0;
  }
  rtcm->buff[rtcm->nbyte++] = data;

  if (rtcm->nbyte == 3) {
    rtcm->len = rtcm_getbitu(rtcm, 14, 10) + 3; /* Length without parity */
  }
  if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3) return 0;
  rtcm->nbyte = 0;

  /* Check parity */
  if (rtk_crc24q(rtcm->buff, sizeof(rtcm->buff), rtcm->len) !=
      rtcm_getbitu(rtcm, rtcm->len * 8, 24)) {
    trace(2, "rtcm3 parity error: len=%d\n", rtcm->len);
    return 0;
  }
  /* Decode RTCM3 message */
  return decode_rtcm3(rtcm);
}
/* Input RTCM 2 message from file ----------------------------------------------
 * Fetch next RTCM 2 message and input a message from file
 * Args   : rtcm_t *rtcm IO   RTCM control struct
 *          FILE  *fp    I    file pointer
 * Return : status (-2: end of file, -1...10: same as above)
 * Notes  : same as above
 *----------------------------------------------------------------------------*/
extern int input_rtcm2f(rtcm_t *rtcm, FILE *fp) {
  trace(4, "input_rtcm2f\n");

  for (int i = 0; i < 4096; i++) {
    int data = fgetc(fp);
    if (data == EOF) return -2;
    int ret = input_rtcm2(rtcm, (uint8_t)data);
    if (ret) return ret;
  }
  return 0; /* Return at every 4k bytes */
}
/* Input RTCM 3 message from file ----------------------------------------------
 * Fetch next RTCM 3 message and input a message from file
 * Args   : rtcm_t *rtcm IO   RTCM control struct
 *          FILE  *fp    I    file pointer
 * Return : status (-2: end of file, -1...10: same as above)
 * Notes  : same as above
 *----------------------------------------------------------------------------*/
extern int input_rtcm3f(rtcm_t *rtcm, FILE *fp) {
  trace(4, "input_rtcm3f\n");

  for (int i = 0; i < 4096; i++) {
    int data = fgetc(fp);
    if (data == EOF) return -2;
    int ret = input_rtcm3(rtcm, (uint8_t)data);
    if (ret) return ret;
  }
  return 0; /* Return at every 4k bytes */
}

static void rtcm_setbitu(rtcm_t *rtcm, unsigned pos, unsigned len, uint32_t data) {
  setbitu(rtcm->buff, sizeof(rtcm->buff), pos, len, data);
}

/* Generate RTCM 2 message -----------------------------------------------------
 * Generate RTCM 2 message
 * Args   : rtcm_t *rtcm   IO RTCM control struct
 *          int    type    I  message type
 *          int    sync    I  sync flag (1:another message follows)
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool gen_rtcm2(rtcm_t *rtcm, int type, int sync) {
  trace(4, "gen_rtcm2: type=%d sync=%d\n", type, sync);

  rtcm->nbit = rtcm->len = rtcm->nbyte = 0;

  /* Not yet implemented */

  return false;
}
/* Generate RTCM 3 message -----------------------------------------------------
 * Generate RTCM 3 message
 * Args   : rtcm_t *rtcm   IO RTCM control struct
 *          int    type    I  message type
 *          int    subtype   I   message subtype
 *          int    sync    I  sync flag (1:another message follows)
 * Return : status (true:ok,false:error)
 * Notes  : For RTCM 3 MSM, the {nsat} x {nsig} in rtcm->obs should not exceed
 *          64. If {nsat} x {nsig} of the input obs data exceeds 64, separate
 *          them to multiple ones and call gen_rtcm3() multiple times as user
 *          responsibility.
 *          ({nsat} = number of valid satellites, {nsig} = number of signals in
 *          the obs data)
 *----------------------------------------------------------------------------*/
extern bool gen_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync) {
  trace(4, "gen_rtcm3: type=%d subtype=%d sync=%d\n", type, subtype, sync);

  rtcm->nbit = rtcm->len = rtcm->nbyte = 0;

  /* Set preamble and reserved */
  int i = 0;
  rtcm_setbitu(rtcm, i, 8, RTCM3PREAMB);
  i += 8;
  rtcm_setbitu(rtcm, i, 6, 0);
  i += 6;
  rtcm_setbitu(rtcm, i, 10, 0);
  /* i += 10; */

  /* Encode RTCM 3 message body */
  if (!encode_rtcm3(rtcm, type, subtype, sync)) return false;

  /* Padding to align 8 bit boundary */
  for (i = rtcm->nbit; i % 8; i++) {
    rtcm_setbitu(rtcm, i, 1, 0);
  }
  /* Message length (header+data) (bytes) */
  if ((rtcm->len = i / 8) >= 3 + 1024) {
    trace(2, "generate rtcm 3 message length error len=%d\n", rtcm->len - 3);
    rtcm->nbit = rtcm->len = 0;
    return false;
  }
  /* Message length without header and parity */
  rtcm_setbitu(rtcm, 14, 10, rtcm->len - 3);

  /* Crc-24q */
  uint32_t crc = rtk_crc24q(rtcm->buff, sizeof(rtcm->buff), rtcm->len);
  rtcm_setbitu(rtcm, i, 24, crc);

  /* Length total (bytes) */
  rtcm->nbyte = rtcm->len + 3;

  return true;
}
