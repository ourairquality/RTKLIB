/* dat.f -- translated by f2c (version 20200916).
   You must link the resulting object file with libf2c:
	on Microsoft Windows system, link with libf2c.lib;
	on Linux or Unix systems, link with .../path/to/libf2c.a -lm
	or, if you install libf2c.a in a standard place, with -lf2c -lm
	-- in that order, at the end of the command line, as in
		cc *.o -lf2c -lm
	Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

		http://www.netlib.org/f2c/libf2c.zip
*/

#include "f2c.h"

/* Subroutine */ int iau_dat__(integer *iy, integer *im, integer *id, 
	doublereal *fd, doublereal *deltat, integer *j)
{
    /* Initialized data */

    static integer idat[84]	/* was [2][42] */ = { 1960,1,1961,1,1961,8,
	    1962,1,1963,11,1964,1,1964,4,1964,9,1965,1,1965,3,1965,7,1965,9,
	    1966,1,1968,2,1972,1,1972,7,1973,1,1974,1,1975,1,1976,1,1977,1,
	    1978,1,1979,1,1980,1,1981,7,1982,7,1983,7,1985,7,1988,1,1990,1,
	    1991,1,1992,7,1993,7,1994,7,1996,1,1997,7,1999,1,2006,1,2009,1,
	    2012,7,2015,7,2017,1 };
    static doublereal dats[42] = { 1.417818,1.422818,1.372818,1.845858,
	    1.945858,3.24013,3.34013,3.44013,3.54013,3.64013,3.74013,3.84013,
	    4.31317,4.21317,10.,11.,12.,13.,14.,15.,16.,17.,18.,19.,20.,21.,
	    22.,23.,24.,25.,26.,27.,28.,29.,30.,31.,32.,33.,34.,35.,36.,37. };
    static doublereal drift[28]	/* was [2][14] */ = { 37300.,.001296,37300.,
	    .001296,37300.,.001296,37665.,.0011232,37665.,.0011232,38761.,
	    .001296,38761.,.001296,38761.,.001296,38761.,.001296,38761.,
	    .001296,38761.,.001296,38761.,.001296,39126.,.002592,39126.,
	    .002592 };

    integer m, n;
    extern /* Subroutine */ int iau_cal2jd__(integer *, integer *, integer *, 
	    doublereal *, doublereal *, integer *);
    doublereal da;
    integer is, js;
    doublereal djm, djm0;
    logical more;

/* + */
/*  - - - - - - - - */
/*   i a u _ D A T */
/*  - - - - - - - - */

/*  For a given UTC date, calculate Delta(AT) = TAI-UTC. */

/*     :------------------------------------------: */
/*     :                                          : */
/*     :                 IMPORTANT                : */
/*     :                                          : */
/*     :  A new version of this routine must be   : */
/*     :  produced whenever a new leap second is  : */
/*     :  announced.  There are five items to     : */
/*     :  change on each such occasion:           : */
/*     :                                          : */
/*     :  1) The parameter NDAT must be           : */
/*     :     increased by 1.                      : */
/*     :                                          : */
/*     :  2) The set of DATA statements that      : */
/*     :     initialize the arrays IDAT and       : */
/*     :     DATS must be extended by one line.   : */
/*     :                                          : */
/*     :  3) The parameter IYV must be set to     : */
/*     :     the current year.                    : */
/*     :                                          : */
/*     :  4) The "Latest leap second" comment     : */
/*     :     below must be set to the new leap    : */
/*     :     second date.                         : */
/*     :                                          : */
/*     :  5) The "This revision" comment, later,  : */
/*     :     must be set to the current date.     : */
/*     :                                          : */
/*     :  Change (3) must also be carried out     : */
/*     :  whenever the routine is re-issued,      : */
/*     :  even if no leap seconds have been       : */
/*     :  added.                                  : */
/*     :                                          : */
/*     :  Latest leap second:  2016 December 31   : */
/*     :                                          : */
/*     :__________________________________________: */

/*  This routine is part of the International Astronomical Union's */
/*  SOFA (Standards of Fundamental Astronomy) software collection. */

/*  Status:  user-replaceable support routine. */

/*  Given: */
/*     IY       i     UTC:  year (Notes 1 and 2) */
/*     IM       i           month (Note 2) */
/*     ID       i           day (Notes 2 and 3) */
/*     FD       d           fraction of day (Note 4) */

/*  Returned: */
/*     DELTAT   d     TAI minus UTC, seconds */
/*     J        i     status (Note 5): */
/*                       1 = dubious year (Note 1) */
/*                       0 = OK */
/*                      -1 = bad year */
/*                      -2 = bad month */
/*                      -3 = bad day (Note 3) */
/*                      -4 = bad fraction (Note 4) */
/*                      -5 = internal error (Note 5) */

/*  Notes: */

/*  1) UTC began at 1960 January 1.0 (JD 2436934.5) and it is improper */
/*     to call the routine with an earlier date.  If this is attempted, */
/*     zero is returned together with a warning status. */

/*     Because leap seconds cannot, in principle, be predicted in */
/*     advance, a reliable check for dates beyond the valid range is */
/*     impossible.  To guard against gross errors, a year five or more */
/*     after the release year of the present routine (see parameter IYV) */
/*     is considered dubious.  In this case a warning status is returned */
/*     but the result is computed in the normal way. */

/*     For both too-early and too-late years, the warning status is J=+1. */
/*     This is distinct from the error status J=-1, which signifies a */
/*     year so early that JD could not be computed. */

/*  2) If the specified date is for a day which ends with a leap second, */
/*     the TAI-UTC value returned is for the period leading up to the */
/*     leap second.  If the date is for a day which begins as a leap */
/*     second ends, the TAI-UTC returned is for the period following the */
/*     leap second. */

/*  3) The day number must be in the normal calendar range, for example */
/*     1 through 30 for April.  The "almanac" convention of allowing */
/*     such dates as January 0 and December 32 is not supported in this */
/*     routine, in order to avoid confusion near leap seconds. */

/*  4) The fraction of day is used only for dates before the introduction */
/*     of leap seconds, the first of which occurred at the end of 1971. */
/*     It is tested for validity (0 to 1 is the valid range) even if not */
/*     used;  if invalid, zero is used and status J=-4 is returned.  For */
/*     many applications, setting FD to zero is acceptable;  the */
/*     resulting error is always less than 3 ms (and occurs only */
/*     pre-1972). */

/*  5) The status value returned in the case where there are multiple */
/*     errors refers to the first error detected.  For example, if the */
/*     month and day are 13 and 32 respectively, J=-2 (bad month) will be */
/*     returned.  The "internal error" status refers to a case that is */
/*     impossible but causes some compilers to issue a warning. */

/*  6) In cases where a valid result is not available, zero is returned. */

/*  References: */

/*  1) For dates from 1961 January 1 onwards, the expressions from the */
/*     file ftp://maia.usno.navy.mil/ser7/tai-utc.dat are used. */

/*  2) The 5ms timestep at 1961 January 1 is taken from 2.58.1 (p87) of */
/*     the 1992 Explanatory Supplement. */

/*  Called: */
/*     iau_CAL2JD   Gregorian calendar to JD */

/*  This revision:  2023 January 17 */

/*  SOFA release 2023-10-11 */

/*  Copyright (C) 2023 IAU SOFA Board.  See notes at end. */

/* ----------------------------------------------------------------------- */
/*  Release year for this version of iau_DAT */
/*  Number of Delta(AT) changes (increase by 1 for each new leap second) */
/*  Number of Delta(AT) expressions before leap seconds were introduced */
/*  Dates (year, month) on which new Delta(AT) came into force */
/*  New Delta(AT) which came into force on the given dates */
/*  Reference dates (MJD) and drift rates (s/day), pre leap seconds */
/*  Miscellaneous local variables */
/*  Dates, Delta(AT)s, reference dates, and drift rates */
/*  Dates and Delta(AT)s */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*  Initialize the result to zero and the status to OK. */
    da = 0.;
    js = 0;
/*  If invalid fraction of a day, set error status and give up. */
    if (*fd < 0. || *fd > 1.) {
	js = -4;
	goto L9000;
    }
/*  Convert the date into an MJD. */
    iau_cal2jd__(iy, im, id, &djm0, &djm, &js);
/*  If invalid year, month, or day, give up. */
    if (js < 0) {
	goto L9000;
    }
/*  If pre-UTC year, set warning status and give up. */
    if (*iy < idat[0]) {
	js = 1;
	goto L9000;
    }
/*  If suspiciously late year, set warning status but proceed. */
    if (*iy > 2028) {
	js = 1;
    }
/*  Combine year and month. */
    m = *iy * 12 + *im;
/*  Find the most recent table entry. */
    is = 0;
    more = TRUE_;
    for (n = 42; n >= 1; --n) {
	if (more) {
	    is = n;
	    more = m < idat[(n << 1) - 2] * 12 + idat[(n << 1) - 1];
	}
/* L1: */
    }
/*  Prevent underflow warnings. */
    if (is < 1) {
	js = -5;
	goto L9000;
    }
/*  Get the Delta(AT). */
    da = dats[is - 1];
/*  If pre-1972, adjust for drift. */
    if (is <= 14) {
	da += (djm + *fd - drift[(is << 1) - 2]) * drift[(is << 1) - 1];
    }
/*  Return the Delta(AT) value and the status. */
L9000:
    *deltat = da;
    *j = js;
/*  Finished. */
/* +---------------------------------------------------------------------- */

/*  Copyright (C) 2023 */
/*  Standards of Fundamental Astronomy Board */
/*  of the International Astronomical Union. */

/*  ===================== */
/*  SOFA Software License */
/*  ===================== */

/*  NOTICE TO USER: */

/*  BY USING THIS SOFTWARE YOU ACCEPT THE FOLLOWING SIX TERMS AND */
/*  CONDITIONS WHICH APPLY TO ITS USE. */

/*  1. The Software is owned by the IAU SOFA Board ("SOFA"). */

/*  2. Permission is granted to anyone to use the SOFA software for any */
/*     purpose, including commercial applications, free of charge and */
/*     without payment of royalties, subject to the conditions and */
/*     restrictions listed below. */

/*  3. You (the user) may copy and distribute SOFA source code to others, */
/*     and use and adapt its code and algorithms in your own software, */
/*     on a world-wide, royalty-free basis.  That portion of your */
/*     distribution that does not consist of intact and unchanged copies */
/*     of SOFA source code files is a "derived work" that must comply */
/*     with the following requirements: */

/*     a) Your work shall be marked or carry a statement that it */
/*        (i) uses routines and computations derived by you from */
/*        software provided by SOFA under license to you; and */
/*        (ii) does not itself constitute software provided by and/or */
/*        endorsed by SOFA. */

/*     b) The source code of your derived work must contain descriptions */
/*        of how the derived work is based upon, contains and/or differs */
/*        from the original SOFA software. */

/*     c) UNLIKE OTHER SOFA SOFTWARE, WHICH IS STRICTLY "READ ONLY", */
/*        USERS ARE PERMITTED TO REPLACE THIS SUBROUTINE WITH ONE USING */
/*        THE SAME NAME BUT DIFFERENT CODE.  This is to allow use of */
/*        locally supported mechanisms for keeping track of leap */
/*        seconds, perhaps file or network based.  It avoids the need for */
/*        applications to be relinked periodically in order to pick up */
/*        SOFA updates. */

/*     c) The names of all routines in your derived work shall not */
/*        include the prefix "iau" or "sofa" or trivial modifications */
/*        thereof such as changes of case. */

/*     d) The origin of the SOFA components of your derived work must */
/*        not be misrepresented;  you must not claim that you wrote the */
/*        original software, nor file a patent application for SOFA */
/*        software or algorithms embedded in the SOFA software. */

/*     e) These requirements must be reproduced intact in any source */
/*        distribution and shall apply to anyone to whom you have */
/*        granted a further right to modify the source code of your */
/*        derived work. */

/*     Note that, as originally distributed, the SOFA software is */
/*     intended to be a definitive implementation of the IAU standards, */
/*     and consequently third-party modifications are discouraged.  All */
/*     variations, no matter how minor, must be explicitly marked as */
/*     such, as explained above. */

/*  4. You shall not cause the SOFA software to be brought into */
/*     disrepute, either by misuse, or use for inappropriate tasks, or */
/*     by inappropriate modification. */

/*  5. The SOFA software is provided "as is" and SOFA makes no warranty */
/*     as to its use or performance.   SOFA does not and cannot warrant */
/*     the performance or results which the user may obtain by using the */
/*     SOFA software.  SOFA makes no warranties, express or implied, as */
/*     to non-infringement of third party rights, merchantability, or */
/*     fitness for any particular purpose.  In no event will SOFA be */
/*     liable to the user for any consequential, incidental, or special */
/*     damages, including any lost profits or lost savings, even if a */
/*     SOFA representative has been advised of such damages, or for any */
/*     claim by any third party. */

/*  6. The provision of any version of the SOFA software under the terms */
/*     and conditions specified herein does not imply that future */
/*     versions will also be made available under the same terms and */
/*     conditions. */

/*  In any published work or commercial product which uses the SOFA */
/*  software directly, acknowledgement (see www.iausofa.org) is */
/*  appreciated. */

/*  Correspondence concerning SOFA software should be addressed as */
/*  follows: */

/*      By email:  sofa@ukho.gov.uk */
/*      By post:   IAU SOFA Center */
/*                 HM Nautical Almanac Office */
/*                 UK Hydrographic Office */
/*                 Admiralty Way, Taunton */
/*                 Somerset, TA1 2DN */
/*                 United Kingdom */

/* ----------------------------------------------------------------------- */
    return 0;
} /* iau_dat__ */

