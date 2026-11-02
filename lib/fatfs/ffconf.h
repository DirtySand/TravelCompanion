/*---------------------------------------------------------------------------/
/  FatFs R0.16 configuration for TravelCompanion
/  (replaces the stock ffconf.h; every option of the stock file is defined)
/---------------------------------------------------------------------------*/

#define FFCONF_DEF	80386	/* Revision ID of FatFs R0.16 */

/* Function configurations */
#define FF_FS_READONLY	0	/* read/write */
#define FF_FS_MINIMIZE	0	/* full API */
#define FF_USE_FIND		1	/* f_findfirst/f_findnext: scan existing log files */
#define FF_USE_MKFS		0	/* card is formatted on the PC (FAT32) */
#define FF_USE_FASTSEEK	0
#define FF_USE_EXPAND	0
#define FF_USE_CHMOD	0
#define FF_USE_LABEL	0
#define FF_USE_FORWARD	0
#define FF_USE_STRFUNC	0
#define FF_PRINT_LLI	0
#define FF_PRINT_FLOAT	0
#define FF_STRF_ENCODE	0

/* Locale and namespace */
#define FF_CODE_PAGE	437	/* only used for 8.3 name case conversion */
#define FF_USE_LFN		0	/* 8.3 names only (TC000001.TCL) */
#define FF_MAX_LFN		255
#define FF_LFN_UNICODE	0
#define FF_LFN_BUF		255
#define FF_SFN_BUF		12
#define FF_FS_RPATH		0
#define FF_PATH_DEPTH	10

/* Drive/volume */
#define FF_VOLUMES		1
#define FF_STR_VOLUME_ID	0
#define FF_VOLUME_STRS		"SD"
#define FF_MULTI_PARTITION	0
#define FF_MIN_SS		512
#define FF_MAX_SS		512
#define FF_LBA64		0
#define FF_MIN_GPT		0x10000000
#define FF_USE_TRIM		0

/* System */
#define FF_FS_TINY		0
#define FF_FS_EXFAT		0	/* FAT12/16/32 only: cards > 32 GB must be reformatted to FAT32 */
#define FF_FS_NORTC		0	/* timestamps from get_fattime() (DS3231 via sd_log.c) */
#define FF_NORTC_MON	1
#define FF_NORTC_MDAY	1
#define FF_NORTC_YEAR	2026
#define FF_FS_CRTIME	0
#define FF_FS_NOFSINFO	0
#define FF_FS_LOCK		0
#define FF_FS_REENTRANT	0
#define FF_FS_TIMEOUT	1000
