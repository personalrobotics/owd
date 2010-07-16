
#ifndef __CANDEFS_HH__
#define __CANDEFS_HH__

#define mbxID               (0)
#define BASE_ID             (0)

#define MAX_BUS             (4)
#define SAFETY_MODULE       (10)

#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

// typedef unsigned long DWORD;

#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (50)
#define RX_TIMEOUT          (50)

// initialize everything to -10.  Only some of these will get set to
// proper values based on the firmware version.  If CANbus::compile is
// called with an unset property for that puck version, it will complain.

int VERS=-10;
int ROLE=-10;
int SN=-10;
int ID=-10;
int ERROR=-10;
int STAT=-10;
int ADDR=-10;
int VALUE=-10;
int MODE=-10;
int TEMP=-10;
int PTEMP=-10;
int OTEMP=-10;
int BAUD=-10;
int _LOCK=-10;
int DIG0=-10;
int DIG1=-10;
int FET0=-10;
int FET1=-10;
int ANA0=-10;
int ANA1=-10;
int THERM=-10;
int VBUS=-10;
int IMOTOR=-10;
int VLOGIC=-10;
int ILOGIC=-10;
int SG=-10;
int GRPA=-10;
int GRPB=-10;
int GRPC=-10;
int CMD=-10;
int SAVE=-10;
int LOAD=-10;
int DEF=-10;
int FIND=-10;
int X0=-10;
int X1=-10;
int X2=-10;
int X3=-10;
int X4=-10;
int X5=-10;
int X6=-10;
int X7=-10;
int COMMON_END=-10;
int ZERO=-10;
int PEN=-10;
int SAFE=-10;
int VL1=-10;
int VL2=-10;
int TL1=-10;
int TL2=-10;
int VOLTL1=-10;
int VOLTL2=-10;
int VOLTH1=-10;
int VOLTH2=-10;
int PWR=-10;
int MAXPWR=-10;
int IFAULT=-10;
int VNOM=-10;
int SAFETY_END=-10;
int T=-10;
int MT=-10;
int V=-10;
int MV=-10;
int MCV=-10;
int MOV=-10;
int P=-10;
int P2=-10;
int DP=-10;
int DP2=-10;
int E=-10;
int E2=-10;
int OT=-10;
int OT2=-10;
int CT=-10;
int CT2=-10;
int M=-10;
int M2=-10;
int _DS=-10;
int MOFST=-10;
int IOFST=-10;
int UPSECS=-10;
int OD=-10;
int MDS=-10;
int MECH=-10;
int MECH2=-10;
int CTS=-10;
int CTS2=-10;
int PIDX=-10;
int HSG=-10;
int LSG=-10;
int IVEL=-10;
int IOFF=-10;
int IOFF2=-10;
int MPE=-10;
int HOLD=-10;
int TSTOP=-10;
int KP=-10;
int KD=-10;
int KI=-10;
int ACCEL=-10;
int TENST=-10;
int TENSO=-10;
int JIDX=-10;
int IPNM=-10;
int HALLS=-10;
int HALLH=-10;
int HALLH2=-10;
int POLES=-10;
int IKP=-10;
int IKI=-10;
int IKCOR=-10;
int EN=-10;
int EN2=-10;
int JP=-10;
int JP2=-10;
int JOFST=-10;
int JOFST2=-10;
int TIE=-10;
int ECMAX=-10;
int ECMIN=-10;
int LFLAGS=-10;
int LCTC=-10;
int LCVC=-10;
int TACT=-10;
int TACTID=-10;
int PROP_END=-10;
int AP=-10;
int TENSION=-10;

// older properties (firmware < 40)
int D=-10;
int TORQ=-10;
int B=-10;
int MD=-10;
int AP2=-10;
int SAMPLE=-10;
int UNITS=-10;
int RATIO=-10;
int LOG=-10;
int DUMP=-10;
int LOG1=-10;
int LOG2=-10;
int LOG3=-10;
int LOG4=-10;
int GAIN1=-10;
int GAIN2=-10;
int GAIN3=-10;
int OFFSET1=-10;
int OFFSET2=-10;
int OFFSET3=-10;

#endif
