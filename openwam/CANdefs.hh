
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

typedef unsigned long DWORD;

#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (50)
#define RX_TIMEOUT          (50)

#define L08       (1)
#define L16       (2)
#define L24       (3)
#define L32       (4)
#define EE        (0x0008)

// keyword, index, readFunction, writeFunction, defaultVal, type
const int dataType[]={
   /* VERS */  L16 ,
   /* ROLE */ L16 | EE ,
   /* SN */ L16 | EE ,
   /* ID */ L16 | EE ,
   /* ERROR */ L16 ,
   /* STAT */ L16 ,
   /* ADDR */ L16 ,
   /* VALUE */ L16 ,
   /* MODE */ L16 ,
   /* D */ L16 ,
   /* TORQ */ L16 ,
   /* V */ L16 ,
   /* B */ L16 ,
   /* P */ L32 ,
   /* P2 */ L16 ,
   /* E */ L32 ,
   /* E2 */ L16 ,
   /* MT */ L16 | EE ,
   /* MV */ L16 | EE ,
   /* MCV */ L16 | EE ,
   /* MOV */ L16 | EE ,
   /* MOFST */ L16 | EE ,
   /* IOFST */ L16 | EE ,
   /* PTEMP */ L16 | EE ,
   /* UPSECS */ L16 | EE ,
   /* OD */ L16 | EE ,
   /* MDS */ L16 | EE ,
   /* AP */ L32 | EE ,
   /* AP2 */ L16 ,
   /* MECH */ L32 ,
   /* MECH2 */ L16 ,
   /* CTS */ L32 | EE ,
   /* CTS2 */ L16 ,
   /* DP */ L32 | EE ,
   /* DP2 */ L16 ,
   /* OT */ L32 | EE ,
   /* OT2 */ L16 ,
   /* CT */ L32 | EE ,
   /* CT2 */ L16 ,
   /* BAUD */ L16 ,
   /* TEMP */ L16 ,
   /* OTEMP */ L16 ,
   /* LOCK */ L16 ,
   /* DIG0 */ L16 ,
   /* DIG1 */ L16 ,
   /* ANA0 */ L16 ,
   /* ANA1 */ L16 ,
   /* THERM */ L16 ,
   /* VBUS */ L16 ,
   /* IMOTOR */ L16 ,
   /* VLOGIC */ L16 ,
   /* ILOGIC */ L16 ,
   /* GRPA */ L16 | EE ,
   /* GRPB */ L16 | EE ,
   /* GRPC */ L16 | EE ,
   /* PIDX */ L16 | EE ,
   /* ZERO */ L16 ,
   /* SG */ L16 ,
   /* HSG */ L16 | EE ,
   /* LSG */ L16 | EE ,
   /* DS */ L16 | EE ,
   /* IVEL */ L16 | EE ,
   /* IOFF */ L16 | EE ,
   /* MPE */ L16 | EE ,
   /* EN */ L16 ,
   /* TSTOP */ L16 | EE ,
   /* KP */ L16 | EE ,
   /* KD */ L16 | EE ,
   /* KI */ L16 | EE ,
   /* SAMPLE */ L16 | EE ,
   /* ACCEL */ L16 | EE ,
   /* TENSION */ L16 ,
   /* UNITS */ L16 | EE ,
   /* RATIO */ L16 | EE ,
   /* LOG */ L16 ,
   /* DUMP */ L16 ,
   /* LOG1 */ L16 ,
   /* LOG2 */ L16 ,
   /* LOG3 */ L16 ,
   /* LOG4 */ L16 ,
   /* GAIN1 */ L16 | EE ,
   /* GAIN2 */ L16 | EE ,
   /* GAIN3 */ L16 | EE ,
   /* OFFSET1 */ L16 | EE ,
   /* OFFSET2 */ L16 | EE ,
   /* OFFSET3 */ L16 | EE ,
   /* PEN */ L16 ,
   /* SAFE */ L16 ,
   /* SAVE */ L16 ,
   /* LOAD */ L16 ,
   /* DEF */ L16 ,
   /* VL1 */ L16 ,
   /* VL2 */ L16 ,
   /* TL1 */ L16 ,
   /* TL2 */ L16 ,
   /* VOLTL1 */ L16 | EE ,
   /* VOLTL2 */ L16 | EE ,
   /* VOLTH1 */ L16 | EE ,
   /* VOLTH2 */ L16 | EE ,
   /* MAXPWR */ L16 ,
   /* PWR */ L16 ,
   /* IFAULT */ L16 ,
   /* IKP */ L16 | EE ,
   /* IKI */ L16 | EE ,
   /* IKCOR */ L16 | EE ,
   /* VNOM */ L16 ,
   /* TENST */ L16 | EE ,
   /* TENSO */ L16 | EE ,
   /* JIDX */ L16 | EE ,
   /* IPNM */ L16 | EE ,
   /* HALLS */ L16 ,
   /* HALLH */ L32 ,
   /* HALLH2 */ L16 ,
   /* POLES */ L16 | EE ,
   /* ECMAX */ L16 ,
   /* ECMIN */ L16 ,
   /* ISQ */ L16 ,
   /* TETAE */ L16 ,
   /* FIND */ L16 ,
   /* LCV */ L16 ,
   /* LCVC */ L16 ,
   /* LFV */ L16 ,
   /* LFS */ L16 ,
   /* LFAP */ L16 ,
   /* LFDP */ L16 ,
   /* LFT */ L16 ,
   /* VALUE32 */ L16
};

int VERS;
int ROLE;
int SN;
int ID;
int ERROR;
int STAT;
int ADDR;
int VALUE;
int MODE;
int D;
int TORQ;
int MD;
int V;
int B;
int P;
int P2;
int E;
int E2;
int MT;
int MV;
int MCV;
int MOV;
int MOFST;
int IOFST;
int PTEMP;
int UPSECS;
int OD;
int MDS;
int AP;
int AP2;
int MECH;
int MECH2;
int CTS;
int CTS2;
int DP;
int DP2;
int OT;
int OT2;
int CT;
int CT2;
int BAUD;
int TEMP;
int OTEMP;
int _LOCK;
int DIG0;
int DIG1;
int ANA0;
int ANA1;
int THERM;
int VBUS;
int IMOTOR;
int VLOGIC;
int ILOGIC;
int GRPA;
int GRPB;
int GRPC;
int PIDX;
int ZERO;
int SG;
int HSG;
int LSG;
int _DS;
int IVEL;
int IOFF;
int MPE;
int EN;
int TSTOP;
int KP;
int KD;
int KI;
int SAMPLE;
int ACCEL;
int TENSION;
int UNITS;
int  RATIO;
int LOG;
int DUMP;
int LOG1;
int LOG2;
int LOG3;
int LOG4;
int GAIN1;
int GAIN2;
int GAIN3;
int OFFSET1;
int OFFSET2;
int OFFSET3;
int PEN;
int SAFE;
int SAVE;
int LOAD;
int DEF;
int VL1;
int VL2;
int TL1;
int TL2;
int VOLTL1;
int VOLTL2;
int VOLTH1;
int VOLTH2;
int MAXPWR;
int PWR;
int IFAULT;
int IKP;
int IKI;
int IKCOR;
int VNOM;
int TENST;
int TENSO;
int JIDX;
int IPNM;
int HALLS;
int HALLH;
int HALLH2;
int POLES;
int ECMAX;
int ECMIN;
int ISQ;
int TETAE;
int FIND;
int LCV;
int LCVC;
int LFV;
int LFS;
int LFAP;
int LFDP;
int LFT;
int VALUE32;
int PROP_END;

int LOCK;
int FET0;
int FET1;
int CMD;
int X0;
int X1;
int X2;
int X3;
int X4;
int X5;
int X6;
int X7;
int COMMON_END;
int SAFETY_END;
int T;
int M;
int M2;
int IOFF2;
int HOLD;
int TIE;
int LFLAGS;
int LCTC;

#endif
