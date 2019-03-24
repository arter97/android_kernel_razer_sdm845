//********************************************************************************
//		<< LC898124 Evaluation Soft>>
//		Program Name	: PhoneDownload.h
//********************************************************************************


//****************************************************
//****************************************************



								
//****************************************************
//	TYPE
//****************************************************

typedef	signed char			 INT_8;
typedef	short				 INT_16;
typedef	long                 INT_32;
typedef	long long            INT_64;
typedef	unsigned char       UINT_8;
typedef	unsigned short      UINT_16;
typedef	unsigned long       UINT_32;
typedef	unsigned long long	UINT_64;

//****************************************************
//	Defines
//****************************************************


//****************************************************
//	Generic memory 
//****************************************************


	
typedef struct {
	UINT_8 Vendor;
	UINT_8 User;
	UINT_8 Model;
	UINT_8 Version;
	UINT_8 SpiMode;
	UINT_8 Reserve1;
	UINT_8 ActType;
	UINT_8 GyroType;
} DSPVER;

typedef struct {
	UINT_16	Cmd ;
	const UINT_8* DataPM;
	UINT_32 LengthPM;
	UINT_32 Parity;
	const UINT_8* DataDM;
	UINT_32 LengthDMA;
	UINT_32 LengthDMB;
}DOWNLOAD_TBL ;


extern unsigned char SelectDownload( struct cam_ois_ctrl_t *o_ctrl, UINT_8 ,  UINT_8 ) ;
extern void RemapMain( struct cam_ois_ctrl_t *o_ctrl ) ;



