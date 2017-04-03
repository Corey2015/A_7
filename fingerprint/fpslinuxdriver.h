//
// FILENAME.
//      fpslinuxdriver.h - FPS linux driver header
//
//      $PATH:
//
// FUNCTIONAL DESCRIPTION.
//      The header define the FPS Linux dirver defintion
//
//
// MODIFICATION HISTORY.
//
// NOTICE.
//      Copyright (C) 2000-2014 EgisTec All Rights Reserved.
//

#ifndef _LINUX_DIRVER_H_
#define _LINUX_DIRVER_H_

#include <asm/ioctl.h>
#include <linux/types.h>
#include "fpslinuxdriver.h"

#define SIGIO_RESUME       SIGRTMIN + 1
#define SIGIO_SUSPEND      SIGRTMIN + 2
#define SIGIO_DISCONNECT   SIGRTMIN + 3


typedef enum _SIG_TYPE
{
  SIG_UNKNOW        = 0,
  SIG_RESUME        = 1,
  SIG_DISCONNECT    = 2,
  SIG_SUSPEND       = 3     // only for test tool, SDK do not use it
}Sig_Type;


//
// Define ioctl code   
//
#define EGIS_IOCTL_MAXNR           17
 

#define EGIS_IOCTL_MAGIC                 'k'
//#define EGIS_IOCTL_SCSI_READ             _IOW(EGIS_IOCTL_MAGIC,  0, int)
//#define EGIS_IOCTL_SCSI_WRITE            _IOW(EGIS_IOCTL_MAGIC,  1, int)
#define EGIS_IOCTL_SCSI_READ             _IOW(EGIS_IOCTL_MAGIC,  1, int)
#define EGIS_IOCTL_SCSI_WRITE            _IOW(EGIS_IOCTL_MAGIC,  2, int)

//#define EGIS_IOCTL_ENTER_SUSPEND         _IO( EGIS_IOCTL_MAGIC,  2	  )
//#define EGIS_IOCTL_SET_AUTOSUSPEND_TIME  _IOW(EGIS_IOCTL_MAGIC,  3, int) 

//#define EGIS_IOCTL_RESET_DEVICE          _IO( EGIS_IOCTL_MAGIC,  4	  )

//#define EGIS_IOCTL_SET_NORMALMODE_REG    _IOW(EGIS_IOCTL_MAGIC,  5, int)
//#define EGIS_IOCTL_SET_CONTACTMODE_REG   _IOW(EGIS_IOCTL_MAGIC,  6, int)

//#define EGIS_IOCTL_CREATE_SIGNAL         _IOW(EGIS_IOCTL_MAGIC,  7, int)  // pass one of Sig_Type type

//
// For Debug information
// 
#define EGIS_IOCTL_ENABLE_DBG_MSG 		 _IOW(EGIS_IOCTL_MAGIC,  8, int)

//
// For Testing 
//
#define EGIS_IOCTL_TS_SWITCH_AUTOSUSPEND _IOW(EGIS_IOCTL_MAGIC, 9, int)
#define EGIS_IOCTL_TS_SWITCH_RMWAKEUP    _IOW(EGIS_IOCTL_MAGIC, 10, int)
#define EGIS_IOCTL_TS_SIGNAL             _IO(EGIS_IOCTL_MAGIC,  11	  )

#define EGIS_IOCTL_BULK_READ             _IOW(EGIS_IOCTL_MAGIC, 13, int)
#define EGIS_IOCTL_BULK_WRITE            _IOW(EGIS_IOCTL_MAGIC, 14, int)
#define EGIS_IOCTL_CTRLXFER_READ         _IOW(EGIS_IOCTL_MAGIC, 15, int)
#define EGIS_IOCTL_CTRLXFER_WRITE        _IOW(EGIS_IOCTL_MAGIC, 16, int)


//
// SCSI Command Size
//
#define CBW_SIZE   31
#define CSW_SIZE   13

//
// SCSI Command Format (Stand Storage CBW 31 bytes)
//

#pragma pack(1)
typedef struct _CBW_FORM_
{
  unsigned char    USBC[4];
  unsigned char    Tag[4];
  unsigned int     Length;
  unsigned char    Direction;
  unsigned char    LUN;
  unsigned char    CommandLength;
  unsigned char    VendCmd[16];
}CBW_Form, *PCBW_Form;

//
// SCSI transfer structure
//

typedef struct _SCSI_BUFFER
{
  CBW_Form            CBW;
  unsigned char*      data;
  unsigned short      datasize;
  unsigned char       CSW[CSW_SIZE];
  unsigned long       timeout;		   // in jiffies, 0 means infinite
}SCSICmd_Buf, *PSCSICmd_Buf;

//
// bulk transfer stuct
//
typedef struct _BULK_BUFFER
{
  unsigned short      DataSize;
  unsigned long       Timeout;		   // in juffies, 0 means infinite
  unsigned char*      Data;
}BULK_BUF, *PBULK_BUF;

//
// Ctrl transfer package
//
typedef	struct _CTRL_SETUP_PACKAGE
{
  unsigned char       bRequest;
  unsigned char       bRequestType;
  unsigned short      wValue;
  unsigned short      wIndex;
  unsigned short      wLength;
}CTRL_SETUP_PACKAGE, *PCTRL_SETUP_PACKAGE;


typedef struct _CTRLXFER_PACKAGE
{
  CTRL_SETUP_PACKAGE   SetupPacket;
  unsigned char        DataPacket[64];
  unsigned long        Timeout;		           // in juffies, 0 means infinite
}CTRLXFER_PACKAGE, *PCTRLXFER_PACKAGE;

#pragma pack()

//
// Driver Version Structure
//
typedef struct _DRIVER_VERSION{
  unsigned int    vnumber;
  char            vstring[16];      //xx.xx.xx.xx
}FPDriver_Version;

struct egis_ioc_transfer {
  __u64     tx_buf;
  __u64     rx_buf;

  __u32     len;
  __u32     speed_hz;

  __u16     delay_usecs;
  __u8      bits_per_word;
  __u8      cs_change;
  __u8      opcode;
  __u8      pad[3];
//__u32		pad;

  // If the contents of 'struct spi_ioc_transfer' ever change
  // incompatibly, then the ioctl number (currently 0) must change;
  // ioctls with constant size fields get a bit more in the way of
  // error checking than ones (like this) where that field varies.
  //
  // NOTE: struct layout is the same in 64bit and 32bit userspace.
  //
};

#define SPI_IOC_MAGIC			'k'
#define SPI_MSGSIZE(N) \
  ((((N)*(sizeof (struct egis_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
    ? ((N)*(sizeof (struct egis_ioc_transfer))) : 0)

#define SPI_IOC_MESSAGE(N)  _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])

//
// Read / Write SPI device default max speed hz 
//
#define EGIS_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOC_MAGIC, 4, __u32)
#define EGIS_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOC_MAGIC, 4, __u32)


//
// FPS OPCODE
//
#define FPS_REGISTER_MASSREAD             0x01
#define FPS_REGISTER_MASSWRITE            0x02
#define FPS_GET_ONE_IMG                   0x03
#define FPS_GET_FULL_IMAGE                0x05
#define FPS_GET_FULL_IMAGE2               0x06


#endif





