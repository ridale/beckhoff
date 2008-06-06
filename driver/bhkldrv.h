/**
Beckhoff CX1000 Kernel level driver
Copyright (C) 2005 Thermo Electron Corporation

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

http://www.gnu.org/copyleft/lesser.html

\file
Beckhoff kernel level driver header defines

To be able to use this driver first create the device entry in /dev. The 
following line makes the /dev/bhio device entry.
sh% mknod /dev/bhio c 250 0

After you have made the entry you can load this driver into the kernel using
the following line at the command line.
sh% insmod -f bhkldrv.o

The following shows how to use the driver.
	FILE* file_desc = open("/dev/bhio","r+"); // open the device
	ioctl(fileno(file_desc),IOCTL_BHKLDRV_RESET,NULL); // start the device
	lseek(fileno(file_desc),GCB_OFFSET,SEEK_SET); // move to the general control block
	read(fileno(file_desc), gcb_buffer, sizeof(gcb_buffer));

\author	Richard Lemon
\date	27/04/2005
*/

#ifndef _BHKLDRV_H_544_685_465_753
#define _BHKLDRV_H_544_685_465_753

// Using ascii 'b' (0x63) as the ioctl, the std ioctl tables showed it as unused.
#define IOCTL_BHKLDRV_RESET			0x6301	//!< Resets the beckhoff
#define IOCTL_BHKLDRV_START			0x6302	//!< Starts data cycle
#define IOCTL_BHKLDRV_STOP			0x6303	//!< Stops data cycle
#define IOCTL_BHKLDRV_WDSTART		0x6304	//!< Starts watchdog
#define IOCTL_BHKLDRV_WDSTOP		0x6305	//!< Stops watchdog
#define IOCTL_BHKLDRV_WDPAT			0x6306	//!< Pats watchdog
#define IOCTL_BHKLDRV_BOARDMAP		0x6307	//!< Sets the IO board map

// beckhoff specific defines
const unsigned long io_physical = 0xD0000; //!< Base address for CX1100 DP-RAM
const unsigned long io_length   = 0x4000;  //!< Total block length for CX1100 DP-RAM

// offsets from base address to CX1100 DP-RAM memory areas
const int GCB_OFFSET 		= 0xFF0;	//!< Offset to General Control Block
const int KBCB_OFFSET 		= 0xFD0;	//!< Offset to KBus Control Block
const int IPCB_OFFSET 		= 0xFB0;	//!< Offset to IPBus Control Block
const int RESERVED_OFFSET	= 0xA00;	//!< Offset to Reserved memory area (unused)
const int IPOP_OFFSET 		= 0x700;	//!< Offset to IPBus output area
const int IPIP_OFFSET 		= 0x400;	//!< Offset to IPBus input area
const int KBOP_OFFSET 		= 0x200;	//!< Offset to KBus output area
const int KBIP_OFFSET 		= 0x000;	//!< Offset to KBus input area
const int AUX_OFFSET		= 0x1000;	//!< Offset to Auxillary control block
const int UPS_OFFSET		= 0x1010;	//!< Offset to UPS control block
const int NVRAM_OFFSET		= 0x2000;	//!< Offset to NVRAM

/**
Structure to hold the number of bytes of each of the IO areas that is actually
in use by the hardware. This enables the driver to read the minimum amount of
data from the DP-RAM as the IO to DP-RAM is quite slow.
\author	Richard Lemon
\date	11/05/05
*/
typedef struct  {
	int bUseKBus;			//!< Are we using the KBus?
	int nKBusInputBytes;	//!< KBus input image size in use (number of cards)
	int nKBusOutputBytes;	//!< KBus output image size in use (number of cards)
	int bUseIPBus;			//!< Are we using the IPBus?
	int nIPBusInputBytes;	//!< IPBus input image size in use (number of cards)
	int nIPBusOutputBytes;	//!< IPBus output image size in use (number of cards)
} BH_IO_BOARD_MAP;

/** General control block structure
\author	Richard Lemon
\date	12/05/05
*/
typedef struct {
	unsigned char  fw_revision1;
	unsigned char  fw_revision2;
	unsigned char  service_request;
	unsigned char  service_response;
	unsigned short watchdog_time;
	unsigned short cycle_time;
	unsigned char  unused1;
	unsigned char  process_data_error;
	unsigned char  process_data_overrun;
	unsigned char  unused2;
	unsigned char  unused3;
	unsigned char  pd_cycle_ready;
	unsigned char  pd_cycle_request;
	unsigned char  unused4;
} GCB;

/** Bus control block structure
Used for both IP-Link and K-Bus as only the base offset address is different
between each bus.
\author	Richard Lemon
\date	12/05/05
*/
typedef struct {
	unsigned short plcif_output;
	unsigned short plcif_input;
	unsigned short diag_output;
	unsigned short diag_input;
	unsigned char  service_request;
	unsigned char  service_response;
	unsigned char  error_code;
	unsigned char  error_argument;
	unsigned short base_ptr_inputs;
	unsigned short base_ptr_outputs;
	unsigned short cycle_count;
	unsigned short cycle_time;
	unsigned char  bus_status;
	unsigned char  reserved[9];
	unsigned char  retry_counter;
} BUSCB;

/** Auxiliary control block structure
\author	Richard Lemon
\date	12/05/05
*/
typedef struct {
	unsigned char  display_cmd;
	unsigned char  display_data;
	unsigned char  in_reg;
	unsigned char  out_reg;
	unsigned long  device_id;
} AUXCB;

#endif // #ifndef _BHKLDRV_H_544_685_465_753

