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
This driver maps the dual port I/O ram to user space. It is not that clever
it just grabs the whole block every update. This is a reasonable performance
hit and it would be better to modify the driver to deal with only the bits
of memory you are interested in. The better solution would be to allow reads
of the data directly from the user code with a good spinlock on the update.
\author Richard Lemon
\date	12/08/04
*/

#include <linux/module.h>
#include <linux/config.h>
#include <linux/init.h>

#include <linux/kernel.h> 
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/reboot.h>

#include <asm/io.h>
#include <asm/errno.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include "bhkldrv.h"

// module level defines
MODULE_AUTHOR("Richard Lemon");
static int bhkldrv_major = 250;
module_param(bhkldrv_major, int, 0);
MODULE_DESCRIPTION("Support for Beckhoff CX1000 embedded PC");
MODULE_SUPPORTED_DEVICE("/dev/bhio");
MODULE_LICENSE("Dual BSD/GPL");

// global variables

/**
Used as a bitmask to signal between kernel
timer interrupt and read / write operations
bit0 = signals to stop the driver (stop scheduling the tasklet)
bit1 = signals that the driver is stopped
bit2 = mutex locks the IO memory between tasklet and user IO
bit3 = Set if the watchdog is active
bit4 = signals new data after write (dirty bit)
*/
unsigned long io_poll = 0;

const unsigned long io_used   = 0x1000;  //!< Used block length for IO

unsigned char* io_new = NULL; //!< User space buffer for Beckhoff data
unsigned char* io_old = NULL; //!< Temporary buffer for Beckhoff data
void* io_base         = NULL; //!< Base address of remapped memory area

int bh_interrupt_count = 0;   //!< Interrupt count of missed interrupts for tasklet
atomic_t bh_watchdog_count;   //!< Counter for watchdog timeouts
atomic_t bh_watchdog_timeout; //!< Counter for watchdog timeouts

//! board map variable initialised with KBus available
BH_IO_BOARD_MAP bh_boardmap = {0x01,0x200,0x200,0x00,0x00,0x00};

//! declare work structure
void bhkldrv_timedout(void *ptr);
DECLARE_WORK(bh_work, bhkldrv_timedout , NULL);

//! Tasklet function to handle bottom half processing for timer interrupt
void bh_do_tasklet(unsigned long);
DECLARE_TASKLET(bh_tasklet, bh_do_tasklet, 0);

//! Wait queue to signal when data is ready to be read.
DECLARE_WAIT_QUEUE_HEAD(read_wait_queue);


// forward function declarations
int bhkldrv_open(struct inode *inode, struct file *filp);
int bhkldrv_release(struct inode *inode, struct file *filp);
int bhkldrv_ioctl(struct inode *node, struct file *filep, unsigned int cmd, unsigned long arg);
ssize_t bhkldrv_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
ssize_t bhkldrv_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);

//! Structure to map the available interfaces to the kernel
struct file_operations bhkldrv_remap_ops = {
	open:    bhkldrv_open,
	release: bhkldrv_release,
	ioctl:   bhkldrv_ioctl,
	read:    bhkldrv_read,
	write:   bhkldrv_write,
};

/**
\brief	Runs the device IO loop, called every jiffy
\author	RDL
\date	17/08/04
*/
void bhkldrv_timedout(void *ptr)
{

	if (test_bit(0,&io_poll)) // check if we are meant to end
	{
		tasklet_schedule(&bh_tasklet);
		bh_interrupt_count++; // record the interrupt
		schedule_delayed_work(&bh_work, 2);// add ourselves back into the timer queue
	}
	else
		clear_bit(1, &io_poll); // signal back that we are finished
}

/**
\brief	Does the actual read, bottom half style processing so we don't lockup
the kernel.
\author	RDL
\date	23/08/04
\bug	Need to copy like this
*/
void bh_do_tasklet(unsigned long something)
{
	if (test_bit(3,&io_poll))
	{
		// watchdog is active
		atomic_inc( &bh_watchdog_count );
		if ( atomic_read(&bh_watchdog_count) > atomic_read(&bh_watchdog_timeout ) )
		{
			//printk("watchdog timed out - reboot in progress\n");
			kernel_restart(NULL);
			return;
		}
	}

	// if the read request is finished then lets get the data
    if (readb(io_base + GCB_OFFSET + 0x0D) == readb(io_base + GCB_OFFSET + 0x0E))
	{

		if (test_and_set_bit(2,&io_poll) != 0)
			return;

		// copy data from IO
		memcpy_fromio(io_old  + GCB_OFFSET, io_base  + GCB_OFFSET, sizeof(GCB));	// from io to swap buffer
		if (bh_boardmap.bUseKBus)
		{
			memcpy_fromio(io_old  + KBCB_OFFSET, io_base  + KBCB_OFFSET, sizeof(BUSCB));	// from io to swap buffer
			if ( bh_boardmap.nKBusInputBytes ) 		memcpy_fromio(io_old  + KBIP_OFFSET, io_base  + KBIP_OFFSET, bh_boardmap.nKBusInputBytes );	// from io to swap buffer
			if ( bh_boardmap.nKBusOutputBytes )		memcpy_fromio(io_old  + KBOP_OFFSET, io_base  + KBOP_OFFSET, bh_boardmap.nKBusOutputBytes);	// from io to swap buffer
		}
		if (bh_boardmap.bUseIPBus)
		{
			memcpy_fromio(io_old  + IPCB_OFFSET, io_base  + IPCB_OFFSET, sizeof(BUSCB));	// from io to swap buffer
			if ( bh_boardmap.nIPBusInputBytes )		memcpy_fromio(io_old  + IPIP_OFFSET, io_base  + IPIP_OFFSET, bh_boardmap.nIPBusInputBytes );	// from io to swap buffer
			if ( bh_boardmap.nIPBusOutputBytes )	memcpy_fromio(io_old  + IPOP_OFFSET, io_base  + IPOP_OFFSET, bh_boardmap.nIPBusOutputBytes);	// from io to swap buffer
		}


		if ( test_bit(4,&io_poll) )// buffer dirty copy data to IO
		{
			if  ((bh_boardmap.bUseKBus) && ( bh_boardmap.nKBusOutputBytes ))
				memcpy_toio(io_base + KBOP_OFFSET, io_new + KBOP_OFFSET, bh_boardmap.nKBusOutputBytes);	// from user to dp ram
			
			if ((bh_boardmap.bUseIPBus) && ( bh_boardmap.nIPBusOutputBytes ))
				memcpy_toio(io_base + IPOP_OFFSET, io_new + IPOP_OFFSET, bh_boardmap.nIPBusOutputBytes);	// from user to dp ram
		}
		else
		{
			// buffer clean, let's sync the output buffer
			if ( (bh_boardmap.bUseKBus) && ( bh_boardmap.nKBusOutputBytes )	)
				memcpy(io_new + KBOP_OFFSET, io_old + KBOP_OFFSET, bh_boardmap.nKBusOutputBytes);	// from user to dp ram
			
			if ( (bh_boardmap.bUseIPBus) && ( bh_boardmap.nIPBusOutputBytes ) )
				memcpy(io_new + IPOP_OFFSET, io_old + IPOP_OFFSET, bh_boardmap.nIPBusOutputBytes);	// from user to dp ram
			// signal that the buffer is ready
			wake_up_interruptible(&read_wait_queue);
		}
		// signal that the buffer is clean
		clear_bit(4,&io_poll);
		// ready for user IO again
		clear_bit(2,&io_poll);
		//start another run
		writeb(readb(io_base + GCB_OFFSET + 0x0D) + 1, io_base + GCB_OFFSET + 0x0E);
	}
}

/**
\brief	Open the device; all we have to do here is to up the usage count and
set the right fops.
\author	RDL
\date	13/08/04
*/
int bhkldrv_open(struct inode *inode, struct file *filp)
{
    unsigned int dev = MINOR(inode->i_rdev);

    if (dev >= 1) 
        return -ENODEV;

    filp->f_op = &bhkldrv_remap_ops;
//    MOD_INC_USE_COUNT;
    return 0;
}

/**
\brief	Close the device; all we have to do here is to decrement the usage
count.
\author	RDL
\date	13/08/04
*/
int bhkldrv_release(struct inode *inode, struct file *filp)
{
//    MOD_DEC_USE_COUNT;

    return 0;
}

/**
\brief	Handle operation, wait for read cycle from HW to finish.
\author	RDL
\date	16/08/04
\todo	See wether we need the schedule timeout code!
*/
ssize_t bhkldrv_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	// handle reading past the end of file
	if (*f_pos >= io_length) // no bytes, start point past eof
		return 0;
	if (*f_pos + count > io_length) // trim bytes, read past eof
		count = io_length - *f_pos;

	// copy data to userland
	if ( count > 0 )
	{
		// wait for data to be ready
		interruptible_sleep_on(&read_wait_queue);
		// Wait for cycle to finish
		while (test_and_set_bit(2,&io_poll) != 0)
		{
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		};
//			udelay(10);

		if ( copy_to_user((void*)buf, io_old + *f_pos , count) )
		{
			test_and_clear_bit(2,&io_poll);
			printk(KERN_INFO "bhkldrv::Read copy to user error\n");
			return -EFAULT;
		}
		test_and_clear_bit(2,&io_poll);
	}

	// increment the position pointer
	*f_pos += count;

	// return number of bytes read
	return count;
}

/**
\brief	Handle operation, wait for read cycle from HW to finish.
\author	RDL
\date	16/08/04
\bug	Found that the copy is straight to IO... oops, should double buffer
and then do the copy up in the tasklet
*/
ssize_t bhkldrv_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	// handle writing past the end of file
	if (*f_pos >= io_length) // no bytes, start point past eof
		return 0;
	if (*f_pos + count > io_length) // trim bytes, write past eof
		count = io_length - *f_pos;

	// copy data from userland
	if ( count > 0 )
	{
		// Wait for cycle to finish
		while (test_and_set_bit(2,&io_poll) != 0)
		{
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		};

		set_bit(4,&io_poll); // signal that the buffer is dirty

		if ( copy_from_user(io_new + *f_pos, buf, count) )
		{
			test_and_clear_bit(2,&io_poll);
			return -EFAULT;
		}
		test_and_clear_bit(2,&io_poll);
	}

	// increment the position pointer
	*f_pos += count;

	// return number of bytes written
	return count;
}

/**
\brief	Handles IOCTL calls to the device.
\author	RDL
\date	14/08/04
*/
int bhkldrv_ioctl(struct inode *node, struct file *filep, unsigned int cmd, unsigned long arg)
{
	BH_IO_BOARD_MAP conf;
	switch(cmd)
	{
	case IOCTL_BHKLDRV_RESET:
		// Wait for cycle to finish
		while (test_and_set_bit(2,&io_poll) != 0)
			udelay(10);
		printk(KERN_INFO "bhkldrv::ioctl reset\n");
		if (arg == 0)
		{
			writeb(1, io_base + AUX_OFFSET + 0x03); // set high
			writeb(0, io_base + AUX_OFFSET + 0x03); // reset low
		}
		else if (arg == 1) // reset KBus
		{
			writeb(1, io_base + KBCB_OFFSET + 0x08); // set high
			writeb(0, io_base + KBCB_OFFSET + 0x08); // reset low
		}
		else if (arg == 2) // reset IPBus
		{
			writeb(1, io_base + IPCB_OFFSET + 0x08); // set high
			writeb(0, io_base + IPCB_OFFSET + 0x08); // reset low
		}
		test_and_clear_bit(2,&io_poll);
		break;
	case IOCTL_BHKLDRV_START:
		// add ourselves into the timer queue
		printk(KERN_INFO "bhkldrv::ioctl start\n");
		if (test_bit(1,&io_poll) != 0) // we are currently running
			return -EFAULT; // or maybe 0 ??
		set_bit(0,&io_poll);
		set_bit(1,&io_poll);
		schedule_delayed_work(&bh_work, 2);// add ourselves into the timer queue
		break;

	case IOCTL_BHKLDRV_STOP:
		printk(KERN_INFO "bhkldrv::ioctl stop\n");
		clear_bit(0,&io_poll); // signal we are done
		while (test_bit(1,&io_poll) != 0) // wait for the timer to finish
			mdelay(1);
		break;

	case IOCTL_BHKLDRV_WDSTART:
		printk(KERN_INFO "bhkldrv::ioctl watchdog start\n");
		if (arg < 50 )
		{
			printk(KERN_INFO "bhkldrv::wdstart < 50\n");
			arg = 50;
		}
		if (arg > 3000)
		{
			printk(KERN_INFO "bhkldrv::wdstart > 300\n");
			arg = 3000;
		}
		atomic_set(&bh_watchdog_timeout, (arg / 20));
		atomic_set(&bh_watchdog_count,0);
		set_bit(3,&io_poll);
		break;

	case IOCTL_BHKLDRV_WDSTOP:
		printk(KERN_INFO "bhkldrv::ioctl watchdog stop\n");
		clear_bit(3,&io_poll);
		atomic_set(&bh_watchdog_count,0);
		break;

	case IOCTL_BHKLDRV_WDPAT:
		//printk(KERN_INFO "bhkldrv::ioctl watchdog pat\n");
		atomic_set(&bh_watchdog_count,0);
		break;

	case IOCTL_BHKLDRV_BOARDMAP:
		// let's copy the board map struct from user memory
		printk(KERN_INFO "bhkldrv::ioctl set boardmap\n");
		if ( copy_from_user((void *)&conf, (void *)arg, sizeof(BH_IO_BOARD_MAP)))
			return -EFAULT;

		bh_boardmap = conf;
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

/**
\brief	Remap a not (necessarily) aligned port region
\author	RDL
\date	16/08/04
*/
void *align_remap(unsigned long phys_addr)
{
    // The code comes mainly from arch/any/mm/ioremap.c
    unsigned long offset, last_addr, size;

    last_addr = phys_addr + io_length - 1;
    offset = phys_addr & ~PAGE_MASK;
    
    // Adjust the begin and end to remap a full page
    phys_addr &= PAGE_MASK;
    size = PAGE_ALIGN(last_addr) - phys_addr;
    return ioremap(phys_addr, size) + offset;
}

/**
\brief	Unmap a region obtained with align_remap
\author	RDL
\date	16/08/04
*/
void align_unmap(void *virt_add)
{
    iounmap((void *)((unsigned long)virt_add & PAGE_MASK));
}

/**
\brief	Module initialization code
\author	RDL
\date	12/08/04
*/
int enter_module(void)
{
    int result;
	result = check_mem_region(io_physical, io_length);
	if (result)
	{
	    printk(KERN_INFO "bhkldrv::Cannot lock I/O mem address 0x%lx length 0x%lx\n", io_physical, io_length);
	    return result;
	}
	request_mem_region(io_physical, io_length, "bhkldrv");
	
	io_base = align_remap(io_physical);
	if (io_base == 0x0000)
		return -1; // need some error

	io_new = (unsigned char*) kmalloc(io_length,GFP_KERNEL);
	io_old = (unsigned char*) kmalloc(io_length,GFP_KERNEL);

//	SET_MODULE_OWNER(&bhkldrv_remap_ops);

	// register our chr dev
    result = register_chrdev(bhkldrv_major, "bhkldrv", &bhkldrv_remap_ops);

	return 0;
}

/**
\brief	Module cleanup code
\author	RDL
\date	12/08/04
*/
void exit_module(void)
{
	clear_bit(0,&io_poll); // signal we are done
	while (test_bit(1,&io_poll) != 0) // wait for the timer to finish
		mdelay(1);
    unregister_chrdev(bhkldrv_major, "bhkldrv");
	kfree(io_new);
	kfree(io_old);
	align_unmap((void *)io_base);
	release_mem_region(io_physical, io_length);
}

module_init(enter_module);
module_exit(exit_module);
