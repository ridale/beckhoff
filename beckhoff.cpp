/**
\file	beckhoff.cpp
\author	Richard Lemon
\date	18/09/07
\file
This should provide a good starting point for anyone wanting to use the 
/dev/bhio driver to run the beckhoff IO busses.

This is example code feel free to do whatever you want with it.

This code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

\bug While writing this example I found a nasty bug in the 
kernel driver code the call to the ioctl IOCTL_BHKLDRV_BOARDMAP
does not check the input and can be overflowed causing kernel
problems when you try to read. Easily fixed in the driver with 
a check of the input data.

*/

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "beckhoff.h"

using namespace beckhoff;

/**
\brief	moves the analog outputs and digital IO if a new analog input is added
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::shuffleAO(int nstart)
{
	std::vector<io_module>::iterator idx;
	for (idx = ao_modules_.begin() ; idx != ao_modules_.end(); idx++)
	{
		(*idx).position = nstart;
		nstart += (*idx).bit_width;
	}
	
	return shuffleDIO(nstart);
}

/**
\brief	moves the digital IO if the analog IO is changed
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::shuffleDIO(int nstart)
{
	int tmp_start = nstart;
	std::vector<io_module>::iterator idx;
	for (idx = di_modules_.begin() ; idx != di_modules_.end() ; idx++)
	{
		(*idx).position = tmp_start;
		tmp_start += (*idx).bit_width;
	}
	
	tmp_start = nstart;
	for (idx = do_modules_.begin() ; idx != do_modules_.end() ; idx++)
	{
		(*idx).position = tmp_start;
		tmp_start += (*idx).bit_width;
	}

	return nstart;
}

/**
\brief	add a module to the io map
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::add_module(module_type module)
{
	int nPos = 0;
	int nHandle = NOK;
	
	switch(module)
	{
	case KL1408:
		// create a module and give it an offset
		if (di_modules_.size())
			nPos = di_modules_.back().position + di_modules_.back().bit_width;
		di_modules_.push_back(io_module(module, 8, 8, nPos));
		// add one byte to the kb_input area
		kb_input_.push_back(0);
		nHandle = FLAG_KBUS + FLAG_DIGITAL + FLAG_INPUT + di_modules_.size() - 1;
		break;
	case KL2408:
		// create a module and give it an offset
		if (do_modules_.size())
			nPos = do_modules_.back().position + do_modules_.back().bit_width;
		do_modules_.push_back(io_module(module, 8, 8, nPos));
		// add one byte to the kb_output area
		kb_output_.push_back(0);
		nHandle = FLAG_KBUS + FLAG_DIGITAL + FLAG_OUTPUT +do_modules_.size() - 1;
		break;
	case KL3062:
		// create a module and give it an offset
		if (ai_modules_.size())
			nPos = ai_modules_.back().position + ai_modules_.back().bit_width;
		ai_modules_.push_back(io_module(module, 48, 2, nPos, 1, 1.0 / 3276.7, 0.0));
		// now we need to shuffle all of the IO
		nPos = ai_modules_.back().position + ai_modules_.back().bit_width;
		shuffleAO(nPos);
		// add three bytes per channel to the kb_input and kb_output area
		// as the analog modules use space in both the input and output
		// areas, probably for configuration
		for (int i = 0 ; i < 6 ; i++)
		{
			kb_input_.push_back(0);
			kb_output_.push_back(0);
		}
		nHandle = FLAG_KBUS + FLAG_ANALOG + FLAG_INPUT + ai_modules_.size() - 1;
		break;
	default:
		std::cerr << "error, add module, module type unknown\n";
		break;
	}
	return nHandle;
}

/**
\brief	writes the output block to the dpram
\author	Richard Lemon
\date	19/09/07
*/
int kbus_map::write(int fd)
{
	if (kb_output_.size())
	{
	/*	printf("kb out mem=");
		for (unsigned i = 0 ; i < kb_output_.size() ; i++)
			printf(":%u", kb_output_[i]);
		printf(":\n");
	*/
		if (lseek(fd, KBOP_OFFSET, SEEK_SET) != KBOP_OFFSET)
		{
			std::cerr << "error, could not seek kbus output area\n";
			return NOK;
		}
		if (::write(fd, &kb_output_[0], kb_output_.size()) != (int)kb_output_.size())
		{
			std::cerr << "error, could not write kbus output area\n";
			return NOK;
		}
	}

	return OK;
}
/**
\brief	reads the dpram into the input block
\author	Richard Lemon
\date	19/09/07
*/
int kbus_map::read(int fd)
{
	if (kb_input_.size())

	{
		if (lseek(fd, KBIP_OFFSET, SEEK_SET) != KBIP_OFFSET)
		{
			std::cerr << "error, could not seek kbus input area\n";
			return NOK;
		}
		if (::read(fd, &kb_input_[0], kb_input_.size()) != (int)kb_input_.size())
		{
			std::cerr << "error, could not read kbus input area\n";
			return NOK;
		}
	/*
		printf("kb in mem =");
		for (unsigned i = 0 ; i < kb_input_.size() ; i++)
			printf(":%u", kb_input_[i]);
		printf(":\n");
	*/
	}
	return OK;
}

/**
\brief	do the IO to the dpram
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::update(int fd)
{
	// first write the outputs
	if (write(fd) == NOK)
		return NOK;
	// now read the inputs
	if (read(fd) == NOK)
		return NOK;

	return OK;
}

/**
\brief	set the output value for the indexed analog channel
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::setAO(unsigned hdl, unsigned channel, double val)
{
	unsigned idx = hdl&0xFF;
	if (idx >= ao_modules_.size())
	{
		std::cerr << "error, setAO handle out of range\n";
		return NOK;
	}
	if (channel >= ao_modules_[idx].channels )
	{
		std::cerr << "error, setAO channel out of range\n";
		return NOK;
	}
	
	unsigned tmp = (ao_modules_[idx].position / 8) + (ao_modules_[idx].bit_width/ao_modules_[idx].channels)*channel + ao_modules_[idx].control_bytes;
	if (tmp >= kb_output_.size())
	{
		std::cerr << "error, setAO channel offset out of range\n";
		return NOK;
	}

	short* pVal = (short*)&kb_output_[tmp];

	if (ao_modules_[idx].scale == 0.0)
	{
		std::cerr << "warning, setAO scale factor 0.0\n";
		*pVal = 0;
		return OK;
	}

	*pVal = (short) ((val -  ao_modules_[idx].offset) / ao_modules_[idx].scale);

	return OK;
}

/**
\brief	get the current output value
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::getAO(unsigned hdl, unsigned channel, double& val)
{
	unsigned idx = hdl&0xFF;
	if (idx >= ao_modules_.size())
	{
		std::cerr << "error, getAO handle out of range\n";
		return NOK;
	}
	if (channel >= ao_modules_[idx].channels )
	{
		std::cerr << "error, getAO channel out of range\n";
		return NOK;
	}
	if (ao_modules_[idx].scale == 0.0)
	{
		std::cerr << "warning, getAO scale factor 0.0\n";
		val = 0.0;
		return OK;
	}
	
	unsigned tmp = (ao_modules_[idx].position / 8) + (ao_modules_[idx].bit_width/ao_modules_[idx].channels)*channel + ao_modules_[idx].control_bytes;
	if (tmp >= kb_output_.size())
	{
		std::cerr << "error, getAO channel offset out of range\n";
		return NOK;
	}
	
	short* pVal = (short*)&kb_output_[tmp];
	val = ((*pVal) * ao_modules_[idx].scale ) + ao_modules_[idx].offset;

	return OK;
}

/**
\brief	get the current input value
\author	Richard Lemon
\date	18/09/07
*/
int kbus_map::getAI(unsigned hdl, unsigned channel, double& val)
{
	unsigned idx = hdl&0xFF;
	if (idx >= ai_modules_.size())
	{
		std::cerr << "error, getAI handle out of range\n";
		return NOK;
	}
	if (channel >= ai_modules_[idx].channels )
	{
		std::cerr << "error, getAI channel out of range\n";
		return NOK;
	}

	unsigned tmp = (ai_modules_[idx].position  + (ai_modules_[idx].bit_width/ai_modules_[idx].channels)*channel) / 8 + ai_modules_[idx].control_bytes;
	if (tmp >= kb_input_.size())
	{
		std::cerr << "error, getAI channel offset out of range\n";
		return NOK;
	}

	short* pVal = (short*)&kb_input_[tmp];
	val = ((*pVal) * ai_modules_[idx].scale ) + ai_modules_[idx].offset;

	return OK;
}

/**
\brief	set the output value
\author	Richard Lemon
\date	18/09/07
\bug	this isn't correct if we are crossing a byte boundary... exercise for the reader
*/
int kbus_map::setDO(unsigned hdl, unsigned channel, bool val)
{
	unsigned idx = hdl&0xFF;
	if (idx >= do_modules_.size())
	{
		std::cerr << "error, setDO handle out of range\n";
		return NOK;
	}
	if (channel >= do_modules_[idx].channels )
	{
		std::cerr << "error, setDO channel out of range\n";
		return NOK;
	}

	if (val)
	{
//		printf("setting module[%u].bit[%u] = %i\n", idx, channel + do_modules_[idx].position % 8,(1 << channel + do_modules_[idx].position % 8));
		kb_output_[do_modules_[idx].position / 8] = kb_output_[do_modules_[idx].position / 8] | (1 << channel + do_modules_[idx].position % 8);
	}
	else
	{
//		printf("setting module[%u].bit[%u] = false\n", idx, channel + do_modules_[idx].position % 8);
		kb_output_[do_modules_[idx].position / 8] &= ((1 << channel + do_modules_[idx].position % 8) ^ 0xFF);
	}

/*	printf("setDO mem=");
	for (unsigned i = 0 ; i < kb_output_.size() ; i++)
		printf(":%u", kb_output_[i]);
	printf(":\n");
*/
	return OK;
}

/**
\brief	get the output value
\author	Richard Lemon
\date	18/09/07
\bug	this isn't correct if we are crossing a byte boundary... exercise for the reader
*/
int kbus_map::getDO(unsigned hdl, unsigned channel, bool& val)
{
	unsigned idx = hdl&0xFF;
	if (idx >= do_modules_.size())
	{
		std::cerr << "error, getDO handle out of range\n";
		return NOK;
	}
	if (channel >= do_modules_[idx].channels )
	{
		std::cerr << "error, getDO channel out of range\n";
		return NOK;
	}

	int nBit = 1;
	nBit <<= channel + do_modules_[idx].position % 8;
	val = (nBit & kb_output_[do_modules_[idx].position / 8]);
	return OK;

}

/**
\brief	get the input value
\author	Richard Lemon
\date	18/09/07
\bug	this isn't correct if we are crossing a byte boundary... exercise for the reader
*/
int kbus_map::getDI(unsigned hdl, unsigned channel, bool& val)
{
	unsigned idx = hdl&0xFF;

	if (idx >= di_modules_.size())
	{
		std::cerr << "error, getDI handle out of range\n";
//		printf("hdl[%u] idx[%u] out of range\n",hdl, idx);
		return NOK;
	}
	if (channel >= di_modules_[idx].channels )
	{
		std::cerr << "error, getDI channel out of range\n";
//		printf("channel too big\n");
		return NOK;
	}

//	printf("hdl[%u]:idx[%u]:ch[%i]:loc[%i]:mem[%i]\n",hdl, idx,channel,di_modules_[idx].position / 8, kb_input_[di_modules_[idx].position / 8]);
	int nBit = 1;
	nBit <<= channel + di_modules_[idx].position % 8; // we can have 1 bit outputs...
	val = (nBit & kb_input_[di_modules_[idx].position / 8]);
	return OK;
}

/**
\brief	ctr
\author	Richard Lemon
\date	18/09/07
*/
beckhoff_io::beckhoff_io()
{
	fd_ = NOK;
	board_map_.bUseKBus				= 0;
	board_map_.nKBusInputBytes		= 0;
	board_map_.nKBusOutputBytes		= 0;
	board_map_.bUseIPBus			= 0;
	board_map_.nIPBusInputBytes		= 0;
	board_map_.nIPBusOutputBytes	= 0;
}

/**
\brief	opens the beckhoff IO dpram device
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::open()
{
	// open the file
	fd_ = ::open("/dev/bhio", O_RDWR);
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio failed opened\n";
		return NOK;
	}
	
	// start the driver
	int ret = ioctl(fd_,IOCTL_BHKLDRV_START, NULL);
	if (ret)
	{
		std::cerr << "error, /dev/bhio failed start\n";
		::close(fd_);
		fd_ = NOK;
		return NOK;
	}

	// reset the data collection busses
	 ret = ioctl(fd_,IOCTL_BHKLDRV_RESET, NULL);
	if (ret)
	{
		std::cerr << "error, /dev/bhio failed reset\n";
		::close(fd_);
		fd_ = NOK;
		return NOK;
	}

	// need to update the board map...
	ioctl(fd_, IOCTL_BHKLDRV_BOARDMAP, &board_map_);

	// need to let the driver settle
	sleep(3);

	return OK;
}

/**
\brief	closes the beckhoff IO dpram device
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::close()
{
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio not opened\n";
		return NOK;
	}
	ioctl(fd_,IOCTL_BHKLDRV_STOP, NULL);
	::close(fd_);
	fd_ = NOK;
	
	return OK;
}

/**
\brief	starts the watchdog
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::start_wd()
{
	int nMsecs = 1000; // milliseconds to watchdog timeout
	
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio not opened\n";
		return NOK;
	}

	int ret = ioctl(fd_,IOCTL_BHKLDRV_WDSTART, nMsecs);
	if (ret)
	{
		std::cerr << "error, could not start watchdog\n";
		return NOK;
	}
	return OK;
}

/**
\brief	stops the watchdog
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::stop_wd()
{
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio not opened\n";
		return NOK;
	}
	
	int ret = ioctl(fd_,IOCTL_BHKLDRV_WDSTOP, NULL);
	if (ret)
	{
		std::cerr << "error, could not stop watchdog\n";
		return NOK;
	}
	return OK;
}

/**
\brief	pats the watchdog
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::pat_wd()
{
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio not opened\n";
		return NOK;
	}
	
	int ret = ioctl(fd_,IOCTL_BHKLDRV_WDPAT, NULL);
	if (ret)
	{
		std::cerr << "error, could not pat watchdog\n";
		return NOK;
	}
	return OK;
	
}

/**
\brief	add a module to the io map
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::add_module(module_type module)
{
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio not opened\n";
		return NOK;
	}

	int ret = NOK;
	switch(module)
	{
	case KL1408:
		ret = kbus_.add_module(module);
		if (ret == NOK)
			return NOK;
		board_map_.bUseKBus = 1;
		board_map_.nKBusInputBytes += 1;
		break;
	case KL2408:
		ret = kbus_.add_module(module);
		if (ret == NOK)
			return NOK;
		board_map_.bUseKBus = 1;
		board_map_.nKBusOutputBytes += 1;
		break;
	case KL3062:
		ret = kbus_.add_module(module);
		if (ret == NOK)
			return NOK;
		// need to always remember that the analog units take
		// space in both the input and output areas as they
		// are configurable.
		board_map_.bUseKBus = 1;
		board_map_.nKBusInputBytes += 6;
		board_map_.nKBusOutputBytes += 6;
		break;
	}

	// need to update the board map...
	ioctl(fd_, IOCTL_BHKLDRV_BOARDMAP, &board_map_);
	return ret;
}

/**
\brief	sync the IO map with the dpram
\author	Richard Lemon
\date	18/09/07
*/
int beckhoff_io::update()
{
	if (fd_ < 0)
	{
		std::cerr << "error, /dev/bhio not opened\n";
		return NOK;
	}
	// if we have any kbus modules update the kbus
	kbus_.update(fd_);
	// if we have any ipbus modules update the ipbus
//	ipbus.update(fd_);
	
	return OK;
}

/**
\brief	
\author	Richard Lemon
\date	19/09/07
*/
int beckhoff_io::setAO(int hdl, int idx, double val)
{
	if (!(FLAG_OUTPUT&hdl) || !(FLAG_ANALOG&hdl))
	{
		std::cerr << "not output or not digital\n";
		return NOK;
	}
	if (FLAG_KBUS&hdl)
	{
//		printf("kbus-setAO()\n");
		return kbus_.setAO(hdl, idx, val);
	}
	else if (FLAG_IPBUS&hdl)
	{
//		return ipbus_.setDO(hdl, idx, val);		
	}
	std::cerr << "setAO() bus type not found\n";
	return NOK;
}

/**
\brief	
\author	Richard Lemon
\date	19/09/07
*/
int beckhoff_io::getAO(int hdl, int idx, double& val)
{
	if (!(FLAG_OUTPUT&hdl) || !(FLAG_ANALOG&hdl))
	{
		std::cerr << "not output or not analog\n";
		return NOK;
	}
	if (FLAG_KBUS&hdl)
	{
//		printf("kbus-getAO()\n");
		return kbus_.getAO(hdl, idx, val);
	}
	else if (FLAG_IPBUS&hdl)
	{
//		return ipbus_.setDO(hdl, idx, val);		
	}
	std::cerr << "getAO() bus type not found\n";
	return NOK;
}

/**
\brief	
\author	Richard Lemon
\date	19/09/07
*/
int beckhoff_io::getAI(int hdl, int idx, double& val)
{
	if (!(FLAG_INPUT&hdl) || !(FLAG_ANALOG&hdl))
	{
		std::cerr << "not input or not analog\n";
		return NOK;
	}
	if (FLAG_KBUS&hdl)
	{
//		printf("kbus-getAI()\n");
		return kbus_.getAI(hdl, idx, val);
	}
	else if (FLAG_IPBUS&hdl)
	{
//		return ipbus_.getAI(hdl, idx, val);		
	}
	std::cerr << "getAI() bus type not found\n";
	return NOK;
}

/**
\brief	
\author	Richard Lemon
\date	19/09/07
*/
int beckhoff_io::setDO(int hdl, int idx, bool val)
{
	if (!(FLAG_OUTPUT&hdl) || !(FLAG_DIGITAL&hdl))
	{
		std::cerr << "not output or not digital\n";
		return NOK;
	}
	
	if (FLAG_KBUS&hdl)
	{
//		printf("kbus-setDO()\n");
		return kbus_.setDO(hdl, idx, val);
	}
	else if (FLAG_IPBUS&hdl)
	{
//		return ipbus_.setDO(hdl, idx, val);		
	}
	std::cerr << "setDO() bus type not found\n";
	return NOK;
}

/**
\brief	
\author	Richard Lemon
\date	19/09/07
*/
int beckhoff_io::getDO(int hdl, int idx, bool& val)
{
	if (!(FLAG_OUTPUT&hdl) || !(FLAG_DIGITAL&hdl))
	{
		std::cerr << "not output or not digital\n";
		return NOK;
	}
	
	if (FLAG_KBUS&hdl)
	{
//		printf("kbus-getDO()\n");
		return kbus_.getDO(hdl, idx, val);
	}
	else if (FLAG_IPBUS&hdl)
	{
//		return ipbus_.getDO(hdl, idx, val);		
	}
	std::cerr << "getDO() bus type not found\n";
	return NOK;
}

/**
\brief	
\author	Richard Lemon
\date	19/09/07
*/
int beckhoff_io::getDI(int hdl, int idx, bool& val)
{
	if (!(FLAG_INPUT&hdl) || !(FLAG_DIGITAL&hdl))
	{
		std::cerr << "not input or not digital\n";
		return NOK;
	}

	if (FLAG_KBUS&hdl)
	{
//		printf("kbus-getDI\n");
		return kbus_.getDI(hdl, idx, val);
	}
	else if (FLAG_IPBUS&hdl)
	{
//		return ipbus_.getDI(hdl, idx, val);		
	}
	std::cerr << "getDI() bus type not found\n";
	return NOK;
}

/**
\brief	
\author	Richard Lemon
\date	18/09/07
*/
int main(void)
{
	beckhoff_io myIO;
	myIO.open();
	int DIN08  = myIO.add_module(KL1408);
	int DOUT08 = myIO.add_module(KL2408);
	int DOUT18 = myIO.add_module(KL2408);
	int AIN02  = myIO.add_module(KL3062);

	do
	{
		myIO.update();
		bool val = false;
		for (int i = 0 ; i < 8; i++)
		{
			if (myIO.getDI(DIN08,i, val) != NOK)
				printf("1408[%i,%i] = %i -- ",DIN08, i ,val);
			if (myIO.setDO(DOUT08, i, val) != NOK)
				printf("2408[%i,%i] = %i -- ", DOUT08, i, val);
			if (myIO.setDO(DOUT18, i, !val) != NOK)
				printf("2408[%i,%i] = %i", DOUT18, i, !val);
			printf("\n");
		}
	
		double dval = 0.0;
		if (myIO.getAI(AIN02, 0, dval) != NOK)
			printf("3062[%i,%i] = %fV\n", AIN02, 0, dval);
		if (myIO.getAI(AIN02, 1, dval) != NOK)
			printf("3062[%i,%i] = %fV\n", AIN02, 1, dval);
			
		puts("0 to exit");
	}
	while(getchar() != '0');

	myIO.close();	
	return EXIT_SUCCESS;
}
