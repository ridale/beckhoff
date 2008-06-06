#ifndef BECKHOFF_H_
#define BECKHOFF_H_
#include <vector>
#include "driver/bhkldrv.h"

namespace beckhoff {

	const int OK  = 0; //! return OK
	const int NOK = -1; //! return Not OK

	// the following flags are used in the
	// handle returned when we add a module
	const int FLAG_KBUS		= 0x1000;
	const int FLAG_IPBUS	= 0x2000;
	const int FLAG_DIGITAL	= 0x4000;
	const int FLAG_ANALOG	= 0x8000;
	const int FLAG_INPUT	= 0x0100;
	const int FLAG_OUTPUT	= 0x0200;
	
	enum module_type {KL1408,KL2408,KL3062};
	
	struct io_module{
		io_module(module_type m, int b, int c, int p, int cb = 0,
					float s = 1.0, float o = 0.0) :
						module(m), bit_width(b), channels(c), position(p), 
							control_bytes(cb), scale(s), offset(o){};
		module_type module;	// enum type of module
		int bit_width;		// size in the memory map
		unsigned channels;	// number of channels of data
		int position;		// position in the memory map
		int control_bytes;	// number of control bytes
		float scale;		// scale factor for data
		float offset;		// zero offset for data
	};

	/**
	 * \class kbus_map map of the kbus IO region and IO modules
	 */ 
	class kbus_map{
		std::vector <io_module> ai_modules_;
		std::vector <io_module> ao_modules_;
		std::vector <io_module> di_modules_;
		std::vector <io_module> do_modules_;
		std::vector <unsigned char> kb_input_;
		std::vector <unsigned char> kb_output_;
		
		int shuffleAO(int nstart);
		int shuffleDIO(int nstart);

	public:
		kbus_map() {};
		//! adds a module to the IO map
		int add_module(module_type module);

		int setAO(unsigned idx, unsigned channel, double val); 	//< set the output value for the indexed analog channel
		int getAO(unsigned idx, unsigned channel, double& val); //< get the current output value

		int getAI(unsigned idx, unsigned channel, double& val); //< get the current input value

		int setDO(unsigned idx, unsigned channel, bool val); 	//< set the output value
		int getDO(unsigned idx, unsigned channel, bool& val); 	//< get the output value

		int getDI(unsigned idx, unsigned channel, bool& val);	//< get the input value

		//! sync the IO map with the dpram
		int update(int fd);
		int write(int fd);
		int read(int fd);
	};

	/**
	 * \class ipbus_map map of the ipbus IO region and IO modules
	 */ 
	class ipbus_map{
		std::vector <io_module> ai_modules_;
		std::vector <io_module> ao_modules_;
		std::vector <io_module> di_modules_;
		std::vector <io_module> do_modules_;
		std::vector <unsigned char> ip_input_;
		std::vector <unsigned char> ip_output_;
		
	public:
		ipbus_map(){};
		//! adds a module to the IO map
		int add_module(module_type module);

		int setAO(unsigned idx, unsigned channel, double val); 	//< set the output value for the indexed analog channel
		int getAO(unsigned idx, unsigned channel, double& val); //< get the current output value

		int getAI(unsigned idx, unsigned channel, double& val); //< get the current input value

		int setDO(unsigned idx, unsigned channel, bool val); 	//< set the output value
		int getDO(unsigned idx, unsigned channel, bool& val); 	//< get the output value

		int getDI(unsigned idx, unsigned channel, bool& val);	//< get the input value

		//! sync the IO map with the dpram
		int update(int fd);
	};
	
	/**
	 * \class beckhoff_io wrapper for access to the IO
	 */ 
	class beckhoff_io {
		int fd_;
		BH_IO_BOARD_MAP board_map_;
		kbus_map kbus_;
		ipbus_map ipbus_;
	public:
		beckhoff_io();
		
		//! open the driver (dpram)
		int open();
		//! close the driver (dpram)
		int close();
		
		//! start the watchdog
		int start_wd();
		//! stop the watchdog
		int stop_wd();
		//! pat the watchdog
		int pat_wd();
		
		//! add a module to the io map
		int add_module(module_type module);
		//! removes a module from the configuration
		int remove_module(int hdl);
		
		//! sync the IO map with the dpram
		int update();

		int setAO(int hdl, int idx, double val);
		int getAO(int hdl, int idx, double& val);

		int getAI(int hdl, int idx, double& val);

		int setDO(int hdl, int idx, bool val);
		int getDO(int hdl, int idx, bool& val);

		int getDI(int hdl, int idx, bool& val);

	};
};
#endif /*BECKHOFF_H_*/
