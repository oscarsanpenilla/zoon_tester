#ifndef __SERIAL_COM
#define __SERIAL_COM

#define SERIAL_COM_WAIT_TIME 10

#include <stdio.h>
#include <vector>
#include <list>
using namespace std;

#ifdef _WINDOWS
#include <windows.h>
#include <stdlib.h>
#endif

#ifdef __linux__
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string.h>
#endif

//#include "serial_port.h"
#include <thread>
#include <mutex>
#include <condition_variable>

#include "timestamp.h"
using namespace aerialx;

#ifndef DEF_DATA_TYPE
#define DEF_DATA_TYPE
using byte = unsigned char;
using word = unsigned short;
using dword = unsigned int;
using bytearray = vector<byte>;
#endif

class read;
class SerialCom
{
private:
	bool connected;
#ifdef _WINDOWS
	HANDLE handle;
	COMSTAT status;
#endif
#ifdef __linux__
	int handle;
#endif

//	SerialPort serial_port_;
	
	mutex serial_io_mutex_;

	list<vector<byte>> read_buffer_;
	mutex read_buffer_mutex_;
	condition_variable read_buffer_cv_;	
	
//	list<vector<byte>> write_buffer_;
//	mutex write_buffer_mutex_;

public:
	SerialCom()
	{
		connected = false;
		handle = 0;
	}

	~SerialCom()
	{
		Close();
	}

	bool Open(const char *portName, int baud)
	{
		connected = false;
		
/*		try {
			serial_port_.connect(portName, baud);
		}
		catch (...) {
			return false;
		}
		
		thread(&SerialCom::ProcessIOThread, this).detach();		
		
		return true;
*/		
		
#ifdef __linux__
/*		speed_t speed;
		switch(baud) {	// from termbits.h
		case 57600 : speed = B57600; break;
		case 115200 : speed = B115200; break;
		case 230400 : speed = B230400; break;
		case 460800 : speed = B460800; break;
		case 500000 : speed = B500000; break;
		case 576000 : speed = B576000; break;
		case 921600 : speed = B921600; break;
		case 1000000 : speed = B1000000; break;
		case 1152000 : speed = B1152000; break;
		case 1500000 : speed = B1500000; break;
		case 2000000 : speed = B2000000; break;
		case 2500000 : speed = B2500000; break;
		case 3000000 : speed = B3000000; break;
		case 3500000 : speed = B3500000; break;
		case 4000000 : speed = B4000000; break;
		default :
			printf("\nerror : not support baud rate (%d)", baud);
			return false;
		}

		handle = open(portName, O_RDWR | O_ASYNC | O_NDELAY);
		if (handle < 0) {
			printf("\nerror : %s\n", strerror(errno));
			return false;
		}		
		
		fcntl(handle, F_SETFL, O_ASYNC | O_NDELAY);	
		
		struct termios tty;
		memset(&tty, 0, sizeof tty);

		// Read in existing settings, and handle any error
		if(tcgetattr(handle, &tty) != 0) {
			printf("\nerror : %s\n", strerror(errno));
			return false;
		}
		
		// Set Baud Rate 
		
		cfsetospeed (&tty, (speed_t)speed);
		cfsetispeed (&tty, (speed_t)speed);

		// Setting other Port Stuff 
		tty.c_cflag     &=  ~PARENB;            // Make 8n1
		tty.c_cflag     &=  ~CSTOPB;
		tty.c_cflag     &=  ~CSIZE;
		tty.c_cflag     |=  CS8;

		tty.c_cflag     &=  ~CRTSCTS;           // no flow control
		tty.c_cc[VMIN]   =  1;                  // read doesn't block
		tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
		tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

		// Make raw 
		cfmakeraw(&tty);

		// Flush Port, then applies attributes 
		tcflush( handle, TCIFLUSH );
		if (tcsetattr (handle, TCSANOW, &tty) != 0) {
			printf("\nerror : %s\n", strerror(errno));
			return false;
		}	
		
//		int flags = fcntl(handle, F_GETFL, 0);
//		fcntl(handle, F_SETFL, flags | );
*/
		// Open serial port
		// O_RDWR - Read and write
		// O_NOCTTY - Ignore special chars like CTRL-C
		handle = open(portName, O_RDWR | O_ASYNC | O_NDELAY);

		// Check for Errors
		if (handle == -1)
		{
			/* Could not open the port. */
			return false;
		}

		// Finalize
	//	else
	//	{
	//	fcntl(handle, F_SETFL, O_ASYNC | O_NDELAY);
	//	}


		int data_bits = 8;
		int stop_bits = 1;
		bool parity = false;
		bool hardware_control = false;

		// Check file descriptor
		if(!isatty(handle))
		{
			fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", handle);
			return false;
		}

		// Read file descritor configuration
		struct termios  config;
		if(tcgetattr(handle, &config) < 0)
		{
			fprintf(stderr, "\nERROR: could not read configuration of handle %d\n", handle);
			return false;
		}

		// Input flags - Turn off input processing
		// convert break to null byte, no CR to NL translation,
		// no NL to CR translation, don't mark parity errors or breaks
		// no input parity check, don't strip high bit off,
		// no XON/XOFF software flow control
		config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
							INLCR | PARMRK | INPCK | ISTRIP | IXON);

		// Output flags - Turn off output processing
		// no CR to NL translation, no NL to CR-NL translation,
		// no NL to CR translation, no column 0 CR suppression,
		// no Ctrl-D suppression, no fill characters, no case mapping,
		// no local output processing
		config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
							ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

		// No line processing:
		// echo off, echo newline off, canonical mode off,
		// extended input processing off, signal chars off
		config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

		// Turn off character processing
		// clear current char size mask, no parity checking,
		// no output processing, force 8 bit input
		config.c_cflag &= ~(CSIZE | PARENB);
		config.c_cflag |= CS8;

		// One input byte is enough to return from read()
		// Inter-character timer off
		config.c_cc[VMIN]  = 1;
		config.c_cc[VTIME] = 10; // was 0

		// Get the current options for the port
		////struct termios options;
		////tcgetattr(handle, &options);

		// Apply baudrate
		switch (baud)
		{
			case 1200:
				if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 1800:
				cfsetispeed(&config, B1800);
				cfsetospeed(&config, B1800);
				break;
			case 9600:
				cfsetispeed(&config, B9600);
				cfsetospeed(&config, B9600);
				break;
			case 19200:
				cfsetispeed(&config, B19200);
				cfsetospeed(&config, B19200);
				break;
			case 38400:
				if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 57600:
				if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 115200:
				if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;

				// These two non-standard (by the 70'ties ) rates are fully supported on
				// current Debian and Mac OS versions (tested since 2010).
			case 460800:
				if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 921600:
				if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			default:
				fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
				return false;

				break;
		}

		// Finally, apply the configuration
		if(tcsetattr(handle, TCSAFLUSH, &config) < 0)
		{
			fprintf(stderr, "\nERROR: could not set configuration of handle %d\n", handle);
			return false;
		}


		
		connected = true;

		thread(&SerialCom::ProcessIOThread, this).detach();

		return true;
		
#endif		

#ifdef _WINDOWS
		DWORD nBaudRate;		
		switch(baud) {	// from WinBase.h
		case 110 : nBaudRate = CBR_110; break;
		case 300 : nBaudRate = CBR_300; break;
		case 600 : nBaudRate = CBR_600; break;
		case 1200 : nBaudRate = CBR_1200; break;
		case 2400 : nBaudRate = CBR_2400; break;
		case 4800 : nBaudRate = CBR_4800; break;
		case 9600 : nBaudRate = CBR_9600; break;
		case 14400 : nBaudRate = CBR_14400; break;
		case 19200 : nBaudRate = CBR_19200; break;
		case 38400 : nBaudRate = CBR_38400; break;
		case 56000 : nBaudRate = CBR_56000; break;
		case 115200 : nBaudRate = CBR_115200; break;
		case 128000 : nBaudRate = CBR_128000; break;
		case 256000 : nBaudRate = CBR_256000; break;
		default :
			printf("\nerror : not support baud rate (%d)\n", baud);
			return false;
		}

		char szDev[256];
		sprintf_s(szDev, "\\\\.\\%s", portName);
		handle = CreateFileA(static_cast<LPCSTR>(szDev),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);
		if (handle == INVALID_HANDLE_VALUE) {
			printf("\nerror : cannot open seriel device (%s)\n", szDev);
			return false;
		}

		DCB dcbSerialParameters = { 0 };

		if (!GetCommState(handle, &dcbSerialParameters)) {
			printf("\nerror : cannot get seriel device states\n");
			return false;
		}

		dcbSerialParameters.BaudRate = nBaudRate;
		dcbSerialParameters.ByteSize = 8;
		dcbSerialParameters.StopBits = ONESTOPBIT;
		dcbSerialParameters.Parity = NOPARITY;
		dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;
		dcbSerialParameters.fRtsControl = RTS_CONTROL_ENABLE;

		if (!SetCommState(handle, &dcbSerialParameters)) {
			printf("\nerror : cannot set seriel device states\n");
			return false;
		}

		COMMTIMEOUTS commTimeout;

		if (!GetCommTimeouts(handle, &commTimeout)) {
			printf("\nerror : cannot get seriel device time out\n");
			return false;
		}
		commTimeout.ReadIntervalTimeout = MAXDWORD;
		commTimeout.ReadTotalTimeoutConstant = 0;
		commTimeout.ReadTotalTimeoutMultiplier = 0;
		commTimeout.WriteTotalTimeoutConstant = 0;
		commTimeout.WriteTotalTimeoutMultiplier = 0;
		if (!SetCommTimeouts(handle, &commTimeout)) {
			printf("\nerror : cannot set seriel device time out\n");
			return false;
		}

		connected = true;
		PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
		Sleep(SERIAL_COM_WAIT_TIME);
		
		thread(&SerialCom::ProcessIOThread, this).detach();		

		return true;
#endif
	}
	void Close()
	{
		if (connected) {
			connected = false;
#ifdef __linux__
			close(handle);
#endif
#ifdef _WINDOWS
			CloseHandle(handle);
#endif
			handle = 0;
		}
	}
	
	void AddReadBuffer(unsigned char *buffer, unsigned int buf_size)
	{
		if (buf_size <= 0) {
			return;
		}
		
		bytearray new_data(buf_size);
		memcpy(new_data.data(), buffer, buf_size);
		
		lock_guard<mutex> lock(read_buffer_mutex_);
		read_buffer_.push_back(move(new_data));
		read_buffer_cv_.notify_one();			
	}
	
	int ReadUntil(byte nChar)
	{
		int read_count = 0;
		unsigned char c;
		while (true) {
			if (Read(&c, 1) > 0) {
				read_count++;
				if (c == nChar) {
					break;
				}
			}
			else {
				fsleep(0.001f);
			}
		}
		
		return read_count;
	}
	
	void Read(byte &byte_)
	{
		bytearray buffer(1);
		Read(buffer);
	
		memcpy(&byte_, buffer.data(), buffer.size());
	}

	void Read(word &word_)
	{
		bytearray buffer(2);
		Read(buffer);
		memcpy(&word_, buffer.data(), buffer.size());
	}
	
	void Read(bytearray &buffer)
	{
		unsigned int buf_size = buffer.size();
		unsigned int total_read_count = 0;
		while (total_read_count < buf_size) {
			int ret = Read(&(buffer[total_read_count]), buf_size - total_read_count);
			if (ret <= 0) {
				fsleep(0.001f);
				continue;
			}			
			
			total_read_count += ret;
		}
	}
	
	int Read(byte *buffer, int buf_size)
	{	
		unique_lock<mutex> lock(read_buffer_mutex_);
		read_buffer_cv_.wait(lock, [this] { return (!read_buffer_.empty()); });		
		
		static int read_buffer_index = 0;
		int index = 0;
		int read_size = 0;
		while(!read_buffer_.empty() && buf_size > 0) {
			auto &data = read_buffer_.front();
			int data_size = data.size();
			int size = min(data_size - read_buffer_index, buf_size);
			
			memcpy(buffer + index, data.data() + read_buffer_index, size);
			buf_size -= size;
			read_size += size;
			index += size;
			read_buffer_index += size;
			
			if (read_buffer_index >= data_size) {
				read_buffer_.pop_front();
				read_buffer_index = 0;
			}
		}
		
		return read_size;
	}
	
	void ProcessIOThread()
	{
		constexpr int buf_size = 4096;
		byte buffer[buf_size];
		
		while (true) {
			int received_size = Receive(buffer, buf_size);
			if (received_size > 0) {
				AddReadBuffer(buffer, received_size);
			}
			else {
				fsleep(0.001f);
			}
/*			
			if(!write_buffer_.empty()) {
				auto &data = write_buffer_.front();
				Send(data);

				lock_guard<mutex> lock(write_buffer_mutex_);
				write_buffer_.pop_front();
			}
*/ 
		}
	} 

	int Receive(byte *buffer, int buf_size)
	{
		lock_guard<mutex> lock(serial_io_mutex_);		
//		return serial_port_.receive(buffer, buf_size);
		
#ifdef __linux__
		return read(handle, buffer, buf_size);
#endif		
#ifdef _WINDOWS
		DWORD bytesRead = 0;
		unsigned int toRead = buf_size;
		DWORD errors;

/*		ClearCommError(handle, &errors, &status);
		if (status.cbInQue > 0) {
			toRead = min(status.cbInQue, buf_size);
		}
*/
//		printf("\nSerial Read Start : %d\n", buf_size);
		if (ReadFile(handle, buffer, toRead, &bytesRead, NULL) == 0) {
			ClearCommError(handle, &errors, &status);
//			printf("\nSerial Read End : error(%x)\n", errors);
			return -1;
		}
		
//		printf("\nSerial Read End : %d\n", bytesRead);
		return bytesRead;
#endif
	}

/*	void Write(const bytearray &buffer)
	{
		lock_guard<mutex> lock(write_buffer_mutex_);
		write_buffer_.push_back(buffer);
	}
*/
	int Send(const bytearray &buffer)
	{
		unsigned int buf_size = buffer.size();
		unsigned int total_written_count = 0;
		while (total_written_count < buf_size) {
			int ret = Send(&(buffer[total_written_count]), buf_size - total_written_count);
			if (ret <= 0) {
				fsleep(0.001f);
				continue;
			}			
			
			total_written_count += ret;
		}
		
		return total_written_count;
	}
	
	int Send(const byte *buffer, int buf_size)
	{
		lock_guard<mutex> lock(serial_io_mutex_);
//		return serial_port_.send((uint8_t *)buffer, buf_size);
		
#ifdef __linux__
//		printf("\nSerial Write Start : %d\n", buf_size);
		int write_count = write(handle, buffer, buf_size);
		tcdrain(handle);
//		printf("\nSerial Write End : %d\n", write_count);
		return write_count;
#endif		
#ifdef _WINDOWS
		DWORD bytesSent = 0;
		DWORD errors;

//		printf("\nSerial Write Start : %d\n", buf_size);
		if (WriteFile(handle, (void*)buffer, buf_size, &bytesSent, 0) == 0) {
			ClearCommError(handle, &errors, &status);
//			printf("\nSerial Write End : error(%x)\n", errors);
			return -1;
		}
		
//		printf("\nSerial Write End : %d\n", bytesSent);
		return bytesSent;
#endif		

		return -1;
	}
	
	bool isConnected()
	{
		return connected;
	}
};

#endif //__SERIAL_COM
