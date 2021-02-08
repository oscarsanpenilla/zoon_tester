#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <mutex>
//#include "serial_port.h"
#include "serial_com.h"
//#include <curses.h>

#include <sys/time.h>
#include <unistd.h>
#include <signal.h>



using namespace std;

void get_cur_time(timeval &cur_time)
{
	gettimeofday(&cur_time, 0);
}

float get_elapsed_time(const timeval &start_time)
{
	timeval cur_time;
	get_cur_time(cur_time);
	
	long nSec = cur_time.tv_sec - start_time.tv_sec;
	long nUSec = cur_time.tv_usec - start_time.tv_usec;
	return ((((nSec * 1000.f) + nUSec / 1000.f) + 0.5f) / 1000.f);
}

//SerialPort serial;//("/dev/ttyS0",115200);
SerialCom serial;//("/dev/ttyS0",115200);

void
signal_callback_handler(int signum)
{
	serial.Close();
    exit(signum);
}

std::mutex  mtx;
int main(int argc, char** argv) {
	string dev = "/dev/ttyACM0";
	bool isPrint = false;
	int baud_rate = 115200;
	float send_rate = 1000.f;
	bool send_only = false;
	bool receive_only = false;
	int data_size = 100;

	const char* param = NULL;
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {
			param = argv[i];
			if (strcmp(param, "-d") == 0) {
				if (i + 1 < argc && argv[i + 1][0] != '-') {
					dev = argv[i + 1];
					i++;
					printf ("Parameter : %s %s (serial device name)\n", param, dev.c_str());
				}					
				else {
					printf ("Parameter : %s, missing seiral device name !!\n", param);
				}
			}else if (strcmp(param, "-print") == 0) {
                isPrint = true;
            }
			else if (strcmp(param, "-b") == 0) {
				if (i + 1 < argc && argv[i + 1][0] != '-') {
					baud_rate = atoi(argv[i + 1]);
					i++;

					printf ("Parameter : %s %d (serial baud rate)\n", param, baud_rate);
				}
				else {
					printf ("Parameter : %s, missing serial baud rate !!\n", param);
				}
			}
			else if (strcmp(param, "-s") == 0) {
				if (i + 1 < argc && argv[i + 1][0] != '-') {
					data_size = atoi(argv[i + 1]);
					i++;

					printf ("Parameter : %s %d (data size)\n", param, data_size);
				}
				else {
					printf ("Parameter : %s, missing send rate !!\n", param);
				}
			}
			else if (strcmp(param, "-r") == 0) {
				if (i + 1 < argc && argv[i + 1][0] != '-') {
					send_rate = atof(argv[i + 1]);
					i++;

					printf ("Parameter : %s %.1f (send rate)\n", param, send_rate);
				}
				else {
					printf ("Parameter : %s, missing send rate !!\n", param);
				}
			}
			else if (strcmp(param, "-send") == 0 && !receive_only) {
				send_only = true;
				printf ("Parameter : %s, enable sending only\n", param);
			}
			else if (strcmp(param, "-receive") == 0 && !send_only) {
				receive_only = true;
				printf ("Parameter : %s, enable receiving only\n", param);
			}
			else {
				printf ("Parameter : %s (invalid)\n\n", param);
				printf("Usage: TestZoon [param list]\n");
				printf("param:\n");
				printf("  -d		: serial device name (/dev/ttyACM0)\n");
				printf("  -b		: serial baud rate (115200)\n");
				printf("  -r		: sending rate (1000 Bps)\n");
				printf("  -send		: enalbe sending only (false)\n");
				printf("  -receive	: enable receving only (false)\n");
                printf("  -print	: prints data to out\n");
				return 0;
			}
		}
	}	
	
    signal(SIGINT, signal_callback_handler);

	serial.Open(dev.c_str(), baud_rate);
 
	if (!send_only) {
		// thread for receiving data
		std::thread([isPrint] {
			printf("==== start receving thread ====\n");
			
			int total_received_data_count = 0;
			int total_missing_data_count = 0;
			uint64_t all_received_data_count = 0,all_missing_data_count = 0;
			uint8_t buf[1024] = {0};
			bool first_data = true;
			
			timeval start_time,iter_time;
			int prev_data = -1;
			while(true) {
				int received_data_count = serial.Read((uint8_t *)buf, 1024);
				if (received_data_count > 0) {
	//                std::lock_guard<std::mutex> lock(mtx);
					// verify data
					int missing_data_count = 0;
                    int prev_missing_data_count = 0;
					for (int i = 0; i < received_data_count; ++i) {
						int cur_data = buf[i];
						bool end_of_message = (cur_data >= 100);
						if (end_of_message) cur_data -= 100;
						
						if (first_data) {
							first_data = false;
							get_cur_time(start_time);
                            get_cur_time(iter_time);
                            prev_data = end_of_message ? -1 : cur_data;
                            continue;
						}
						else{
							auto tcur_data = cur_data < prev_data ? cur_data + 100 : cur_data;
							missing_data_count += (tcur_data - prev_data - 1);
							if(missing_data_count != prev_missing_data_count) {
								prev_missing_data_count = missing_data_count;
								printf("\033[0;33m");
								printf("* missing : %d (cur : %d, prev : %d)\n", missing_data_count, cur_data,
									   prev_data);
								printf("\033[0m");
							}
							
							prev_data = end_of_message ? -1 : cur_data;
                        }
						if(isPrint) {
							if (end_of_message) {
								printf("%d*", cur_data);
							}
							else {
								printf("%d,",cur_data);
							}
                        }
					}
                    if(isPrint) {
                        printf("\n");
                    }
//                    if(missing_data_count) {
//                        printf("* missing : %d received  %d \n", missing_data_count, received_data_count);
//                    }
					
					total_missing_data_count += missing_data_count;
					total_received_data_count += received_data_count + missing_data_count;
					
					float elapted_time = get_elapsed_time(start_time);
					float transfer_rate = total_received_data_count / get_elapsed_time(iter_time);

					if(get_elapsed_time(iter_time) >= 1){
                        all_missing_data_count += (uint64_t) total_missing_data_count;
                        all_received_data_count += (uint64_t) total_received_data_count;
                        printf("\033[0;31m");
                        printf("[%.03f] Received %d data. receive rate : %.0f Bps, missing : %d, total missing rate : %.1f %% (%d / %d)\n",
                               elapted_time, received_data_count, transfer_rate, missing_data_count,
                               100.f * total_missing_data_count / total_received_data_count, total_missing_data_count, total_received_data_count);
                        printf("\033[1;31m");
                        printf("[%.03f] Average missing rate : %.1f \n",
                               elapted_time,
                               100.f * all_missing_data_count / all_received_data_count);
                        printf("\033[0m");
                        fflush(stdout);

                        total_missing_data_count = 0;
                        total_received_data_count = 0;
                        get_cur_time(iter_time);
					}
				}
				else {
					usleep(1000); // sleep for 1 ms
				}
			}

		}).detach();
	}

	// main thread for sending data
	if (!receive_only) {
		printf("==== start sending thread ====\n");
		
		// initialize data with 0 to 99  
		int data_count = data_size;
		uint8_t *buf = new uint8_t[data_count];
		for (int i = 0; i < data_count; i++) {
			buf[i] = i % 100;
		}
		buf[data_count - 1] += 100;		// add 100 to last character

		uint64_t send_delay = ((uint64_t)1e6) / (send_rate / data_count);
		
		int total_sent_data_count = 0;
		
		timeval start_time;
		get_cur_time(start_time);

		printf("send delay: %ld us\n",send_delay);
		printf("send data size: %d\n",data_count);
        fflush(stdout);
		while(true) {
			usleep(send_delay);
            //clock_nanosleep(CLOCK_REALTIME, 0, {tv_sec=10, tv_nsec=send_delay}, NULL);

			int cur_sent_data_count = 0;
			while(cur_sent_data_count < data_count) {
				int sent_data_count = serial.Send((uint8_t *)(buf + cur_sent_data_count), data_count - cur_sent_data_count);
				if (sent_data_count > 0) {
					cur_sent_data_count += sent_data_count;
					total_sent_data_count += sent_data_count;
							
					float elapted_time = get_elapsed_time(start_time);
					float transfer_rate = total_sent_data_count / elapted_time;		
					
					printf("[%.03f] * Sent %d data. send rate : %.0f Bps, total sent data : %d\n", elapted_time, cur_sent_data_count, transfer_rate, total_sent_data_count);					
					fflush(stdout);
				}
				else {
					usleep(1000); // sleep for 1 ms
				}
			}			
		}
		
		delete [] buf;
	}
	else {
		while(true) {
			usleep(1e9);
		}
	}
	
    return 0;
}
