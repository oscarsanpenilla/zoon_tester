#pragma once

#include <string>
#include <chrono>
#include <ctime>
#ifdef __linux__
#include <unistd.h>
#endif

using namespace std;
using namespace std::chrono;

namespace aerialx {
#ifndef _FSLEEP_SEC
#define _FSLEEP_SEC
#ifdef __linux__
inline void fsleep(float sec)
{
	usleep(sec * 1e6);
}
#endif
#ifdef _WINDOWS
#define _FSLEEP_SEC
inline void fsleep(float sec)
{
//	clock_t goal = sec * 1e3 + clock();
//	while (goal > clock());
/*	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * (__int64)(sec*1e6));

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);	
*/
	Sleep(sec * 1000);
}
#endif
#endif
	template <class Duration>
	using sys_time = time_point<system_clock, Duration>;
	using sys_nanoseconds = sys_time<nanoseconds>;

	class TimeStamp {
	private:
		int year_;
		int month_;
		int day_;
		int hour_;
		int min_;
		int sec_;
		int millisec_;
		int microsec_;
		int nanosec_;

		int64_t duration_nanosec_;
		char format_[32];

	public:
		TimeStamp(bool bSetNow = false) {
			if(bSetNow) {
				Now();
			}
			else {
				Init();
			}
		}
		TimeStamp(int64_t time_stamp) {
			Now(time_stamp);
		}

		~TimeStamp() {
			Clear(false);
		}
		void Init() {
			year_ = 0;
			month_ = 0;
			day_ = 0;
			hour_ = 0;
			min_ = 0;
			sec_ = 0;
			millisec_ = 0;
			microsec_ = 0;
			nanosec_ = 0;
		}
		void Clear(bool init = true)
		{
			if (init) {
				Init();
			}
		}

		TimeStamp operator -(const TimeStamp &time_stamp) const
		{
			TimeStamp result;
			result.Now(duration_nanosec_ - time_stamp.duration_nanosec_);
			return result;
		}
		
		int64_t operator -(int64_t duration_nanosec) const
		{
			return (duration_nanosec_ - duration_nanosec);
		}

		TimeStamp &operator -=(int64_t duration_nanosec)
		{
			duration_nanosec_ -= duration_nanosec;
			_SetTime();
			
			return *this;
		}
		
		int64_t Now() {
			duration_nanosec_ = Now_ns();

			_SetTime();

			return duration_nanosec_;
		}

		void Now(int64_t now)
		{
			duration_nanosec_ = now;
			_SetTime();
		}
		
		int64_t GetValue() {
			return duration_nanosec_;
		}

		const char *Format()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%04d-%02d-%02d_%02d-%02d-%02d]",
				year_, month_, day_, hour_, min_, sec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%04d-%02d-%02d_%02d-%02d-%02d]",
				year_, month_, day_, hour_, min_, sec_);
#endif
			return format_;
		}
		const char *Format_Millisec()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%04d-%02d-%02d_%02d-%02d-%02d.%03d]",
				year_, month_, day_, hour_, min_, sec_, millisec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%04d-%02d-%02d_%02d-%02d-%02d.%03d]",
				year_, month_, day_, hour_, min_, sec_, millisec_);
#endif
			return format_;
		}
		const char *Format_Microsec()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%04d-%02d-%02d_%02d-%02d-%02d.%06d]",
				year_, month_, day_, hour_, min_, sec_, microsec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%04d-%02d-%02d_%02d-%02d-%02d.%06d]",
				year_, month_, day_, hour_, min_, sec_, microsec_);
#endif
			return format_;
		}
		const char *Format_Nanosec()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%04d-%02d-%02d_%02d-%02d-%02d.%09d]",
				year_, month_, day_, hour_, min_, sec_, nanosec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%04d-%02d-%02d_%02d-%02d-%02d.%09d]",
				year_, month_, day_, hour_, min_, sec_, nanosec_);
#endif
			return format_;
		}
		const char *Format_Short_Millisec()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%02d.%03d]", sec_, millisec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%02d.%03d]", sec_, millisec_);
#endif
			return format_;
		}
		const char *Format_Short_Microsec()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%02d.%06d]", sec_, microsec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%02d.%06d]", sec_, microsec_);
#endif
			return format_;
		}
		const char *Format_Short_Nanosec()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "[%02d.%09d]", sec_, nanosec_);
#endif
#ifdef __linux__    
			sprintf(format_, "[%02d.%09d]", sec_, nanosec_);
#endif
			return format_;
		}
		const char *Format_Filename()
		{
#ifdef _WINDOWS    
			sprintf_s(format_, "%04d-%02d-%02d_%02d-%02d-%02d_%03d",
				year_, month_, day_, hour_, min_, sec_, millisec_);
#endif
#ifdef __linux__    
			sprintf(format_, "%04d-%02d-%02d_%02d-%02d-%02d_%03d",
				year_, month_, day_, hour_, min_, sec_, millisec_);
#endif
			return format_;
		}

		int Year() {
			return year_;
		}
		int Month() {
			return month_;
		}
		int Day() {
			return day_;
		}
		int Hour() {
			return hour_;
		}
		int Min() {
			return min_;
		}
		int Sec() {
			return sec_;
		}
		int MilliSec() {
			return millisec_;
		}
		int MicroSec() {
			return microsec_;
		}
		int NanoSec() {
			return nanosec_;
		}
		
		static int64_t Now_ns() {
			sys_nanoseconds now = system_clock::now();
			nanoseconds duration = duration_cast<nanoseconds>(now.time_since_epoch());
			return duration.count();
		}

	private:
		void _SetTime() {
			time_t duration_sec = duration_nanosec_ / 1000000000;
			nanosec_ = duration_nanosec_ % 1000000000;
			microsec_ = nanosec_ / 1000;
			millisec_ = microsec_ / 1000;

#ifdef _WINDOWS    
			tm tm_;
			gmtime_s(&tm_, &duration_sec);
			tm *ptm = &tm_;
#endif
#ifdef __linux__    
			tm *ptm = gmtime(&duration_sec);
#endif

			year_ = ptm->tm_year + 1900;
			month_ = ptm->tm_mon + 1;
			day_ = ptm->tm_mday;
			hour_ = ptm->tm_hour;
			min_ = ptm->tm_min;
			sec_ = ptm->tm_sec;
		}
	};
}