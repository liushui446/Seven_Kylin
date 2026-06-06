#pragma once

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <windows.h>
//#include <ppl.h>
#include <thread>
#include <mutex>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "core/CommonCore.hpp"
#include "process/AtomicArray.hpp"

namespace seven
{
	class SEVEN_EXPORTS CalcProcessThread
	{
	public:
		CalcProcessThread();
		~CalcProcessThread();

	public:
		static std::shared_ptr<CalcProcessThread> GetInstance();

	public:
		void Init();
		void UnInit();
		void ReloadAllThread();
		unsigned int GetThreadNum();
		void StartWork(bool work);
		bool SerSwitchTaskParam(int formation_id, Formation_Type input);
		bool SerTurnTaskParam(int formation_id, double input);
		bool SetAddNodeTaskParam(int formation_id, vector<UUVNode>& input);
		bool SetRemoveNodeTaskParam(int formation_id, int num);
		bool SubmitTask(HANDLE hPipe, const Json::Value& input, Json::Value& output);
		bool WakeUpAThread(int noThread);
		bool Interrupted();
		void ResetInterruptFlag(int noThread);
		bool IsInterrupted(int noThread);

	private:
		void ThreadFunc(int noThread);

	private:
		struct Pimple;
		std::shared_ptr<Pimple> pMem_;
		std::condition_variable m_con_var;
		std::mutex m_mut;

	private:
		static std::once_flag flag_;
		static std::shared_ptr<CalcProcessThread> instance_;

	public:
		CalcProcessThread(const CalcProcessThread&) = delete;
		CalcProcessThread(CalcProcessThread&&) = delete;
		CalcProcessThread& operator=(const CalcProcessThread&) = delete;
		CalcProcessThread& operator=(CalcProcessThread&&) = delete;
	};
}