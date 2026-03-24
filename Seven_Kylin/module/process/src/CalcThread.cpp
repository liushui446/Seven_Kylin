//#include <windows.h>
//#include <processthreadsapi.h>
//#include <timeapi.h>
//#include <winnt.h>

#include "process/CalcThread.hpp"
#include "barrage/barrage.hpp"
#include <deception/deception.hpp>
#include <transformation/transformation.hpp>
#include <process/process.hpp>
#include "process/CalcParamRes.hpp"

#pragma comment(lib, "winmm.lib") 

namespace seven
{
	// 静态成员变量初始化
	std::shared_ptr<CalcProcessThread> CalcProcessThread::instance_ = nullptr;    // 静态成员变量，保存单例实例  
	std::once_flag CalcProcessThread::flag_;					// 静态成员变量，用于同步访问 

	static std::vector<std::shared_ptr<CalcTaskParam>> g_task_queue;
	// 全局任务队列（线程安全）
	static std::mutex g_task_mutex;
	static std::condition_variable g_task_cv;

	static size_t GetNumThread()
	{
		return 1;
	}

	struct CalcProcessThread::Pimple
	{
	public:
		enum class ThreadStatus
		{
			UNINIT = -1,
			DORMANT = 0,	 // 休眠
			READY = 1,		 // 准备
			BUSY = 2,		 // 忙碌
			INTERRUPTED = 3, // 打断
			QUIT = 4,		 // 退出
		};

		std::atomic<int> iThreadNum_;			  // 线程数
		std::vector<std::thread*> pThreads_;     // 线程指针
		AtomicIntArray ThdStats_;		  // 各个线程的状态
		std::atomic<bool> bStartWork_;			  // 是否线程工作
		// 新增：每个线程的中断请求标志（线程安全）
		std::vector<bool> vecInterruptRequest_;

		Pimple()
			: iThreadNum_(GetNumThread())
			, pThreads_(iThreadNum_, nullptr)
			, ThdStats_(iThreadNum_)
			, bStartWork_(false)
			, vecInterruptRequest_(iThreadNum_, false) // 初始化中断标志为false
		{
			for (int i = 0; i < iThreadNum_; ++i)
			{
				ThdStats_[i].SetValue(static_cast<int>(ThreadStatus::UNINIT));
			}
		}

		~Pimple()
		{
		}
	};

	CalcProcessThread::CalcProcessThread()
		: pMem_(std::make_shared<Pimple>())
	{
		this->Init();
	}

	CalcProcessThread::~CalcProcessThread()
	{
		UnInit();

		for (auto itr = pMem_->pThreads_.begin(); itr != pMem_->pThreads_.end(); ++itr)
		{
			if ((*itr) != nullptr)
			{
				delete (*itr);
				*itr = nullptr;
			}
		}
	}

	std::shared_ptr<CalcProcessThread> CalcProcessThread::GetInstance()
	{
		std::call_once(flag_, [&]() {
			instance_.reset(new CalcProcessThread());
			});
		return instance_;
	}

	void CalcProcessThread::Init()
	{
		for (size_t i = 0; i < pMem_->iThreadNum_; ++i)
		{
			if (pMem_->pThreads_[i] != nullptr)
			{
				continue;
			}
			pMem_->pThreads_[i] = new std::thread(std::bind(&CalcProcessThread::ThreadFunc, this, i));
			// Linux 设置线程名（最多 15 个字符）
			std::string name = "CalProcessThread_" + std::to_string(i);
			pthread_setname_np(pthread_self(), name.c_str());
			//SetThreadDescription(pMem_->pThreads_[i]->native_handle(), L"ImageProcessThread_" + i);
		}
		return;
	}

	void CalcProcessThread::UnInit()
	{
		for (int noThread = 0; noThread < pMem_->pThreads_.size(); ++noThread)
		{
			if (pMem_->pThreads_[noThread] == nullptr)
			{
				continue;
			}

			{
				std::unique_lock<std::mutex> lk(m_mut);
				pMem_->ThdStats_[noThread].SetValue(static_cast<int>(CalcProcessThread::Pimple::ThreadStatus::QUIT));
				m_con_var.notify_all();
			}

			if (pMem_->pThreads_[noThread]->joinable())
			{
				pMem_->pThreads_[noThread]->join();
			}
		}
		return;
	}

	void CalcProcessThread::ReloadAllThread()
	{
		for (size_t i = 0; i < pMem_->iThreadNum_; ++i)
		{
			if (pMem_->pThreads_[i] == nullptr)
			{
				continue;
			}
		}
		return;
	}

	unsigned int CalcProcessThread::GetThreadNum()
	{
		return pMem_->iThreadNum_;
	}

	void CalcProcessThread::StartWork(bool work)
	{
		pMem_->bStartWork_.store(work, std::memory_order_release);
	}

	bool CalcProcessThread::SerSwitchTaskParam(Formation_Type input) {
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		SwitchFormation(input);
		return true;
	}

	bool CalcProcessThread::SerTurnTaskParam(double input)
	{
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		TurnFormation(input);
		return true;
	}

	bool CalcProcessThread::SetAddNodeTaskParam(UUVNode input)
	{
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		AddNode(input.pos_.lon_deg, input.pos_.lat_deg, input.speed, input.heading, input.join_total_frames);
		return true;
	}

	bool CalcProcessThread::SetRemoveNodeTaskParam()
	{
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		RemoveLastNode();
		return true;
	}

	// 新增：提交计算任务并唤醒线程
	//bool CalcProcessThread::SubmitTask(HANDLE hPipe, const Json::Value& input, Json::Value& output) {
	//	if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
	//		return false;
	//	}

	//	// 1. 查找空闲线程
	//	int idle_thread = -1;
	//	for (int i = 0; i < pMem_->iThreadNum_; ++i) {
	//		if (pMem_->ThdStats_[i].GetValue() == static_cast<int>(Pimple::ThreadStatus::DORMANT)) {
	//			idle_thread = i;
	//			break;
	//		}
	//	}

	//	if (idle_thread == -1) {
	//		return false; // 无空闲线程
	//	}

	//	// 2. 创建任务参数
	//	auto task_param = std::make_shared<CalcTaskParam>();
	//	task_param->hPipe = hPipe;
	//	task_param->max_frames = CalcParamManager::Ins().GetCalcParam().sim_time_;
	//	task_param->run_frames = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
	//	task_param->return_frames = CalcParamManager::Ins().GetCalcParam().return_frames;
	//	task_param->serveral_plat = CalcParamManager::Ins().GetPlatform();
	//	task_param->input = input;
	//	task_param->task_finished = false;

	//	// 3. 将任务加入队列
	//	{
	//		std::lock_guard<std::mutex> lk(g_task_mutex);
	//		g_task_queue.push_back(task_param);
	//	}

	//	// 4. 唤醒指定空闲线程
	//	if (WakeUpAThread(idle_thread)) {
	//		// 等待任务完成
	//		std::unique_lock<std::mutex> lk(g_task_mutex);
	//		g_task_cv.wait(lk, [&]() {
	//			return task_param->task_finished.load(std::memory_order_acquire);
	//			});
	//		//output = task_param->trajectory_result;
	//		return true;
	//	}

	//	return false;
	//}

	// 新增：提交计算任务并唤醒线程（无等待版本）
	bool CalcProcessThread::SubmitTask(HANDLE hPipe, const Json::Value& input, Json::Value& output) {
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}

		// 1. 查找空闲线程
		int idle_thread = -1;
		for (int i = 0; i < pMem_->iThreadNum_; ++i) {
			if (pMem_->ThdStats_[i].GetValue() == static_cast<int>(Pimple::ThreadStatus::DORMANT)) {
				idle_thread = i;
				break;
			}
		}

		if (idle_thread == -1) {
			return false; // 无空闲线程
		}

		// 2. 创建任务参数
		auto task_param = std::make_shared<CalcTaskParam>();
		task_param->hPipe = hPipe;
		task_param->max_frames = CalcParamManager::Ins().GetCalcParam().sim_time_;
		task_param->run_frames = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
		task_param->return_frames = CalcParamManager::Ins().GetCalcParam().return_frames;
		task_param->serveral_plat = CalcParamManager::Ins().GetPlatform();
		task_param->input = input;
		task_param->task_finished = false;

		// 3. 将任务加入队列
		{
			std::lock_guard<std::mutex> lk(g_task_mutex);
			g_task_queue.push_back(task_param);
		}

		// 4. 唤醒指定空闲线程
		if (WakeUpAThread(idle_thread)) {
			// ===================== 关键移除：删除等待逻辑 =====================
			// 原来的 wait 会阻塞在这里，现在直接返回 true
			// 任务会在线程后台异步执行
			// ==================================================================
			return true;
		}

		return false;
	}

	bool CalcProcessThread::WakeUpAThread(int noThread)
	{
		if (!pMem_->bStartWork_.load(std::memory_order_acquire))
		{
			return false;
		}

		int threadState = pMem_->ThdStats_[noThread].GetValue();
		if (threadState != static_cast<int>(Pimple::ThreadStatus::DORMANT))
		{
			return false;
		}

		int try_times = 0;
		do
		{
			try_times++;
			if (try_times > 10)
			{
				return false;
			}
			//timeBeginPeriod(1);
			std::this_thread::sleep_for(std::chrono::microseconds(1));
			//timeEndPeriod(1);
		} while ((!pMem_->ThdStats_[noThread].CAS(threadState, static_cast<int>(Pimple::ThreadStatus::READY))) &&
			(threadState == static_cast<int>(Pimple::ThreadStatus::DORMANT)));

		std::unique_lock<std::mutex> lk(m_mut);
		m_con_var.notify_all();

		return true;
	}

	bool CalcProcessThread::Interrupted()
	{
		// 边界检查
		/*if (noThread < 0 || noThread >= pMem_->iThreadNum_) {
			std::cerr << "Interrupted: 线程编号无效，编号：" << noThread << std::endl;
			return false;
		}*/

		for (int noThread = 0; noThread < pMem_->pThreads_.size(); ++noThread)
		{
			if (pMem_->pThreads_[noThread] == nullptr)
			{
				continue;
			}

			int threadState = pMem_->ThdStats_[noThread].GetValue();
			// 只有BUSY状态的线程才能被中断
			if (threadState != static_cast<int>(Pimple::ThreadStatus::BUSY))
			{
				std::cerr << "Interrupted: 线程" << noThread << "非BUSY状态，当前状态：" << threadState << std::endl;
				return false;
			}

			// 1. 设置中断请求标志（核心：让线程能感知到中断）
			//pMem_->vecInterruptRequest_[noThread].store(true, std::memory_order_release);
			pMem_->vecInterruptRequest_[noThread] = true;

			// 2. 切换线程状态为INTERRUPTED
			pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::INTERRUPTED));

			std::cout << "已发起线程" << noThread << "中断请求" << std::endl;
		}

		return true;
	}

	// 新增：重置中断标志（线程完成/中断后清理）
	void CalcProcessThread::ResetInterruptFlag(int noThread)
	{
		if (noThread >= 0 && noThread < pMem_->iThreadNum_) {
			//pMem_->vecInterruptRequest_[noThread].store(false, std::memory_order_release);
			pMem_->vecInterruptRequest_[noThread] = false;
		}
	}

	// 新增：检查当前线程是否被中断（供ThreadFunc内部调用）
	bool CalcProcessThread::IsInterrupted(int noThread)
	{
		if (noThread < 0 || noThread >= pMem_->iThreadNum_) {
			return false;
		}
		// 双重检查：状态 + 标志
		return (pMem_->ThdStats_[noThread].GetValue() == static_cast<int>(Pimple::ThreadStatus::INTERRUPTED)) || pMem_->vecInterruptRequest_[noThread];
			/*pMem_->vecInterruptRequest_[noThread].load(std::memory_order_acquire);*/
	}

	void CalcProcessThread::ThreadFunc(int noThread)
	{
		// 初始化线程状态为休眠
		pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));

		while (true)
		{
			std::unique_lock<std::mutex> lk(m_mut);
			// 等待唤醒条件：线程退出 或 线程就绪
			m_con_var.wait(lk, [&]()
				{
					int status = pMem_->ThdStats_[noThread].GetValue();
					// 如果不是退出状态，重置为休眠状态
					if (status != static_cast<int>(Pimple::ThreadStatus::QUIT)) {
						if (status != static_cast<int>(Pimple::ThreadStatus::READY)) {
							pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
						}
					}
					// 唤醒条件：退出 或 就绪
					return status == static_cast<int>(Pimple::ThreadStatus::QUIT) ||
						status == static_cast<int>(Pimple::ThreadStatus::READY);
				});

			// 判断是否是退出线程的命令
			int current_status = pMem_->ThdStats_[noThread].GetValue();
			if (current_status == static_cast<int>(Pimple::ThreadStatus::QUIT))
			{
				break;
			}

			// 线程进入忙碌状态
			pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::BUSY));
			lk.unlock(); // 释放锁，避免阻塞其他操作

			/*try
			{*/
				auto start = std::chrono::high_resolution_clock::now();

				// ========== 核心工作区：执行计算任务 ==========
				std::shared_ptr<CalcTaskParam> task_param;
				{
					std::lock_guard<std::mutex> task_lk(g_task_mutex);
					if (!g_task_queue.empty()) {
						task_param = g_task_queue.front();
						g_task_queue.erase(g_task_queue.begin());
					}
				}

				//临时数据
				CalcTempParam TempParam;
				TempParam.input = task_param->input;
				TempParam.max_frames.store(task_param->max_frames);
				TempParam.run_frames.store(task_param->run_frames);
				TempParam.return_frames.store(task_param->return_frames);
				TempParam.serveral_plat = task_param->serveral_plat;


				if (task_param) {
					CalcParamManager::Ins().GetCalcParam();
					// 执行核心计算
					int cmd_int = task_param->input.get("cmd", 4).asInt();
					Cmd_Type type = static_cast<Cmd_Type>(cmd_int);
					if (type == Cmd_Type::Barrage)
					{
						SimConfig barrage_config = ContextManager::Ins().GetBarrageParams();
						while (true) {
							//判断是否运行到最大帧数
							UINT run_frames_cnt_ = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
							if (run_frames_cnt_ >= (task_param->max_frames - 1)) {
								break;
							}

							//判断是否被打断
							bool is_interrupted = IsInterrupted(noThread);
							if (is_interrupted) {
								// 重置中断标志
								ResetInterruptFlag(noThread);
								// 切换状态为DORMANT（而非直接退出）
								pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
								break;
							}
							/*std::vector<InputPlatParam> server_platform_data;
							for (int cnt = 0; cnt < task_param->serveral_plat.size(); cnt++)
							{
								server_platform_data.push_back(task_param->serveral_plat[cnt]);
							}*/
							//std::vector<InputPlatParam> server_platform_data = barrage_config.platsparam;
							Barrage_Test_1(TempParam, barrage_config);
							CalcParamManager::Ins().SetRunFramesCnt(TempParam.run_frames);
							sendResultData(task_param->hPipe, TempParam.trajectory_result);
						}
					}
					else if (type == Cmd_Type::Deception)
					{
						SimParams deception_config = ContextManager::Ins().GetDeceptionParams();
						while (true) {
							//判断是否运行到最大帧数
							UINT run_frames_cnt_ = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
							if (run_frames_cnt_ >= (task_param->max_frames - 1)) {
								break;
							}

							//判断是否被打断
							bool is_interrupted = IsInterrupted(noThread);
							if (is_interrupted) {
								// 重置中断标志
								ResetInterruptFlag(noThread);
								// 切换状态为DORMANT（而非直接退出）
								pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
								break;
							}

							/*std::vector<InputPlatParam> server_platform_data;
							for (int cnt = 0; cnt < task_param->serveral_plat.size(); cnt++)
							{
								server_platform_data.push_back(task_param->serveral_plat[cnt]);
							}*/
							Deception_Use(TempParam, deception_config);
							CalcParamManager::Ins().SetRunFramesCnt(TempParam.run_frames);
							sendResultData(task_param->hPipe, TempParam.trajectory_result);
						}
					}
					else if (type == Cmd_Type::Transformation)
					{
						while (true) {
							//判断是否运行到最大帧数
							UINT run_frames_cnt_ = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
							/*if (run_frames_cnt_ >= (task_param->max_frames - 1)) {
								break;
							}*/

							//判断是否被打断
							bool is_interrupted = IsInterrupted(noThread);
							if (is_interrupted) {
								// 重置中断标志
								ResetInterruptFlag(noThread);
								// 切换状态为DORMANT（而非直接退出）
								pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
								break;
							}
							FormationConfig params = ContextManager::Ins().GetFormationParams();
							Transformation_Use(TempParam);
							CalcParamManager::Ins().SetRunFramesCnt(TempParam.run_frames);
							sendResultData(task_param->hPipe, TempParam.trajectory_result);
						}
					}

					// 标记任务完成
					task_param->task_finished = true;
					g_task_cv.notify_all();
				}

				// 并行计算示例（保留原框架）
				int num = 3;
				//concurrency::parallel_for(0, num, [&](size_t index)
				//	{
				//		// 可添加并行辅助计算逻辑
				//	});

				auto stop = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

				// ========== 工作完成时间 ==========
				std::string str = "CalcProcessThread::ThreadFunc(), Barrage_Test_1 Time Cost: [" +
					std::to_string(duration) + "]ms";

			//}
			//catch (const std::exception& e)
			//{
			//	// 异常处理
			//	std::string err_str = "CalcProcessThread::ThreadFunc() exception: " + std::string(e.what());
			//	std::cerr << err_str << std::endl;
			//}

			// 重置线程状态为休眠
			if (pMem_->ThdStats_[noThread].GetValue() != static_cast<int>(Pimple::ThreadStatus::QUIT) &&
				pMem_->ThdStats_[noThread].GetValue() != static_cast<int>(Pimple::ThreadStatus::INTERRUPTED)) {
				pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
			}
		}
	}
}
