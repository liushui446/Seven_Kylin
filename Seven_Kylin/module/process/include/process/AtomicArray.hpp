#pragma once
#pragma pack(push,4)

#include <atomic>
#include <iosfwd>
#include <memory>

namespace seven
{
	class AtomicIntData
	{
	public:
		typedef int value_type;

	private:
		std::atomic<int> _Data;
		AtomicIntData& operator = (AtomicIntData& rhs);

	public:
		AtomicIntData();
		value_type GetValue() const throw();
		value_type GetValue() throw();
		void SetValue(value_type&& value) throw();

		/// <summary>
		/// 存在修改好了但是返回false的情况
		/// </summary>
		/// <param name="expected"></param>
		/// <param name="desired"></param>
		/// <param name="mem_order"></param>
		/// <returns></returns>
		bool CAS(value_type& expected, value_type desired, std::memory_order mem_order = std::memory_order::memory_order_seq_cst);
	};

	class AtomicIntArray
	{
	private:
		std::shared_ptr<AtomicIntData> _Data;
		std::atomic<int> _size;
		AtomicIntArray  operator=(const AtomicIntArray& rhs) {};

	public:

		/// <summary>
		/// 初始化
		/// </summary>
		/// <param name="size">多大的size</param>
		explicit AtomicIntArray(size_t size = 1);

		AtomicIntData& operator[](int rank)throw();
		size_t size() { return _size; }
	};

	class AtomicDoubleData
	{
	private:
		std::atomic<double> _Data;
		AtomicDoubleData& operator = (AtomicDoubleData& rhs);

	public:
		AtomicDoubleData();
		double GetValue() const throw();
		double GetValue() throw();
		void SetValue(double&& value) throw();

		/// <summary>
		/// 存在修改好了但是返回false的情况
		/// </summary>
		/// <param name="expected"></param>
		/// <param name="desired"></param>
		/// <param name="mem_order"></param>
		/// <returns></returns>
		bool CAS(double& expected, double desired, std::memory_order mem_order = std::memory_order::memory_order_seq_cst);
	};

	class AtomicDoubleArray
	{
	private:
		std::shared_ptr<AtomicDoubleData> _Data;
		std::atomic<int> _size;
		AtomicDoubleArray  operator=(const AtomicDoubleArray& rhs) {};

	public:
		/// <summary>
		/// 初始化
		/// </summary>
		/// <param name="size">多大的size</param>
		explicit AtomicDoubleArray(size_t size = 1);

		AtomicDoubleData& operator[](int rank)throw();
		size_t size() { return _size; }
	};
}

#pragma pack(pop,4)
