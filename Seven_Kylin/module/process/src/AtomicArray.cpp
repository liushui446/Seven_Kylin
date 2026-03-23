#include "process/AtomicArray.hpp"

namespace seven
{
    AtomicIntData::AtomicIntData() : _Data(0)
    {
    }
    
    AtomicIntData::value_type AtomicIntData::GetValue() throw()
    {
        return _Data.load(std::memory_order_acquire);
    }

    AtomicIntData::value_type AtomicIntData::GetValue() const throw()
    {
        return _Data.load(std::memory_order_acquire);
    }

    void AtomicIntData::SetValue(AtomicIntData::value_type &&value) throw()
    {
        AtomicIntData::value_type v = std::forward<AtomicIntData::value_type>(value);
        _Data.store(v, std::memory_order_release);
    }

    bool AtomicIntData::CAS(AtomicIntData::value_type &expected, AtomicIntData::value_type desired, std::memory_order mem_order)
    {
        return _Data.compare_exchange_weak(expected, desired, mem_order);
    }

    AtomicIntData &AtomicIntData::operator=(AtomicIntData &rhs)
    {
        return *this;
    }

    AtomicIntArray::AtomicIntArray(size_t size)
    {
        _Data.reset(new AtomicIntData[size](), std::default_delete<AtomicIntData[]>());
        _size.store(size, std::memory_order::memory_order_release);
    }

    AtomicIntData &AtomicIntArray::operator[](int rank) noexcept
    {
        return _Data.get()[rank];
    }

    AtomicDoubleData::AtomicDoubleData() : _Data(0)
    {
    }

    double AtomicDoubleData::GetValue() throw()
    {
        return _Data.load(std::memory_order_acquire);
    }

    double AtomicDoubleData::GetValue() const throw()
    {
        return _Data.load(std::memory_order_acquire);
    }

    void AtomicDoubleData::SetValue(double&& value) throw()
    {
        double v = std::forward<double>(value);
        _Data.store(v, std::memory_order_release);
    }

    bool AtomicDoubleData::CAS(double& expected, double desired, std::memory_order mem_order)
    {
        return _Data.compare_exchange_weak(expected, desired, mem_order);
    }

    AtomicDoubleData& AtomicDoubleData::operator=(AtomicDoubleData& rhs)
    {
        return *this;
    }

    AtomicDoubleArray::AtomicDoubleArray(size_t size)
    {
        _Data.reset(new AtomicDoubleData[size](), std::default_delete<AtomicDoubleData[]>());
        _size.store(size, std::memory_order::memory_order_release);
    }

    AtomicDoubleData& AtomicDoubleArray::operator[](int rank) noexcept
    {
        return _Data.get()[rank];
    }
}