#include "core/CommonCore.hpp"

namespace seven
{
	PlatformData::PlatformData(UINT platform_id, LLA pos)
	{
		platform_id_ = platform_id;
		position_ = pos;
	}
}