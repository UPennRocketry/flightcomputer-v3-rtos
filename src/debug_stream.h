#pragma once

#include "app_types.h"

namespace debug_stream
{

void writeRecord(const debug_record_t &record);
void task(void *);

} // namespace debug_stream
