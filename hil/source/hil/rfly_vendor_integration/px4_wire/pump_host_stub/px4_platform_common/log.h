#pragma once
// HOST fixture error output only; live builds use the existing PX4 logger.
#include <cstdio>
#define PX4_ERR(...) do {std::fprintf(stderr,__VA_ARGS__);std::fputc('\n',stderr);} while(0)
