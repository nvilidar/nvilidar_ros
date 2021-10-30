#include "console.h"

namespace nvilidar
{
void
disableStdoutStream() {
    std::cout.setstate(std::ios::failbit);
    std::cerr.setstate(std::ios::failbit);
};

} 
