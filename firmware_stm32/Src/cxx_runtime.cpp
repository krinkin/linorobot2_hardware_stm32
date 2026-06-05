// Minimal freestanding C++ runtime so the driver/control C++ TUs link WITHOUT
// pulling in libstdc++ (kept out of the tight flash/RAM budget). We build with
// -fno-exceptions -fno-rtti -fno-use-cxa-atexit, so all that is needed is:
//   - operator new/delete backed by the C heap (malloc is already used by rcl)
//   - __cxa_pure_virtual for the (never-called) pure-virtual slot in vtables
// No exception/RTTI/static-guard machinery is referenced.
#include <cstddef>
#include <cstdlib>

void* operator new(std::size_t n)        { return std::malloc(n); }
void* operator new[](std::size_t n)      { return std::malloc(n); }
void  operator delete(void* p) noexcept            { std::free(p); }
void  operator delete[](void* p) noexcept          { std::free(p); }
void  operator delete(void* p, std::size_t) noexcept   { std::free(p); }
void  operator delete[](void* p, std::size_t) noexcept { std::free(p); }

extern "C" void __cxa_pure_virtual(void) { for (;;) {} }
