#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

TEST_CASE("sanity: the host test harness builds and runs") {
    CHECK(1 + 1 == 2);
}
