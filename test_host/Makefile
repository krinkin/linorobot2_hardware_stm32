CXX      ?= g++
CXXFLAGS ?=
CXXFLAGS += -std=c++17 -Wall -Wextra -O0 -g
LIBDIR   := ../firmware/lib
INCLUDES := -Ishim \
            -I$(LIBDIR)/kinematics \
            -I$(LIBDIR)/pid \
            -I$(LIBDIR)/odom_integrator

# Test translation units — picked up as files are added in later tasks.
TESTS := $(wildcard test_*.cpp)

# Portable sources under test (header-only libs need no entry here)
SRCS  := $(LIBDIR)/kinematics/kinematics.cpp \
         $(LIBDIR)/pid/pid.cpp

run_tests: $(TESTS) $(SRCS) doctest.h shim/Arduino.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TESTS) $(SRCS) -o run_tests

.PHONY: test clean
test: run_tests
	./run_tests
clean:
	rm -f run_tests
