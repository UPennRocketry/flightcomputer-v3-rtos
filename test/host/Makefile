# Host build of the hardware-independent flight modules.
# Usage: make -C test/host && make -C test/host run
ROOT := ../..
CXX ?= g++
CXXFLAGS := -std=gnu++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -I stubs -I $(ROOT)/include -I $(ROOT)/src -I $(ROOT)/lib/Fusion/src
CC ?= gcc
CFLAGS := -O1 -g -Wall -I $(ROOT)/lib/Fusion/src

SRCS := $(ROOT)/src/flight_logic.cpp $(ROOT)/src/state_estimator.cpp $(ROOT)/src/vertical_filter.cpp \
        $(ROOT)/src/telemetry.cpp $(ROOT)/src/persist.cpp $(ROOT)/src/supervisor.cpp test_flight.cpp
FUSION := $(wildcard $(ROOT)/lib/Fusion/src/*.c)

all: test_flight

test_flight: $(SRCS) $(FUSION)
	$(CXX) $(CXXFLAGS) -x c++ $(SRCS) -x c $(FUSION) -o $@ -lm

run: test_flight
	./test_flight
	python3 -I $(ROOT)/ground/telemetry_protocol.py --selftest state.bin health.bin expected.json

clean:
	rm -f test_flight state.bin health.bin expected.json
