CXX      = g++
LDFLAGS  = -lpthread
CXXFLAGS = -std=c++17 -Wall -Wextra -Wfatal-errors -O3
TARGET   = serial 
SRCFILES = main.cpp serial.cpp
OBJECTS  = $(patsubst %.cpp, %.o, $(SRCFILES))

ifeq ($(shell uname -p),unknown)
   CXXFLAGS += -march=armv8-a+crc+fp+simd -mtune=cortex-a53
endif

all: $(TARGET)

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -f *.o $(TARGET)
