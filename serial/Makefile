CXX      = g++
LDFLAGS  = -lpthread
CXXFLAGS = -std=c++17 -Wall -Wextra -Wfatal-errors -O3 -funroll-loops -march=native
CXXFLAGS += $(INCLUDE)
TARGET   = serial 
SRCFILES = main.cpp serial.cpp
OBJECTS  = $(patsubst %.cpp, %.o, $(SRCFILES))

all: $(TARGET)

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -f *.o $(TARGET)
