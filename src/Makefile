#arch: install opencv, hdf5, and boost
CXX      = g++
LDFLAGS  = $(shell pkg-config --libs opencv) -lpthread -lboost_system
INCLUDE  = 
CXXFLAGS = -std=gnu++17 -Wall -Wextra -Wfatal-errors -O3 -g
CXXFLAGS += $(INCLUDE)
TARGET   = vision
SRCFILES = main.cpp
OBJECTS  = $(patsubst %.cpp, %.o, $(SRCFILES))

all: $(TARGET)

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -f *.o $(TARGET)
