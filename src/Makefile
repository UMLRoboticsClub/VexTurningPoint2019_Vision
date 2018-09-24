#arch: install "opencv" and "hdf5"
CXX      = g++
LDFLAGS  = $(shell pkg-config --libs opencv)
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
