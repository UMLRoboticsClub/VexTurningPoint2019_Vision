#arch: install opencv, hdf5, and boost
CXX      = g++
CVFLAGS  = -I/usr/include/opencv -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dpm -lopencv_face -lopencv_photo -lopencv_freetype -lopencv_fuzzy -lopencv_img_hash -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_plot -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ml -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_flann -lopencv_xobjdetect -lopencv_imgcodecs -lopencv_objdetect -lopencv_xphoto -lopencv_imgproc -lopencv_core
#LDFLAGS  = $(shell pkg-config --libs opencv) -lpthread -lboost_system
LDFLAGS  = $(CVFLAGS) -lpthread
CXXFLAGS = -std=c++17 -Wall -Wextra -Wfatal-errors -O3 -funroll-loops -march=native
CXXFLAGS += $(INCLUDE)
TARGET   = vision
SRCFILES = main.cpp crc.cpp
OBJECTS  = $(patsubst %.cpp, %.o, $(SRCFILES))

ifeq ($(shell uname -p),aarch64)
        INCLUDE  = -I/usr/local/include/opencv4
else
        INCLUDE  =
endif

all: $(TARGET)

.c.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -f *.o $(TARGET)
