CC = g++
AR = ar

LIBLOGCONFIGTIME_PATH = ../../Framework/libLogConfigTime
OPENCV_CFLAGS = `pkg-config --cflags opencv`

CFLAGS = -c -Wall -I include/ -I $(LIBLOGCONFIGTIME_PATH)/include $(OPENCV_CFLAGS)
LDFLAGS = 

all: lib3dWorld.a


LIB3DWORLD_OBJECTS = src/camera.o src/chessboard.o src/chessboarddetector.o src/ray2d.o src/ray.o src/showhelper.o

lib3dWorld.a: $(LIB3DWORLD_OBJECTS)
	$(AR) rcs lib3dWorld.a $(LIB3DWORLD_OBJECTS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean:
	rm -rf src/*.o lib3dWorld.a
	
.PHONY: all clean