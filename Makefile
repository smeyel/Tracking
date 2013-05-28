all: lib3dWorld
 
lib3dWorld:
	make -C lib3dWorld/
	
clean:
	make -C lib3dWorld/ clean
	
.PHONY: all clean lib3dWorld