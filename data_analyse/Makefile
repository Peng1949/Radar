#CC := g++                                                                                                                                                                                               
#CFLAGS += -fPIC -std=c++11 -O3 -Wall 
#LIBRARY := -L. -lcontrolcan -lpthread -L/usr/lib/x86_64-linux-gnu -lopencv_core -lopencv_highgui -lopencv_imgproc
#INCLUDE := -I/usr/include

#src := $(wildcard *.cpp)
#obj := $(patsubst %.cpp,%.o, $(src))

#all:main

#main:main.o $(obj)
#	$(CC) -o $@ $< $(LIBRARY)

#$(obj):%o:%cpp
#	$(CC) -c $(CFLAGS)  $< -o $@ $(INCLUDE)
   
#.PHONY:clean
#clean:
#	rm main.o main

all:
	#gcc -o hello -L. -lcontrolcan -lSetting_202 -lpthread -lusb main.c
	#arm-none-linux-gnueabi-gcc -o hello -L. -L /home/caidunqing/controlcan -lcontrolcan -lSetting_202 -lpthread main.c
	g++ -o main main.cpp  -L. -lcontrolcan -lpthread -lopencv_highgui -lopencv_imgproc -lopencv_core -lstdc++
	
clean:
	rm -f *.o main
