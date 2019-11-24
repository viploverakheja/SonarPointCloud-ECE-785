CC = g++ -std=c++11
CFLAGS = -c -Wall -g -march=armv7-a -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a8 -mtune=cortex-a8 -O1

compass: imu_read.o
	$(CC) imu_read.o -g  -lm -o $@

# geometry_list.s: geometry.c
#	$(CC) -Wa,-adhln -g geometry.c -c > geometry_list.s

clean:
	rm -f *.o compass
