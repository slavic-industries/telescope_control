output: main.o si5351.o tmc2130.o tmc429.o
	g++ main.o si5351.o tmc2130.o tmc429.o -lpigpio -o output

main.o: main.cpp
	g++ -c main.cpp

si5351.o: si5351.cpp si5351.h
	g++ -c si5351.cpp

tmc2130.o: tmc2130.cpp tmc2130.h
	g++ -c tmc2130.cpp

tmc429.o: tmc429.cpp tmc429.h
	g++ -c tmc429.cpp

clean:
	rm *.o output
