output: main.o si5351.o
	g++ main.o si5351.o -lpigpio -o output

main.o: main.cpp
	g++ -c main.cpp

si5351.o: si5351.cpp si5351.h
	g++ -c si5351.cpp

clean:
	rm *.o output
