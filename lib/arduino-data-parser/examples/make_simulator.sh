g++ -c ../src/sewparser.cpp ../src/sewprotocol.cpp
ar rvs sewparser.a sewparser.o sewprotocol.o
g++ -o simulator simulator.cpp sewparser.a -I../src
rm -f *.o *.a
