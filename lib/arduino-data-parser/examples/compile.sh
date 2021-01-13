g++ -c ../src/sewparser.cpp ../src/sewprotocol.cpp
ar rvs sewparser.a sewparser.o sewprotocol.o
g++ -o sample sewsample.cpp sewparser.a -I../src
rm -f *.o *.a
