# Makefile

CXX = g++
CXXFLAGS = -std=c++11 -O2 -I /usr/local/include/eigen3

TARGET = 4_axis_jacobian
SRC = 4_axis_jacobian.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)