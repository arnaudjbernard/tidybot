# Makefile for Tidybot
#g++ -o main main.cpp Vect.cpp `pkg-config opencv playerc++ --cflags --libs`

.PHONY: clean mrproper map
.SUFFIXES: .o .cpp
CXX =g++
CXXFLAGS =-c `pkg-config --cflags opencv playerc++`# -g
LDFLAGS=`pkg-config --libs opencv playerc++`
SOURCES=$(wildcard *.cpp)
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@

main.o: main.cpp main.hpp Vect.hpp PathPlanner.hpp

Vect.o: Vect.cpp Vect.hpp

PathPlanner.o: PathPlanner.cpp PathPlanner.hpp Obstacle.hpp Node.hpp

Obstacle.o: Obstacle.cpp Obstacle.hpp

Node.o: Node.cpp Node.hpp

.cpp.o: $(HEADERS)
	$(CXX) $(CXXFLAGS) $< -o $@

map:
	python map/convert.py
	python map/convertL.py*

texture:
	python gazebo/textures.py

mrproper: clean
	find . -name "*~" -exec rm {} \;

clean:
	rm -f $(EXECUTABLE) *.o
