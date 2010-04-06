# Makefile for Hand_Gesture
#g++ -o Hand_Gesture Hand_Gesture.cpp `pkg-config opencv --cflags --libs`

.PHONY: clean mrproper
.SUFFIXES: .o .cpp
CXX =g++
CXXFLAGS =-c `pkg-config --cflags opencv playerc++`# -g
LDFLAGS=`pkg-config --libs opencv playerc++`
SOURCES=main.cpp Vect.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o: $(HEADERS)
	$(CXX) $(CXXFLAGS) $< -o $@

mrproper:
	rm -f *.o *~

clean:
	rm -f $(EXECUTABLE) *.o
