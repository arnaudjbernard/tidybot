# Makefile for Tidybot
#g++ -o main main.cpp Vect.cpp `pkg-config opencv playerc++ --cflags --libs`

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
