LIB=-lwiringPi

BIN=sensor
OBJS=sensor.o main.o
SRCS=sensor.cpp main.cpp

all: $(OBJS)
	g++ $(LIB) -o $(BIN) $(OBJS)

$(OBJS):
	g++ -c $(SRCS)

test: all
	./$(BIN)

