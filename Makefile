CFLAGS := -Wall -g -O3 -std=c++0x
CPP := g++
CPP_FILES := $(shell find src -name \*.cpp -print)
OBJS := $(CPP_FILES:.cpp=.o)
TARGETS := sample

all: $(OBJS)
	$(CPP) $(CFLAGS) -o $(TARGETS) $(OBJS)

clean:
	rm -f $(OBJS) $(TARGETS)

.cpp.o:
	$(CPP) -c $(CFLAGS) $< -o $@
