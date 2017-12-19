CXX = clang++
LD = clang++ 

CXXFLAGS = -std=c++1y -stdlib=libc++ -c -g -O0 -Wall -Wextra -pedantic
LDFLAGS = -std=c++1y -stdlib=libc++
EXE1 = main
OBJ1 = flow_free.o

all: $(EXE1)

$(EXE1): $(OBJ1)
	$(LD) $(LDFLAGS) -o $@ $<

%.o : %.c
	$(CXX) $(CXXFLAGS) -o $@ $<

.PHONY: clean

clean:
	rm -rf *.o $(EXE1) $(EXE2)