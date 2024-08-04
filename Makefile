
# compiler
CC := g++

# other variables
OUTPUT := tst.e
CC_FLAGS := -g -O3 -Wall -Wextra -Wpedantic
LD_FLAGS := -O3

SRC_FILES := $(wildcard src/*.cpp)
H_FILES := $(wildcard src/*.h)
OBJ_FILES := $(SRC_FILES:.c=.o)

all: $(OUTPUT)

$(OUTPUT): $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) $(LD_FLAGS) -o ./bin/$@ && chmod -c u+x ./bin/$@

.cpp.o:
	$(CC) -c $(CC_FLAGS) $< -o $@

clean:
	rm -f $(OBJ_FILES) ./bin/*.e

run: $(OUTPUT)
	cd bin && ./tst.e && cd ..
