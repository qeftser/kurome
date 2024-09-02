
# compiler
CC := g++

# other variables
OUTPUT := tst.e
CC_FLAGS := -g -O3 -Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Wno-unused-parameter -Wno-format
LD_FLAGS := -O3 

SRC_FILES := $(wildcard src/*.cpp)
H_FILES := $(wildcard src/*.h)
OBJ_FILES := $(SRC_FILES:.c=.o)

$(OUTPUT): $(OBJ_FILES) Makefile
	$(CC) $(CC_FLAGS) $(OBJ_FILES) $(H_FILES) $(LD_FLAGS) -o ./bin/$@ && chmod -c u+x ./bin/$@

all: ex1 ex2 ex3 ex4 ex5 ex6 ex7 ex8

ex1: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex2: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex3: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex4: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex5: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex6: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex7: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e
ex8: $(OBJ_FILES) Makefile
	$(CC) $(OBJ_FILES) $(H_FILES) ./examples/$@.cpp $(LD_FLAGS) -o ./bin/$@.e && chmod -c u+x ./bin/$@.e

.cpp.o:
	$(CC) -c $(CC_FLAGS) $< -o $@

clean:
	rm -f $(OBJ_FILES) ./bin/*.e

run: $(OUTPUT)
	cd bin && ./tst.e && cd ..
