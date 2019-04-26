FLAGS = -pedantic -Wall -Wextra -std=c++17 -g -I /usr/include

all:conics
clean:
	rm -rf *.o ./conics

# Dependencies
test.o : trajectory.hpp body.hpp

%.o : %.cpp
	g++ $(FLAGS) -c $< -o $@

conics: test.o
	g++ $(FLAGS) $^ -o $@
