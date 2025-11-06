CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall 

SRC = main.cpp graph.cpp shortest_path.cpp

EXE = phase1

all: $(EXE)

$(EXE): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(EXE)

clean:
	rm -f $(EXE)
