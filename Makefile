# Compiler
CXX = g++

# Compiler flags:

CXXFLAGS = -std=c++17 -g -Wall -O3 -fopenmp

# Include directory
INCDIR = Phase-1/include
INCDIR2 = Phase-2/include
INCDIR3 = Phase-3/include

# Source directory
SRCDIR = Phase-1/src
SRCDIR2 = Phase-2/src
SRCDIR3 = Phase-3/src

# Object directory
OBJDIR = Phase-1/obj
OBJDIR2 = Phase-2/obj
OBJDIR3 = Phase-3/obj

EXEC = phase1
EXEC2 = phase2
EXEC3 = phase3

SOURCES = $(wildcard $(SRCDIR)/*.cpp)
SOURCES2 = $(wildcard $(SRCDIR2)/*.cpp)
SOURCES3 = $(wildcard $(SRCDIR3)/*.cpp)

OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJECTS2 = $(SOURCES2:$(SRCDIR2)/%.cpp=$(OBJDIR2)/%.o)
OBJECTS3 = $(SOURCES3:$(SRCDIR3)/%.cpp=$(OBJDIR3)/%.o)

.PHONY: all
all: $(EXEC) $(EXEC2) $(EXEC3)

$(EXEC): $(OBJECTS)
	@echo "Linking Phase 1..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -o $(EXEC) $(OBJECTS)
	@echo "Build complete: ./$(EXEC)"

$(EXEC2): $(OBJECTS2)
	@echo "Linking Phase 2..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR2) -o $(EXEC2) $(OBJECTS2)
	@echo "Build complete: ./$(EXEC2)"

$(EXEC3): $(OBJECTS3)
	@echo "Linking Phase 3..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR3) -o $(EXEC3) $(OBJECTS3)
	@echo "Build complete: ./$(EXEC3)"

$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(INCDIR)/graph.hpp | $(OBJDIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -c $< -o $@

$(OBJECTS2): $(OBJDIR2)/%.o: $(SRCDIR2)/%.cpp $(INCDIR2)/graph.hpp | $(OBJDIR2)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR2) -c $< -o $@

$(OBJECTS3): $(OBJDIR3)/%.o: $(SRCDIR3)/%.cpp $(INCDIR3)/graph.hpp $(INCDIR3)/utils.hpp | $(OBJDIR3)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR3) -c $< -o $@

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR2):
	@mkdir -p $(OBJDIR2)

$(OBJDIR3):
	@mkdir -p $(OBJDIR3)

.PHONY: clean
clean:
	@echo "Cleaning up..."
	@rm -rf $(OBJDIR) $(EXEC)
	@rm -rf $(OBJDIR2) $(EXEC2)
	@rm -rf $(OBJDIR3) $(EXEC3)