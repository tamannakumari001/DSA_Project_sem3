# Compiler
CXX = g++

# Compiler flags:

CXXFLAGS = -std=c++17 -g -Wall -O3 -fopenmp

# Include directory
INCDIR = Phase-1/include
INCDIR2 = Phase-2/include

# Source directory
SRCDIR = Phase-1/src
SRCDIR2 = Phase-2/src

# Object directory
OBJDIR = Phase-1/obj
OBJDIR2 = Phase-2/obj

EXEC = phase1
EXEC2 = phase2

SOURCES = $(wildcard $(SRCDIR)/*.cpp)
SOURCES2 = $(wildcard $(SRCDIR2)/*.cpp)

OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJECTS2 = $(SOURCES2:$(SRCDIR2)/%.cpp=$(OBJDIR2)/%.o)

.PHONY: all
all: $(EXEC) $(EXEC2)

$(EXEC): $(OBJECTS)
	@echo "Linking Phase 1..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -o $(EXEC) $(OBJECTS)
	@echo "Build complete: ./$(EXEC)"

$(EXEC2): $(OBJECTS2)
	@echo "Linking Phase 2..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR2) -o $(EXEC2) $(OBJECTS2)
	@echo "Build complete: ./$(EXEC2)"

$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(INCDIR)/graph.hpp | $(OBJDIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -c $< -o $@

$(OBJECTS2): $(OBJDIR2)/%.o: $(SRCDIR2)/%.cpp $(INCDIR2)/graph.hpp | $(OBJDIR2)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR2) -c $< -o $@

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR2):
	@mkdir -p $(OBJDIR2)

.PHONY: clean
clean:
	@echo "Cleaning up..."
	@rm -rf $(OBJDIR) $(EXEC)
	@rm -rf $(OBJDIR2) $(EXEC2)