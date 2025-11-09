# Compiler
CXX = g++

# Compiler flags:
# -std=c++17: Use the C++17 standard (needed for nlohmann/json)
# -g: Include debugging symbols
# -Wall: Enable all compiler warnings
# -O2: Optimization level 2
CXXFLAGS = -std=c++17 -g -Wall -O3

# Include directory
INCDIR = Phase-1/include
INCDIR2 = Phase-2/include

# Source directory
SRCDIR = Phase-1/src
SRCDIR2 = Phase-2/src

# Object directory (to store intermediate .o files)
OBJDIR = Phase-1/obj
OBJDIR2 = Phase-2/obj

# Name of the final executable
EXEC = phase1
EXEC2 = phase2

# List of source files
SOURCES = $(SRCDIR)/main.cpp $(SRCDIR)/graph.cpp $(SRCDIR)/shortest_path.cpp
SOURCES2 = $(SRCDIR2)/main.cpp $(SRCDIR2)/graph.cpp $(SRCDIR2)/k_shortest_paths.cpp $(SRCDIR2)/utils.cpp $(SRCDIR2)/k_shortest_paths_heuristic.cpp

# Generate object file names from source file names
# (e.g., Phase-1/src/main.cpp -> obj/main.o)
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJECTS2 = $(SOURCES2:$(SRCDIR2)/%.cpp=$(OBJDIR2)/%.o)

# Add the include directory to the compiler flags
# CXXFLAGS += -I$(INCDIR)

# Default target: build the executable
.PHONY: all
all: $(EXEC) $(EXEC2)

# Rule to link the final executable
$(EXEC): $(OBJECTS)
	@echo "Linking Phase 1..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -o $(EXEC) $(OBJECTS)
	@echo "Build complete: ./$(EXEC)"

$(EXEC2): $(OBJECTS2)
	@echo "Linking Phase 2..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR2) -o $(EXEC2) $(OBJECTS2)
	@echo "Build complete: ./$(EXEC2)"

# *** CHANGED RULE ***
# Use a static pattern rule for more robust matching.
# This explicitly tells make how to build the files listed in $(OBJECTS).
$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(INCDIR)/graph.hpp | $(OBJDIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR) -c $< -o $@

$(OBJECTS2): $(OBJDIR2)/%.o: $(SRCDIR2)/%.cpp $(INCDIR2)/graph.hpp | $(OBJDIR2)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -I$(INCDIR2) -c $< -o $@

# Rule to create the object directory
$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR2):
	@mkdir -p $(OBJDIR2)

# Rule to clean up build files
.PHONY: clean
clean:
	@echo "Cleaning up..."
	@rm -rf $(OBJDIR) $(EXEC)
	@rm -rf $(OBJDIR2) $(EXEC2)