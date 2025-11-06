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

# Source directory
SRCDIR = Phase-1/src

# Object directory (to store intermediate .o files)
OBJDIR = obj

# Name of the final executable
EXEC = phase1

# List of source files
SOURCES = $(SRCDIR)/main.cpp $(SRCDIR)/graph.cpp $(SRCDIR)/shortest_path.cpp

# Generate object file names from source file names
# (e.g., Phase-1/src/main.cpp -> obj/main.o)
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# Add the include directory to the compiler flags
CXXFLAGS += -I$(INCDIR)

# Default target: build the executable
.PHONY: all
all: $(EXEC)

# Rule to link the final executable
$(EXEC): $(OBJECTS)
	@echo "Linking..."
	$(CXX) $(CXXFLAGS) -o $(EXEC) $(OBJECTS)
	@echo "Build complete: ./$(EXEC)"

# *** CHANGED RULE ***
# Use a static pattern rule for more robust matching.
# This explicitly tells make how to build the files listed in $(OBJECTS).
$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(INCDIR)/graph.hpp | $(OBJDIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to create the object directory
$(OBJDIR):
	@mkdir -p $(OBJDIR)

# Rule to clean up build files
.PHONY: clean
clean:
	@echo "Cleaning up..."
	@rm -rf $(OBJDIR) $(EXEC)