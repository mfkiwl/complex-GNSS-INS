# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -march=native
DEBUG_FLAGS = -g -DDEBUG

# Threading flags
THREAD_FLAGS = -pthread

# Eigen library flags (might need adjustment based on your system)
EIGEN_INCLUDE = -I/usr/include/eigen3

# Include directories
INCLUDES = $(EIGEN_INCLUDE)

# Source files
SRCS = main.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Output binary
TARGET = uav_navigation

# Installation directory
INSTALL_DIR = /usr/local/bin

# Build type
BUILD_TYPE ?= release

ifeq ($(BUILD_TYPE),debug)
    CXXFLAGS += $(DEBUG_FLAGS)
    TARGET := $(TARGET)_debug
endif

# Create directory for dependencies
$(shell mkdir -p deps)

# Define make_depend function
define make_depend
    @$(CXX) $(CXXFLAGS) $(INCLUDES) -MM $< > deps/$*.d
endef

# Default target
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)..."
	@$(CXX) $(OBJS) -o $(TARGET) $(THREAD_FLAGS)
	@echo "Build complete!"

# Compilation
%.o: %.cpp
	@echo "Compiling $<..."
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@
	$(call make_depend)

# Include dependency files
-include $(OBJS:.o=.d)

# Clean build files
clean:
	@echo "Cleaning build files..."
	@rm -f $(OBJS) $(TARGET) deps/*.d
	@rm -rf deps
	@echo "Clean complete!"

# Deep clean (including backup files)
distclean: clean
	@echo "Deep cleaning..."
	@rm -f *~ *.bak
	@echo "Deep clean complete!"

# Install the program
install: $(TARGET)
	@echo "Installing $(TARGET)..."
	@install -m 755 $(TARGET) $(INSTALL_DIR)
	@echo "Installation complete!"

# Uninstall the program
uninstall:
	@echo "Uninstalling $(TARGET)..."
	@rm -f $(INSTALL_DIR)/$(TARGET)
	@echo "Uninstallation complete!"

# Run tests (if available)
test: $(TARGET)
	@echo "Running tests..."
	./$(TARGET) test
	@echo "Tests complete!"

# Generate documentation using Doxygen (if available)
docs:
	@command -v doxygen >/dev/null 2>&1 || { echo >&2 "Doxygen not installed. Aborting."; exit 1; }
	@echo "Generating documentation..."
	@doxygen Doxyfile
	@echo "Documentation generated!"

# Help target
help:
	@echo "Available targets:"
	@echo "  all        - Build the project (default)"
	@echo "  clean      - Remove build files"
	@echo "  distclean  - Remove build files and backup files"
	@echo "  install    - Install the program to $(INSTALL_DIR)"
	@echo "  uninstall  - Remove the installed program"
	@echo "  test       - Run tests"
	@echo "  docs       - Generate documentation"
	@echo ""
	@echo "Build options:"
	@echo "  BUILD_TYPE=release (default) or debug"
	@echo ""
	@echo "Example usage:"
	@echo "  make BUILD_TYPE=debug"
	@echo "  make install"

# Build requirements check
check-requirements:
	@echo "Checking build requirements..."
	@command -v $(CXX) >/dev/null 2>&1 || { echo >&2 "$(CXX) not installed. Aborting."; exit 1; }
	@echo "Checking for Eigen..."
	@test -d /usr/include/eigen3 || { echo >&2 "Eigen headers not found. Please install libeigen3-dev"; exit 1; }

# Phony targets
.PHONY: all clean distclean install uninstall test docs help check-requirements