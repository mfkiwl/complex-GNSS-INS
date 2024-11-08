# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -march=native
DEBUG_FLAGS = -g -DDEBUG

# Threading flags
THREAD_FLAGS = -pthread

# Eigen library flags
EIGEN_INCLUDE = -I/usr/include/eigen3

# Include directories
INCLUDES = $(EIGEN_INCLUDE)

# Source files
SRCS = main.cpp data_logger.cpp logger.cpp
HEADERS = data_logger.hpp logger.hpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Output binary
TARGET = uav_navigation

# Data directory for outputs
DATA_DIR = data

# Installation directory
INSTALL_DIR = /usr/local/bin

# Build type
BUILD_TYPE ?= release

ifeq ($(BUILD_TYPE),debug)
    CXXFLAGS += $(DEBUG_FLAGS)
    TARGET := $(TARGET)_debug
endif

# Required tools
REQUIRED_TOOLS = gnuplot

# Create directories for dependencies and data
$(shell mkdir -p deps)
$(shell mkdir -p $(DATA_DIR))

# Define make_depend function
define make_depend
    @$(CXX) $(CXXFLAGS) $(INCLUDES) -MM $< > deps/$*.d
endef

# Default target
all: check-requirements $(TARGET)

# Linking
$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)..."
	@$(CXX) $(OBJS) -o $(TARGET) $(THREAD_FLAGS)
	@echo "Build complete!"

# Compilation
%.o: %.cpp $(HEADERS)
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

# Clean data files
clean-data:
	@echo "Cleaning data files..."
	@rm -f $(DATA_DIR)/*.csv $(DATA_DIR)/*.png navigation_report.txt plot_navigation.gnuplot
	@echo "Data files cleaned!"

# Deep clean
distclean: clean clean-data
	@echo "Deep cleaning..."
	@rm -f *~ *.bak
	@echo "Deep clean complete!"

# Install
install: $(TARGET)
	@echo "Installing $(TARGET)..."
	@install -m 755 $(TARGET) $(INSTALL_DIR)
	@echo "Installation complete!"

# Uninstall
uninstall:
	@echo "Uninstalling $(TARGET)..."
	@rm -f $(INSTALL_DIR)/$(TARGET)
	@echo "Uninstallation complete!"

# Run simulation
run: $(TARGET)
	@echo "Running navigation simulation..."
	./$(TARGET) terrain.txt

# Run tests
test: $(TARGET)
	@echo "Running tests..."
	./$(TARGET) test
	@echo "Tests complete!"

# Generate documentation
docs:
	@command -v doxygen >/dev/null 2>&1 || { echo >&2 "Doxygen not installed. Aborting."; exit 1; }
	@echo "Generating documentation..."
	@doxygen Doxyfile
	@echo "Documentation generated!"

# Check requirements
check-requirements:
	@echo "Checking build requirements..."
	@command -v $(CXX) >/dev/null 2>&1 || { echo >&2 "$(CXX) not installed. Aborting."; exit 1; }
	@echo "Checking for Eigen..."
	@test -d /usr/include/eigen3 || { echo >&2 "Eigen headers not found. Please install libeigen3-dev"; exit 1; }
	@echo "Checking for required tools..."
	@for tool in $(REQUIRED_TOOLS); do \
		command -v $$tool >/dev/null 2>&1 || { echo >&2 "$$tool not found. Please install it."; exit 1; }; \
	done
	@echo "All requirements satisfied."

# Create data directory
create-data-dir:
	@mkdir -p $(DATA_DIR)

# Help target
help:
	@echo "Available targets:"
	@echo "  all        - Build the project (default)"
	@echo "  clean      - Remove build files"
	@echo "  clean-data - Remove generated data files"
	@echo "  distclean  - Remove build files, backup files, and data"
	@echo "  install    - Install the program to $(INSTALL_DIR)"
	@echo "  uninstall  - Remove the installed program"
	@echo "  run        - Run the simulation with default route"
	@echo "  test       - Run tests"
	@echo "  docs       - Generate documentation"
	@echo ""
	@echo "Build options:"
	@echo "  BUILD_TYPE=release (default) or debug"
	@echo ""
	@echo "Example usage:"
	@echo "  make BUILD_TYPE=debug"
	@echo "  make run"
	@echo "  make clean-data"

# Dependencies handling
deps/%.d: ;
.PRECIOUS: deps/%.d

# Phony targets
.PHONY: all clean clean-data distclean install uninstall run test docs help check-requirements create-data-dir