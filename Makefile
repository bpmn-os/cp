# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++23 -Wall -Wextra -Werror  -fmax-errors=1

# Conditionally add LIMEX path and define if it is set
ifneq ($(LIMEX_PATH),)
    CXXFLAGS += -DUSE_LIMEX -I$(LIMEX_PATH)
endif

# Source files
SRCS = main.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Executable name
TARGET = test

# Rule to build the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Rule to compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to clean object files and executable
clean:
	rm -f $(OBJS) $(TARGET)

