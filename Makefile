CXX = g++
CXXFLAGS = -std=c++17 -I.
SRCS = $(wildcard src/*.cpp)
OBJS = $(SRCS:.cpp=.o)
TEST_SRCS = $(wildcard tests/*.cpp)
TEST_TARGETS = $(TEST_SRCS:.cpp=)

all: $(TEST_TARGETS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%: %.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $< $(OBJS) -o $@

clean:
	rm -f $(OBJS) $(TEST_TARGETS) 