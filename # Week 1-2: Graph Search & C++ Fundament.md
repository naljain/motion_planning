# Week 1-2: Graph Search & C++ Fundamentals
## Implementation Challenges

This document is structured to help you learn by doing. First, you'll see what to implement with hints, then you can check your work against the provided solutions.

---

## Part 1: Conceptual Foundations

[Same as before - Configuration Space, Graph Search algorithms explanation]

### Configuration Space (C-space)

**What is Configuration Space?**

Configuration space is the set of all possible configurations of a robot. A configuration uniquely specifies the position of every point on the robot.

**Examples:**
- **Point robot in 2D**: q = (x, y) → 2D C-space
- **Point robot in 3D**: q = (x, y, z) → 3D C-space  
- **Car (x, y, θ)**: Position + heading → 3D C-space
- **2-link arm**: q = (θ₁, θ₂) → 2D C-space
- **6-DOF robot arm**: q = (θ₁, θ₂, θ₃, θ₄, θ₅, θ₆) → 6D C-space

**Why use C-space?**

Planning in C-space transforms the robot from a complex geometric shape into a point. This simplifies collision checking:
- **Workspace**: "Does this robot shape intersect obstacles?"
- **C-space**: "Is this configuration in the free space?"

### C-obstacles

When we map workspace obstacles into C-space, we get **C-obstacles** - the set of configurations where the robot collides.

**Free space**: C_free = C_space \ C_obstacles

### Graph Search Overview

**Problem**: Find path from q_start to q_goal in C_free

**Solution**: Discretize C-space into a graph
- **Nodes** = Configurations (grid cells)
- **Edges** = Valid transitions (collision-free)
- **Weights** = Cost of transition (distance, time, energy)

**Then**: Use graph search algorithms!

---

## Graph Search Algorithms

### Dijkstra's Algorithm

**Idea**: Always expand the node with lowest cost-so-far

**Algorithm:**
```
1. Initialize: g(start) = 0, g(all others) = ∞
2. Priority queue: [(0, start)]
3. While queue not empty:
   a. Pop node with lowest g-value
   b. If goal reached: reconstruct path, done
   c. For each neighbor:
      - Compute tentative g = g(current) + cost(current, neighbor)
      - If tentative g < g(neighbor):
          Update g(neighbor)
          Add to queue
```

**Properties:**
- ✅ Complete and optimal
- ❌ Explores equally in all directions (no goal bias)

**Time**: O((|V| + |E|) log |V|) with binary heap  

### A* Search

**Key Insight**: Use heuristic to guide search toward goal!

**Cost function**: f(n) = g(n) + h(n)
- g(n) = cost from start to n (known)
- h(n) = estimated cost from n to goal (heuristic)
- f(n) = estimated total cost through n

**Heuristic requirements:**
- **Admissible**: Never overestimates (h(n) ≤ h*(n))
- **Consistent**: h(n) ≤ cost(n, n') + h(n') for all neighbors n'

If heuristic is admissible, A* is optimal!

**Common heuristics:**
- **Euclidean**: √((x₁-x₂)² + (y₁-y₂)²) - admissible for any path
- **Manhattan**: |x₁-x₂| + |y₁-y₂| - admissible for 4-connected grid

**Why A* is better:**
Dijkstra explores everywhere. A* focuses on promising directions!

---

## Part 2: C++ Fundamentals

Before jumping into implementation, you need to understand these C++ concepts. Study this section carefully!

### Namespaces

**Why?** Avoid name collisions in large projects

```cpp
namespace planner {
    class Grid { /* ... */ };
    void plan() { /* ... */ }
}

namespace visualization {
    class Grid { /* ... */ };  // Different Grid!
}

// Usage
planner::Grid grid;
visualization::Grid viz_grid;

// Or use "using"
using namespace planner;
Grid grid;  // Now refers to planner::Grid
```

### Header Guards

**Problem:** If header included multiple times, get redefinition errors

```cpp
// grid.h
#ifndef PLANNER_GRID_H
#define PLANNER_GRID_H

class Grid {
    // ...
};

#endif  // PLANNER_GRID_H
```

**Modern alternative:** `#pragma once` (not standard but widely supported)
```cpp
// grid.h
#pragma once

class Grid {
    // ...
};
```

### Struct vs Class

In C++, `struct` and `class` are identical except:
- **struct**: Default public
- **class**: Default private

```cpp
struct Point {
    int x, y;  // Public by default
};

class Point {
    int x, y;  // Private by default
public:
    int get_x() const { return x; }
};
```

**Convention**: 
- Use `struct` for plain data (no invariants)
- Use `class` for objects with behavior

### Const Correctness

```cpp
class Grid {
public:
    // Const member function: promises not to modify object
    bool is_obstacle(int x, int y) const {
        return grid_[y][x];  // Can only read
    }
    
    // Non-const: can modify
    void set_obstacle(int x, int y, bool value) {
        grid_[y][x] = value;
    }
    
private:
    std::vector<std::vector<bool>> grid_;
};

// Usage
const Grid& grid = ...;
grid.is_obstacle(0, 0);  // ✅ OK
grid.set_obstacle(0, 0, true);  // ❌ Error: can't call non-const on const object
```

**Why const?**
- Documents intent (this function won't modify state)
- Compiler can optimize better
- Catches bugs at compile time

### References vs Pointers

```cpp
int value = 100;

// POINTER
int* ptr;           // Uninitialized pointer (dangerous!)
ptr = &value;       // Assign address of value
int* ptr2 = &value; // Initialize at declaration

// REFERENCE  
int& ref = value;   // MUST initialize at declaration
int& ref2;          // ❌ ERROR: references must be initialized

// Using them
std::cout << *ptr;   // Dereference with *
std::cout << ref;    // No special syntax needed!

// Modifying
*ptr = 30;           // value is now 30 (through pointer)
ref = 40;            // value is now 40 (through reference)
```

**Key differences:**

| Feature | Pointer | Reference |
|---------|---------|-----------|
| Can be null | ✅ Yes | ❌ No |
| Can be reassigned | ✅ Yes | ❌ No |
| Must initialize | ❌ No | ✅ Yes |
| Syntax for access | `*ptr` or `ptr->` | Just use it |

**When to use:**
- **Reference**: Default choice (safer, cleaner)
- **Pointer**: When you need nullptr or rebinding

### Smart Pointers

**Problem with raw pointers:**
```cpp
void bad_code() {
    int* ptr = new int(42);
    // ... use ptr ...
    // Forgot to delete! Memory leak!
}
```

**Solution: Smart pointers (RAII)**

#### std::unique_ptr - Unique ownership

```cpp
#include <memory>

void good_code() {
    std::unique_ptr<int> ptr = std::make_unique<int>(42);
    // Use ptr
}  // Automatically deleted here!

// Can't copy (unique ownership)
std::unique_ptr<int> p1 = std::make_unique<int>(5);
std::unique_ptr<int> p2 = p1;  // ❌ Error

// But can move (transfer ownership)
std::unique_ptr<int> p2 = std::move(p1);  // ✅ OK
// Now p1 is nullptr, p2 owns the memory
```

#### std::shared_ptr - Shared ownership

```cpp
std::shared_ptr<int> p1 = std::make_shared<int>(42);
std::shared_ptr<int> p2 = p1;  // ✅ OK: both own it

// Reference counting
std::cout << p1.use_count() << std::endl;  // 2

p2.reset();  // p2 releases ownership
// p1 still owns it, so not deleted yet

// When last shared_ptr goes out of scope → deleted
```

#### std::weak_ptr - Weak reference

```cpp
std::shared_ptr<int> shared = std::make_shared<int>(42);
std::weak_ptr<int> weak = shared;

// weak_ptr doesn't increase reference count
std::cout << shared.use_count() << std::endl;  // 1

// To use, must convert to shared_ptr
if (auto temp_shared = weak.lock()) {
    std::cout << *temp_shared << std::endl;
} else {
    std::cout << "Object was deleted\n";
}
```

**When to use:**
- **unique_ptr**: Default choice (95% of cases)
- **shared_ptr**: When multiple owners needed
- **weak_ptr**: Break circular references

### STL Containers

#### std::vector - Dynamic array

```cpp
#include <vector>

std::vector<int> vec;
vec.push_back(1);      // Add element
vec.push_back(2);

vec[0] = 10;           // Access (no bounds check)
vec.at(1) = 20;        // Access (with bounds check)

vec.size();            // Number of elements
vec.capacity();        // Allocated space
vec.reserve(100);      // Pre-allocate space
vec.clear();           // Remove all elements

// Iteration
for (int x : vec) {
    std::cout << x << " ";
}

for (size_t i = 0; i < vec.size(); ++i) {
    std::cout << vec[i] << " ";
}
```

**Performance:**
- Random access: O(1)
- Push back: O(1) amortized
- Insert/delete middle: O(n)

#### std::unordered_map - Hash table

```cpp
#include <unordered_map>

std::unordered_map<std::string, int> scores;
scores["Alice"] = 100;
scores["Bob"] = 95;

// Check if key exists
if (scores.find("Alice") != scores.end()) {
    std::cout << scores["Alice"] << std::endl;
}

// Or use count
if (scores.count("Charlie") > 0) {
    // key exists
}

// Iterate
for (const auto& [name, score] : scores) {
    std::cout << name << ": " << score << std::endl;
}
```

**Performance:**
- Insert: O(1) average
- Lookup: O(1) average
- Requires hashable keys

#### std::priority_queue - Binary heap

```cpp
#include <queue>

// Max heap by default
std::priority_queue<int> max_heap;
max_heap.push(3);
max_heap.push(1);
max_heap.push(4);

max_heap.top();   // 4
max_heap.pop();   // Remove 4

// Min heap (for Dijkstra/A*)
std::priority_queue<int, std::vector<int>, std::greater<int>> min_heap;
min_heap.push(3);
min_heap.push(1);
min_heap.push(4);

min_heap.top();   // 1
```

**Important for graph search:** You'll need min-heap for Dijkstra/A*!

### Templates

**Function template:**
```cpp
template<typename T>
T max(T a, T b) {
    return (a > b) ? a : b;
}

int x = max(5, 3);           // T = int
double y = max(3.14, 2.71);  // T = double
```

**Class template:**
```cpp
template<typename T>
class Stack {
public:
    void push(const T& value) {
        data_.push_back(value);
    }
    
    T pop() {
        T value = data_.back();
        data_.pop_back();
        return value;
    }
    
private:
    std::vector<T> data_;
};

Stack<int> int_stack;
Stack<std::string> string_stack;
```

### Lambda Functions

```cpp
// Basic lambda
auto add = [](int a, int b) { return a + b; };

// Capture by value
int threshold = 10;
auto filter = [threshold](int x) { return x > threshold; };

// Capture by reference
std::vector<int> results;
auto process = [&results](int x) { results.push_back(x * 2); };

// For priority queue comparison
auto compare = [](const std::pair<double, Cell>& a, 
                  const std::pair<double, Cell>& b) {
    return a.first > b.first;  // Min-heap
};
```

**You'll use lambdas for:** Custom comparators in priority queues!

---

## Part 3: Implementation Challenges

Now it's time to implement! Work through these challenges in order. After each one, you can check the solution code at the bottom of this document.

### Project Structure

First, create this directory structure:

```
planner/
├── CMakeLists.txt
├── include/
│   └── planner/
│       ├── types.h
│       ├── grid_world.h
│       └── graph_search.h
├── src/
│   ├── grid_world.cpp
│   ├── graph_search.cpp
│   └── main.cpp
└── tests/
    └── test_planner.cpp
```

---

## Challenge 1: Basic Types (types.h)

**What to implement:**

Create a `Cell` struct to represent a grid cell with x, y coordinates.

**Requirements:**
1. Two integer fields: `x` and `y`
2. Default constructor that initializes to (0, 0)
3. Constructor that takes x and y parameters
4. Equality operator (`operator==`) to compare two cells
5. Output operator (`operator<<`) to print as "(x, y)"

**Also implement these heuristic functions:**
1. `euclidean_distance(Cell a, Cell b)` - returns Euclidean distance
2. `manhattan_distance(Cell a, Cell b)` - returns Manhattan distance

**Advanced (required for A*):**
Create a hash function for Cell so it can be used in `std::unordered_map`:
- Specialize `std::hash<planner::Cell>`
- Use a simple hash like: `h1 ^ (h2 << 1)` where h1 and h2 are hashes of x and y

**Hints:**
- Use a namespace called `planner`
- Remember to include `<cmath>` for `std::sqrt()`
- The hash specialization goes in the `std` namespace, not `planner`
- Use `#pragma once` as header guard

**Test your implementation:**
```cpp
Cell a(0, 0);
Cell b(3, 4);
std::cout << a << " to " << b << "\n";
std::cout << "Euclidean: " << euclidean_distance(a, b) << "\n";
std::cout << "Manhattan: " << manhattan_distance(a, b) << "\n";

// Should work in hash map
std::unordered_map<Cell, int> map;
map[a] = 10;
```

---

## Challenge 2: Grid World (grid_world.h and grid_world.cpp)

**What to implement:**

Create a `GridWorld` class that represents a 2D grid with obstacles.

**Requirements:**

**Constructor:**
- `GridWorld(int width, int height)` - creates a grid with all cells free

**Public methods:**
1. `void set_obstacle(int x, int y)` - mark a cell as obstacle
2. `void clear_obstacle(int x, int y)` - mark a cell as free
3. `bool is_obstacle(int x, int y) const` - check if cell is obstacle
4. `bool is_valid(const Cell& cell) const` - check if cell is in bounds and not obstacle
5. `std::vector<Cell> get_neighbors(const Cell& cell, bool eight_connected = true) const`
   - Return all valid neighbors of a cell
   - If `eight_connected = true`: return up to 8 neighbors (including diagonals)
   - If `eight_connected = false`: return up to 4 neighbors (only cardinal directions)
6. `double transition_cost(const Cell& from, const Cell& to) const`
   - Return the Euclidean distance between adjacent cells
   - Diagonal moves cost √2 ≈ 1.414, cardinal moves cost 1.0
7. `int width() const` - return width
8. `int height() const` - return height

**Private data:**
- Store width and height as integers
- Store obstacles as `std::vector<std::vector<bool>>` (2D array)
- Helper function: `bool in_bounds(int x, int y) const`

**Error handling:**
- Throw `std::invalid_argument` if width or height are ≤ 0
- Throw `std::out_of_range` if trying to access out-of-bounds cell

**Hints:**
- For `get_neighbors`, use direction arrays:
  ```cpp
  const int dx4[] = {0, 0, 1, -1};
  const int dy4[] = {1, -1, 0, 0};
  ```
- Out-of-bounds cells should be treated as obstacles
- Use `reserve()` on the neighbors vector for efficiency
- For transition cost, calculate Euclidean distance: √(dx² + dy²)

**Test your implementation:**
```cpp
GridWorld world(10, 10);

// Add wall
for (int y = 0; y < 10; y++) {
    world.set_obstacle(5, y);
}

// Check cells
std::cout << world.is_obstacle(5, 5) << "\n";  // true
std::cout << world.is_valid(Cell(5, 5)) << "\n";  // false
std::cout << world.is_valid(Cell(4, 5)) << "\n";  // true

// Get neighbors
auto neighbors = world.get_neighbors(Cell(4, 5));
std::cout << "Neighbors: " << neighbors.size() << "\n";
```

---

## Challenge 3: Graph Search Base Class (graph_search.h)

**What to implement:**

Create a base class for planners and a result structure.

**PlanResult struct:**
```cpp
struct PlanResult {
    bool success;
    std::vector<Cell> path;
    double cost;
    int nodes_expanded;
};
```

**GraphSearchPlanner base class:**
- Constructor: `GraphSearchPlanner(const GridWorld& world)`
- Pure virtual method: `virtual PlanResult plan(const Cell& start, const Cell& goal) = 0`
- Protected helper: `std::vector<Cell> reconstruct_path(...)`
  - Takes a `came_from` map and start/goal cells
  - Returns the path from start to goal
  - Remember to reverse it!

**Hints:**
- Store a const reference to the GridWorld: `const GridWorld& world_;`
- Pure virtual function uses `= 0` at the end
- Make destructor virtual: `virtual ~GraphSearchPlanner() = default;`
- `reconstruct_path` should walk backward from goal to start using parent pointers

---

## Challenge 4: Dijkstra's Algorithm (graph_search.cpp)

**What to implement:**

Create a `DijkstraPlanner` class that inherits from `GraphSearchPlanner`.

**Algorithm to implement:**
1. Create a priority queue that stores (cost, Cell) pairs, ordered by cost (min-heap)
2. Create a map to store g-scores (cost from start): `std::unordered_map<Cell, double>`
3. Create a map to store parent pointers: `std::unordered_map<Cell, Cell>`
4. Initialize: set g(start) = 0, add (0, start) to queue
5. Loop while queue not empty:
   - Pop cell with lowest cost
   - If it's the goal, reconstruct path and return success
   - For each neighbor:
     - Calculate tentative_g = g(current) + transition_cost(current, neighbor)
     - If tentative_g is better than current g(neighbor):
       - Update came_from[neighbor] = current
       - Update g_score[neighbor] = tentative_g
       - Add (tentative_g, neighbor) to queue

**Tricky parts:**

**Priority Queue:**
By default, `std::priority_queue` is a max-heap. You need a min-heap:
```cpp
auto compare = [](const std::pair<double, Cell>& a, 
                  const std::pair<double, Cell>& b) {
    return a.first > b.first;  // Note: > for min-heap
};

std::priority_queue<std::pair<double, Cell>,
                    std::vector<std::pair<double, Cell>>,
                    decltype(compare)> open_set(compare);
```

**Handling duplicates:**
The queue might contain outdated entries. Skip them:
```cpp
if (current_cost > g_score[current]) {
    continue;  // Found a better path already
}
```

**Checking if key exists in map:**
```cpp
if (g_score.find(neighbor) == g_score.end() || 
    tentative_g < g_score[neighbor]) {
    // Update
}
```

**Hints:**
- Track `nodes_expanded` to see how many nodes you visited
- Initialize g_score with infinity for unvisited nodes (or use `.find()`)
- Don't forget to increment nodes_expanded each time you pop from queue

**Test your implementation:**
```cpp
GridWorld world(20, 20);
// Add some obstacles
DijkstraPlanner planner(world);

Cell start(0, 0);
Cell goal(19, 19);

auto result = planner.plan(start, goal);

if (result.success) {
    std::cout << "Path found!\n";
    std::cout << "Cost: " << result.cost << "\n";
    std::cout << "Nodes: " << result.nodes_expanded << "\n";
    std::cout << "Length: " << result.path.size() << "\n";
}
```

---

## Challenge 5: A* Search (graph_search.cpp)

**What to implement:**

Create an `AStarPlanner` class similar to Dijkstra, but with a heuristic.

**Differences from Dijkstra:**
1. Store f_scores in addition to g_scores: `f(n) = g(n) + h(n)`
2. Priority queue orders by f-score instead of g-score
3. Take a heuristic function as a parameter

**Class definition:**
```cpp
class AStarPlanner : public GraphSearchPlanner {
public:
    using HeuristicFunc = std::function<double(const Cell&, const Cell&)>;
    
    AStarPlanner(const GridWorld& world, 
                 HeuristicFunc heuristic = euclidean_distance);
    
    PlanResult plan(const Cell& start, const Cell& goal) override;
    
private:
    HeuristicFunc heuristic_;
};
```

**Algorithm:**
Same as Dijkstra, but:
- Priority queue uses f-score: `f = g + h(cell, goal)`
- When updating a neighbor:
  ```cpp
  g_score[neighbor] = tentative_g;
  f_score[neighbor] = tentative_g + heuristic_(neighbor, goal);
  open_set.push({f_score[neighbor], neighbor});
  ```

**Hints:**
- Still return the g_score (actual cost) in result, not f_score
- The heuristic guides search but doesn't affect path cost
- Test with different heuristics (Euclidean vs Manhattan)

**Test your implementation:**
```cpp
GridWorld world(30, 20);
// Add obstacles

DijkstraPlanner dijkstra(world);
AStarPlanner astar(world, euclidean_distance);

auto d_result = dijkstra.plan(start, goal);
auto a_result = astar.plan(start, goal);

std::cout << "Dijkstra expanded: " << d_result.nodes_expanded << "\n";
std::cout << "A* expanded: " << a_result.nodes_expanded << "\n";
// A* should expand fewer nodes!
```

---

## Challenge 6: Visualization (main.cpp)

**What to implement:**

Create a function to visualize the path on the grid.

**Requirements:**
```cpp
void visualize_path(const GridWorld& world, 
                   const PlanResult& result,
                   const Cell& start, 
                   const Cell& goal);
```

**Output format:**
- `.` = free cell
- `#` = obstacle
- `*` = path
- `S` = start
- `G` = goal

**Example output:**
```
S * * . . . . . . . . . . . . . . . . .
. . * . . . . . . . . . . . . . . . . .
. . * . . . . . . . . . . . . . . . . .
. . * * * . . . . . . . . . . . . . . .
. . . . * . . . . . . . . . . . . . . .
. . . . * # # # # # . . . . . . . . . .
. . . . * # # # # # . . . . . . . . . .
. . . . * * * * * * * . . . . . . . . .
. . . . . . . . . . * . . . . . . . . .
. . . . . . . . . . * * * * * * * * * G
```

**Hints:**
- Create a 2D vector to store display characters
- First mark obstacles, then path, then start/goal
- Use nested loops to print

---

## Challenge 7: CMakeLists.txt

**What to implement:**

Create a CMake build system.

**Requirements:**
1. Minimum CMake version 3.14
2. Project name: `motion_planner`
3. C++17 standard
4. Create a library called `planner_lib` with:
   - `src/grid_world.cpp`
   - `src/graph_search.cpp`
5. Create an executable called `planner_demo` that links to `planner_lib`
6. Include directory: `include/`
7. Add compiler warnings (`-Wall -Wextra -Wpedantic`)

**Bonus:** Add Google Test for unit testing

**Hints:**
```cmake
cmake_minimum_required(VERSION 3.14)
project(motion_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add library...
# Add executable...
# Link them together...
```

---

## Challenge 8: Unit Tests

**What to implement:**

Write unit tests for your implementation using Google Test.

**Tests to write:**

1. **GridWorld tests:**
   - Constructor creates correct size
   - Invalid dimensions throw exception
   - Setting/clearing obstacles works
   - Boundary checking works correctly
   - Neighbor count is correct (4 vs 8 connected)

2. **Dijkstra tests:**
   - Finds path in empty grid
   - Path cost is correct
   - Finds path around obstacle
   - Returns failure when no path exists
   - Path is collision-free

3. **A* tests:**
   - Finds path with correct cost
   - Expands fewer nodes than Dijkstra
   - Works with different heuristics

**Example test structure:**
```cpp
TEST(GridWorldTest, ConstructorSetsSize) {
    GridWorld world(10, 20);
    EXPECT_EQ(world.width(), 10);
    EXPECT_EQ(world.height(), 20);
}

TEST(DijkstraTest, FindsPathInEmptyGrid) {
    GridWorld world(10, 10);
    DijkstraPlanner planner(world);
    
    auto result = planner.plan(Cell(0, 0), Cell(9, 9));
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 0);
}
```

---

## Building and Running Your Code

Once you've implemented everything:

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake ..

# Build
cmake --build .

# Run demo
./planner_demo

# Run tests (if you added them)
ctest --verbose
```

---

## Debugging Tips

If something isn't working:

1. **Compilation errors:**
   - Check include guards
   - Make sure all headers are included
   - Check namespace usage

2. **Linker errors:**
   - Make sure CMakeLists.txt includes all .cpp files
   - Check that you're linking libraries correctly

3. **Runtime crashes:**
   - Add `std::cout` statements to trace execution
   - Check for out-of-bounds access
   - Make sure pointers aren't null
   - Use a debugger!

4. **Wrong results:**
   - Print the path to see if it makes sense
   - Check that costs are calculated correctly
   - Verify neighbor generation
   - Make sure priority queue is min-heap

---

# SOLUTIONS BELOW

## ⚠️ Try to implement everything above first! ⚠️

Scroll down only after you've attempted the challenges yourself.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

.

---

# Solution Code

## Solution 1: Basic Types (types.h)

```cpp
#pragma once
#include <iostream>
#include <cmath>

namespace planner {

// 2D grid cell / configuration
struct Cell {
    int x;
    int y;
    
    Cell() : x(0), y(0) {}
    Cell(int x_, int y_) : x(x_), y(y_) {}
    
    // Equality comparison (needed for hash maps)
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
    
    // For printing
    friend std::ostream& operator<<(std::ostream& os, const Cell& c) {
        return os << "(" << c.x << ", " << c.y << ")";
    }
};

// Heuristic functions
inline double euclidean_distance(const Cell& a, const Cell& b) {
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double manhattan_distance(const Cell& a, const Cell& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

} // namespace planner

// Hash function for Cell (needed for std::unordered_map)
namespace std {
    template<>
    struct hash<planner::Cell> {
        size_t operator()(const planner::Cell& c) const {
            // Cantor pairing function
            size_t h1 = std::hash<int>{}(c.x);
            size_t h2 = std::hash<int>{}(c.y);
            return h1 ^ (h2 << 1);
        }
    };
}
```

---

## Solution 2: Grid World (grid_world.h)

```cpp
#pragma once
#include "types.h"
#include <vector>
#include <stdexcept>

namespace planner {

class GridWorld {
public:
    GridWorld(int width, int height);
    
    // Set/clear obstacles
    void set_obstacle(int x, int y);
    void clear_obstacle(int x, int y);
    
    // Query functions
    bool is_obstacle(int x, int y) const;
    bool is_valid(const Cell& cell) const;
    
    // Get valid neighbors (4-connected or 8-connected)
    std::vector<Cell> get_neighbors(const Cell& cell, 
                                     bool eight_connected = true) const;
    
    // Cost of transition
    double transition_cost(const Cell& from, const Cell& to) const;
    
    // Dimensions
    int width() const { return width_; }
    int height() const { return height_; }
    
private:
    int width_;
    int height_;
    std::vector<std::vector<bool>> grid_;  // true = obstacle
    
    bool in_bounds(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
};

} // namespace planner
```

## Solution 2b: Grid World Implementation (grid_world.cpp)

```cpp
#include "planner/grid_world.h"

namespace planner {

GridWorld::GridWorld(int width, int height) 
    : width_(width), height_(height),
      grid_(height, std::vector<bool>(width, false)) {
    
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
}

void GridWorld::set_obstacle(int x, int y) {
    if (!in_bounds(x, y)) {
        throw std::out_of_range("Cell coordinates out of bounds");
    }
    grid_[y][x] = true;
}

void GridWorld::clear_obstacle(int x, int y) {
    if (!in_bounds(x, y)) {
        throw std::out_of_range("Cell coordinates out of bounds");
    }
    grid_[y][x] = false;
}

bool GridWorld::is_obstacle(int x, int y) const {
    if (!in_bounds(x, y)) return true;  // Out of bounds = obstacle
    return grid_[y][x];
}

bool GridWorld::is_valid(const Cell& cell) const {
    return in_bounds(cell.x, cell.y) && !is_obstacle(cell.x, cell.y);
}

std::vector<Cell> GridWorld::get_neighbors(const Cell& cell, 
                                            bool eight_connected) const {
    std::vector<Cell> neighbors;
    neighbors.reserve(eight_connected ? 8 : 4);
    
    // Direction vectors
    static const int dx4[] = {0, 0, 1, -1};
    static const int dy4[] = {1, -1, 0, 0};
    
    static const int dx8[] = {0, 0, 1, -1, 1, 1, -1, -1};
    static const int dy8[] = {1, -1, 0, 0, 1, -1, 1, -1};
    
    const int* dx = eight_connected ? dx8 : dx4;
    const int* dy = eight_connected ? dy8 : dy4;
    int count = eight_connected ? 8 : 4;
    
    for (int i = 0; i < count; ++i) {
        Cell neighbor(cell.x + dx[i], cell.y + dy[i]);
        if (is_valid(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

double GridWorld::transition_cost(const Cell& from, const Cell& to) const {
    // Euclidean distance between cells
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace planner
```

---

## Solution 3: Graph Search Base (graph_search.h)

```cpp
#pragma once
#include "types.h"
#include "grid_world.h"
#include <vector>
#include <unordered_map>
#include <functional>

namespace planner {

// Result of planning
struct PlanResult {
    bool success;
    std::vector<Cell> path;
    double cost;
    int nodes_expanded;
    
    PlanResult() : success(false), cost(0.0), nodes_expanded(0) {}
};

// Base class for planners
class GraphSearchPlanner {
public:
    explicit GraphSearchPlanner(const GridWorld& world);
    virtual ~GraphSearchPlanner() = default;
    
    // Pure virtual: must implement in derived classes
    virtual PlanResult plan(const Cell& start, const Cell& goal) = 0;
    
protected:
    const GridWorld& world_;
    
    // Helper: reconstruct path from parent pointers
    std::vector<Cell> reconstruct_path(
        const std::unordered_map<Cell, Cell>& came_from,
        const Cell& start,
        const Cell& goal) const;
};

// Dijkstra's algorithm
class DijkstraPlanner : public GraphSearchPlanner {
public:
    explicit DijkstraPlanner(const GridWorld& world);
    
    PlanResult plan(const Cell& start, const Cell& goal) override;
};

// A* search
class AStarPlanner : public GraphSearchPlanner {
public:
    using HeuristicFunc = std::function<double(const Cell&, const Cell&)>;
    
    explicit AStarPlanner(const GridWorld& world,
                          HeuristicFunc heuristic = euclidean_distance);
    
    PlanResult plan(const Cell& start, const Cell& goal) override;
    
private:
    HeuristicFunc heuristic_;
};

} // namespace planner
```

---

## Solution 4 & 5: Graph Search Implementation (graph_search.cpp)

```cpp
#include "planner/graph_search.h"
#include <queue>
#include <algorithm>
#include <limits>

namespace planner {

GraphSearchPlanner::GraphSearchPlanner(const GridWorld& world) 
    : world_(world) {}

std::vector<Cell> GraphSearchPlanner::reconstruct_path(
    const std::unordered_map<Cell, Cell>& came_from,
    const Cell& start,
    const Cell& goal) const {
    
    std::vector<Cell> path;
    Cell current = goal;
    
    while (!(current == start)) {
        path.push_back(current);
        auto it = came_from.find(current);
        if (it == came_from.end()) {
            return {};  // Path broken
        }
        current = it->second;
    }
    
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// ============================================================================
// Dijkstra's Algorithm
// ============================================================================

DijkstraPlanner::DijkstraPlanner(const GridWorld& world)
    : GraphSearchPlanner(world) {}

PlanResult DijkstraPlanner::plan(const Cell& start, const Cell& goal) {
    PlanResult result;
    
    // Priority queue: (cost, cell)
    using QueueElement = std::pair<double, Cell>;
    auto compare = [](const QueueElement& a, const QueueElement& b) {
        return a.first > b.first;  // Min heap
    };
    std::priority_queue<QueueElement,
                        std::vector<QueueElement>,
                        decltype(compare)> open_set(compare);
    
    // Cost from start
    std::unordered_map<Cell, double> g_score;
    
    // Parent pointers
    std::unordered_map<Cell, Cell> came_from;
    
    // Initialize
    open_set.push({0.0, start});
    g_score[start] = 0.0;
    
    while (!open_set.empty()) {
        auto [current_cost, current] = open_set.top();
        open_set.pop();
        
        result.nodes_expanded++;
        
        // Goal test
        if (current == goal) {
            result.success = true;
            result.cost = current_cost;
            result.path = reconstruct_path(came_from, start, goal);
            return result;
        }
        
        // Skip if we found better path
        if (current_cost > g_score[current]) {
            continue;
        }
        
        // Expand neighbors
        for (const Cell& neighbor : world_.get_neighbors(current)) {
            double tentative_g = g_score[current] + 
                                 world_.transition_cost(current, neighbor);
            
            // If better path found
            if (g_score.find(neighbor) == g_score.end() ||
                tentative_g < g_score[neighbor]) {
                
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                open_set.push({tentative_g, neighbor});
            }
        }
    }
    
    // No path found
    result.success = false;
    return result;
}

// ============================================================================
// A* Search
// ============================================================================

AStarPlanner::AStarPlanner(const GridWorld& world, HeuristicFunc heuristic)
    : GraphSearchPlanner(world), heuristic_(heuristic) {}

PlanResult AStarPlanner::plan(const Cell& start, const Cell& goal) {
    PlanResult result;
    
    // Priority queue: (f_score, cell)
    using QueueElement = std::pair<double, Cell>;
    auto compare = [](const QueueElement& a, const QueueElement& b) {
        return a.first > b.first;
    };
    std::priority_queue<QueueElement,
                        std::vector<QueueElement>,
                        decltype(compare)> open_set(compare);
    
    std::unordered_map<Cell, double> g_score;
    std::unordered_map<Cell, double> f_score;
    std::unordered_map<Cell, Cell> came_from;
    
    // Initialize
    g_score[start] = 0.0;
    f_score[start] = heuristic_(start, goal);
    open_set.push({f_score[start], start});
    
    while (!open_set.empty()) {
        auto [current_f, current] = open_set.top();
        open_set.pop();
        
        result.nodes_expanded++;
        
        if (current == goal) {
            result.success = true;
            result.cost = g_score[current];
            result.path = reconstruct_path(came_from, start, goal);
            return result;
        }
        
        // Skip if outdated
        if (current_f > f_score[current]) {
            continue;
        }
        
        for (const Cell& neighbor : world_.get_neighbors(current)) {
            double tentative_g = g_score[current] +
                                 world_.transition_cost(current, neighbor);
            
            if (g_score.find(neighbor) == g_score.end() ||
                tentative_g < g_score[neighbor]) {
                
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + heuristic_(neighbor, goal);
                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }
    
    result.success = false;
    return result;
}

} // namespace planner
```

---

## Solution 6: Visualization and Main (main.cpp)

```cpp
#include "planner/grid_world.h"
#include "planner/graph_search.h"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace planner;

// Visualize path on grid
void visualize_path(const GridWorld& world, 
                   const PlanResult& result,
                   const Cell& start, 
                   const Cell& goal) {
    
    // Create display grid
    std::vector<std::vector<char>> display(
        world.height(), std::vector<char>(world.width(), '.'));
    
    // Mark obstacles
    for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
            if (world.is_obstacle(x, y)) {
                display[y][x] = '#';
            }
        }
    }
    
    // Mark path
    for (const Cell& cell : result.path) {
        display[cell.y][cell.x] = '*';
    }
    
    // Mark start and goal
    display[start.y][start.x] = 'S';
    display[goal.y][goal.x] = 'G';
    
    // Print
    for (const auto& row : display) {
        for (char c : row) {
            std::cout << c << ' ';
        }
        std::cout << '\n';
    }
}

int main() {
    // Create 30x20 grid
    GridWorld world(30, 20);
    
    // Add vertical wall with gap
    for (int y = 5; y < 15; ++y) {
        world.set_obstacle(15, y);
    }
    world.clear_obstacle(15, 10);  // Gap
    
    Cell start(2, 10);
    Cell goal(28, 10);
    
    std::cout << "Grid: " << world.width() << "x" << world.height() << "\n";
    std::cout << "Start: " << start << "\n";
    std::cout << "Goal: " << goal << "\n\n";
    
    // ===== Dijkstra =====
    std::cout << "=== Dijkstra's Algorithm ===\n";
    DijkstraPlanner dijkstra(world);
    
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dijkstra_result = dijkstra.plan(start, goal);
    auto t2 = std::chrono::high_resolution_clock::now();
    
    if (dijkstra_result.success) {
        std::cout << "✓ Path found!\n";
        std::cout << "  Cost: " << std::fixed << std::setprecision(2) 
                  << dijkstra_result.cost << "\n";
        std::cout << "  Path length: " << dijkstra_result.path.size() << "\n";
        std::cout << "  Nodes expanded: " << dijkstra_result.nodes_expanded << "\n";
        std::cout << "  Time: " 
                  << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
                  << " μs\n\n";
        
        visualize_path(world, dijkstra_result, start, goal);
    } else {
        std::cout << "✗ No path found\n";
    }
    
    // ===== A* with Euclidean =====
    std::cout << "\n=== A* (Euclidean heuristic) ===\n";
    AStarPlanner astar_euclidean(world, euclidean_distance);
    
    t1 = std::chrono::high_resolution_clock::now();
    auto astar_result = astar_euclidean.plan(start, goal);
    t2 = std::chrono::high_resolution_clock::now();
    
    if (astar_result.success) {
        std::cout << "✓ Path found!\n";
        std::cout << "  Cost: " << std::fixed << std::setprecision(2)
                  << astar_result.cost << "\n";
        std::cout << "  Path length: " << astar_result.path.size() << "\n";
        std::cout << "  Nodes expanded: " << astar_result.nodes_expanded << "\n";
        std::cout << "  Time: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
                  << " μs\n\n";
        
        visualize_path(world, astar_result, start, goal);
    }
    
    // ===== Comparison =====
    std::cout << "\n=== Comparison ===\n";
    std::cout << "Dijkstra expanded: " << dijkstra_result.nodes_expanded << " nodes\n";
    std::cout << "A* (Euclidean): " << astar_result.nodes_expanded << " nodes\n";
    
    double efficiency = 100.0 * (1.0 - 
        static_cast<double>(astar_result.nodes_expanded) / 
        dijkstra_result.nodes_expanded);
    
    std::cout << "\nA* is " << std::fixed << std::setprecision(1)
              << efficiency << "% more efficient than Dijkstra\n";
    
    return 0;
}
```

---

## Solution 7: CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.14)
project(motion_planner VERSION 1.0 LANGUAGES CXX)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optimization flags for Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(NOT MSVC)
        add_compile_options(-O3 -march=native)
    endif()
endif()

# Library
add_library(planner_lib
    src/grid_world.cpp
    src/graph_search.cpp
)

target_include_directories(planner_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Executable
add_executable(planner_demo src/main.cpp)
target_link_libraries(planner_demo PRIVATE planner_lib)

# Google Test
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(planner_tests tests/test_planner.cpp)
target_link_libraries(planner_tests PRIVATE planner_lib gtest_main)

include(GoogleTest)
gtest_discover_tests(planner_tests)
```

---

## Solution 8: Unit Tests (tests/test_planner.cpp)

```cpp
#include "planner/grid_world.h"
#include "planner/graph_search.h"
#include <gtest/gtest.h>

using namespace planner;

// ===== GridWorld Tests =====

TEST(GridWorldTest, Construction) {
    GridWorld world(10, 10);
    EXPECT_EQ(world.width(), 10);
    EXPECT_EQ(world.height(), 10);
}

TEST(GridWorldTest, InvalidDimensions) {
    EXPECT_THROW(GridWorld(-5, 10), std::invalid_argument);
    EXPECT_THROW(GridWorld(10, 0), std::invalid_argument);
}

TEST(GridWorldTest, ObstacleManipulation) {
    GridWorld world(10, 10);
    
    EXPECT_FALSE(world.is_obstacle(5, 5));
    
    world.set_obstacle(5, 5);
    EXPECT_TRUE(world.is_obstacle(5, 5));
    
    world.clear_obstacle(5, 5);
    EXPECT_FALSE(world.is_obstacle(5, 5));
}

TEST(GridWorldTest, BoundaryChecking) {
    GridWorld world(10, 10);
    
    EXPECT_TRUE(world.is_valid(Cell(0, 0)));
    EXPECT_TRUE(world.is_valid(Cell(9, 9)));
    EXPECT_FALSE(world.is_valid(Cell(-1, 0)));
    EXPECT_FALSE(world.is_valid(Cell(0, -1)));
    EXPECT_FALSE(world.is_valid(Cell(10, 0)));
    EXPECT_FALSE(world.is_valid(Cell(0, 10)));
}

TEST(GridWorldTest, Neighbors4Connected) {
    GridWorld world(10, 10);
    Cell center(5, 5);
    
    auto neighbors = world.get_neighbors(center, false);
    EXPECT_EQ(neighbors.size(), 4);
}

TEST(GridWorldTest, Neighbors8Connected) {
    GridWorld world(10, 10);
    Cell center(5, 5);
    
    auto neighbors = world.get_neighbors(center, true);
    EXPECT_EQ(neighbors.size(), 8);
}

// ===== Dijkstra Tests =====

TEST(DijkstraTest, EmptyGridStraightLine) {
    GridWorld world(10, 10);
    DijkstraPlanner planner(world);
    
    Cell start(0, 0);
    Cell goal(9, 0);
    
    auto result = planner.plan(start, goal);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.path.size(), 10);
    EXPECT_DOUBLE_EQ(result.cost, 9.0);
}

TEST(DijkstraTest, WithObstacle) {
    GridWorld world(10, 10);
    
    // Create wall with gap
    for (int y = 0; y < 10; ++y) {
        if (y != 5) {
            world.set_obstacle(5, y);
        }
    }
    
    DijkstraPlanner planner(world);
    Cell start(0, 5);
    Cell goal(9, 5);
    
    auto result = planner.plan(start, goal);
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.size(), 10);  // Must go around
}

TEST(DijkstraTest, NoPathExists) {
    GridWorld world(10, 10);
    
    // Create impassable wall
    for (int y = 0; y < 10; ++y) {
        world.set_obstacle(5, y);
    }
    
    DijkstraPlanner planner(world);
    Cell start(0, 5);
    Cell goal(9, 5);
    
    auto result = planner.plan(start, goal);
    
    EXPECT_FALSE(result.success);
}

// ===== A* Tests =====

TEST(AStarTest, ComparisonWithDijkstra) {
    GridWorld world(30, 20);
    
    // Add obstacles
    for (int y = 5; y < 15; ++y) {
        world.set_obstacle(15, y);
    }
    world.clear_obstacle(15, 10);
    
    Cell start(2, 10);
    Cell goal(28, 10);
    
    DijkstraPlanner dijkstra(world);
    AStarPlanner astar(world, euclidean_distance);
    
    auto dijkstra_result = dijkstra.plan(start, goal);
    auto astar_result = astar.plan(start, goal);
    
    EXPECT_TRUE(dijkstra_result.success);
    EXPECT_TRUE(astar_result.success);
    
    // Same cost (both optimal)
    EXPECT_NEAR(dijkstra_result.cost, astar_result.cost, 1e-6);
    
    // A* should expand fewer nodes
    EXPECT_LT(astar_result.nodes_expanded, dijkstra_result.nodes_expanded);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

## Exercises

Now that you've implemented the basics, here are additional exercises to deepen your understanding:

### Basic Exercises

1. **Weighted A***: Implement weighted A* where f = g + ε·h (ε > 1)
   - Compare ε = 1.0, 1.5, 2.0
   - What happens to optimality? Speed?

2. **Different Connectivity**: Test 4-connected vs 8-connected grids
   - How does path cost change?
   - Which expands fewer nodes?

3. **Tie-Breaking**: When f-scores are equal, prefer cells closer to goal
   ```cpp
   // In A*, modify comparison
   if (f_a == f_b) {
       return h(a, goal) > h(b, goal);
   }
   ```

4. **Bidirectional Search**: Search from both start and goal
   - Stop when frontiers meet
   - Combine two half-paths

### Intermediate Exercises

5. **Jump Point Search (JPS)**: 
   - Skip unnecessary cells on uniform cost grids
   - Much faster than A* on open spaces

6. **Theta***: Allow any-angle paths
   - Check line-of-sight between cells
   - Don't restrict to grid edges

7. **Time-Dependent Costs**: Obstacles appear/disappear
   - Cost depends on time: c(e, t)
   - Used for traffic-aware routing

8. **Multi-Goal Planning**: Find path visiting multiple goals
   - Convert to TSP (Traveling Salesman Problem)
   - Use A* for each leg

### Advanced Exercises

9. **3D Grid Search**: Extend to 3D
   - 6-connected (faces) or 26-connected (all neighbors)
   - Visualize with voxels

10. **Anytime Weighted A***: 
    - Start with high ε, decrease over time
    - Return best path found when time runs out

11. **Memory-Bounded A* (SMA*)**: 
    - Limit memory usage
    - Forget nodes when memory full

12. **Custom Heuristic**: Design heuristic for specific map
    - Precompute distances to landmarks
    - Use as better h(n)

---

## Common Mistakes & Debugging

### Mistake 1: Forgot to Check Bounds

```cpp
// ❌ Bad: Segmentation fault
bool is_obstacle(int x, int y) {
    return grid_[y][x];  // What if x, y out of bounds?
}

// ✅ Good: Always check
bool is_obstacle(int x, int y) {
    if (!in_bounds(x, y)) return true;
    return grid_[y][x];
}
```

### Mistake 2: Using Wrong Data Structure

```cpp
// ❌ Bad: O(n) to find minimum
std::vector<std::pair<double, Cell>> open_set;

// ✅ Good: O(log n) with priority queue
std::priority_queue<...> open_set;
```

### Mistake 3: Not Handling Duplicates

```cpp
// ❌ Bad: Process same cell multiple times
while (!queue.empty()) {
    auto current = queue.top();
    queue.pop();
    // ... process ...
}

// ✅ Good: Skip if outdated
while (!queue.empty()) {
    auto [cost, current] = queue.top();
    queue.pop();
    
    if (cost > g_score[current]) continue;  // Skip!
    // ... process ...
}
```

### Mistake 4: Memory Leaks

```cpp
// ❌ Bad: Forgot to delete
Cell* cell = new Cell();

// ✅ Good: Use smart pointers
auto cell = std::make_unique<Cell>();
```

### Mistake 5: Wrong Priority Queue Comparison

```cpp
// ❌ Bad: Creates max-heap (want min-heap!)
auto compare = [](const auto& a, const auto& b) {
    return a.first < b.first;
};

// ✅ Good: Min-heap
auto compare = [](const auto& a, const auto& b) {
    return a.first > b.first;  // Note: > not <
};
```

---

## Performance Tips

1. **Reserve vector capacity**: Avoid reallocations
   ```cpp
   neighbors.reserve(8);
   ```

2. **Pass by const reference**: Avoid copies
   ```cpp
   void process(const std::vector<int>& data);  // ✅
   void process(std::vector<int> data);         // ❌ Copies!
   ```

3. **Use `emplace_back` instead of `push_back`**:
   ```cpp
   vec.emplace_back(x, y);  // Construct in-place
   vec.push_back(Cell(x, y));  // Construct then copy
   ```

4. **Profile before optimizing**: Use Valgrind, perf, gprof
   ```bash
   valgrind --tool=callgrind ./planner_demo
   ```

5. **Static const for lookup tables**:
   ```cpp
   static const int dx[] = {0, 0, 1, -1};  // Computed once
   ```

---

## Debugging Tips

### Using GDB

```bash
# Compile with debug symbols
g++ -g -O0 src/*.cpp -o planner

# Run with GDB
gdb ./planner

# Inside GDB:
(gdb) break main
(gdb) run
(gdb) next        # Step over
(gdb) step        # Step into
(gdb) print x     # Print variable
(gdb) backtrace   # Call stack
```

### Print Debugging

```cpp
// Add strategic prints
std::cout << "Exploring cell: " << current << "\n";
std::cout << "g_score: " << g_score[current] << "\n";
std::cout << "Queue size: " << open_set.size() << "\n";
```

### Common Issues

**Infinite loop:**
- Check your while condition
- Make sure queue is actually being emptied
- Verify you're not adding same cell repeatedly

**Wrong path cost:**
- Print costs as you explore
- Check transition_cost calculation
- Verify diagonal vs cardinal distances

**Crash on path reconstruction:**
- Check that came_from map is populated
- Verify start cell is in the path
- Make sure equality operator works

---

This completes Week 1-2! You now have:
- ✅ Understanding of configuration space and graph search
- ✅ C++ fundamentals (smart pointers, STL, templates)
- ✅ Working grid-based planner with A* and Dijkstra
- ✅ Testing infrastructure
- ✅ Build system (CMake)
- ✅ Exercises to extend your learning

**Next up:** Week 3-4 with RRT and performance optimization!