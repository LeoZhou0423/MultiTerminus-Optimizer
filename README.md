# MultiTerminus Optimizer 🚀

**Revolutionizing Multi-Origin/Destination Path Planning**

## 🎯 Core Innovation
Breakthrough algorithm for complex routing scenarios with **multiple starts and multiple ends** - going beyond traditional single-point limitations.

## ⚡ Technical Architecture

### Adaptive Algorithm Fusion
- **Exact Computation**: Full permutation search for small datasets (n ≤ 12)
- **Metaheuristic Hybrid**: Simulated annealing + greedy construction for large-scale problems
- **Local Optimization**: 2-opt refinement for path smoothing

### Multi-Point Global Optimization
```cpp
// Simultaneously evaluates all start-end combinations
for (all starts × all ends) {
    find_optimal_path(waypoints);
    update_global_best();
}
