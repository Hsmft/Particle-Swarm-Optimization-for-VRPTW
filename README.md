# Particle Swarm Optimization for the Vehicle Routing Problem with Time Windows (VRPTW)

![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)

This repository contains a C++ implementation of the **Particle Swarm Optimization (PSO)** metaheuristic to solve the **Vehicle Routing Problem with Time Windows (VRPTW)**. The algorithm aims to find near-optimal solutions by minimizing the number of vehicles and the total travel distance.

---

## 📋 Project Overview

This solver uses the Particle Swarm Optimization algorithm, a population-based stochastic optimization technique inspired by the social behavior of bird flocking or fish schooling.

The solver implements the following workflow:

1.  **Particle Representation & Decoding:**
    * Each particle in the swarm represents a potential solution. A particle's "position" is a vector of continuous values.
    * The **Smallest Position Value (SPV)** rule is used to decode this continuous position vector into a discrete permutation of customers, which represents the order of service.

2.  **Fitness Evaluation:**
    * The fitness of each particle is evaluated by reconstructing a set of feasible routes from its decoded customer sequence.
    * The objective function aims to minimize the number of vehicles first, then the total travel distance.

3.  **Swarm Intelligence & Movement:**
    * The swarm is initialized with a diverse set of particles generated using greedy, heuristic, and random strategies.
    * In each iteration, every particle updates its velocity and position based on two key factors:
        * Its own best-known position (`pBest`).
        * The best-known position in the entire swarm (`gBest`).
    * This process guides the swarm towards promising areas of the solution space.

---

## 🛠️ Technologies Used

* **Language:** C++ (utilizing C++11 features like `<chrono>` and `<random>`)
* **Libraries:** C++ Standard Library only. No external optimization libraries were used.

---

## 🚀 How to Compile and Run

### Compilation
You can compile the source code using a standard C++ compiler like g++.

```bash
g++ -std=c++11 -o solver PSO.cpp
```

### Execution
The program is run from the command line with the following arguments:

```bash
./solver [instance-file-path] [max-execution-time] [max-evaluations]
```
* `instance-file-path`: The path to the problem instance file (e.g., `instances/C101.txt`).
* `max-execution-time`: The maximum run time in seconds. Use `0` for no time limit.
* `max-evaluations`: The maximum number of objective function evaluations. Use `0` for no limit.

**Example:**
```bash
./solver instances/C101.txt 60 0
```
This command runs the solver on the `C101.txt` instance for a maximum of 60 seconds.

---

## 📄 License
This project is licensed under the custom **Hesameddin Fathi Non‑Commercial & Academic Co‑Authorship License 1.0**. Please see the [LICENSE](./LICENSE) file for full details.
