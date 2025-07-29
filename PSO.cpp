#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <random>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <unordered_set>
#include <queue>
#include <map>
#include <unordered_map>
#include <cstdlib>
#include <atomic>

using namespace std;

// Global variable for controlling algorithm termination
atomic<bool> stopAlgorithm(false);
bool timeLimitMessagePrinted = false;

// Customer structure
struct Customer {
    int id;
    double x, y;
    int demand;
    double serviceTime;
    double earliest;
    double latest;
};

// Route structure
struct Route {
    vector<int> customerIds;
    double totalDistance;
    vector<double> arrivalTimes;
    vector<double> departureTimes;
    int load;
    vector<double> min_slacks; // Minimum slack time from each position to the end
    Route() : totalDistance(0.0), load(0) {}
};

// Problem data structure
struct ProblemData {
    vector<Customer> customers;
    int vehicleCapacity;
    int maxVehicles;
    double maxRouteDuration;
    vector<vector<double>> distanceMatrix;
    unordered_map<int, int> idToIndex;
};

// Particle structure for PSO
struct Particle {
    vector<double> position;   // Current position (continuous values for SPV)
    vector<double> velocity;   //velocity for each dimension
    vector<double> pBest;      //Personal best position
    pair<int, double> pBestFitness; //Fitness of personal best
    vector<Route> pBestRoutes;
};

//Helper to compare solutions
bool isBetterThan(const pair<int, double>& a, const pair<int, double>& b) {
    if (a.first < b.first) return true;
    if (a.first == b.first && a.second < b.second) return true;
    return false;
}


void printSolutionInfo(const pair<int, double>& fitness, const string& label) {
    cout << label << ": Vehicles = " << fitness.first << ", Total Distance = " << fixed << setprecision(2) << fitness.second << endl;
}

//Check time limit
bool checkTimeLimit(chrono::steady_clock::time_point startTime, int maxTime) {
    if (stopAlgorithm) return false;
    auto currentTime = chrono::steady_clock::now();
    double elapsedTime = chrono::duration_cast<chrono::seconds>(currentTime - startTime).count();
    if (maxTime > 0 && elapsedTime >= maxTime) {
        stopAlgorithm = true;
        if (!timeLimitMessagePrinted) {
            cout << "Time limit reached: " << elapsedTime << " seconds" << endl;
            timeLimitMessagePrinted = true;
        }
        return false;
    }
    return true;
}

//Read instance file and calculate distance matrix
ProblemData readInstance(const string& filename) {
    ProblemData data;
    ifstream infile(filename);
    if (!infile) {
        cerr << "Cannot open file: " << filename << endl;
        exit(1);
    }
    string line;
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            if (getline(infile, line)) {
                istringstream iss(line);
                iss >> data.maxVehicles >> data.vehicleCapacity;
            }}
        if (line.find("CUST NO.") != string::npos) {
            break;
        }}
    while (getline(infile, line)) {
        if (line.empty() || line.find("CUST NO.") != string::npos) continue;
        istringstream issCust(line);
        Customer cust;
        if (issCust >> cust.id >> cust.x >> cust.y >> cust.demand >> cust.earliest >> cust.latest >> cust.serviceTime) {
            data.customers.push_back(cust);
        }
    }
    if (!data.customers.empty()) {
        data.maxRouteDuration = data.customers[0].latest;
    }
    for (size_t i = 0; i < data.customers.size(); ++i) {
        data.idToIndex[data.customers[i].id] = i;}
    size_t n = data.customers.size();
    data.distanceMatrix.resize(n, vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            double dx = data.customers[i].x - data.customers[j].x;
            double dy = data.customers[i].y - data.customers[j].y;
            data.distanceMatrix[i][j] = sqrt(dx * dx + dy * dy);
        }}
    infile.close();
    return data;}

//Update route details
void updateRoute(Route& route, const ProblemData& data, chrono::steady_clock::time_point startTime, int maxTime, bool isFinal = false) {
    if (!isFinal && !checkTimeLimit(startTime, maxTime)) return;
    route.arrivalTimes.clear();
    route.departureTimes.clear();
    route.load = 0;
    route.totalDistance = 0;
    if (route.customerIds.empty()) return;

    double currentTime = 0.0;
    int currentIndex = 0; 

    for (int custId : route.customerIds) {
        if (!isFinal && !checkTimeLimit(startTime, maxTime)) return;
        int index = data.idToIndex.at(custId);
        double travelTime = data.distanceMatrix[currentIndex][index];
        route.totalDistance += travelTime;
        currentTime += travelTime;

        route.arrivalTimes.push_back(max(currentTime, data.customers[index].earliest));
        currentTime = max(currentTime, data.customers[index].earliest);
        
        currentTime += data.customers[index].serviceTime;
        route.departureTimes.push_back(currentTime);
        route.load += data.customers[index].demand;
        currentIndex = index;}
    double returnDist = data.distanceMatrix[currentIndex][0];
    route.totalDistance += returnDist;
}

//check if a customer can be inserted into a route at a given position
bool canInsert(const Route& route, int pos, int customerId, const ProblemData& data) {
    int custIndex = data.idToIndex.at(customerId);
    if (route.load + data.customers[custIndex].demand > data.vehicleCapacity) {
        return false;}
    double prevDepartureTime;
    int prevCustIndex;
    if (pos == 0) {
        prevDepartureTime = 0.0; // Departure from depot
        prevCustIndex = 0;
    } else {
        prevDepartureTime = route.departureTimes[pos - 1];
        prevCustIndex = data.idToIndex.at(route.customerIds[pos - 1]);}
    double arrivalAtNewCust = prevDepartureTime + data.distanceMatrix[prevCustIndex][custIndex];
    double startServiceNewCust = max(arrivalAtNewCust, data.customers[custIndex].earliest);

    if (startServiceNewCust > data.customers[custIndex].latest) {
        return false;}
    double departureFromNewCust = startServiceNewCust + data.customers[custIndex].serviceTime;
    double currentTime = departureFromNewCust;
    int currentCustIndex = custIndex;
    for (size_t i = pos; i < route.customerIds.size(); ++i) {
        int nextCustId = route.customerIds[i];
        int nextCustIndex = data.idToIndex.at(nextCustId);
        currentTime += data.distanceMatrix[currentCustIndex][nextCustIndex];
        double startServiceNextCust = max(currentTime, data.customers[nextCustIndex].earliest);
        if (startServiceNextCust > data.customers[nextCustIndex].latest) {
            return false;}
        currentTime = startServiceNextCust + data.customers[nextCustIndex].serviceTime;
        currentCustIndex = nextCustIndex;}
    currentTime += data.distanceMatrix[currentCustIndex][0];
    if (currentTime > data.maxRouteDuration) {
        return false;}
    return true;}

//Check route feasibility
bool isRouteFeasible(const Route& route, const ProblemData& data) {
    if (route.customerIds.empty()) return true;

    int capacityUsed = 0;
    double currentTime = 0.0;
    int currentIndex = 0;

    for (int custId : route.customerIds) {
        int index = data.idToIndex.at(custId);
        capacityUsed += data.customers[index].demand;
        double travelTime = data.distanceMatrix[currentIndex][index];
        currentTime += travelTime;
        currentTime = max(currentTime, data.customers[index].earliest);
        if (currentTime > data.customers[index].latest) return false;
        currentTime += data.customers[index].serviceTime;
        currentIndex = index;}
    if (capacityUsed > data.vehicleCapacity) return false;
    currentTime += data.distanceMatrix[currentIndex][0];
    if (currentTime > data.maxRouteDuration) return false;
    return true;
}

//Check solution feasibility
bool isSolutionFeasible(const vector<Route>& routes, const ProblemData& data) {
    int usedVehicles = 0;
    for (const auto& r : routes) {
        if (!r.customerIds.empty()) ++usedVehicles;
    }
    if (usedVehicles > data.maxVehicles) return false;

    unordered_set<int> visitedCustomers;
    for (const auto& route : routes) {
        if (!isRouteFeasible(route, data)) return false;
        for (int custId : route.customerIds) {
            if (visitedCustomers.count(custId)) return false; // Visited more than once
            visitedCustomers.insert(custId);}}
    for (size_t i = 1; i < data.customers.size(); ++i) {
        if (visitedCustomers.find(data.customers[i].id) == visitedCustomers.end()) {
            return false; // Not all customers visited
            }}
    return true;
}

// Objective function
pair<int, double> objectiveFunction(const vector<Route>& routes) {
    int vehicles = 0;
    double totalDistance = 0.0;
    for (const auto& route : routes) {
        if (!route.customerIds.empty()) {
            vehicles++;
            totalDistance += route.totalDistance;}}
    return {vehicles, totalDistance};}

// Calculate insertion cost
double calculateInsertionCost(const Route& route, int pos, int customerId, const ProblemData& data) {
    int prevCustId = (pos == 0) ? 0 : route.customerIds[pos - 1];
    int nextCustId = (pos == route.customerIds.size()) ? 0 : route.customerIds[pos];
    
    int prevIndex = data.idToIndex.at(prevCustId);
    int nextIndex = data.idToIndex.at(nextCustId);
    int custIndex = data.idToIndex.at(customerId);

    return data.distanceMatrix[prevIndex][custIndex] + data.distanceMatrix[custIndex][nextIndex] - data.distanceMatrix[prevIndex][nextIndex];
}

// Reconstruct routes with stochastic choice
vector<Route> reconstructRoutes(const vector<int>& sequence, const ProblemData& data, mt19937& rng, chrono::steady_clock::time_point startTime, int maxTime, bool isFinal = false) {
    if (!isFinal && !checkTimeLimit(startTime, maxTime)) return {};
    vector<Route> routes;
    for (int customerId : sequence) {
        if (!isFinal && !checkTimeLimit(startTime, maxTime)) return routes;
        struct InsertionOption {
            double cost;
            int routeIdx;
            int position;};
        vector<InsertionOption> candidates;
        for (size_t r = 0; r < routes.size(); ++r) {
            for (size_t pos = 0; pos <= routes[r].customerIds.size(); ++pos) {
                if (canInsert(routes[r], pos, customerId, data)) {
                    double cost = calculateInsertionCost(routes[r], pos, customerId, data);
                    candidates.push_back({cost, (int)r, (int)pos});                }}}
        if (!candidates.empty()) {
            sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b){
                return a.cost < b.cost;            });
            int k = min((int)candidates.size(), 3);
            uniform_int_distribution<int> dist(0, k - 1);
            InsertionOption best = candidates[dist(rng)];
            routes[best.routeIdx].customerIds.insert(routes[best.routeIdx].customerIds.begin() + best.position, customerId);
            updateRoute(routes[best.routeIdx], data, startTime, maxTime, isFinal);
        } else {
            Route newRoute;
            newRoute.customerIds.push_back(customerId);
            if (isRouteFeasible(newRoute, data)) {
                 updateRoute(newRoute, data, startTime, maxTime, isFinal);
                 routes.push_back(newRoute);
            } else {
                 cout << "Warning: Customer " << customerId << " cannot be placed in a new route." << endl;}}}
    return routes;}

// Greedy construction of sequence
vector<int> greedyConstruction(const ProblemData& data, mt19937& rng) {
    vector<int> sequence;
    unordered_set<int> unrouted;
    for (size_t i = 1; i < data.customers.size(); ++i) {
        unrouted.insert(data.customers[i].id);}
    uniform_int_distribution<size_t> dist(0, unrouted.size() - 1);
    auto it = unrouted.begin();
    advance(it, dist(rng));
    int current_cust = *it;
    sequence.push_back(current_cust);
    unrouted.erase(current_cust);
    while (!unrouted.empty()) {
        int current_cust_idx = data.idToIndex.at(current_cust);
        int best_next_cust = -1;
        double min_dist = numeric_limits<double>::max();
        for (int next_cust_id : unrouted) {
            int next_cust_idx = data.idToIndex.at(next_cust_id);
            if (data.distanceMatrix[current_cust_idx][next_cust_idx] < min_dist) {
                min_dist = data.distanceMatrix[current_cust_idx][next_cust_idx];
                best_next_cust = next_cust_id;}}
        current_cust = best_next_cust;
        sequence.push_back(current_cust);
        unrouted.erase(current_cust);}
    return sequence;}

// Border construction by time windows
vector<int> borderConstructionByTime(const ProblemData& data, bool sortByEarliest, bool ascending) {
    vector<pair<double, int>> sortedCustomers;
    for (size_t i = 1; i < data.customers.size(); ++i) {
        double value = sortByEarliest ? data.customers[i].earliest : data.customers[i].latest;
        sortedCustomers.push_back({value, data.customers[i].id});}
    sort(sortedCustomers.begin(), sortedCustomers.end(), [ascending](const auto& a, const auto& b) {
        return ascending ? a.first < b.first : a.first > b.first;});
    vector<int> sequence;
    for (const auto& pair : sortedCustomers) {
        sequence.push_back(pair.second);}
    return sequence;}

// Border construction by coordinates 
vector<int> borderConstructionByCoords(const ProblemData& data, bool sortByX, bool ascending) {
    vector<pair<double, int>> sortedCustomers;
    for (size_t i = 1; i < data.customers.size(); ++i) {
        double value = sortByX ? data.customers[i].x : data.customers[i].y;
        sortedCustomers.push_back({value, data.customers[i].id});}
    sort(sortedCustomers.begin(), sortedCustomers.end(), [ascending](const auto& a, const auto& b) {
        return ascending ? a.first < b.first : a.first > b.first;});
    vector<int> sequence;
    for (const auto& pair : sortedCustomers) {
        sequence.push_back(pair.second);}
    return sequence;}

//Decode particle into customer sequence
vector<int> decodeParticle(const vector<double>& position, const ProblemData& data) {
    int numCustomers = data.customers.size() - 1;
    vector<pair<double, int>> order;
    for (int i = 0; i < numCustomers; ++i) 
        order.emplace_back(position[i], data.customers[i + 1].id);
    sort(order.begin(), order.end());
    vector<int> customerSeq;
    for (auto& pr : order) customerSeq.push_back(pr.second);
    return customerSeq;}

//Evaluate solution
pair<int, double> evaluateSolution(const vector<double>& position, const ProblemData& data, mt19937& rng, long long& evalCount, int maxEvaluations, chrono::steady_clock::time_point startTime, int maxTime, vector<Route>& generatedRoutes, bool isFinal = false) {
    const pair<int, double> WORST_FITNESS = {numeric_limits<int>::max(), numeric_limits<double>::max()};
    if (stopAlgorithm || (!isFinal && !checkTimeLimit(startTime, maxTime))) return WORST_FITNESS;
    if (maxEvaluations > 0 && evalCount >= maxEvaluations) {
        stopAlgorithm = true;
        return WORST_FITNESS;}
    evalCount++;
    vector<int> sequence = decodeParticle(position, data);
    vector<Route> routes = reconstructRoutes(sequence, data, rng, startTime, maxTime, isFinal);
    generatedRoutes = routes;
    unordered_set<int> assigned;
    for (const auto& r : routes) for (int id : r.customerIds) assigned.insert(id);
    if (assigned.size() != data.customers.size() - 1) {
        return WORST_FITNESS;}
    if (!isSolutionFeasible(routes, data)) {
        return WORST_FITNESS;}
    return objectiveFunction(routes);}

//Update particle
void updateParticle(Particle& particle, const vector<double>& gBest, double w, double c1, double c2, mt19937& rng, double Vmax) {
    if (particle.position.empty()) return;
    uniform_real_distribution<double> dist(0.0, 1.0);

    for (size_t i = 0; i < particle.velocity.size(); ++i) {
        double r1 = dist(rng);
        double r2 = dist(rng);
        particle.velocity[i] = w * particle.velocity[i] +
                               c1 * r1 * (particle.pBest[i] - particle.position[i]) +
                               c2 * r2 * (gBest[i] - particle.position[i]);
        if (particle.velocity[i] > Vmax) particle.velocity[i] = Vmax;
        else if (particle.velocity[i] < -Vmax) particle.velocity[i] = -Vmax;}
    for (size_t i = 0; i < particle.position.size(); ++i) {
        particle.position[i] += particle.velocity[i];}}

//Generates an initial swarm 
vector<Particle> generateInitialSwarm(const ProblemData& data, mt19937& rng, int swarmSize) {
    int numCustomers = data.customers.size() - 1;
    int numVehicles = data.maxVehicles;
    int dims = numCustomers + 2 * numVehicles;
    vector<Particle> swarm(swarmSize);
    uniform_real_distribution<double> pos_dist(0.0, 4.0);
    for (auto& p : swarm) {
        p.position.assign(dims, 0.0);
        p.velocity.assign(dims, 0.0);
        p.pBest.assign(dims, 0.0);
        p.pBestFitness = {numeric_limits<int>::max(), numeric_limits<double>::max()};}
    int greedyCount = static_cast<int>(swarmSize * .2); 
    int borderCount = static_cast<int>(swarmSize * .15); 
    int randomCount = swarmSize - greedyCount - borderCount;
    int particle_idx = 0;
    auto encodeSequence = [&](const vector<int>& sequence) {
        vector<double> position(dims);
        unordered_map<int, int> customer_to_rank;
        for (int i = 0; i < sequence.size(); ++i) {
            customer_to_rank[sequence[i]] = i;}
        for (int i = 0; i < numCustomers; ++i) {
            int customerId = data.customers[i + 1].id;
            position[i] = static_cast<double>(customer_to_rank[customerId]);}
        for (int j = numCustomers; j < dims; ++j) {
            position[j] = pos_dist(rng);}
        return position;};
    //Greedy
    for (int i = 0; i < greedyCount; ++i) {
        vector<int> greedy_sequence = greedyConstruction(data, rng);
        swarm[particle_idx].position = encodeSequence(greedy_sequence);
        swarm[particle_idx].velocity = vector<double>(dims, 0.0);
        swarm[particle_idx].pBest = swarm[particle_idx].position;
        swarm[particle_idx].pBestFitness = {numeric_limits<int>::max(), numeric_limits<double>::max()};
        particle_idx++;}
    //border
    for (int i = 0; i < borderCount; ++i) {
        vector<int> border_sequence;
        if (i % 4 == 0) border_sequence = borderConstructionByTime(data, true, true);
        else if (i % 4 == 1) border_sequence = borderConstructionByTime(data, true, false);
        else if (i % 4 == 2) border_sequence = borderConstructionByTime(data, false, true);
        else border_sequence = borderConstructionByTime(data, false, false);
        
        swarm[particle_idx].position = encodeSequence(border_sequence);
        swarm[particle_idx].velocity = vector<double>(dims, 0.0);
        swarm[particle_idx].pBest = swarm[particle_idx].position;
        swarm[particle_idx].pBestFitness = {numeric_limits<int>::max(), numeric_limits<double>::max()};
        particle_idx++;}
    int hBorderCount = 0;
    if (greedyCount == 0 && borderCount == 0) {
        hBorderCount = static_cast<int>(swarmSize * 0.4);}
    for (int i = 0; i < randomCount; ++i) {
        if (i < hBorderCount) {
            vector<int> border_sequence;
            if (i % 4 == 0) border_sequence = borderConstructionByCoords(data, true, true);
            else if (i % 4 == 1) border_sequence = borderConstructionByCoords(data, true, false);
            else if (i % 4 == 2) border_sequence = borderConstructionByCoords(data, false, true);
            else border_sequence = borderConstructionByCoords(data, false, false);
            swarm[particle_idx].position = encodeSequence(border_sequence);
        } else {
            for (int j = 0; j < dims; ++j) {
                swarm[particle_idx].position[j] = pos_dist(rng);}}
        swarm[particle_idx].velocity = vector<double>(dims, 0.0);
        swarm[particle_idx].pBest = swarm[particle_idx].position;
        swarm[particle_idx].pBestFitness = {numeric_limits<int>::max(), numeric_limits<double>::max()};
        particle_idx++;}
    return swarm;}

// tournament Selection
int tournamentSelection(const vector<Particle>& swarm, int k, mt19937& rng, const unordered_set<int>& tabu_guides) {
    if (swarm.empty()) {
        return -1;}
    vector<int> eligible_indices;
    for (int i = 0; i < swarm.size(); ++i) {
        if (tabu_guides.find(i) == tabu_guides.end()) {
            eligible_indices.push_back(i);}}
    if (eligible_indices.size() < k) {
        return -1;}
    //Randomly shuffle the eligible indices to pick random participants
    shuffle(eligible_indices.begin(), eligible_indices.end(), rng);
    int winner_index = -1;
    pair<int, double> winner_fitness = {numeric_limits<int>::max(), numeric_limits<double>::max()};
    for (int i = 0; i < k; ++i) {
        int participant_index = eligible_indices[i];
        if (isBetterThan(swarm[participant_index].pBestFitness, winner_fitness)) {
            winner_fitness = swarm[participant_index].pBestFitness;
            winner_index = participant_index;}}
    return winner_index;}

// PSO algorithm 
vector<Route> particleSwarmOptimization(const ProblemData& data, mt19937& rng, int swarmSize, int maxIterations, double c1, double c2, double Vmax, int maxTime, int maxEvaluations, chrono::steady_clock::time_point startTime) {
    stopAlgorithm = false;
    timeLimitMessagePrinted = false;
    long long evalCount = 0;
    vector<Particle> swarm = generateInitialSwarm(data, rng, swarmSize);
    if (swarm.empty()) return {};
    vector<double> gBest;
    pair<int, double> gBestFitness = {numeric_limits<int>::max(), numeric_limits<double>::max()};
    vector<Route> bestRoutesSoFar; 
    //Initial evaluation of all particles to determine their pBest
    for (int i = 0; i < swarmSize; ++i) {
        if (stopAlgorithm) break;
        vector<Route> currentRoutes;
        pair<int, double> fitness = evaluateSolution(swarm[i].position, data, rng, evalCount, maxEvaluations, startTime, maxTime, currentRoutes);  
        swarm[i].pBest = swarm[i].position;
        swarm[i].pBestFitness = fitness;
        if (fitness.first != numeric_limits<int>::max()) {
            swarm[i].pBestRoutes = currentRoutes;
        }}
    //Attempt to select the guide
    vector<int> feasible_random_indices;
    int greedyCount = static_cast<int>(swarmSize * 0); 
    int borderCount = static_cast<int>(swarmSize * 0); 
    int random_start_index = greedyCount + borderCount;

    for (int i = random_start_index; i < swarmSize; ++i) {
        if (swarm[i].pBestFitness.first != numeric_limits<int>::max()) {
            feasible_random_indices.push_back(i);}}
    
    if (!feasible_random_indices.empty()) {
        uniform_int_distribution<int> dist(0, feasible_random_indices.size() - 1);
        int random_choice_index = feasible_random_indices[dist(rng)];
        gBestFitness = swarm[random_choice_index].pBestFitness;
        gBest = swarm[random_choice_index].pBest;
        bestRoutesSoFar = swarm[random_choice_index].pBestRoutes;
        cout << "Initial guide chosen randomly from " << feasible_random_indices.size() << " PURELY RANDOM feasible particles." << endl;
    } else {
        cout << "Warning: No feasible solution found among random particles. Falling back to the absolute best particle." << endl;
        for (const auto& particle : swarm) {
            if (isBetterThan(particle.pBestFitness, gBestFitness)) {
                gBestFitness = particle.pBestFitness;
                gBest = particle.pBest;
                bestRoutesSoFar = particle.pBestRoutes;}}
        if (bestRoutesSoFar.empty()) {
            cout << "Warning: No feasible solution found in the entire initial swarm." << endl;
    }}
    if (!bestRoutesSoFar.empty()) {
        printSolutionInfo(gBestFitness, "Initial");
    }
    //Main PSO Loop 
    const double w_max = 2.0; const double w_min = 0.4; double w = w_max;
    const int tournamentSize_k = 2; 
    unordered_set<int> tabu_guides;
    for (int iteration = 0; iteration < maxIterations && !stopAlgorithm; ++iteration) {
        // Evaluate each particle and update pBest
        for (int i = 0; i < swarmSize; ++i) {
            if (stopAlgorithm) break;
            vector<Route> currentRoutes;
            pair<int, double> newFitness = evaluateSolution(swarm[i].position, data, rng, evalCount, maxEvaluations, startTime, maxTime, currentRoutes);
            if (isBetterThan(newFitness, swarm[i].pBestFitness)) {
                swarm[i].pBest = swarm[i].position;
                swarm[i].pBestFitness = newFitness;
                swarm[i].pBestRoutes = currentRoutes;}}
        if (stopAlgorithm) break;
        // Update gBest and bestRoutesSoFar
        for (int i = 0; i < swarmSize; ++i) {
            if (isBetterThan(swarm[i].pBestFitness, gBestFitness)) {
                gBestFitness = swarm[i].pBestFitness;
                gBest = swarm[i].pBest;
                bestRoutesSoFar = swarm[i].pBestRoutes;}}
        if (swarm.size() - tabu_guides.size() < tournamentSize_k) {
            tabu_guides.clear();
        }
        int winner_index = tournamentSelection(swarm, tournamentSize_k, rng, tabu_guides);
        vector<double> guide_gBest = (winner_index != -1) ? swarm[winner_index].pBest : gBest;
        if(winner_index != -1) tabu_guides.insert(winner_index);

        for (int i = 0; i < swarmSize; ++i) {
            updateParticle(swarm[i], guide_gBest, w, c1, c2, rng, Vmax);}
        w = w_max - (w_max - w_min) * static_cast<double>(iteration) / maxIterations;
        cout << "Iteration " << iteration + 1 << ": ";
        printSolutionInfo(gBestFitness, "Best Solution");}
    return bestRoutesSoFar;
}

// Print solution
void printSolution(const string& instanceFile, const vector<Route>& routes, const ProblemData& data) {
    string outputFileName = instanceFile;
    size_t pos = outputFileName.find_last_of(".");
    if (pos != string::npos) {
        outputFileName = outputFileName.substr(0, pos);}
    outputFileName += "_output.txt";
    ofstream outfile(outputFileName);
    if (!outfile) {
        cerr << "Cannot open output file: " << outputFileName << endl;
        return;}
    int routeNumber = 0;
    int vehicles = 0;
    double totalDistance = 0.0;
    for (const auto& route : routes) {
        if (!route.customerIds.empty()) {
            outfile << "Route " << ++routeNumber << ":";
            for (int custId : route.customerIds) {
                outfile << " " << custId;}
            outfile << "\n";
            totalDistance += route.totalDistance;
            vehicles++;}}
    outfile << "Vehicles: " << vehicles << "\n";
    outfile << "Distance: " << fixed << setprecision(2) << totalDistance << "\n";
    cout << "Solution written to " << outputFileName << endl;
    outfile.close();}

// Main function
int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <instanceFile> <maxTime> <maxEvaluations>" << endl;
        return 1;}
    const string instanceFile = argv[1];
    const int maxTime = stoi(argv[2]);
    const int maxEvaluations = (string(argv[3]) == "0") ? numeric_limits<int>::max() : stoi(argv[3]);
    cout << "Running with instance: " << instanceFile << ", maxTime: " << maxTime << ", maxEvaluations: " << maxEvaluations << endl;
    ProblemData data = readInstance(instanceFile);
    if (data.customers.empty()) {
        cerr << "Failed to read problem data." << endl;
        return 1;}
    const int swarmSize = 30;
    const int maxIterations = 10000000;
    const double c1 = 2.0;
    const double c2 = 2.0;
    const double Vmax = 2.0;
    cout << "Parameters: swarmSize = " << swarmSize << ", maxIterations = " << maxIterations
         << ", c1 = " << c1 << ", c2 = " << c2 << ", Vmax = " << Vmax << endl;
    mt19937 rng(chrono::high_resolution_clock::now().time_since_epoch().count());
    auto startTime = chrono::steady_clock::now();
    vector<Route> solution = particleSwarmOptimization(data, rng, swarmSize, maxIterations, c1, c2, Vmax, maxTime, maxEvaluations, startTime);
    auto endTime = chrono::steady_clock::now();
    double executionTime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count() / 1000.0;
    if (!solution.empty()) {
        bool feasible = isSolutionFeasible(solution, data);
        cout << "Final solution check: " << (feasible ? "FEASIBLE" : "INFEASIBLE!") << endl;
        printSolution(instanceFile, solution, data);
        int vehicles = 0;
        double totalDistance = 0.0;
        for (const auto& route : solution) {
            if (!route.customerIds.empty()) {
                vehicles++;
                totalDistance += route.totalDistance; }}
        cout << "----------------------------------------" << endl;
        cout << "Final Solution Summary:" << endl;
        cout << "Number of vehicles used: " << vehicles << endl;
        cout << "Total distance traveled: " << fixed << setprecision(2) << totalDistance << endl;
        cout << "----------------------------------------" << endl;
    } else {
        cout << "No feasible solution found." << endl;
    }
    cout << "Execution Time: " << fixed << setprecision(2) << executionTime << " seconds" << endl;
    return 0;}