#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using StateId = std::uint64_t;

struct State {
    StateId id{};
    std::vector<double> embedding;
};

struct Transition {
    StateId id{};
    StateId from{};
    StateId to{};
    double cost{1.0};
    double safety{1.0};       // 0 = unsafe, 1 = safest
    double reliability{1.0};  // probability-like score in (0, 1]
    bool available{true};
};

struct PlanningProblem {
    StateId initialState{};
    StateId goalState{};
    std::vector<StateId> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

struct PlanningResult {
    bool success{false};
    std::vector<StateId> statePath;
    std::vector<StateId> transitionPath;
    double totalCost{0.0};
    double safetyScore{0.0};
    double minBadStateDistance{std::numeric_limits<double>::infinity()};
    double cumulativeReliability{1.0};
    std::size_t exploredStates{0};
    double planningTimeMs{0.0};
    std::string message;
};

struct PlannerConfig {
    // Objective edge weight = cost + safetyWeight/(distance-to-bad + epsilon)
    // + reliabilityWeight*(-log(reliability)) + transitionSafetyWeight*(1-safety).
    double safetyWeight{1.0};
    double reliabilityWeight{1.0};
    double transitionSafetyWeight{0.25};
    double epsilon{0.1};
};

class SafePlanner {
public:
    explicit SafePlanner(PlannerConfig config = {}) : config_(config) {}

    PlanningResult plan(const PlanningProblem& problem) const {
        const auto started = std::chrono::steady_clock::now();
        PlanningResult result;
        auto finish = [&]() {
            result.planningTimeMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            return result;
        };

        std::unordered_map<StateId, const State*> stateById;
        for (const auto& state : problem.states) {
            if (!stateById.emplace(state.id, &state).second) {
                result.message = "Invalid problem: duplicate state ID.";
                return finish();
            }
        }
        if (!stateById.count(problem.initialState) || !stateById.count(problem.goalState)) {
            result.message = "Invalid problem: initial or goal state does not exist.";
            return finish();
        }

        std::unordered_set<StateId> bad(problem.badStates.begin(), problem.badStates.end());
        if (bad.count(problem.initialState) || bad.count(problem.goalState)) {
            result.message = "No safe plan: initial state or goal is marked as bad.";
            return finish();
        }
        if (problem.initialState == problem.goalState) {
            result.success = true;
            result.statePath = {problem.initialState};
            result.minBadStateDistance = distanceToNearestBad(problem.initialState, stateById, bad);
            result.safetyScore = std::isinf(result.minBadStateDistance) ? 1.0 : result.minBadStateDistance;
            result.message = "Already at goal.";
            return finish();
        }

        std::unordered_map<StateId, std::vector<const Transition*>> outgoing;
        for (const auto& tr : problem.transitions) {
            if (!stateById.count(tr.from) || !stateById.count(tr.to)) continue;
            if (!tr.available || bad.count(tr.from) || bad.count(tr.to)) continue;
            if (!std::isfinite(tr.cost) || tr.cost < 0.0 ||
                !std::isfinite(tr.safety) || tr.safety < 0.0 || tr.safety > 1.0 ||
                !std::isfinite(tr.reliability) || tr.reliability <= 0.0 || tr.reliability > 1.0) continue;
            outgoing[tr.from].push_back(&tr);
        }

        struct Node { double score; StateId id; };
        struct Greater { bool operator()(const Node& a, const Node& b) const { return a.score > b.score; } };
        std::priority_queue<Node, std::vector<Node>, Greater> frontier;
        std::unordered_map<StateId, double> dist;
        std::unordered_map<StateId, StateId> previousState;
        std::unordered_map<StateId, StateId> previousTransition;
        dist[problem.initialState] = 0.0;
        frontier.push({0.0, problem.initialState});

        while (!frontier.empty()) {
            const Node current = frontier.top(); frontier.pop();
            auto known = dist.find(current.id);
            if (known == dist.end() || current.score > known->second + 1e-12) continue;
            ++result.exploredStates;
            if (current.id == problem.goalState) break;

            for (const Transition* tr : outgoing[current.id]) {
                const double distance = distanceToNearestBad(tr->to, stateById, bad);
                const double safetyPenalty = bad.empty() ? 0.0 : config_.safetyWeight / (distance + config_.epsilon);
                const double reliabilityPenalty = config_.reliabilityWeight * (-std::log(tr->reliability));
                const double edgeScore = tr->cost + safetyPenalty + reliabilityPenalty +
                    config_.transitionSafetyWeight * (1.0 - tr->safety);
                const double candidate = current.score + edgeScore;
                auto it = dist.find(tr->to);
                if (it == dist.end() || candidate + 1e-12 < it->second) {
                    dist[tr->to] = candidate;
                    previousState[tr->to] = current.id;
                    previousTransition[tr->to] = tr->id;
                    frontier.push({candidate, tr->to});
                }
            }
        }

        if (!dist.count(problem.goalState)) {
            result.message = "No safe path found with the current available transitions.";
            return finish();
        }

        StateId cursor = problem.goalState;
        result.statePath.push_back(cursor);
        while (cursor != problem.initialState) {
            if (!previousState.count(cursor) || !previousTransition.count(cursor)) {
                result.statePath.clear(); result.transitionPath.clear();
                result.message = "Path reconstruction failed.";
                return finish();
            }
            result.transitionPath.push_back(previousTransition[cursor]);
            cursor = previousState[cursor];
            result.statePath.push_back(cursor);
        }
        std::reverse(result.statePath.begin(), result.statePath.end());
        std::reverse(result.transitionPath.begin(), result.transitionPath.end());

        result.totalCost = 0.0;
        result.cumulativeReliability = 1.0;
        result.minBadStateDistance = std::numeric_limits<double>::infinity();
        std::unordered_map<StateId, const Transition*> transitionById;
        for (const auto& tr : problem.transitions) transitionById[tr.id] = &tr;
        for (StateId sid : result.statePath) {
            result.minBadStateDistance = std::min(result.minBadStateDistance,
                distanceToNearestBad(sid, stateById, bad));
        }
        for (StateId tid : result.transitionPath) {
            const Transition& tr = *transitionById.at(tid);
            result.totalCost += tr.cost;
            result.cumulativeReliability *= tr.reliability;
        }
        result.safetyScore = std::isinf(result.minBadStateDistance) ? 1.0 : result.minBadStateDistance;
        result.success = true;
        result.message = "Safe path found.";
        return finish();
    }

private:
    PlannerConfig config_;

    static double euclidean(const std::vector<double>& a, const std::vector<double>& b) {
        if (a.size() != b.size()) return std::numeric_limits<double>::infinity();
        double sum = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const double delta = a[i] - b[i]; sum += delta * delta;
        }
        return std::sqrt(sum);
    }

    static double distanceToNearestBad(StateId id,
            const std::unordered_map<StateId, const State*>& stateById,
            const std::unordered_set<StateId>& bad) {
        if (bad.empty()) return std::numeric_limits<double>::infinity();
        const State* current = stateById.at(id);
        double best = std::numeric_limits<double>::infinity();
        for (StateId badId : bad) {
            auto it = stateById.find(badId);
            if (it != stateById.end()) best = std::min(best, euclidean(current->embedding, it->second->embedding));
        }
        return best;
    }
};

static State makeState(StateId id, double x, double y) { return State{id, {x, y}}; }
static Transition edge(StateId id, StateId from, StateId to, double cost = 1.0,
                       double safety = 1.0, double reliability = 0.99, bool available = true) {
    return Transition{id, from, to, cost, safety, reliability, available};
}

static void printResult(const std::string& name, const PlanningResult& r) {
    std::cout << "\n=== " << name << " ===\n";
    std::cout << "Status: " << (r.success ? "SUCCESS" : "NO PATH") << "\n";
    std::cout << "Message: " << r.message << "\n";
    if (r.success) {
        std::cout << "State path: ";
        for (std::size_t i = 0; i < r.statePath.size(); ++i) std::cout << (i ? " -> " : "") << r.statePath[i];
        std::cout << "\nTransition IDs: ";
        for (std::size_t i = 0; i < r.transitionPath.size(); ++i) std::cout << (i ? ", " : "") << r.transitionPath[i];
        std::cout << "\nTotal transition cost: " << r.totalCost;
        std::cout << "\nMinimum distance to bad state: ";
        if (std::isinf(r.minBadStateDistance)) std::cout << "N/A (no bad states specified)";
        else std::cout << r.minBadStateDistance;
        std::cout << "\nCumulative reliability: " << r.cumulativeReliability;
    }
    std::cout << "\nExplored states: " << r.exploredStates
              << "\nPlanning time (ms): " << r.planningTimeMs << "\n";
}

int main() {
    std::cout << std::fixed << std::setprecision(4);
    SafePlanner planner;

    // Test 1: basic reachability: S(0) -> A(1) -> B(2) -> G(3).
    PlanningProblem basic{0, 3, {},
        {makeState(0,0,0), makeState(1,1,0), makeState(2,2,0), makeState(3,3,0)},
        {edge(10,0,1), edge(11,1,2), edge(12,2,3)}};
    printResult("Test 1 - Basic reachability", planner.plan(basic));

    // Test 2: route through bad state 2 must be rejected; route 0->3->4->5 is valid.
    PlanningProblem avoidBad{0, 5, {2},
        {makeState(0,0,0), makeState(1,1,0), makeState(2,2,0), makeState(3,0,2), makeState(4,1,2), makeState(5,2,2)},
        {edge(20,0,1), edge(21,1,2), edge(22,2,5), edge(23,0,3), edge(24,3,4), edge(25,4,5)}};
    printResult("Test 2 - Bad-state avoidance", planner.plan(avoidBad));

    // Test 3: two routes; the safety-aware objective can prefer a longer route away from bad state 9.
    PlanningProblem safety{0, 4, {9},
        {makeState(0,0,0), makeState(1,1,0), makeState(2,2,0), makeState(3,1,2), makeState(4,2,2), makeState(9,1,0.1)},
        {edge(30,0,1,1.0), edge(31,1,2,1.0), edge(32,2,4,1.0),
         edge(33,0,3,1.3), edge(34,3,4,1.3)}};
    printResult("Test 3 - Safety/cost trade-off", planner.plan(safety));

    // Test 4: transition (A,G) becomes unavailable, so the planner uses the alternative route.
    PlanningProblem dynamic{0, 3, {},
        {makeState(0,0,0), makeState(1,1,0), makeState(2,0,1), makeState(3,2,0)},
        {edge(40,0,1), edge(41,1,3,1.0,1.0,0.99,false), edge(42,0,2,1.2), edge(43,2,3,1.2)}};
    printResult("Test 4 - Dynamic transition unavailable", planner.plan(dynamic));

    // Test 5: changing the goal and replanning with the updated problem.
    PlanningProblem goalUpdate{0, 2, {},
        {makeState(0,0,0), makeState(1,1,0), makeState(2,2,0), makeState(3,1,1)},
        {edge(50,0,1), edge(51,1,2), edge(52,0,3), edge(53,3,2)}};
    printResult("Test 5a - Original goal", planner.plan(goalUpdate));
    goalUpdate.goalState = 3;
    printResult("Test 5b - Updated goal (replan)", planner.plan(goalUpdate));

    // Test 6: adding a shortcut improves the path after replanning.
    PlanningProblem shortcut{0, 3, {},
        {makeState(0,0,0), makeState(1,1,0), makeState(2,2,0), makeState(3,3,0)},
        {edge(60,0,1,2.0), edge(61,1,2,2.0), edge(62,2,3,2.0)}};
    printResult("Test 6a - Before shortcut", planner.plan(shortcut));
    shortcut.transitions.push_back(edge(63,0,3,1.0));
    printResult("Test 6b - After shortcut (replan)", planner.plan(shortcut));

    return 0;
}
