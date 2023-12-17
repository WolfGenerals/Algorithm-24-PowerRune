#ifndef STABILIZER_HPP
#define STABILIZER_HPP
#include <list>

class StabilizedDouble {
    struct Node {
        double value;
        bool   excluded = false;
    };


    std::list<Node> history{};

    double average  = 0.0;
    double variance = 0.0;

    int    historySize;
    double excludedThreshold;

public:
    explicit StabilizedDouble(const int historySize = 10, const double excludedThreshold = 3)
        : historySize(historySize),
          excludedThreshold(excludedThreshold) {}

private:
    void insert(const double value) {
        if (history.size() >= historySize)
            history.pop_front();
        history.push_back({value});
    }


    [[nodiscard]] bool valid(const double value) const {
        return (value - average) * (value - average) <= variance * excludedThreshold * excludedThreshold;
    }

    bool excludeErrorData() {
        bool changed = false;
        for (auto &[value, excluded]: history) {
            if (excluded)
                continue;
            if (!valid(value)) {
                excluded = true;
                changed  = true;
            }
        }
        return changed;
    }

    [[nodiscard]] double computeAverage() const {
        double sum   = 0.0;
        int    count = 0;
        for (const auto &[value, excluded]: history) {
            if (excluded)
                continue;
            sum += value;
            count++;
        }
        return sum / count;
    }

    [[nodiscard]] double computeVariance() const {
        double sum   = 0.0;
        int    count = 0;
        for (const auto &[value, excluded]: history) {
            if (excluded)
                continue;
            sum += value * value;
            count++;
        }
        return sum / count - average * average;
    }

public:
    StabilizedDouble &operator=(const double value) {
        insert(value);
        bool hasError;
        do {
            average  = computeAverage();
            variance = computeVariance();
            hasError = excludeErrorData();
        } while (hasError);

        return *this;
    }

    [[nodiscard]] operator double() const {// NOLINT(*-explicit-constructor)
        return average;
    }

    [[nodiscard]] bool valid() const {
        return valid(history.back().value);
    }
};


#endif//STABILIZER_HPP
