#ifndef STABILIZER_HPP
#define STABILIZER_HPP
#include <list>

template<int historySize>
class StabilizedDouble {
    std::list<double> history{};

    bool   mask[historySize]{true};
    double average  = 0.0;
    double variance = 0.0;


    void insert(const double value) {
        if (history.size() >= historySize)
            history.pop_front();
        history.push_back(value);
    }

    void resetMask() {
        std::fill(mask, mask + historySize, true);
    }
    bool excludeErrorData() {
        bool changed = false;
        int  index   = 0;
        for (const double element: history) {
            if (!mask[index])
                continue;
            if ((element - average) * (element - average) > 9 * variance) {
                mask[index] = false;
                changed     = true;
            }
        }
        return changed;
    }

    [[nodiscard]] double computeAverage() const {
        double sum   = 0.0;
        int    index = 0;
        for (const double element: history) {
            if (!mask[index++])
                continue;
            sum += element;
        }
        return sum / index;
    }

    [[nodiscard]] double computeVariance() const {
        double sum   = 0.0;
        int    index = 0;
        for (const double element: history) {
            if (!mask[index++])
                continue;
            const double diff = element - average;
            sum += diff * diff;
        }
        return sum / index;
    }

public:
    StabilizedDouble &operator=(const double value) {
        insert(value);
        resetMask();
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
        return history.back() - average * 9 < variance;
    }
};


#endif//STABILIZER_HPP
