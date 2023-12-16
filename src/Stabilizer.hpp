//
// Created by mojiw on 2023/12/16.
//

#ifndef STABILIZER_HPP
#define STABILIZER_HPP


template<int historySize>
class StabilizedDouble {
    double history[historySize] = {0};
    int    index                = 0;
    int    currentSize          = 0;

    double current = 0.0;

    void insert(double value) {
        history[index] = value;
        index          = (index + 1) % historySize;
        if (currentSize < historySize)
            currentSize++;
    }

    double caculate() {
        bool mask[historySize] = {false};
        while (true) {
            bool         changed    = false;
            const double avr        = average(mask);
            const double maxDeviate = 9 * variance(mask);
            for (int i = 0; i < currentSize; i++) {
                if (!mask[i])
                    continue;
                const double deviate = (history[i] - avr);
                if (deviate * deviate > maxDeviate) {
                    changed = true;
                    mask[i] = false;
                }
            }
            if (!changed)
                return avr;
        }
    }

    [[nodiscard]] double average(const bool *mask) const {
        double sum = 0.0;
        for (int i = 0; i < currentSize; i++) {
            if (!mask[i])
                continue;
            sum += history[i];
        }
        return sum / currentSize;
    }

    [[nodiscard]] double variance(const bool *mask) const {
        double sum = 0.0;
        for (int i = 0; i < currentSize; i++) {
            if (!mask[i])
                continue;
            const double deviate = (history[i] - average());
            sum += deviate * deviate;
        }
        return sum / currentSize;
    }

public:
    StabilizedDouble &operator=(double value) {
        insert(value);
        current = caculate();
        return *this;
    }

    [[nodiscard]] operator double() const {// NOLINT(*-explicit-constructor)
        return current;
    }
};


#endif//STABILIZER_HPP
