#ifndef CACHE_HPP
#define CACHE_HPP
template<typename T>
class Cache {
    std::deque<T> cache;
    unsigned int size;

public:
    explicit Cache(const int size)
        : size(size) {}

    void update(T value) {
        cache.push_back(value);
        if ( cache.size() >size)
            cache.pop_front();
    }

    [[nodiscard]] T avrage() const{
        return accumulate(cache.begin(), cache.end(), T{}) / static_cast<double>(cache.size());
    }

    [[nodiscard]] bool empty() const{
        return cache.empty();
    }

    [[nodiscard]] T last() const {
        return cache.back();
    }

};
#endif //CACHE_HPP
