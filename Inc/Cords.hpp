#include <cstdint>

struct Point {
    int32_t x;
    int32_t y;
    uint32_t t;

    Point() = default;
    Point(int32_t _x, int32_t _y, uint32_t _t);

    bool inRange(Point& other);
};

int64_t hypotsqrd(Point& a, Point& b);

#ifdef __WIN32

constexpr std::size_t MAX_CORDS_SIZE = 43200;

#elif STM32H7A3xx

constexpr std::size_t MAX_CORDS_SIZE = 6000; // 10 minutes of driving at 10hz

#endif

struct Cords {
    std::size_t size{};
    Point* data;

    void push_back(Point& Point);
    void push_back(Point Point);
};


