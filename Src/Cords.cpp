#include "Cords.hpp"

int64_t hypotsqrd(Point& a, Point& b)
{
    int64_t dx = a.x - b.x;
    int64_t dy = a.y - b.y;
    return (dx * dx + dy * dy);
}

Point::Point(int32_t _x, int32_t _y, uint32_t _t, int32_t _headMot) : x(_x), y(_y), t(_t), headMot(_headMot) 
{

}

bool Point::inRange(Point& other)
{
    constexpr int64_t tolerance = 6000 * 6000; 

    return hypotsqrd(*this, other) < tolerance ? true : false;
}

bool Point::isSameDirection(const Point& other) const
{
    constexpr int32_t FULL_CIRCLE_DEG = 36000000; // * 10^5
    constexpr int32_t HALF_CIRCLE_DEG = 18000000; // * 10^5
    constexpr int32_t MAX_HEADING_DIFF_DEG = 4500000; // 45 deg * 10^5

    int32_t diff = headMot - other.headMot;
    if (diff < 0)
    {
        diff = -diff;
    }

    diff %= FULL_CIRCLE_DEG;
    
    if (diff > HALF_CIRCLE_DEG)
    {
        diff = FULL_CIRCLE_DEG - diff;
    }

    return diff <= MAX_HEADING_DIFF_DEG;
}

void Cords::push_back(const Point& point)
{
    if (size < MAX_CORDS_SIZE)
    {
        data[size] = point;
        size++;
    }
}
