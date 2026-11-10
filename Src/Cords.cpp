#include "Cords.hpp"

int64_t hypotsqrd(Point& a, Point& b)
{
    int64_t dx = a.x - b.x;
    int64_t dy = a.y - b.y;
    return (dx * dx + dy * dy);
}

Point::Point(int32_t _x, int32_t _y, uint32_t _t) : x(_x), y(_y), t(_t) 
{

}

bool Point::inRange(Point& other)
{
    constexpr int64_t tolerance = 6000 * 6000; 

    return hypotsqrd(*this, other) < tolerance ? true : false;
}

void Cords::push_back(Point& point)
{
    if (size < MAX_CORDS_SIZE)
    {
        data[size] = point;
        size++;
    }
}

void Cords::push_back(Point point)
{
    if (size < MAX_CORDS_SIZE)
    {
        data[size] = point;
        size++;
    }
}

