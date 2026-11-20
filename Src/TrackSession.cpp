#include "TrackSession.hpp"

#include <math.h>
#include <algorithm>
#include <tuple>

#ifdef __WIN32
#include <chrono>

auto start_time = std::chrono::high_resolution_clock::now();
auto end_time = std::chrono::high_resolution_clock::now();

#elif STM32H7A3xx

#include "FreeRTOS.h"
#include "task.h"
TickType_t start_time;
TickType_t end_time;

#endif

void TrackSession::startTimer()
{
    #ifdef __WIN32

    start_time = std::chrono::high_resolution_clock::now();

    #elif STM32H7A3xx

    start_time = xTaskGetTickCount();

    #endif
}

void TrackSession::resetTimer(uint32_t offset_ms)
{
    #ifdef __WIN32

    start_time = std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(offset_ms);

    #elif STM32H7A3xx

    start_time = xTaskGetTickCount() - pdMS_TO_TICKS(offset_ms);

    #endif
}

std::pair<int32_t, int32_t> TrackSession::gpstoxy(int32_t& lat, int32_t& lon)
{
    constexpr int64_t R_MM = 6378137LL * 1000LL; // earths radius [mm]
    constexpr int64_t RAD_TO_DEG1E7 = 572957795LL; // (180 * 10^7) / pi

    int32_t x = static_cast<int32_t>((((R_MM * (static_cast<int64_t>(lon) - lon0)) / RAD_TO_DEG1E7) * cos_lat0) / 1000000); // [mm]
    int32_t y = static_cast<int32_t>((R_MM * (static_cast<int64_t>(lat) - lat0)) / RAD_TO_DEG1E7); // [mm]
    return { x, y };
}

int64_t perpendicular_distance_squared(Point& p0, Point& p1, Point& p2)
{
    int64_t dx = (static_cast<int64_t>(p2.x) - p1.x) / 100; // [dm] 
    int64_t dy = (static_cast<int64_t>(p2.y) - p1.y) / 100; // [dm]
    int64_t den = dx * dx + dy * dy;
    if (den == 0)
    {
        return hypotsqrd(p0, p1);
    }
    int64_t px = (static_cast<int64_t>(p0.x) - p1.x) / 100; // [dm]
    int64_t py = (static_cast<int64_t>(p0.y) - p1.y) / 100; // [dm]
    int64_t num = dy * px - dx * py;
    return ((num * num) / den) * 10000; // [mm^2]
}

Cords DouglasPeucker(Point* data, std::size_t size, int64_t eps_squared)
{
    if(size <= 2)
    {
        return Cords{ size, data };
    }

    std::size_t idx{};
    int64_t d_max{};

    for(std::size_t i{1}; i < size - 1; i++)
    {
        int64_t d = perpendicular_distance_squared(data[i], data[0], data[size - 1]);
        if(d > d_max) 
        {
            d_max = d;
            idx = i; 
        }
    }

    if(d_max > eps_squared)
    {
        auto left_part = DouglasPeucker(data, idx + 1, eps_squared);
        auto right_part = DouglasPeucker(data + idx, size - idx, eps_squared);

        if(hypotsqrd(left_part.data[0], left_part.data[left_part.size - 1]) > hypotsqrd(right_part.data[0], right_part.data[right_part.size - 1]))
        {
            return left_part;
        }
        else
        {
            return right_part;
        }
    }
    else
    {
        return Cords{ size, data };
    }

}

std::tuple<Point, std::size_t, int32_t, int32_t, int64_t, int64_t> TrackSession::findStartFinishPoint(std::size_t& p0_idx, int64_t eps) // put out of class
{
    Point* lap_data = ref_lap.data + p0_idx;
    std::size_t lap_size = ref_lap.size - p0_idx;

    if (lap_size < 3)
    {
        Point p = lap_size > 0 ? lap_data[0] : Point{0, 0, 0};
        return { p, p0_idx, p.x, p.y, 0, 0 };
    }

    std::size_t top_split_idx{};
    std::size_t bot_split_idx{};
    int64_t max_dist{};

    for (std::size_t i{1}; i < lap_size - 1; i++)
    {
        int64_t dist = hypotsqrd(lap_data[0], lap_data[i]);
        if (dist > max_dist)
        {
            max_dist = dist;
            top_split_idx = i;
        }
    }
    max_dist = 0;

    for (std::size_t i{1}; i < lap_size - 1; i++)
    {
        int64_t dist = hypotsqrd(lap_data[top_split_idx], lap_data[i]);
        if (dist > max_dist)
        {
            max_dist = dist;
            bot_split_idx = i;
        }
    }

    auto mid_x = static_cast<int32_t>((static_cast<int64_t>(lap_data[top_split_idx].x) + lap_data[bot_split_idx].x) / 2);
    auto mid_y = static_cast<int32_t>((static_cast<int64_t>(lap_data[top_split_idx].y) + lap_data[bot_split_idx].y) / 2);

    std::rotate(lap_data, lap_data + bot_split_idx, lap_data + lap_size);
    top_split_idx = (top_split_idx + lap_size - bot_split_idx) % lap_size;

    eps *= eps; // making eps^2

    Cords first_half = DouglasPeucker(lap_data, top_split_idx + 1, eps);
    Cords second_half = DouglasPeucker(lap_data + top_split_idx, lap_size - top_split_idx, eps);

    int64_t first_len = hypotsqrd(first_half.data[0], first_half.data[first_half.size - 1]);
    int64_t second_len = hypotsqrd(second_half.data[0], second_half.data[second_half.size - 1]);

    Cords best = (first_len >= second_len) ? first_half : second_half;

    Point& A = best.data[0];
    Point& B = best.data[best.size - 1];

    auto dir_x = static_cast<int64_t>(B.x) - A.x;
    auto dir_y = static_cast<int64_t>(B.y) - A.y;

    Point* sf_ptr = best.data + best.size / 2;
    Point sf = *sf_ptr;

    std::size_t orig_lap_sf_idx = (sf_ptr - lap_data + bot_split_idx) % lap_size;
    std::size_t orig_ref_sf_idx = p0_idx + orig_lap_sf_idx;               

    std::rotate(lap_data, sf_ptr, lap_data + lap_size);

    return { sf, orig_ref_sf_idx, mid_x, mid_y, dir_x, dir_y };
}

void TrackSession::findFirstPos(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot)
{
    constexpr uint32_t MIN_MOVING_SAMPLES = 5;
    moving_samples_count++;

    if (moving_samples_count < MIN_MOVING_SAMPLES)
    {
        return;
    }

    lon0 = static_cast<int64_t>(lon);
    lat0 = static_cast<int64_t>(lat);

    double lat0_rad = (static_cast<double>(lat0) / 10000000.0) * (M_PI / 180.0);
    cos_lat0 = static_cast<int64_t>(std::cos(lat0_rad) * 1000000.0);

    session_state = SessionState::LEARNING;

    startTimer();
}

void TrackSession::learning(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot)
{
    if (ref_lap.size < MAX_CORDS_SIZE)
    {
        #ifdef __WIN32

        uint32_t t = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::high_resolution_clock::now() - start_time).count());
        
        #elif STM32H7A3xx

        TickType_t ticks_elapsed = xTaskGetTickCount() - start_time;    
        uint32_t t = static_cast<uint32_t>(ticks_elapsed * portTICK_PERIOD_MS);

        #endif

        auto [x, y] = gpstoxy(lat, lon);
        ref_lap.push_back(Point(x, y, t, headMot));
    }

    if (ref_lap.size < 2)
    {
        return;
    }

    auto& curr_point = ref_lap.data[ref_lap.size - 1];

    constexpr int64_t MIN_DEPARTURE_DIST_SQ = 20000 * 20000; // 20 m

    std::size_t search_limit = ref_lap.size - 1;

    while (search_limit > 0 && hypotsqrd(curr_point, ref_lap.data[search_limit]) <= MIN_DEPARTURE_DIST_SQ) // Until we pass 20 meters, we won't proceed
    {
        search_limit--;
    }

    for (std::size_t i = 0; i < search_limit; i++)
    {
        if (curr_point.inRange(ref_lap.data[i]) && curr_point.isSameDirection(ref_lap.data[i]))
        {
            session_state = SessionState::NORMAL;

            auto t_loop_start = ref_lap.data[i].t;
            auto t_loop_end   = curr_point.t;
            
            std::size_t lap_size  = ref_lap.size - i;

            std::size_t sf_idx;
            std::tie(sf_point, sf_idx, map_mid_x, map_mid_y, sf_dir_x, sf_dir_y) = findStartFinishPoint(i, 3000);

            uint32_t sf_t = sf_point.t;
            for (std::size_t j = 0; j < lap_size; j++)
            {
                Point& temp_point = ref_lap.data[i + j];
                if (temp_point.t >= sf_t)
                {
                    temp_point.t = temp_point.t - sf_t;
                }
                else
                {
                    temp_point.t = (temp_point.t - t_loop_start) + (t_loop_end - sf_t);
                }
                ref_lap.data[j] = temp_point;
            }
            ref_lap.size = lap_size;
            fastest_lap  = t_loop_end - t_loop_start;
            curr_lap.size = 0;
            has_prev_sf_proj = false;
            wait_first_cross = true;
            break;
        }
    }
}

void TrackSession::normal(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot)
{
    #ifdef __WIN32

    uint32_t t = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - start_time).count());
        
    #elif STM32H7A3xx

    TickType_t ticks_elapsed = xTaskGetTickCount() - start_time;    
    uint32_t t = static_cast<uint32_t>(ticks_elapsed * portTICK_PERIOD_MS);

    #endif

    auto [x, y] = gpstoxy(lat, lon);
    Point curr_point(x, y, t, headMot);

    in_sf_gate = hypotsqrd(curr_point, sf_point) < SF_GATE_RADIUS_SQ;

    int64_t curr_sf_proj = (static_cast<int64_t>(curr_point.x) - sf_point.x) * sf_dir_x +
                       (static_cast<int64_t>(curr_point.y) - sf_point.y) * sf_dir_y;

    

    bool crossed_sf_plane = has_prev_sf_proj && in_sf_gate && curr_point.isSameDirection(sf_point) && (curr_sf_proj >= 0) && (prev_sf_proj < 0);

    const int64_t last_sf_proj = prev_sf_proj;
    const uint32_t last_t = prev_t;

    prev_sf_proj = curr_sf_proj;
    prev_t = t;
    has_prev_sf_proj = true;

    if(crossed_sf_plane)
    {
        auto t_prev = static_cast<int64_t>(last_t);
        auto t_curr = static_cast<int64_t>(t);
        int64_t den = curr_sf_proj - last_sf_proj;
        int64_t t_cross = t_prev + ((t_curr - t_prev) * -last_sf_proj) / den;
        auto dt_after = static_cast<uint32_t>(t_curr - t_cross);

        resetTimer(dt_after);
        prev_t = dt_after;
        curr_point.t = dt_after;

        if (wait_first_cross)
        {
            wait_first_cross = false;
            curr_lap.size = 0;
            curr_lap.push_back(curr_point);
            last_ref_idx = 0;
            return;
        }

        auto lap_time = static_cast<uint32_t>(t_cross);
        lap_count++;

        if(curr_lap.size > 1 && lap_time < fastest_lap)
        {
            fastest_lap = lap_time;
            std::swap(curr_lap.data, ref_lap.data);
            ref_lap.size = curr_lap.size;
        }

        last_ref_idx = 0;
        delta = 0;
        curr_lap.size = 0;
        curr_lap.push_back(curr_point);

        return;
    }

    if(wait_first_cross) { return; }

    if (ref_lap.size == 0)
    {
        curr_lap.push_back(curr_point);
        return;
    }

    constexpr std::size_t WINDOW_BACK = 50; // how many points forward and backwards to check
    constexpr std::size_t WINDOW_FWD  = 50;
    // add, when max distance is at last idx, take larger probe!!!!!!
    std::size_t start_idx = (last_ref_idx > WINDOW_BACK) ? (last_ref_idx - WINDOW_BACK) : 0; // to eliminate before 0
    std::size_t end_idx   = std::min(ref_lap.size - 1, last_ref_idx + WINDOW_FWD); // to get maximum last point

    int64_t min_dist = INT64_MAX;
    std::size_t min_idx = start_idx;
    for(std::size_t i{start_idx}; i <= end_idx; i++)
    {
        int64_t dist = hypotsqrd(curr_point, ref_lap.data[i]);
        if (dist < min_dist)
        {
            min_dist = dist;
            min_idx = i;
        }
    }

    last_ref_idx = min_idx;
    
    delta = static_cast<int32_t>(static_cast<int64_t>(curr_point.t) - static_cast<int64_t>(ref_lap.data[min_idx].t));

    curr_lap.push_back(curr_point);
}


void TrackSession::start()
{
    // find nearest track, if not set to first_pos to find reference lon0 & lat0
    session_state = SessionState::FIRST_POS;
    static Point ref_buf[MAX_CORDS_SIZE];
    static Point curr_buf[MAX_CORDS_SIZE];

    ref_lap.data = ref_buf;
    curr_lap.data = curr_buf;
    ref_lap.size = 0;
    curr_lap.size = 0;

    moving_samples_count = 0;
    sf_dir_x = 0;
    sf_dir_y = 0;

    in_sf_gate = false;
    wait_first_cross = true;
    prev_sf_proj = 0;
    prev_t = 0;
    has_prev_sf_proj = false;

    last_ref_idx = 0;
    fastest_lap = UINT32_MAX;
    delta = 0;
}

void TrackSession::processData(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot,
                               uint8_t fixType, bool gnssFixOK, uint32_t sAcc)
{
    constexpr uint32_t MAX_SPEED_ACC_MM_S = 400; // [mm/s] (~1.44 km/h)
    if (!gnssFixOK || fixType < 3 || sAcc > MAX_SPEED_ACC_MM_S)
    {
        moving_samples_count = 0;
        return;
    }

    constexpr int64_t VEL_LIMIT = 555LL * 555LL; // [mm/s]
    if((static_cast<int64_t>(velN) * static_cast<int64_t>(velN)) + (static_cast<int64_t>(velE) * static_cast<int64_t>(velE)) < VEL_LIMIT)
    {
        moving_samples_count = 0;
        return; // below 2 km/h not analysing anything
    }

    switch(session_state)
    {
        case SessionState::NORMAL:
        {
            normal(lat, lon, height, velN, velE, headMot);
            break;
        }

        case SessionState::LEARNING:
        {
            learning(lat, lon, height, velN, velE, headMot);
            break;
        }
        case SessionState::FIRST_POS:
        {
            findFirstPos(lat, lon, height, velN, velE, headMot);
            break;
        }
    }
}

int32_t TrackSession::getDelta()
{
    return delta;
}
