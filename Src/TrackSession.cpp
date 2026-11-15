#include "TrackSession.hpp"

#include <math.h>
#include <iostream>

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

Point TrackSession::findStartFinishPoint(std::size_t& p0_idx, int64_t eps)
{
    Point* lap_data = ref_lap.data + p0_idx;
    std::size_t lap_size = ref_lap.size - p0_idx;

    if (lap_size < 2)
    {
        sf_dir_x = 0;
        sf_dir_y = 0;
        return lap_size > 0 ? lap_data[0] : Point{0, 0, 0};
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

    // for (std::size_t i{1}; i < lap_size - 1; i++)
    // {
    //     int64_t dist = hypotsqrd(lap_data[top_split_idx], lap_data[i]);
    //     if (dist > max_dist)
    //     {
    //         max_dist = dist;
    //         bot_split_idx = i;
    //     }
    // } to do later, not that important

    eps *= eps; // making eps^2

    Cords first_half = DouglasPeucker(lap_data, top_split_idx + 1, eps);
    Cords second_half = DouglasPeucker(lap_data + top_split_idx, lap_size - top_split_idx, eps);

    int64_t first_len = hypotsqrd(first_half.data[0], first_half.data[first_half.size - 1]);
    int64_t second_len = hypotsqrd(second_half.data[0], second_half.data[second_half.size - 1]);

    Cords best = (first_len >= second_len) ? first_half : second_half;

    Point& A = best.data[0];
    Point& B = best.data[best.size - 1];

    sf_dir_x = static_cast<int64_t>(B.x) - A.x;
    sf_dir_y = static_cast<int64_t>(B.y) - A.y;

    return best.data[best.size / 2];
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

    constexpr int64_t MIN_DEPARTURE_DIST_MM = 20000; // 20 m
    constexpr int64_t MIN_DEPARTURE_DIST_SQ = MIN_DEPARTURE_DIST_MM * MIN_DEPARTURE_DIST_MM;

    std::size_t search_limit = ref_lap.size - 1;
    while (search_limit > 0 && hypotsqrd(curr_point, ref_lap.data[search_limit]) <= MIN_DEPARTURE_DIST_SQ)
    {
        search_limit--;
    }

    for (std::size_t i = 0; i < search_limit; i++)
    {
        if (curr_point.inRange(ref_lap.data[i]) && curr_point.isSameDirection(ref_lap.data[i]))
        {
            session_state = SessionState::NORMAL;

            Point close_point = curr_point;
            start_finish_point = findStartFinishPoint(i, 3000);

            std::size_t sf_idx = 0;
            while (sf_idx < ref_lap.size && ref_lap.data[sf_idx].t < start_finish_point.t)
            {
                sf_idx++;
            }

            std::size_t new_size = ref_lap.size - sf_idx;
            for (std::size_t j = 0; j < new_size; j++)
            {
                ref_lap.data[j] = ref_lap.data[sf_idx + j];
                ref_lap.data[j].t -= start_finish_point.t;
            }
            ref_lap.size = new_size;
            curr_lap.size = 0;
            waiting_first_cross = true;

            constexpr int64_t SF_GATE_RADIUS_MM = 12000;
            constexpr int64_t SF_GATE_RADIUS_SQ = SF_GATE_RADIUS_MM * SF_GATE_RADIUS_MM;
            is_crossing_startfinishpoint = (hypotsqrd(close_point, start_finish_point) < SF_GATE_RADIUS_SQ);
            prev_norm_point = close_point;
            has_prev_norm_point = true;

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

    constexpr int64_t SF_GATE_RADIUS_MM = 12000; // 12 m promien bramki start/meta (dla szerokich torow i wysokich predkosci)
    constexpr int64_t SF_GATE_RADIUS_SQ = SF_GATE_RADIUS_MM * SF_GATE_RADIUS_MM;

    bool in_sf_gate = hypotsqrd(curr_point, start_finish_point) < SF_GATE_RADIUS_SQ;
    int64_t curr_sf_proj = (static_cast<int64_t>(curr_point.x) - start_finish_point.x) * sf_dir_x +
                           (static_cast<int64_t>(curr_point.y) - start_finish_point.y) * sf_dir_y;
    bool crossed_sf_plane = in_sf_gate &&
                            curr_point.isSameDirection(start_finish_point) &&
                            (curr_sf_proj >= 0);

    auto computeCrossingOffsetMs = [&]() -> uint32_t {
        if (!has_prev_norm_point || curr_point.t <= prev_norm_point.t)
        {
            return 0;
        }
        int64_t prev_sf_proj = (static_cast<int64_t>(prev_norm_point.x) - start_finish_point.x) * sf_dir_x +
                               (static_cast<int64_t>(prev_norm_point.y) - start_finish_point.y) * sf_dir_y;
        if (prev_sf_proj < 0 && (curr_sf_proj - prev_sf_proj) > 0)
        {
            uint32_t step_dt = curr_point.t - prev_norm_point.t;
            return static_cast<uint32_t>(
                (curr_sf_proj * static_cast<int64_t>(step_dt)) / (curr_sf_proj - prev_sf_proj)
            );
        }
        return 0;
    };

    if (waiting_first_cross)
    {
        if (!in_sf_gate)
        {
            is_crossing_startfinishpoint = false;
        }
        else if (crossed_sf_plane && !is_crossing_startfinishpoint)
        {
            uint32_t dt_after_cross = computeCrossingOffsetMs();
            is_crossing_startfinishpoint = true;
            waiting_first_cross = false;
            uint32_t cross_t = (t > dt_after_cross) ? (t - dt_after_cross) : t;
            fastest_lap = cross_t - start_finish_point.t;
            curr_lap.size = 0;
            resetTimer(dt_after_cross);
            curr_point.t = dt_after_cross;
            prev_norm_point = curr_point;
            has_prev_norm_point = true;
            curr_lap.push_back(std::move(curr_point));
            return;
        }

        ref_lap.push_back(Point(x, y, t - start_finish_point.t, headMot));
        prev_norm_point = curr_point;
        has_prev_norm_point = true;
        return;
    }

    if (crossed_sf_plane && !is_crossing_startfinishpoint)
    {
        uint32_t dt_after_cross = computeCrossingOffsetMs();
        uint32_t lap_time = (t > dt_after_cross) ? (t - dt_after_cross) : t;

        is_crossing_startfinishpoint = true;

        if (lap_time < fastest_lap)
        {
            fastest_lap = lap_time;

            std::swap(ref_lap.data, curr_lap.data);
            ref_lap.size = curr_lap.size;
        }

        curr_lap.size = 0;

        resetTimer(dt_after_cross);
        curr_point.t = dt_after_cross;
    }
    else if (!in_sf_gate)
    {
        is_crossing_startfinishpoint = false;
    }

    if (ref_lap.size > 0)
    {
        int64_t closest_dist = INT64_MAX;
        std::size_t closest_dist_idx{};
        bool found_same_dir = false;

        for (std::size_t i = 0; i < ref_lap.size; i++)
        {
            if (!curr_point.isSameDirection(ref_lap.data[i]))
            {
                continue;
            }
            int64_t dist = hypotsqrd(ref_lap.data[i], curr_point);
            if (dist < closest_dist)
            {
                closest_dist = dist;
                closest_dist_idx = i;
                found_same_dir = true;
            }
        }

        if (!found_same_dir)
        {
            for (std::size_t i = 0; i < ref_lap.size; i++)
            {
                int64_t dist = hypotsqrd(ref_lap.data[i], curr_point);
                if (dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_dist_idx = i;
                }
            }
        }

        int32_t ref_t = static_cast<int32_t>(ref_lap.data[closest_dist_idx].t);

        auto interpolateSegmentTime = [&](std::size_t idx_a, std::size_t idx_b) -> bool {
            const Point& pa = ref_lap.data[idx_a];
            const Point& pb = ref_lap.data[idx_b];
            int64_t ux = static_cast<int64_t>(pb.x) - pa.x;
            int64_t uy = static_cast<int64_t>(pb.y) - pa.y;
            int64_t len_sq = ux * ux + uy * uy;
            if (len_sq <= 0)
            {
                return false;
            }
            int64_t wx = static_cast<int64_t>(curr_point.x) - pa.x;
            int64_t wy = static_cast<int64_t>(curr_point.y) - pa.y;
            int64_t dot = wx * ux + wy * uy;
            if (dot < 0 || dot > len_sq)
            {
                return false;
            }
            int64_t dt = static_cast<int64_t>(pb.t) - static_cast<int64_t>(pa.t);
            ref_t = static_cast<int32_t>(static_cast<int64_t>(pa.t) + (dot * dt) / len_sq);
            return true;
        };

        if (closest_dist_idx + 1 < ref_lap.size)
        {
            if (!interpolateSegmentTime(closest_dist_idx, closest_dist_idx + 1) && closest_dist_idx > 0)
            {
                interpolateSegmentTime(closest_dist_idx - 1, closest_dist_idx);
            }
        }
        else if (closest_dist_idx > 0)
        {
            interpolateSegmentTime(closest_dist_idx - 1, closest_dist_idx);
        }

        delta = static_cast<int32_t>(curr_point.t) - ref_t;
    }

    prev_norm_point = curr_point;
    has_prev_norm_point = true;
    curr_lap.push_back(std::move(curr_point));
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
    has_prev_norm_point = false;
    waiting_first_cross = false;
    is_crossing_startfinishpoint = false;

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
