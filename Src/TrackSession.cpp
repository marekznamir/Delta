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

void TrackSession::resetTimer()
{
    #ifdef __WIN32

    start_time = std::chrono::high_resolution_clock::now();

    #elif STM32H7A3xx

    start_time = xTaskGetTickCount();

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

Point TrackSession::findStartFinishPoint() // temporary ai slop, to verify
{
    if (ref_lap.size < 2) {
        return ref_lap.size > 0 ? ref_lap.data[0] : Point{0, 0, 0};
    }
    constexpr int64_t STEP_DIST_MM = 5000;                      // krok geometryczny [5 m] wygladzajacy szum GPS
    constexpr int64_t STEP_DIST_SQ = STEP_DIST_MM * STEP_DIST_MM;
    constexpr int64_t TAN_INV = 8;                              // ~7 stopni tolerancji kata (1/8 = 0.125)
    constexpr int64_t MAX_DEV_MM = 4000;                        // maks. odchylenie od osi prostej [4 m]
    const std::size_t N = ref_lap.size;
    std::size_t best_start = 0;
    std::size_t best_end = 0;
    int64_t max_len_sq = -1;

    auto advanceByStep = [&](std::size_t from_idx, std::size_t limit_idx) -> std::size_t {
        std::size_t next_idx = from_idx + 1;
        while (next_idx < limit_idx &&
               hypotsqrd(ref_lap.data[from_idx % N], ref_lap.data[next_idx % N]) < STEP_DIST_SQ)
        {
            next_idx++;
        }
        return next_idx;
    };

    // Przeszukujemy petle krokiem geometrycznym (z uwzglednieniem zawiniecia na koncu tablicy)
    for (std::size_t i = 0; i < N; i = advanceByStep(i, N))
    {
        const std::size_t limit = i + N;
        std::size_t prev_j = i;
        std::size_t j = advanceByStep(prev_j, limit);

        while (j < limit)
        {
            const Point& p_start = ref_lap.data[i];
            const Point& p_prev  = ref_lap.data[prev_j % N];
            const Point& p_curr  = ref_lap.data[j % N];
            // Wektor calej prostej V = P_curr - P_start
            int64_t vx = static_cast<int64_t>(p_curr.x) - p_start.x;
            int64_t vy = static_cast<int64_t>(p_curr.y) - p_start.y;
            // Wektor ostatniego kroku geometrycznego dv = P_curr - P_prev
            int64_t dvx = static_cast<int64_t>(p_curr.x) - p_prev.x;
            int64_t dvy = static_cast<int64_t>(p_curr.y) - p_prev.y;
            int64_t dot   = dvx * vx + dvy * vy;
            int64_t cross = dvx * vy - dvy * vx;
            if (cross < 0) cross = -cross;
            // Jesli kierunek lokalny odchyla sie o wiecej niz ~7 stopni od osi prostej -> koniec prostej
            if (dot <= 0 || cross * TAN_INV > dot) {
                break;
            }
            // Dodatkowe sprawdzenie odchylenia srodkowego punktu od cieciwy (odrzuca dlugie, lagodne luki)
            const Point& p_mid_idx = ref_lap.data[((i + j) / 2) % N];
            int64_t mx = static_cast<int64_t>(p_mid_idx.x) - p_start.x;
            int64_t my = static_cast<int64_t>(p_mid_idx.y) - p_start.y;
            int64_t mid_cross = mx * vy - my * vx;
            int64_t len_sq = vx * vx + vy * vy;
            // |mid_cross| / sqrt(len_sq) > MAX_DEV_MM  <=>  (mid_cross / MAX_DEV_MM)^2 > len_sq
            int64_t scaled_cross = mid_cross / MAX_DEV_MM;
            if (scaled_cross * scaled_cross > len_sq) {
                break;
            }
            if (len_sq > max_len_sq) {
                max_len_sq = len_sq;
                best_start = i;
                best_end = j;
            }
            prev_j = j;
            j = advanceByStep(prev_j, limit);
        }
    }
    const Point& A = ref_lap.data[best_start % N];
    const Point& B = ref_lap.data[best_end % N];
    // Idealny srodek geometryczny na osi prostej
    int32_t mid_x = (A.x + B.x) / 2;
    int32_t mid_y = (A.y + B.y) / 2;
    Point sf(mid_x, mid_y, 0);

    // Pierwsza probka na prostej wjezdzajaca w strefe start/finish (zgodnie z detekcja w normal())
    std::size_t best_k = best_start % N;
    for (std::size_t k = best_start; k <= best_end; ++k)
    {
        if (ref_lap.data[k % N].inRange(sf)) {
            best_k = k % N;
            break;
        }
    }

    sf.t = ref_lap.data[best_k].t;
    sf.headMot = ref_lap.data[best_k].headMot;
    return sf;
}

void TrackSession::findFirstPos(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot)
{
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

            start_finish_point = findStartFinishPoint();

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
    bool curr_in_range = curr_point.inRange(start_finish_point);

    if (waiting_first_cross)
    {
        if (curr_in_range && !is_crossing_startfinishpoint)
        {
            is_crossing_startfinishpoint = true;
            waiting_first_cross = false;
            fastest_lap = t - start_finish_point.t;
            curr_lap.size = 0;
            resetTimer();
            curr_lap.push_back(Point(x, y, 0, headMot));
            return;
        }

        ref_lap.push_back(Point(x, y, t - start_finish_point.t, headMot));
        return;
    }

    if (curr_in_range && !is_crossing_startfinishpoint)
    {
        #ifdef __WIN32

        uint32_t lap_time = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::high_resolution_clock::now() - start_time).count());

        #elif STM32H7A3xx

        uint32_t lap_time = static_cast<uint32_t>((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS);
        
        #endif

        

        is_crossing_startfinishpoint = true;

        if (lap_time < fastest_lap)
        {
            fastest_lap = lap_time;

            std::swap(ref_lap.data, curr_lap.data);
            ref_lap.size = curr_lap.size;
        }

        curr_lap.size = 0;

        resetTimer();
        curr_point.t = 0;
    }
    else if (!curr_in_range)
    {
        is_crossing_startfinishpoint = false;
    }

    if (ref_lap.size > 0)
    {
        int64_t closest_dist = INT64_MAX;
        std::size_t closest_dist_idx{};

        for (std::size_t i = 0; i < ref_lap.size; i++)
        {
            int64_t dist = hypotsqrd(ref_lap.data[i], curr_point);
            if (dist < closest_dist)
            {
                closest_dist = dist;
                closest_dist_idx = i;
            }
        }

        delta = ref_lap.data[closest_dist_idx].t - curr_point.t;
    }

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

    delta = 0;
}

void TrackSession::processData(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot)
{
    constexpr int64_t VEL_LIMIT = 555LL * 555LL; // [mm/s]
    if((static_cast<int64_t>(velN) * static_cast<int64_t>(velN)) + (static_cast<int64_t>(velE) * static_cast<int64_t>(velE)) < VEL_LIMIT) { return; } // below 2 km/h not analysing anything

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
