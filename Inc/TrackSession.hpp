#pragma once

#include <cstdint>
#include <utility>
#include <tuple>

#include "Cords.hpp"

enum class SessionState {
    NORMAL,
    LEARNING,
    FIRST_POS
};

class TrackSession {
private:
    SessionState session_state;

    int64_t lat0;
    int64_t lon0;

    int64_t cos_lat0; // * 10^6

    Cords ref_lap;
    Cords curr_lap;

    Point sf_point;
    int64_t sf_dir_x = 0;
    int64_t sf_dir_y = 0;
    int32_t map_mid_x;
    int32_t map_mid_y;

    Point prev_norm_point{};
    bool has_prev_norm_point = false;
    int32_t delta;
    uint32_t fastest_lap = UINT32_MAX;
    uint32_t moving_samples_count = 0;
    bool waiting_first_cross = false;
    bool is_crossing_startfinishpoint = false;

    void startTimer();
    void resetTimer(uint32_t offset_ms = 0);

    std::pair<int32_t, int32_t> gpstoxy(int32_t& lat, int32_t& lon);

    std::tuple<Point, std::size_t, int32_t, int32_t, int64_t, int64_t> findStartFinishPoint(std::size_t& p0_idx, int64_t eps);

    void findFirstPos(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);
    void learning(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);
    void normal(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);

    
public:
    void start();

    void processData(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot,
                     uint8_t fixType = 3, bool gnssFixOK = true, uint32_t sAcc = 0);   

    int32_t getDelta();
};
