#pragma once

#include <cstdint>
#include <utility>
#include <tuple>

#include "Cords.hpp"

constexpr int64_t SF_GATE_RADIUS_MM = 12000;
constexpr int64_t SF_GATE_RADIUS_SQ = SF_GATE_RADIUS_MM * SF_GATE_RADIUS_MM;

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

    uint32_t moving_samples_count = 0;

    int32_t delta;
    uint32_t fastest_lap = UINT32_MAX;
    
    uint16_t lap_count = 0;

    bool in_sf_gate = false;
    bool wait_first_cross = true;

    int64_t prev_sf_proj = 0;
    uint32_t prev_t = 0;   
    bool has_prev_sf_proj = false;

    std::size_t last_ref_idx{};

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
