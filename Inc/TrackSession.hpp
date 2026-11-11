#include <cstdint>
#include <utility>

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
    Point start_finish_point;
    int32_t delta;
    uint32_t fastest_lap = UINT32_MAX;
    uint32_t moving_samples_count = 0;
    bool waiting_first_cross = false;
    bool is_crossing_startfinishpoint = false;

    void startTimer();
    void resetTimer();

    std::pair<int32_t, int32_t> gpstoxy(int32_t& lat, int32_t& lon);

    Point findStartFinishPoint();

    void findFirstPos(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);
    void learning(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);
    void normal(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);

    
public:
    void start();

    void processData(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot,
                     uint8_t fixType = 3, bool gnssFixOK = true, uint32_t sAcc = 0);   

    int32_t getDelta();
};
