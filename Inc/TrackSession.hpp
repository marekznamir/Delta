#include <cstdint>
#include <utility>

enum class SessionState {
    LEARNING,
    DEFAULT,
    FIRST_POS
};

class TrackSession {
private:
    SessionState session_state;
    int64_t lat0;
    int64_t lon0;
    int64_t cos_lat0; // * 10^6

    std::pair<int32_t, int32_t> gpstoxy(int32_t& lat, int32_t& lon);
public:
    void start();

    void processData(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot);   
};