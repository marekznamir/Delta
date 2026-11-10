#include "TrackSession.hpp"
#include <math.h>
#include <iostream>

std::pair<int32_t, int32_t> TrackSession::gpstoxy(int32_t& lat, int32_t& lon)
{
    constexpr int64_t R_MM = 6378137LL * 1000LL; // earths radius [mm]
    constexpr int64_t RAD_TO_DEG1E7 = 572957795LL; // (180 * 10^7) / pi

    int32_t x = static_cast<int32_t>((((R_MM * (static_cast<int64_t>(lon) - lon0)) / RAD_TO_DEG1E7) * cos_lat0) / 1000000); // [mm]
    int32_t y = static_cast<int32_t>((R_MM * (static_cast<int64_t>(lat) - lat0)) / RAD_TO_DEG1E7); // [mm]
    return { x, y };
}

void TrackSession::start()
{
    // find nearest track, if not set to first_pos to find reference lon0 & lat0
    session_state = SessionState::FIRST_POS;
}

void TrackSession::processData(int32_t& lat, int32_t& lon, int32_t& height, int32_t& velN, int32_t& velE, int32_t& headMot)
{
    switch(session_state)
    {
        case SessionState::DEFAULT:
        break;
        case SessionState::LEARNING:
        {
            auto [x, y] = gpstoxy(lat, lon);
            std::cout << x << ", " << y << std::endl;
            break;
        }
        case SessionState::FIRST_POS:
        {
            lon0 = static_cast<int64_t>(lon);
            lat0 = static_cast<int64_t>(lat);
            double lat0_rad = (static_cast<double>(lat0) / 10000000.0) * (M_PI / 180.0);
            cos_lat0 = static_cast<int64_t>(std::cos(lat0_rad) * 1000000.0);
            session_state = SessionState::LEARNING;
        }
        break;
    }
}