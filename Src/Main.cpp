#ifdef __WIN32

#include <iostream>
#include <fstream>
#include <filesystem>
#include <cstdint>
#include <memory>
#include <chrono>
#include <cstdio>

#include "TrackSession.hpp"

union UBX_NAV_PVT {
    struct PVT {
        uint32_t iTOW;          // [ms] GPS time of week of the navigation epoch
        uint16_t year;          // [y] Year (UTC)
        uint8_t  month;         // [month] Month, range 1..12 (UTC)
        uint8_t  day;           // [d] Day of month, range 1..31 (UTC)
        uint8_t  hour;          // [h] Hour of day, range 0..23 (UTC)
        uint8_t  min;           // [min] Minute of hour, range 0..59 (UTC)
        uint8_t  sec;           // [s] Seconds of minute, range 0..60 (UTC)
        // Validity flags
        uint8_t  validDate     : 1; // 1 = valid UTC Date
        uint8_t  validTime     : 1; // 1 = valid UTC time of day
        uint8_t  fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty)
        uint8_t  validMag      : 1; // 1 = valid magnetic declination
        uint8_t                : 4;
        uint32_t tAcc;          // [ns] Time accuracy estimate (UTC)
        int32_t  nano;          // [ns] Fraction of second, range -1e9 .. 1e9 (UTC)
        uint8_t  fixType;       // GNSSfix Type
        // Fix status flags
        uint8_t  gnssFixOK     : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
        uint8_t  diffSoln      : 1; // 1 = differential corrections were applied
        uint8_t  psmState      : 3; // Power save mode state
        uint8_t  headVehValid  : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
        uint8_t  carrSoln      : 2; // Carrier phase range solution status
        // Additional flags
        uint8_t                : 5;
        uint8_t  confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
        uint8_t  confirmedDate : 1; // 1 = UTC Date validity could be confirmed
        uint8_t  confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
        uint8_t  numSV;         // Number of satellites used in Nav Solution
        int32_t  lon;           // [1e-7 deg] Longitude
        int32_t  lat;           // [1e-7 deg] Latitude
        int32_t  height;        // [mm] Height above ellipsoid
        int32_t  hMSL;          // [mm] Height above mean sea level
        uint32_t hAcc;          // [mm] Horizontal accuracy estimate
        uint32_t vAcc;          // [mm] Vertical accuracy estimate
        int32_t  velN;          // [mm/s] NED north velocity
        int32_t  velE;          // [mm/s] NED east velocity
        int32_t  velD;          // [mm/s] NED down velocity
        int32_t  gSpeed;        // [mm/s] Ground Speed (2-D)
        int32_t  headMot;       // [1e-5 deg] Heading of motion (2-D)
        uint32_t sAcc;          // [mm/s] Speed accuracy estimate
        uint32_t headAcc;       // [1e-5 deg] Heading accuracy estimate (both motion and vehicle)
        uint16_t pDOP;          // [0.01] Position DOP
        // Additional flags
        uint8_t  invalidLlh    : 1; // 1 = Invalid lon, lat, height and hMSL
        uint8_t                : 7;
        uint8_t  reserved0[5];  // Reserved
        int32_t  headVeh;       // [1e-5 deg] Heading of vehicle (2-D)
        int16_t  magDec;        // [1e-2 deg] Magnetic declination
        uint16_t magAcc;        // [1e-2 deg] Magnetic declination accuracy
    } pvt;

    uint8_t data[92];
};

namespace fs = std::filesystem;

int main()
{
    fs::path data_dir_path = "data";
    fs::path lap_data_path = "2026-11-08_22-23-59_ks_red_bull_ring-ks_red_bull_ring_bmw_m3_e92.ubx"; // this ubx file has no necessary frame elements, ex: ck_a/b, pure payload
    fs::path p = fs::current_path() / data_dir_path / lap_data_path;
    
    if(!fs::exists(p))
    {
        std::cerr << "Lap data file does not exist" << std::endl;
        return 1;
    }

    auto file_size = fs::file_size(p);
    auto num_of_elements = file_size / sizeof(UBX_NAV_PVT);

    std::unique_ptr<UBX_NAV_PVT[]> buf = std::make_unique<UBX_NAV_PVT[]>(num_of_elements);



    std::ifstream file(p, std::ifstream::binary);
    if(!file)
    {
        std::cerr << "Error during opening file " << p << std::endl;
        return 1;
    }

    file.read(reinterpret_cast<char*>(buf.get()->data), file_size);

    TrackSession track_session;
    track_session.start();

    auto start_time = std::chrono::steady_clock::now();
    uint32_t start_iTOW = buf[0].pvt.iTOW;

    std::size_t i{};
    while(i < num_of_elements)
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        if(elapsed_ms >= static_cast<int64_t>(buf[i].pvt.iTOW - start_iTOW))
        {
            while (i < num_of_elements && elapsed_ms >= static_cast<int64_t>(buf[i].pvt.iTOW - start_iTOW))
            {
                track_session.processData(buf[i].pvt.lat, buf[i].pvt.lon, buf[i].pvt.height, buf[i].pvt.velN, buf[i].pvt.velE, buf[i].pvt.headMot);
                i++;
            }

            // std::cout << "\x1B[2J\x1B[H";
            // std::cout << "Delta [ms]: " << (static_cast<double>(track_session.getDelta()) / 1000.0) << std::endl;
            // std::cout << "Elapsed time [s]: " << elapsed_ms / 1000 << std::endl;
            printf("\x1B[2J\x1B[H");
            printf("Delta [ms]: %f \n", (static_cast<double>(track_session.getDelta()) / 1000.0));
            printf("Elapsed time [s]: %d \n", elapsed_ms / 1000);
        }
    }

    return 0;
}

#endif
