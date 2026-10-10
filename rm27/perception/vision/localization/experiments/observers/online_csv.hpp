// Instrumentation only: copied public API returns, no estimator mutation.
#pragma once
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace rm27_observer {
using Clock = std::chrono::steady_clock;
inline std::int64_t ns(Clock::time_point value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();
}
class Csv {
    std::ofstream output;
public:
    Csv() : output("online.csv", std::ios::out | std::ios::trunc) {
        if (!output) throw std::runtime_error("cannot open online observation log");
        output.exceptions(std::ios::badbit | std::ios::failbit);
        output << "input_index,backend_timestamp,processing_started_ns,processing_completed_ns,observation_completed_ns,raw_state,pose_present";
        for (int row=0; row<4; ++row)
            for (int col=0; col<4; ++col) output << ",t" << row << col;
        output << std::endl;
    }
    template<class Matrix>
    void write(unsigned index, double timestamp, Clock::time_point started,
               Clock::time_point completed, const std::string& state, const Matrix* pose) {
        const auto observed = Clock::now();
        output << index << ',' << std::setprecision(17) << timestamp << ','
               << ns(started) << ',' << ns(completed) << ',' << ns(observed) << ','
               << state << ',' << (pose ? 1 : 0);
        for (int row=0; row<4; ++row)
            for (int col=0; col<4; ++col) {
                output << ',';
                if (pose) output << (*pose)(row,col);
            }
        // Flush this runtime observation before accepting the next frame.
        output << std::endl;
    }
};
}
