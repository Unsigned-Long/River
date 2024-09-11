// River: A Tightly-Coupled Radar-Inertial Velocity Estimator Based on Continuous-Time Optimization
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/River.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RIVER_SAMPLING_HPP
#define RIVER_SAMPLING_HPP

#include <iostream>
#include <random>
#include <vector>

namespace ns_river {

    /**
     * @brief sampling the samples without replacement
     *
     * @param num the num of the samples to sampling
     * @param engine the random engine
     * @param start the start index
     * @param end the end index
     * @param step the step
     * @attention range: [start, end](step) i.e. for [1, 5](2) -> pool: {1, 3, 5}
     * @return std::vector<std::size_t>
     */
    static std::vector<std::size_t> SamplingWoutReplace(std::default_random_engine &engine,
                                                        std::size_t num,
                                                        std::size_t start,
                                                        std::size_t end,
                                                        std::size_t step = 1) {
        // create the pool for sampling
        std::vector<std::size_t> idxPool((end - start) / step + 1);
        for (int i = 0; i != static_cast<int>(idxPool.size()); ++i) {
            idxPool.at(i) = start + i * step;
        }
        std::vector<std::size_t> res(num);
        // the engine
        for (std::size_t i = 0; i != num; ++i) {
            // generate the random index
            std::uniform_int_distribution<std::size_t> ui(0, idxPool.size() - 1);
            std::size_t ridx = ui(engine);
            // record it
            res.at(i) = idxPool.at(ridx);
            // remove it
            idxPool.at(ridx) = idxPool.back();
            idxPool.pop_back();
        }
        return res;
    }

    /**
     * @brief sampling the samples without replacement
     *
     * @tparam ElemType the element type
     * @param engine the random engine
     * @param dataVec the data vector
     * @param num the num of the samples to sampling
     * @return std::vector<std::size_t>
     */
    template<typename ElemType>
    std::vector<std::size_t> SamplingWoutReplace(std::default_random_engine &engine,
                                                 const std::vector<ElemType> &dataVec,
                                                 std::size_t num) {
        return SamplingWoutReplace(engine, num, 0, dataVec.size() - 1, 1);
    }

    /**
     * @brief sampling the samples without replacement
     *
     * @tparam ElemType the element type
     * @param engine the random engine
     * @param dataVec the data vector
     * @param num the num of the samples to sampling
     * @return std::vector<ElemType>
     */
    template<typename ElemType>
    std::vector<ElemType> SamplingWoutReplace2(std::default_random_engine &engine,
                                               const std::vector<ElemType> &dataVec,
                                               std::size_t num) {
        std::vector<std::size_t> res = SamplingWoutReplace(engine, dataVec, num);
        std::vector<ElemType> samples(num);
        for (int i = 0; i != static_cast<int>(num); ++i) {
            samples.at(i) = dataVec.at(res.at(i));
        }
        return samples;
    }
}

#endif //RIVER_SAMPLING_HPP
