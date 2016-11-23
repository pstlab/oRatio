/*
* Copyright (C) 2016 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "../solver/solver.h"

int main(int argc, char** argv) {
    std::cout << "Starting oRatio solver.." << std::endl;
    std::vector<std::string> fs;
    for (int i = 1; i < argc; i++) {
        fs.push_back(argv[i]);
    }

    if (fs.empty()) {
        std::cerr << "no input files.." << std::endl;
        return -1;
    }

    oratio::solver s;

    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    std::cout << "Reading " << argc - 1 << " files.." << std::endl;

    bool r = s.read(fs);

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds parsing_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);

    std::cout << "Parsing time: " << parsing_time.count() << "ms" << std::endl;

    if (!r) {
        std::cout << "The initial problem is inconsistent.." << std::endl;
        return 0;
    }

    std::cout << "Solving the problem.." << std::endl;
    bool slv = s.solve();
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds solving_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

    std::cout << "Solving time: " << solving_time.count() << "ms" << std::endl;

    std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds total_time = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0);

    std::cout << "Total processing time: " << total_time.count() << "ms" << std::endl;

    if (slv) {
        std::cout << "We have found a solution!!" << std::endl;
    }
    else {
        std::cout << "The problem has no solution!!" << std::endl;
    }

    return 0;
}