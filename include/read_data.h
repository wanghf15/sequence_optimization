//
// Created by wanghf on 17-12-2.
//

#ifndef OPTIMIZATION_READ_DATA_H
#define OPTIMIZATION_READ_DATA_H

#include "json.h"
#include <vector>
#include <string>
using namespace std;

namespace sequence_optimization {
    class JsonReader {
    public:
        vector<string> globVector(const string& pattern);
        map<int, vector<vector<double >>> read_by_tracklets_from_path(string path);
        map<int, vector<vector<double >>> read_by_frames_from_path(string path);
        map<int, vector<vector<double >>> read_by_frames_from_file(string path, string filename);
        map<int, vector<vector<double >>> readgt_by_tacklets_from_path(string path);

    private:
        // 观测量,{x-foex, y-foey, w, true_depth}
        map<int, vector<vector<double >>> measurements;
    };
}

#endif //OPTIMIZATION_READ_DATA_H
