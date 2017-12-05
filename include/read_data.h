//
// Created by wanghf on 17-12-2.
//

#ifndef OPTIMIZATION_READ_DATA_H
#define OPTIMIZATION_READ_DATA_H

#include "json.h"
#include <vector>
#include <string>
#include "utils.h"
using namespace std;

namespace sequence_optimization {
    class JsonReader {
    public:
        vector<string> globVector(const string& pattern);
        /**
         * 遍历文件夹读取数据,存储形式为tracklets
         * @param path
         * @return map: trackid -> measurements
         */
        map<int, vector<MeasurementObject>> read_by_tracklets_from_path(string path);

        /**
         * 遍历文件夹读取数据,存储形式为frames
         * @param path
         * @return map: video num -> measurements
         */
        map<string, map<int, vector<MeasurementObject >>> read_by_frames_from_path(string path);

    private:
        Utils utils;
    };
}

#endif //OPTIMIZATION_READ_DATA_H
