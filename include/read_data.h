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
        map<int, vector<vector<double >>> read_by_tracklets_from_path(string path);

        vector<vector<double >> read_by_tracklets_from_file(string filename);

        /**
         * 遍历文件夹读取数据,存储形式为frames
         * @param path
         * @return map: video num -> measurements
         */
        map<int, vector<vector<double >>> read_by_frames_from_path(string path);


        map<int, vector<vector<double >>> read_by_frames_from_file(string filename);


    private:
        // 观测量,{x-foex, y-foey, w, true_depth}
        map<int, vector<vector<double >>> measurements;
        Utils utils;
    };
}

#endif //OPTIMIZATION_READ_DATA_H
