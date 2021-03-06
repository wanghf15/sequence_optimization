//
// Created by wanghf on 17-12-1.
//

#ifndef OPTIMIZATION_LOG_ERRO_INFO_H
#define OPTIMIZATION_LOG_ERRO_INFO_H

#include <map>
#include "vector"
#include "ls_optimization.h"
using namespace std;

namespace sequence_optimization {
    class Log_Error_Info {
    public:
        void showErrorStat(map<int,vector<double> > truth, map<int, vector<MeasurementObject >> estimation);
    };
}


#endif //OPTIMIZATION_LOG_ERRO_INFO_H
