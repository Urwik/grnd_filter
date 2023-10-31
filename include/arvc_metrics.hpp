#pragma once

#include <iostream>
#include <vector>
#include "arvc_utils_v2.hpp"

namespace arvc{
    class metrics
    {

    public:
        metrics(/* args */){}
        ~metrics(){}
        
        std::vector<float> precision;
        std::vector<float> recall;
        std::vector<float> f1_score;
        std::vector<float> accuracy;
        std::vector<int> tp_vector;
        std::vector<int> tn_vector;
        std::vector<int> fp_vector;
        std::vector<int> fn_vector;


        void show(){
            cout << endl;
            cout << "-----------------------------------------------" << endl;
            cout << "Metrics: " << endl;
            cout << " - Accuracy: " << arvc::mean(this->accuracy) << endl;
            cout << " - Precision: " << arvc::mean(this->precision) << endl;
            cout << " - Recall: " << arvc::mean(this->recall) << endl;
            cout << " - F1 Score: " << arvc::mean(this->f1_score) << endl;
            cout << " - TP: " << arvc::mean(this->tp_vector) << endl;
            cout << " - TN: " << arvc::mean(this->tn_vector) << endl;
            cout << " - FP: " << arvc::mean(this->fp_vector) << endl;
            cout << " - FN: " << arvc::mean(this->fn_vector) << endl;
        }

    };
}
