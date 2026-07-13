#ifndef FT_COMPENSATOR_HPP
#define FT_COMPENSATOR_HPP

#include <vector>
#include <cmath>
#include "model_params.h" 

class FTCompensator {
public:
    FTCompensator() {}

    std::vector<double> predict(const std::vector<double>& joints) {
        if (joints.size() != 6) return std::vector<double>(6, 0.0);

        // 1. 标准化输入
        double x[6];
        for (int i = 0; i < 6; ++i) {
            x[i] = (joints[i] - scaler_x_mean[i]) / scaler_x_scale[i];
        }

        // 2. 网络前向传播 (根据你的新结构：6 -> 128 -> 128 -> 6)

        // Layer 1: 6 -> 128 (ReLU)
        double h1[128];
        dense_layer(x, 6, h1, 128, w1, b1, true);

        // Layer 2: 128 -> 128 (ReLU) 
        // 注意：这里输出维度改为了 128，原先可能是 128
        double h2[128]; 
        dense_layer(h1, 128, h2, 128, w2, b2, true);

        // Layer 3: 128 -> 6 (Linear / No Activation)
        // 这是最后一层，直接输出
        double out_scaled[6];
        dense_layer(h2, 128, out_scaled, 6, w3, b3, false);

        // 3. 反标准化输出
        std::vector<double> result(6);
        for (int i = 0; i < 6; ++i) {
            result[i] = out_scaled[i] * scaler_y_scale[i] + scaler_y_mean[i];
        }

        return result;
    }

private:
    void dense_layer(const double* input, int input_size, double* output, int output_size, 
                     const double* weights, const double* bias, bool relu) {
        for (int i = 0; i < output_size; ++i) {
            double sum = bias[i];
            for (int j = 0; j < input_size; ++j) {
                sum += weights[i * input_size + j] * input[j];
            }
            if (relu) {
                output[i] = (sum > 0.0) ? sum : 0.0;
            } else {
                output[i] = sum;
            }
        }
    }
};

#endif
