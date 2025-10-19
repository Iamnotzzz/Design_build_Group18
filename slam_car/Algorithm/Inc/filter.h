#ifndef __FILTER_H
#define __FILTER_H

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define IIR_MAX_ORDER 10                // 最大阶数，根据需要调整

// 复数结构体
typedef struct {
    double real;
    double imag;
} Complex;

typedef struct {
    int order;                    // 滤波器阶数
    double cutoff_freq;           // 截止频率(Hz)
    double sample_rate;           // 采样率(Hz)
    double poles_real[IIR_MAX_ORDER]; // 极点实部
    double poles_imag[IIR_MAX_ORDER]; // 极点虚部
    double a[IIR_MAX_ORDER + 1];      // 分母系数
    double b[IIR_MAX_ORDER + 1];      // 分子系数
} ButterworthLPF;

typedef struct {
    int order;                         // 阶数
    double b[IIR_MAX_ORDER + 1];       // 分子系数
    double a[IIR_MAX_ORDER + 1];       // 分母系数（a[0] 应该为 1）
    double x_hist[IIR_MAX_ORDER + 1];  // 输入历史
    double y_hist[IIR_MAX_ORDER];      // 输出历史
} IIRFilter;

void IIRFilter_Init(IIRFilter* f, int order, const float* b, const float* a);
int IIRFilter_Auto_Init(IIRFilter* f, int order, float fs, float fc);
float IIRFilter_Calculate(IIRFilter* f, float input);
void IIRFilter_Reset(IIRFilter* f);
int butterworth_lowpass(int order, float fs, float fc, float* b_out, float* a_out);

#endif
