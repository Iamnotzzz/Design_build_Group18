#include "filter.h"
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <complex.h>

/**
 * @brief 初始化滤波器
 * @param f 滤波器对象
 * @param order 阶数
 * @param b 分子系数数组（长度 = order+1）
 * @param a 分母系数数组（长度 = order+1）
 */
void IIRFilter_Init(IIRFilter* f, int order, const float* b, const float* a) {
    f->order = order;
    memcpy(f->b, b, (order + 1) * sizeof(float));
    memcpy(f->a, a, (order + 1) * sizeof(float));
    memset(f->x_hist, 0, sizeof(f->x_hist));
    memset(f->y_hist, 0, sizeof(f->y_hist));

    // 归一化 a[0]
    if (f->a[0] != 1.0f) {
        float a0 = f->a[0];
        for (int i = 0; i <= order; i++) {
            f->b[i] /= a0;
            f->a[i] /= a0;
        }
    }
}

int IIRFilter_Auto_Init(IIRFilter* f, int order, float fs, float fc) {
    f->order = order;
    int result;
    float a[order + 1], b[order + 1];
    result = butterworth_lowpass(order, fs, fc, b, a);
    if(result != 0) return result;

    memcpy(f->b, b, (order + 1) * sizeof(float));
    memcpy(f->a, a, (order + 1) * sizeof(float));
    memset(f->x_hist, 0, sizeof(f->x_hist));
    memset(f->y_hist, 0, sizeof(f->y_hist));

    // 归一化 a[0]
    if (f->a[0] != 1.0f) {
        float a0 = f->a[0];
        for (int i = 0; i <= order; i++) {
            f->b[i] /= a0;
            f->a[i] /= a0;
        }
    }
    return 0;
}

/**
 * @brief 计算滤波输出
 * @param f 滤波器对象
 * @param input 输入样本
 * @return 输出结果
 */
float IIRFilter_Calculate(IIRFilter* f, float input) {
    int n = f->order;

    // 输入历史右移
    for (int i = n; i > 0; i--) {
        f->x_hist[i] = f->x_hist[i - 1];
    }
    f->x_hist[0] = input;

    // 计算输出
    float output = f->b[0] * input;
    for (int i = 1; i <= n; i++) {
        output += f->b[i] * f->x_hist[i];
        output -= f->a[i] * f->y_hist[i - 1];
    }

    // 输出历史右移
    for (int i = n - 1; i > 0; i--) {
        f->y_hist[i] = f->y_hist[i - 1];
    }
    f->y_hist[0] = output;

    return output;
}

/**
 * @brief 重置滤波器
 */
void IIRFilter_Reset(IIRFilter* f) {
    memset(f->x_hist, 0, sizeof(f->x_hist));
    memset(f->y_hist, 0, sizeof(f->y_hist));
}

/**
 * @brief 计算 n 阶低通巴特沃斯滤波器的系数（双线性变换法）
 * @param order 滤波器阶数，建议 <= IIR_MAX_ORDER
 * @param fs 采样频率 Hz
 * @param fc 截止频率 Hz
 * @param b_out 分子系数数组，长度 order+1
 * @param a_out 分母系数数组，长度 order+1
 */
// 复数运算函数
Complex complex_polar(float r, float theta) {
    Complex result;
    result.real = r * cosf(theta);
    result.imag = r * sinf(theta);
    return result;
}

Complex complex_add(Complex a, Complex b) {
    Complex result;
    result.real = a.real + b.real;
    result.imag = a.imag + b.imag;
    return result;
}

Complex complex_subtract(Complex a, Complex b) {
    Complex result;
    result.real = a.real - b.real;
    result.imag = a.imag - b.imag;
    return result;
}

Complex complex_multiply(Complex a, Complex b) {
    Complex result;
    result.real = a.real * b.real - a.imag * b.imag;
    result.imag = a.real * b.imag + a.imag * b.real;
    return result;
}

Complex complex_divide(Complex a, Complex b) {
    Complex result;
    float denominator = b.real * b.real + b.imag * b.imag;
    result.real = (a.real * b.real + a.imag * b.imag) / denominator;
    result.imag = (a.imag * b.real - a.real * b.imag) / denominator;
    return result;
}

Complex complex_scale(Complex a, float scalar) {
    Complex result;
    result.real = a.real * scalar;
    result.imag = a.imag * scalar;
    return result;
}

Complex complex_negate(Complex a) {
    Complex result;
    result.real = -a.real;
    result.imag = -a.imag;
    return result;
}

/**
 * 上层封装函数: 计算巴特沃斯低通滤波器系数
 * 
 * @param order   滤波器阶数 (1 到 MAX_ORDER)
 * @param fs      采样率 (Hz)
 * @param fc      截止频率 (Hz)
 * @param b_out   输出分子系数数组 (至少需要 order+1 个元素)
 * @param a_out   输出分母系数数组 (至少需要 order+1 个元素)
 * @return        0: 成功, -1: 参数错误
 * 
 * 使用示例:
 *   float b[5], a[5];
 *   int result = butterworth_lowpass(4, 8000.0, 1000.0, b, a);
 */
int butterworth_lowpass(int order, float fs, float fc, float* b_out, float* a_out) {
    // 参数验证
    if (order < 1 || order > IIR_MAX_ORDER) {
//        printf("错误: 阶数必须在1到%d之间\n", MAX_ORDER);
        return -1;
    }
    
    if (fc <= 0 || fs <= 0) {
//        printf("错误: 频率必须大于0\n");
        return -1;
    }
    
    if (fc >= fs / 2.0f) {
//        printf("错误: 截止频率必须小于采样率的一半(奈奎斯特频率)\n");
        return -1;
    }
    
    if (b_out == NULL || a_out == NULL) {
//        printf("错误: 输出数组不能为空\n");
        return -1;
    }
    
    // 计算预畸变系数 k
    float const k = tanf(PI * fc / fs);
    
    // 初始化复数分母系数数组
    Complex complex_a[IIR_MAX_ORDER + 1];
    memset(complex_a, 0, sizeof(complex_a));
    complex_a[0].real = 1.0f;
    complex_a[0].imag = 0.0f;
    
    int deg = 0;
    
    // 计算每个极点并更新分母系数
    for (int m = 0; m < order; ++m) {
        // 计算极点角度
        float const theta = PI * (2.0f * (float)m + 1.0f + (float)order) / 
                           (2.0f * (float)order);
        
        // 极点 p = e^(j*theta)
        Complex const p = complex_polar(1.0f, theta);
        
        // 双线性变换: z = (1 + k*p) / (1 - k*p)
        Complex numerator;
        numerator.real = 1.0f + k * p.real;
        numerator.imag = k * p.imag;
        
        Complex denominator;
        denominator.real = 1.0f - k * p.real;
        denominator.imag = -k * p.imag;
        
        Complex const z = complex_divide(numerator, denominator);
        
        // 更新多项式系数: 乘以 (z - z_pole)
        ++deg;
        Complex const neg_z = complex_negate(z);
        
        for (int r = deg; r > 0; --r) {
            Complex temp = complex_multiply(neg_z, complex_a[r - 1]);
            complex_a[r] = complex_add(complex_a[r], temp);
        }
    }
    
    // 计算分子系数 b (二项式系数)
    b_out[0] = 1.0f;
    for (int i = 1; i <= order; ++i) {
        b_out[i] = b_out[i - 1] * (float)(order - (i - 1)) / (float)i;
    }
    
    // 计算增益归一化因子
    float sumA = 0.0f;
    for (int i = 0; i <= order; ++i) {
        sumA += complex_a[i].real;
    }
    
    float sumB = 0.0f;
    for (int i = 0; i <= order; ++i) {
        sumB += b_out[i];
    }
    
    float const g = sumA / sumB;
    
    // 应用增益到分子系数
    for (int i = 0; i <= order; ++i) {
        b_out[i] *= g;
    }
    
    // 归一化使得 a[0] = 1
    float const a0 = complex_a[0].real;
    float const inv_a0 = 1.0f / a0;
    
    for (int i = 0; i <= order; ++i) {
        complex_a[i] = complex_scale(complex_a[i], inv_a0);
        b_out[i] *= inv_a0;
    }
    
    // 提取实部作为分母系数
    for (int i = 0; i <= order; ++i) {
        a_out[i] = complex_a[i].real;
    }
    
    return 0;
}




















