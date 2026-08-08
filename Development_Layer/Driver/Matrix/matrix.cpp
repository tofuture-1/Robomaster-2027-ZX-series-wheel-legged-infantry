#include "matrix.h"
#include <cmath>

namespace
{
static bool Matrix_Check_Same_Shape(const Matrix_TypeDef *left,
                                    const Matrix_TypeDef *right,
                                    const Matrix_TypeDef *result)
{
    return Matrix_Is_Valid(left) &&
           Matrix_Is_Valid(right) &&
           Matrix_Is_Valid(result) &&
           (left->Rows == right->Rows) &&
           (left->Cols == right->Cols) &&
           (result->Rows == left->Rows) &&
           (result->Cols == left->Cols);
}

static bool Matrix_Check_Index(const Matrix_TypeDef *matrix, uint16_t row, uint16_t col)
{
    return Matrix_Is_Valid(matrix) && (row < matrix->Rows) && (col < matrix->Cols);
}
}

/**
* @brief 初始化矩阵对象
*
* @param matrix 目标矩阵对象
* @param rows 矩阵行数
* @param cols 矩阵列数
* @param data 外部提供的存储缓冲区
* @return bool 初始化是否成功
*/
bool Matrix_Init(Matrix_TypeDef *matrix, uint16_t rows, uint16_t cols, float *data)
{
    if ((matrix == nullptr) || (data == nullptr) || (rows == 0U) || (cols == 0U))
    {
        return false;
    }

    matrix->Rows = rows;
    matrix->Cols = cols;
    matrix->Data = data;
    return true;
}

/**
* @brief 判断矩阵对象是否有效
*
* @param matrix 目标矩阵对象
* @return bool 有效返回 true，否则返回 false
*/
bool Matrix_Is_Valid(const Matrix_TypeDef *matrix)
{
    return (matrix != nullptr) &&
           (matrix->Data != nullptr) &&
           (matrix->Rows > 0U) &&
           (matrix->Cols > 0U);
}

/**
* @brief 将矩阵所有元素清零
*
* @param matrix 目标矩阵对象
* @return bool 操作是否成功
*/
bool Matrix_Fill_Zero(Matrix_TypeDef *matrix)
{
    return Matrix_Fill(matrix, 0.0f);
}

/**
* @brief 将矩阵所有元素填充为指定值
*
* @param matrix 目标矩阵对象
* @param value 需要填充的值
* @return bool 操作是否成功
*/
bool Matrix_Fill(Matrix_TypeDef *matrix, float value)
{
    if (!Matrix_Is_Valid(matrix))
    {
        return false;
    }

    const uint32_t element_count = (uint32_t) matrix->Rows * (uint32_t) matrix->Cols;
    for (uint32_t i = 0; i < element_count; i++)
    {
        matrix->Data[i] = value;
    }

    return true;
}

/**
* @brief 将矩阵设置为单位阵
*
* @param matrix 目标矩阵对象，必须为方阵
* @return bool 操作是否成功
*/
bool Matrix_Set_Identity(Matrix_TypeDef *matrix)
{
    if (!Matrix_Is_Valid(matrix) || (matrix->Rows != matrix->Cols))
    {
        return false;
    }

    Matrix_Fill_Zero(matrix);
    for (uint16_t row = 0; row < matrix->Rows; row++)
    {
        matrix->Data[MATRIX_INDEX(matrix, row, row)] = 1.0f;
    }

    return true;
}

/**
* @brief 拷贝矩阵内容
*
* @param source 源矩阵
* @param destination 目标矩阵
* @return bool 操作是否成功
*/
bool Matrix_Copy(const Matrix_TypeDef *source, Matrix_TypeDef *destination)
{

    if (!Matrix_Is_Valid(source) ||
        !Matrix_Is_Valid(destination) ||
        (source->Rows != destination->Rows) ||
        (source->Cols != destination->Cols))
    {
        return false;
    }

    const uint32_t element_count = (uint32_t) source->Rows * (uint32_t) source->Cols;
    for (uint32_t i = 0; i < element_count; i++)
    {
        destination->Data[i] = source->Data[i];
    }

    return true;
}

/**
* @brief 设置指定元素
*
* @param matrix 目标矩阵对象
* @param row 行号，从 0 开始
* @param col 列号，从 0 开始
* @param value 需要写入的值
* @return bool 操作是否成功
*/
bool Matrix_Set_Element(Matrix_TypeDef *matrix, uint16_t row, uint16_t col, float value)
{

    if (!Matrix_Check_Index(matrix, row, col))
    {
        return false;
    }

    matrix->Data[MATRIX_INDEX(matrix, row, col)] = value;
    return true;
}

/**
* @brief 读取指定元素
*
* @param matrix 目标矩阵对象
* @param row 行号，从 0 开始
* @param col 列号，从 0 开始
* @param value 输出元素值的指针
* @return bool 操作是否成功
*/
bool Matrix_Get_Element(const Matrix_TypeDef *matrix, uint16_t row, uint16_t col, float *value)
{
    if (!Matrix_Check_Index(matrix, row, col) || (value == nullptr))
    {
        return false;
    }

    *value = matrix->Data[MATRIX_INDEX(matrix, row, col)];
    return true;
}

/**
* @brief 矩阵加法
*
* @param left 左操作数
* @param right 右操作数
* @param result 结果矩阵
* @return bool 操作是否成功
*/
bool Matrix_Add(const Matrix_TypeDef *left, const Matrix_TypeDef *right, Matrix_TypeDef *result)
{
    if (!Matrix_Check_Same_Shape(left, right, result))
    {
        return false;
    }

    const uint32_t element_count = (uint32_t) left->Rows * (uint32_t) left->Cols;
    for (uint32_t i = 0; i < element_count; i++)
    {
        result->Data[i] = left->Data[i] + right->Data[i];
    }

    return true;
}


/**
* @brief 矩阵减法
*
* @param left 左操作数
* @param right 右操作数
* @param result 结果矩阵
* @return bool 操作是否成功
*/
bool Matrix_Subtract(const Matrix_TypeDef *left, const Matrix_TypeDef *right, Matrix_TypeDef *result)
{
    if (!Matrix_Check_Same_Shape(left, right, result))
    {
        return false;
    }

    const uint32_t element_count = (uint32_t) left->Rows * (uint32_t) left->Cols;
    for (uint32_t i = 0; i < element_count; i++)
    {
        result->Data[i] = left->Data[i] - right->Data[i];
    }

    return true;
}

/**
* @brief 矩阵数乘
*
* @param matrix 输入矩阵
* @param scalar 标量系数
* @param result 结果矩阵
* @return bool 操作是否成功
*/
bool Matrix_Scale(const Matrix_TypeDef *matrix, float scalar, Matrix_TypeDef *result)
{
    if (!Matrix_Is_Valid(matrix) ||
        !Matrix_Is_Valid(result) ||
        (matrix->Rows != result->Rows) ||
        (matrix->Cols != result->Cols))
    {
        return false;
    }

    const uint32_t element_count = (uint32_t) matrix->Rows * (uint32_t) matrix->Cols;
    for (uint32_t i = 0; i < element_count; i++)
    {
        result->Data[i] = matrix->Data[i] * scalar;
    }

    return true;
}

/**
* @brief 矩阵乘法
*
* @param left 左操作数
* @param right 右操作数
* @param result 结果矩阵
* @return bool 操作是否成功
*/
bool Matrix_Multiply(const Matrix_TypeDef *left, const Matrix_TypeDef *right, Matrix_TypeDef *result)
{
    if (!Matrix_Is_Valid(left) ||
        !Matrix_Is_Valid(right) ||
        !Matrix_Is_Valid(result) ||
        (left->Cols != right->Rows) ||
        (result->Rows != left->Rows) ||
        (result->Cols != right->Cols))
    {
        return false;
    }

    for (uint16_t row = 0; row < result->Rows; row++)
    {
        for (uint16_t col = 0; col < result->Cols; col++)
        {
            float sum = 0.0f;
            for (uint16_t k = 0; k < left->Cols; k++)
            {
                sum += left->Data[MATRIX_INDEX(left, row, k)] *
                       right->Data[MATRIX_INDEX(right, k, col)];
            }
            result->Data[MATRIX_INDEX(result, row, col)] = sum;
        }
    }

    return true;
}

/**
* @brief 矩阵转置
*
* @param source 输入矩阵
* @param result 结果矩阵
* @return bool 操作是否成功
*/
bool Matrix_Transpose(const Matrix_TypeDef *source, Matrix_TypeDef *result)
{
    if (!Matrix_Is_Valid(source) ||
        !Matrix_Is_Valid(result) ||
        (result->Rows != source->Cols) ||
        (result->Cols != source->Rows))
    {
        return false;
    }

    for (uint16_t row = 0; row < source->Rows; row++)
    {
        for (uint16_t col = 0; col < source->Cols; col++)
        {
            result->Data[MATRIX_INDEX(result, col, row)] =
                    source->Data[MATRIX_INDEX(source, row, col)];
        }
    }

    return true;
}


/**
* @brief 矩阵与列向量相乘
*
* @param matrix 输入矩阵
* @param vector 输入列向量缓冲区
* @param vector_length 列向量长度
* @param result 输出结果缓冲区
* @return bool 操作是否成功
*/
bool Matrix_Multiply_Vector(const Matrix_TypeDef *matrix,
                            const float *vector,
                            uint16_t vector_length,
                            float *result)
{
    if (!Matrix_Is_Valid(matrix) ||
        (vector == nullptr) ||
        (result == nullptr) ||
        (matrix->Cols != vector_length))
    {
        return false;
    }

    for (uint16_t row = 0; row < matrix->Rows; row++)
    {
        float sum = 0.0f;
        for (uint16_t col = 0; col < matrix->Cols; col++)
        {
            sum += matrix->Data[MATRIX_INDEX(matrix, row, col)] * vector[col];
        }
        result[row] = sum;
    }

    return true;
}

/**
* @brief 求 3x3 矩阵逆
*
* @param source 输入矩阵，必须为 3x3
* @param result 结果矩阵，必须为 3x3
* @return bool 操作是否成功
*/
bool Matrix_Invert_3x3(const Matrix_TypeDef *source, Matrix_TypeDef *result)
{

    if (!Matrix_Is_Valid(source) ||
        !Matrix_Is_Valid(result) ||
        (source->Rows != 3U) ||
        (source->Cols != 3U) ||
        (result->Rows != 3U) ||
        (result->Cols != 3U))
    {
        return false;
    }

    const float a = source->Data[MATRIX_INDEX(source, 0U, 0U)];
    const float b = source->Data[MATRIX_INDEX(source, 0U, 1U)];
    const float c = source->Data[MATRIX_INDEX(source, 0U, 2U)];
    const float d = source->Data[MATRIX_INDEX(source, 1U, 0U)];
    const float e = source->Data[MATRIX_INDEX(source, 1U, 1U)];
    const float f = source->Data[MATRIX_INDEX(source, 1U, 2U)];
    const float g = source->Data[MATRIX_INDEX(source, 2U, 0U)];
    const float h = source->Data[MATRIX_INDEX(source, 2U, 1U)];
    const float i = source->Data[MATRIX_INDEX(source, 2U, 2U)];

    const float cofactor00 = e * i - f * h;
    const float cofactor01 = -(d * i - f * g);
    const float cofactor02 = d * h - e * g;
    const float cofactor10 = -(b * i - c * h);
    const float cofactor11 = a * i - c * g;
    const float cofactor12 = -(a * h - b * g);
    const float cofactor20 = b * f - c * e;
    const float cofactor21 = -(a * f - c * d);
    const float cofactor22 = a * e - b * d;

    const float determinant = a * cofactor00 + b * cofactor01 + c * cofactor02;
    if (std::fabs(determinant) < 1.0e-8f)
    {
        return false;
    }

    const float inv_determinant = 1.0f / determinant;
    result->Data[MATRIX_INDEX(result, 0U, 0U)] = cofactor00 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 0U, 1U)] = cofactor10 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 0U, 2U)] = cofactor20 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 1U, 0U)] = cofactor01 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 1U, 1U)] = cofactor11 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 1U, 2U)] = cofactor21 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 2U, 0U)] = cofactor02 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 2U, 1U)] = cofactor12 * inv_determinant;
    result->Data[MATRIX_INDEX(result, 2U, 2U)] = cofactor22 * inv_determinant;
    return true;
}
