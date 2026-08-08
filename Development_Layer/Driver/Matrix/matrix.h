#ifndef C_BOARD_MATRIX_H
#define C_BOARD_MATRIX_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t Rows;
    uint16_t Cols;
    float *Data;
} Matrix_TypeDef;

#define MATRIX_INDEX(matrix, row, col) ((uint32_t) (row) * (uint32_t) ((matrix)->Cols) + (uint32_t) (col))

#ifdef __cplusplus
extern "C"
{
#endif

bool Matrix_Init(Matrix_TypeDef *matrix, uint16_t rows, uint16_t cols, float *data);

bool Matrix_Is_Valid(const Matrix_TypeDef *matrix);

bool Matrix_Fill_Zero(Matrix_TypeDef *matrix);

bool Matrix_Fill(Matrix_TypeDef *matrix, float value);

bool Matrix_Set_Identity(Matrix_TypeDef *matrix);

bool Matrix_Copy(const Matrix_TypeDef *source, Matrix_TypeDef *destination);

bool Matrix_Set_Element(Matrix_TypeDef *matrix, uint16_t row, uint16_t col, float value);

bool Matrix_Get_Element(const Matrix_TypeDef *matrix, uint16_t row, uint16_t col, float *value);

bool Matrix_Add(const Matrix_TypeDef *left, const Matrix_TypeDef *right, Matrix_TypeDef *result);

bool Matrix_Subtract(const Matrix_TypeDef *left, const Matrix_TypeDef *right, Matrix_TypeDef *result);

bool Matrix_Scale(const Matrix_TypeDef *matrix, float scalar, Matrix_TypeDef *result);

bool Matrix_Multiply(const Matrix_TypeDef *left, const Matrix_TypeDef *right, Matrix_TypeDef *result);

bool Matrix_Transpose(const Matrix_TypeDef *source, Matrix_TypeDef *result);

bool Matrix_Multiply_Vector(const Matrix_TypeDef *matrix,
                            const float *vector,
                            uint16_t vector_length,
                            float *result);

bool Matrix_Invert_3x3(const Matrix_TypeDef *source, Matrix_TypeDef *result);

#ifdef __cplusplus
}
#endif

#endif // C_BOARD_MATRIX_H
