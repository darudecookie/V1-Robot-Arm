#include <iostream>
#include <stdio.h>
#include <chrono>
#include <cmath>

float angles[6] = {0};

float joint_dh_params[6][4] = {
    {angles[0], 90, 0, 217.75},
    {angles[1], 180, 255.5, 0},
    {angles[2], -90, 0, 97.75},
    {angles[3], 90, 0, 123.8},
    {angles[4], 90, 0, -70},
    {angles[5], 0, 5, 22.9},
};

void cross_mult(float output_matrix[4][4], float matrix1[4][4], float matrix2[4][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j, 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                output_matrix[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }
}

unsigned char *populate_transform_matrix(float dh_params[4])
{
    unsigned char *transform_matrix_address = (unsigned char *)malloc(sizeof(float[4][4]));

    float transform_matrix[4][4] = {
        {std::cos(0), -std::sin(dh_params[0]) * std::cos(dh_params[1]), std::sin(dh_params[0]) * std::sin(dh_params[1]), dh_params[2] * std::cos(dh_params[0])},
        {std::sin(dh_params[0]), std::cos(dh_params[0]) * std::cos(dh_params[1]), -std::cos(dh_params[0]) * std::sin(dh_params[1])},
        {0, std::sin(dh_params[1]), std::cos(dh_params[1]), dh_params[3]},
        {0, 0, 0, 1},
    };
    return (unsigned char *) (&transform_matrix);
}


int main()
{
    const long start_time = std::clock();

    float final_matrix[4][4] = {0};

    for (int i = 0; i < (sizeof(angles) / sizeof(angles[0])); i++)
    {
        cross_mult(final_matrix, joint_dh_params[i], final_matrix);
    }

    std::cout << std::clock - start_time << std::endl;

    return 0;
}