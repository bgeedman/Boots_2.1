#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

void print_matrix(gsl_matrix *matrix) {
    int i, j;
    for (i = 0; i< matrix->size1; i++) {
        for (j=0; j<matrix->size2; j++) {
            printf("%4g ", gsl_matrix_get(matrix, i, j));
        }
        printf("\n");
    }
}

int main(int argc, char *argv[]) {
    gsl_matrix *matrix1, *matrix2;
    if (NULL == (matrix1 = gsl_matrix_calloc(3,3))) {
        printf("Failed to create matrix1\n");
        exit(1);
    }
    if (NULL == (matrix2 = gsl_matrix_calloc(3,1))) {
        printf("Failed to create matrix2\n");
        gsl_matrix_free(matrix1);
        exit(1);
    }
    gsl_matrix *product;
    if (NULL == (product = gsl_matrix_calloc(3, 1))) {
        printf("Failed to create product matrix\n");
        gsl_matrix_free(matrix1);
        gsl_matrix_free(matrix2);
        exit(1);
    }
    gsl_matrix_set(matrix1, 0, 0, 4);
    gsl_matrix_set(matrix1, 0, 2, 5);
    gsl_matrix_set(matrix1, 1, 1, 2);
    gsl_matrix_set(matrix1, 2, 2, 1);
    gsl_matrix_set(matrix2, 0, 0, 4);
    gsl_matrix_set(matrix2, 1, 0, 5);
    gsl_matrix_set(matrix2, 2, 0, 6);
    printf("Matrix 1:\n");
    print_matrix(matrix1);
    printf("\nMatrix 2:\n");
    print_matrix(matrix2);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, matrix1, matrix2, 0.0, product);
    printf("\n\nProduct:\n");
    print_matrix(product);
    gsl_matrix_free(matrix1);
    gsl_matrix_free(matrix2);
    gsl_matrix_free(product);
    exit(0);
}
