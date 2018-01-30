#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)


int main(int argc, char *argv[]) {
    gsl_matrix *pt;
    gsl_matrix *mult;
    gsl_matrix *ret;

    int i;
    int angle;
    int body_shift = 0;
    int arg_start = 1;

    if (argc < 4) {
        fprintf(stderr,"Usage: %s [-s shift] <x> <y> <angle> [additional angles...]\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    if (!strncmp(argv[1], "-s", 2)) {
        if (argc < 6) {
            fprintf(stderr,"Usage: %s [-s shift] <x> <y> <angle> [additional angles...]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
        body_shift = atoi(argv[2]);
        arg_start = 3;
    }

    if ((pt = gsl_matrix_alloc(2, 1)) == NULL ) {
        perror("Failed to allocate pt matrix");
        exit(EXIT_FAILURE);
    }
    if ((mult = gsl_matrix_alloc(2, 2)) == NULL) {
        perror("Failed to allocate mult matrix");
        exit(EXIT_FAILURE);
    }
    if ((ret = gsl_matrix_alloc(2, 1)) == NULL) {
        perror("Failed to allocate ret matrix");
        exit(EXIT_FAILURE);
    }

    gsl_matrix_set(pt, 0, 0, atoi(argv[arg_start])); // set x
    gsl_matrix_set(pt, 1, 0, atoi(argv[arg_start + 1])); // set y

    int count = 0;

    for (i = arg_start + 2; i < argc; i++) {
        angle = atoi(argv[i]);
        gsl_matrix_set(mult, 0, 0, cos(degreesToRadians(angle)));
        gsl_matrix_set(mult, 0, 1, -1 * sin(degreesToRadians(angle)));
        gsl_matrix_set(mult, 1, 0, sin(degreesToRadians(angle)));
        gsl_matrix_set(mult, 1, 1, cos(degreesToRadians(angle)));
#ifdef DEBUG
    printf("Original point:\n\t[[%.5f]\n\t[%.5f]]\n", gsl_matrix_get(pt, 0, 0), gsl_matrix_get(pt, 1, 0));
#endif

#ifdef DEBUG
        printf("[DEBUG] -- mult:\n\t[[%.5f, %.5f]\n\t[%.5f, %.5f]]\n",
                gsl_matrix_get(mult, 0, 0),
                gsl_matrix_get(mult, 0, 1),
                gsl_matrix_get(mult, 1, 0),
                gsl_matrix_get(mult, 1, 1));
#endif

        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mult, pt, 0.0, ret);
        printf("Rotation for angle %d. (%.2f, %.2f)\n", angle,
            gsl_matrix_get(ret, 0, 0) - body_shift,
            gsl_matrix_get(ret, 1, 0) - body_shift);
        count++;
        if (!((count) % 10)) {
            printf("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n");
        }
    }

    gsl_matrix_free(pt);
    gsl_matrix_free(mult);
    gsl_matrix_free(ret);
    exit(EXIT_SUCCESS);
}

