#include <gsl/gsl_matrix.h>
#include "tools.h"
#include "logger.h"



void tools_matrix_print(char *label, gsl_matrix *matrix) {
    if (matrix == NULL) {
        log_info("Null matrix");
        return;
    }
    char buf[1024];
    int i, j;
    int offset = 0;
    offset += snprintf(buf, sizeof(buf), "%s\n", label);
    for (i = 0; i < (int)matrix->size1; i++) {
        for (j = 0; j < (int)matrix->size2; j++) {
            offset += snprintf(buf + offset, sizeof(buf) - offset, "%6.3g ",
                              gsl_matrix_get(matrix, i, j));
        }
        offset += snprintf(buf+offset, sizeof(buf) - offset, "\n");
    }
    log_info("%s", buf);
}
