#include <stdio.h>
#include <time.h> 
#include <unistd.h>
#include <stdlib.h>
#include "bloom.h"

#define F_NAME "/mnt/md0/sean/gene.txt"
int main(void)
{

    FILE* fin = fopen(F_NAME, "r");

    double time_spent = 0.0;
    clock_t begin = clock();
/************************************************************/
    //init
    struct bloom bloom;
    double gb_size = 16.0;

    // (1GB / kmer-size / increased_size_by_hashing * xGB)
    int target_num = 1024 * 1024 * 1024 / 32 / 2 * (int)gb_size; // (8hash, 32bit length, 16GB test)
    char* buffer = (char *)malloc(sizeof(char) * (target_num + 256));
    fread(buffer, sizeof(char), target_num + 256, fin);
    bloom_init(&bloom, target_num, 0.01);

    //Do Bloomfilter
    for (int i = 0; i < target_num; ++i) {
        bloom_add(&bloom, buffer + i, 32);
    }
/************************************************************/
    clock_t end = clock();
    bloom_print(&bloom);

    time_spent += (double)(end - begin) / CLOCKS_PER_SEC;

    printf("The elapsed time is %f seconds", time_spent);
    printf("\nThe performance is %fGB/sec\n", gb_size / time_spent);
    return 0;


}

/** const void *buffer = "actgacctg";
    if (bloom_check(&bloom, buffer, 9)) {
        printf("It may be there!\n");
    }
 * bloom_add(&bloom, buffer, 9); */

