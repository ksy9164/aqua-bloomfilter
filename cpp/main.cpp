#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <string>

#include "bdbmpcie.h"

#define FILE_NAME "./sample.txt"

using namespace std;

int main(int argc, char** argv) {
    BdbmPcie* pcie = BdbmPcie::getInstance();
    uint8_t* dmabuf = (uint8_t*)pcie->dmaBuffer();

    FILE *fin = fopen(FILE_NAME, "rb");
    uint32_t file_size = 0;
    uint32_t buff_size = 0;

    /* Get file size */
    fseek(fin, 0, SEEK_END);
    file_size = ftell(fin);
    buff_size = file_size;
    rewind(fin);

    /* Read data from the file */
    char *gene_data = (char *)malloc(buff_size * sizeof(char));
    fread(gene_data, sizeof(char), file_size, fin);

    /* Put size */
    pcie->userWriteWord(0, file_size);

    /* Write to DMA-Buffer and Send Signal */
    printf("Send data to the Host! \n");
    fflush(stdout);
    uint32_t gene_idx = 0;
    while(gene_idx < file_size) {
        for (uint32_t i = 0; i < 16 * 512; i++) {
            if (gene_idx >= file_size)
                break;
            dmabuf[i] = (uint8_t)gene_data[gene_idx++];
        }

        if (gene_idx == file_size) {
            int left_data = gene_idx % (512 * 16);
            if (left_data % 16 == 0)
                pcie->userWriteWord(4, left_data / 16); 
            else
                pcie->userWriteWord(4, left_data / 16 + 1); 
        }
        else 
            pcie->userWriteWord(4, 512); // 512 x 16bytes
        uint32_t target = pcie->userReadWord(4);
    }
    printf("Data sending is done \n");
    fflush(stdout);
    sleep(1);

    ////////////////////////////data receiving////////////////////////////////////
    printf("Get data from the Host! \n");
    fflush(stdout);
    sleep(1);
    uint32_t output_cnt = 0;
    uint32_t bloomfilter_size = 512 * 1024 * 1024;
    uint32_t bl_idx = 0;

    char *bloomfilter = (char *)malloc(sizeof(char) * 1024 * 1024 * 512);

    while (output_cnt < bloomfilter_size) {
        pcie->userWriteWord(8, 512); // 512 x 16bytes
        uint32_t target = pcie->userReadWord(0);
        output_cnt += 512 * 16;
        for (uint32_t i = 0; i < 16 * 512; i++) {
            bloomfilter[bl_idx++] = dmabuf[i];
            printf("%c",dmabuf[i]);
        }
    }
    printf("FINISHED !!!!:) \n");
    sleep(1);
    fflush(stdout);

    ////////////////////////////////////////////////////////////////
    return 0;
}
