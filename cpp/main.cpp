#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <string>

#include "bdbmpcie.h"

/* #define FILE_NAME "./sample.txt" */
#define FILE_NAME "/mnt/md0/sean/sample.txt"

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

    uint32_t comp_buff_size = buff_size / 4;
    uint8_t *comp_gene_data = (uint8_t *)malloc(comp_buff_size * sizeof(uint8_t));

    cout << "\n size is \n" << comp_buff_size;
    fflush(stdout);

    int miss_data = 0;

    for (int i = 0; i < comp_buff_size; ++i) { // transform file to compressed format
        int idx = i * 4;
        uint8_t val = 0;
        for (int j = 0; j < 4; j++) {
            int key = idx + j;
            char gene_char = gene_data[key];

            switch (gene_char) {
                case 'A':
                case 'a':
                    val = (val << 2) | 0;
                    break;

                case 'T':
                case 't':
                    val = (val << 2) | 1;
                    break;

                case 'C':
                case 'c':
                    val = (val << 2) | 2;
                    break;

                case 'G':
                case 'g':
                    val = (val << 2) | 3;
                    break;

                default :
                    cout << "inaccurate values" << val << endl;
                    miss_data++;
                    j++;
            }
        }
        comp_gene_data[i] = val;
    }

    cout << "miss data is " << miss_data << endl;
    /* Put size */
    pcie->userWriteWord(0, comp_buff_size);

    /* Write to DMA-Buffer and Send Signal */
    printf("Send data to the Host! \n");
    fflush(stdout);
    uint32_t gene_idx = 0;
    while(gene_idx < comp_buff_size) {
        for (uint32_t i = 0; i < 16 * 512; i++) { // 8KB
            if (gene_idx >= comp_buff_size)
                break;
            dmabuf[i] = (uint8_t)comp_gene_data[gene_idx++];
        }

        cout << "@@@@ " << gene_idx << " " << comp_buff_size << " \n";
        fflush(stdout);
        if (gene_idx == comp_buff_size) {
            int left_data = gene_idx % (512 * 16);
            if (left_data % 16 == 0)
                pcie->userWriteWord(4, left_data / 16); 
            else
                pcie->userWriteWord(4, left_data / 16 + 1); 
        }
        else 
            pcie->userWriteWord(4, 512); // 512 x 16bytes
        uint32_t target = pcie->userReadWord(4); // to get writeDone signal
        fflush(stdout);
    }
    printf("Data sending is done \n");
    fflush(stdout);
    sleep(100);

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
