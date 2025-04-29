///////////////////////////////////
///// simulation of D Cache + mem
//////////////////////////////////
#include <stdint.h>

#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

/* memory */
static vector<uint8_t> memory;

extern "C" void mem_init() {  // init memory from program.mem
    ifstream memfile("program.mem");
    //printf("++++++++++++ MEMORY INIT +++++++++++++ \n");
    memory.resize(8192 * 8);
    for (int i = 0; i < 8192; i++) {
        uint64_t x;
        memfile >> hex >> x;
        // printf("%lx\n", x);
        for (int j = 0; j < 8; j++) {
            memory[i * 8 + j] = x % 256;
            x = x >> 8;
        }
    }
    memfile.close();
}

extern "C" void mem_write(int addr_int, int data_int, int byte3, int byte2,
                          int byte1, int byte0) {
    uint32_t addr = (uint32_t)addr_int;
    uint32_t data = (uint32_t)data_int;
    if (byte3) {
        memory[addr + 3] = data >> 24;
        //printf("MEM[%d]=%2x ", addr + 3, memory[addr + 3]);
    }
    if (byte2) {
        memory[addr + 2] = (data >> 16) % 256;
    }
    if (byte1) {
        memory[addr + 1] = (data >> 8) % 256;
    }
    if (byte0) {
        memory[addr + 0] = data % 256;
    }
}

extern "C" int mem_read(int addr) {
    int data;
    data = memory[addr];
    data += memory[addr + 1] << 8;
    data += memory[addr + 2] << 16;
    data += memory[addr + 3] << 24;
    return data;
}

extern "C" void mem_print() {
    for (int i = 0; i < 8192; i++) {
        
        uint64_t data=0;
        for (int j = 7; j >=0; j--) {
            data += (uint64_t)memory[i * 8 + j] << (j*8);
        }

    }
}
