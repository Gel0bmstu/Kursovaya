#ifndef IMU_BLOCK_H_
#define IMU_BLOCK_H_

#include <iostream>
#include <stdint.h>

class ImuBlock {
public:
    ImuBlock();
    ~ImuBlock();

    void setAngles(float psi, float teta, float gamma);
    int8_t *getAxelsData();
private:
};

#endif // IMU_BLOCK_H_