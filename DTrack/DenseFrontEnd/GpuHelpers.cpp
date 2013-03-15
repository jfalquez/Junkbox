#include "GpuHelpers.h"

unsigned int CheckMemoryCUDA() {
    cudaError_t err;

    size_t avail;
    size_t total;

    err = cudaMemGetInfo( &avail, &total );
    if(err != cudaSuccess) {
        std::cerr << "CheckMemoryCUDA: cudaMemGetInfo failed: " << err << std::endl;

        err = cudaDeviceReset();
        if(err != cudaSuccess) {
            std::cerr << "CheckMemoryCUDA: Unable to reset device: " << err << std::endl;
        } else {
            err = cudaMemGetInfo( &avail, &total );
        }
    }

    if( err == cudaSuccess ) {
        size_t used = total - avail;
        const unsigned bytes_per_mb = 1024*1000;
        std::cout << "CheckMemoryCUDA: Total = " << total/bytes_per_mb << ", Available = " << avail/bytes_per_mb << ", Used = " << used/bytes_per_mb << std::endl;
        return avail/bytes_per_mb;
    } else {
        std::cerr << "CheckMemoryCUDA: there is an irrecoverable error: " << err << std::endl;
    }
    return 0;
}
