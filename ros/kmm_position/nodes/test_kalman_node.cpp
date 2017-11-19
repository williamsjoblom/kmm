#include "kmm_position/Kalman.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main(int argc, char* argv[]) {
    // Construct the filter
    KalmanFilter kf;

    // List of noisy position measurements (y)
    std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
    };

    // Best guess of initial states
    Eigen::Vector3f x0;
    int initXPos = 0;
    int initYPos = 0;
    int initTheta = 0;
    x0 << initXPos, initYPos, initTheta;
    kf.init(x0);

    // Feed measurements into filter, output estimated states
    Eigen::Vector3f y(3);
    Eigen::Vector3f x_hat = kf.getState();
    std::cout << "x_hat[0]: " << x_hat.transpose() << std::endl;
    for (int i = 0; i < measurements.size(); i++) {
        x_hat = kf.getState();
        y << measurements[i];
        kf.predict(y);
        std::cout << "y[" << i << "] = " << y.transpose()
            << ", x_hat[" << i << "] = " << x_hat.transpose() << std::endl;
    }
    return 0;
}
