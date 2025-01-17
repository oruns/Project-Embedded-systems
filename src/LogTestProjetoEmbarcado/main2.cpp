#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#define GYRO_TOLERANCE 0.05 // Limite de tolerância do giroscópio
#define PI 3.14159265358979323846

class Odometry {
private:
    double _gyroOffset = 0.0; // Offset do giroscópio

public:
    void processCSV(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Erro ao abrir o arquivo: " << filePath << std::endl;
            return;
        }

        std::string line;
        std::getline(file, line); // Ignorar o cabeçalho

        std::cout << "Lendo dados do arquivo..." << std::endl;
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string value;
            double local_w = 0.0, odometry_theta = 0.0;

            // Lê os valores da linha
            if (std::getline(ss, value, ',')) {
                local_w = std::stod(value);
            }
            if (std::getline(ss, value, ',')) {
                odometry_theta = std::stod(value);
            }
            // Processa os dados (Tira o ruído offset)
            double k = 0.05;
            int count = 0;

            while (count <= 10 && fabs(local_w) < GYRO_TOLERANCE) {
                _gyroOffset += local_w * k;
                count++;
            }
            //Transforma velocidade angular w em de º/s para rad/s
            local_w=(local_w - _gyroOffset) * (PI / 180.0);

            // Exibe os dados processados
            std::cout << "Velocidade Angular (local_w): " << local_w << " rad/s, "<< std::endl;
            std::cout<< "Movimento Angular (odometry_theta): " << odometry_theta << " rad" << std::endl;
        }

        file.close();
    }
};

int main() {
    Odometry odometry;
    std::string filePath = "filtered_robot_log.csv";

    odometry.processCSV(filePath);

    return 0;
}
