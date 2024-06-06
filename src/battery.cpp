#include "battery.hpp"
#include <algorithm>

LiPoBattery::LiPoBattery(double initialVoltage, double totalCapacity)
    : voltage(initialVoltage), totalCapacity(totalCapacity), remainingCapacity(totalCapacity) {
    cellCount = calculateCellCount(initialVoltage);
    voltageHistory.push_back(std::make_pair(std::time(0), initialVoltage));
}

LiPoBattery::LiPoBattery(double totalCapacity)
    : LiPoBattery(0.0, totalCapacity) {} // Delegate to the main constructor

void LiPoBattery::updateVoltage(double newVoltage) {
    voltage = newVoltage;
    voltageHistory.push_back(std::make_pair(std::time(0), newVoltage));

    if (voltageHistory.size() > maxEntries + 1) {
        voltageHistory.pop_front();
    }

    remainingCapacity = calculateRemainingCapacity(newVoltage);
}

int LiPoBattery::getCellCount() const {
    return cellCount;
}

double LiPoBattery::getVoltage() const {
    return voltage;
}

double LiPoBattery::estimateTimeLeft() const {
    if (voltageHistory.size() < 2) {
        return -1; // Not enough data to estimate
    }

    double totalDischargeTime = 0.0;
    for (size_t i = 1; i < voltageHistory.size(); ++i) {
        double voltageDiff = voltageHistory[i-1].second - voltageHistory[i].second;
        time_t timeDiff = voltageHistory[i].first - voltageHistory[i-1].first;
        if (voltageDiff != 0) {
            totalDischargeTime += timeDiff / voltageDiff;
        }
    }

    double averageDischargeRate = totalDischargeTime / (voltageHistory.size() - 1);
    double remainingVoltage = voltage - (cellCount * 3.7); // assuming 3.7V is the minimum safe voltage per cell
    return remainingVoltage * averageDischargeRate;
}

double LiPoBattery::getPercentage() const {
    return voltageToPercentage(voltage);
}

double LiPoBattery::getTotalCapacity() const {
    return totalCapacity;
}

double LiPoBattery::getRemainingCapacity() const {
    return remainingCapacity;
}

int LiPoBattery::calculateCellCount(double voltage) const {
    return static_cast<int>(voltage / 3.7 + 0.5); // 3.7V per cell (nominal voltage)
}

double LiPoBattery::voltageToPercentage(double voltage) const {
    double maxVoltage = cellCount * 4.2; // 4.2V per cell (fully charged)
    double minVoltage = cellCount * 3.7; // 3.7V per cell (nominal voltage)
    return std::clamp((voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0, 0.0, 100.0);
}

double LiPoBattery::calculateRemainingCapacity(double voltage) const {
    double percentage = voltageToPercentage(voltage) / 100.0;
    return totalCapacity * percentage;
}
