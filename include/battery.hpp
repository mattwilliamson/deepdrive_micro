#ifndef LIPOBATTERY_HPP
#define LIPOBATTERY_HPP

#include <deque>
#include <ctime>

class LiPoBattery {
public:
    LiPoBattery(double initialVoltage = 0.0, double totalCapacity = 1000.0); // Default initial voltage is 0.0V and default capacity is 1000mAh
    LiPoBattery(double totalCapacity); // Overloaded constructor for capacity only

    void updateVoltage(double newVoltage);
    int getCellCount() const;
    double getVoltage() const;
    double estimateTimeLeft() const;
    double getPercentage() const;
    double getTotalCapacity() const;
    double getRemainingCapacity() const;

private:
    double voltage;
    int cellCount;
    double totalCapacity; // in mAh
    double remainingCapacity; // in mAh
    std::deque<std::pair<time_t, double>> voltageHistory;

    int calculateCellCount(double voltage) const;
    double estimateDischargeTime(double voltage) const;
    double voltageToPercentage(double voltage) const;
    double calculateRemainingCapacity(double voltage) const;

    static const size_t maxEntries = 10;
};

#endif // LIPOBATTERY_HPP
