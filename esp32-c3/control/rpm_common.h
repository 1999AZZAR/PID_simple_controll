#ifndef RPM_COMMON_H
#define RPM_COMMON_H

inline void updateEMA(float& filteredValue, float newValue, float alpha) {
    filteredValue = (alpha * newValue) + ((1.0 - alpha) * filteredValue);
}

#endif // RPM_COMMON_H
